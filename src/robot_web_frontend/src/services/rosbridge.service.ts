import ROSLIB from 'roslib';
import { ROBOT_CONFIG } from '../config/robot.config';

type ConnectionCallback = (connected: boolean) => void;

const RECONNECT_DELAY_MS = 3000;

interface TFTransform {
  translation: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number; w: number };
}

class RosbridgeService {
  private ros: ROSLIB.Ros | null = null;
  private connectPromise: Promise<void> | null = null;
  private reconnectTimer: number | null = null;
  private intentionalDisconnect = false;

  private cmdVelPublisher: ROSLIB.Topic | null = null;
  private mapSubscriber: ROSLIB.Topic | null = null;
  private odomSubscriber: ROSLIB.Topic | null = null;
  private voltageSubscriber: ROSLIB.Topic | null = null;
  private currentASubscriber: ROSLIB.Topic | null = null;
  private currentBSubscriber: ROSLIB.Topic | null = null;
  private tfSubscriber: ROSLIB.Topic | null = null;

  // 訂閱登記表：記錄「想要訂閱」的 callback。
  // 連線尚未建立時先登記，連線（或斷線重連）成功後由 restoreSubscriptions() 統一重建。
  private mapCallback: ((map: OccupancyGridData) => void) | null = null;
  private odomCallback: ((odom: OdomData) => void) | null = null;
  private voltageCallback: ((voltage: number) => void) | null = null;
  private currentsCallback: ((currentA: number, currentB: number) => void) | null = null;
  private robotPoseCallback: ((pose: RobotPoseInMap) => void) | null = null;

  // TF 累積資料 (用於計算 map->base_footprint)
  private tfData: { [key: string]: TFTransform } = {};
  private connected = false;
  private connectionCallbacks: Set<ConnectionCallback> = new Set();

  connect(): Promise<void> {
    // 冪等：已連線直接成功，連線中回傳既有 Promise
    if (this.connected) {
      return Promise.resolve();
    }
    if (this.connectPromise) {
      return this.connectPromise;
    }

    this.intentionalDisconnect = false;

    // 清除殘留的舊連線，避免 WebSocket 洩漏
    if (this.ros) {
      try {
        this.ros.close();
      } catch {
        // ignore
      }
      this.ros = null;
    }

    this.connectPromise = new Promise((resolve, reject) => {
      const ros = new ROSLIB.Ros({ url: ROBOT_CONFIG.ROSBRIDGE_URL });
      this.ros = ros;
      let settled = false;

      ros.on('connection', () => {
        if (this.ros !== ros) return; // 已被更新的連線取代
        console.log('Connected to rosbridge');
        this.connectPromise = null;
        this.setupTopics();
        this.setConnected(true);
        this.restoreSubscriptions();
        if (!settled) {
          settled = true;
          resolve();
        }
      });

      ros.on('error', (error) => {
        if (this.ros !== ros) return;
        console.error('Rosbridge error:', error);
        if (!settled) {
          settled = true;
          this.connectPromise = null;
          reject(error ?? new Error('rosbridge connection error'));
        }
      });

      ros.on('close', () => {
        if (this.ros !== ros) return; // 被孤立的舊連線關閉，忽略
        console.log('Rosbridge connection closed');
        this.connectPromise = null;
        this.setConnected(false);
        this.clearTopicHandles();
        if (!settled) {
          settled = true;
          reject(new Error('rosbridge connection closed'));
        }
        if (!this.intentionalDisconnect) {
          this.scheduleReconnect();
        }
      });
    });

    return this.connectPromise;
  }

  disconnect(): void {
    this.intentionalDisconnect = true;
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    if (this.ros) {
      try {
        this.ros.close();
      } catch {
        // ignore
      }
      this.ros = null;
    }
    this.connectPromise = null;
    this.clearTopicHandles();
    this.setConnected(false);
  }

  isConnected(): boolean {
    return this.connected;
  }

  onConnectionChange(callback: ConnectionCallback): () => void {
    this.connectionCallbacks.add(callback);
    callback(this.connected);
    return () => this.connectionCallbacks.delete(callback);
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer !== null) return;
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect().catch(() => {
        // 連線失敗時由 close handler 再度排程重連
      });
    }, RECONNECT_DELAY_MS);
  }

  private setConnected(value: boolean): void {
    if (this.connected !== value) {
      this.connected = value;
      this.connectionCallbacks.forEach((cb) => cb(value));
    }
  }

  // 連線斷開後，舊的 Topic 物件已失效，清除引用（登記表保留，重連後重建）
  private clearTopicHandles(): void {
    this.cmdVelPublisher = null;
    this.mapSubscriber = null;
    this.odomSubscriber = null;
    this.voltageSubscriber = null;
    this.currentASubscriber = null;
    this.currentBSubscriber = null;
    this.tfSubscriber = null;
  }

  private setupTopics(): void {
    if (!this.ros) return;

    // cmd_vel publisher
    this.cmdVelPublisher = new ROSLIB.Topic({
      ros: this.ros,
      name: ROBOT_CONFIG.TOPICS.CMD_VEL,
      messageType: 'geometry_msgs/msg/Twist',
    });
  }

  // 依登記表重建所有訂閱（連線建立與斷線重連時呼叫）
  private restoreSubscriptions(): void {
    if (this.mapCallback) this.doSubscribeMap();
    if (this.odomCallback) this.doSubscribeOdom();
    if (this.voltageCallback) this.doSubscribeVoltage();
    if (this.currentsCallback) this.doSubscribeCurrents();
    if (this.robotPoseCallback) this.doSubscribeRobotPoseInMap();
  }

  // Publish velocity command
  publishCmdVel(linear: number, angular: number): void {
    if (!this.cmdVelPublisher || !this.connected) return;

    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    });

    this.cmdVelPublisher.publish(twist);
  }

  // Subscribe to map with throttling for performance
  subscribeToMap(callback: (map: OccupancyGridData) => void): void {
    this.mapCallback = callback;
    if (this.ros && this.connected) {
      this.doSubscribeMap();
    }
  }

  private doSubscribeMap(): void {
    if (!this.ros || !this.mapCallback) return;

    // 取消現有訂閱，避免重複
    if (this.mapSubscriber) {
      this.mapSubscriber.unsubscribe();
    }

    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    this.mapSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: ROBOT_CONFIG.TOPICS.MAP,
      messageType: 'nav_msgs/msg/OccupancyGrid',
      throttle_rate: 500,  // 每 500ms 最多收一次 (2Hz)
    } as any);

    this.mapSubscriber.subscribe((message: unknown) => {
      this.mapCallback?.(message as OccupancyGridData);
    });
  }

  unsubscribeFromMap(): void {
    this.mapCallback = null;
    if (this.mapSubscriber) {
      this.mapSubscriber.unsubscribe();
      this.mapSubscriber = null;
    }
  }

  // Subscribe to odometry
  subscribeToOdom(callback: (odom: OdomData) => void): void {
    this.odomCallback = callback;
    if (this.ros && this.connected) {
      this.doSubscribeOdom();
    }
  }

  private doSubscribeOdom(): void {
    if (!this.ros || !this.odomCallback) return;

    if (this.odomSubscriber) {
      this.odomSubscriber.unsubscribe();
    }

    this.odomSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: ROBOT_CONFIG.TOPICS.ODOM,
      messageType: 'nav_msgs/msg/Odometry',
    });

    this.odomSubscriber.subscribe((message: unknown) => {
      this.odomCallback?.(message as OdomData);
    });
  }

  unsubscribeFromOdom(): void {
    this.odomCallback = null;
    if (this.odomSubscriber) {
      this.odomSubscriber.unsubscribe();
      this.odomSubscriber = null;
    }
  }

  // Subscribe to robot pose in map frame via TF
  // Directly subscribe to /tf topic and compute map->base_footprint transform
  subscribeToRobotPoseInMap(callback: (pose: RobotPoseInMap) => void): void {
    this.robotPoseCallback = callback;
    if (this.ros && this.connected) {
      this.doSubscribeRobotPoseInMap();
    }
  }

  private doSubscribeRobotPoseInMap(): void {
    if (!this.ros || !this.robotPoseCallback) return;

    if (this.tfSubscriber) {
      this.tfSubscriber.unsubscribe();
    }
    this.tfData = {};

    this.tfSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/tf',
      messageType: 'tf2_msgs/msg/TFMessage',
    });

    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    this.tfSubscriber.subscribe((message: any) => {
      // 儲存所有 TF 變換
      for (const transform of message.transforms) {
        const key = `${transform.header.frame_id}->${transform.child_frame_id}`;
        this.tfData[key] = {
          translation: transform.transform.translation,
          rotation: transform.transform.rotation,
        };
      }

      // 計算 map->base_footprint (通過 map->odom->base_footprint)
      // hs_motor_controller 發布 odom->base_footprint (base_link_frame: base_footprint)
      // slam_toolbox / AMCL 發布 map->odom
      const mapToOdom = this.tfData['map->odom'];
      const odomToBase = this.tfData['odom->base_footprint'];

      if (!odomToBase || !this.robotPoseCallback) return;

      if (mapToOdom) {
        this.robotPoseCallback(this.composeTF(mapToOdom, odomToBase));
      } else {
        // map->odom 尚未發布時（如僅遙控、SLAM 未啟動）視為 identity，退化為 odom 位姿
        this.robotPoseCallback(this.tfToPose(odomToBase));
      }
    });
  }

  private quatToYaw(q: { x: number; y: number; z: number; w: number }): number {
    return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  private tfToPose(tf: TFTransform): RobotPoseInMap {
    return {
      x: tf.translation.x,
      y: tf.translation.y,
      yaw: this.quatToYaw(tf.rotation),
    };
  }

  // 組合兩個 TF 變換
  private composeTF(tf1: TFTransform, tf2: TFTransform): RobotPoseInMap {
    // 簡化計算：假設只有 yaw 旋轉 (2D 導航)
    const yaw1 = this.quatToYaw(tf1.rotation);
    const yaw2 = this.quatToYaw(tf2.rotation);

    // 旋轉 tf2 的 translation
    const cos1 = Math.cos(yaw1);
    const sin1 = Math.sin(yaw1);
    const rotatedX = tf2.translation.x * cos1 - tf2.translation.y * sin1;
    const rotatedY = tf2.translation.x * sin1 + tf2.translation.y * cos1;

    return {
      x: tf1.translation.x + rotatedX,
      y: tf1.translation.y + rotatedY,
      yaw: yaw1 + yaw2,
    };
  }

  unsubscribeFromRobotPoseInMap(): void {
    this.robotPoseCallback = null;
    if (this.tfSubscriber) {
      this.tfSubscriber.unsubscribe();
      this.tfSubscriber = null;
    }
    this.tfData = {};
  }

  // Subscribe to motor voltage
  subscribeToVoltage(callback: (voltage: number) => void): void {
    this.voltageCallback = callback;
    if (this.ros && this.connected) {
      this.doSubscribeVoltage();
    }
  }

  private doSubscribeVoltage(): void {
    if (!this.ros || !this.voltageCallback) return;

    if (this.voltageSubscriber) {
      this.voltageSubscriber.unsubscribe();
    }

    this.voltageSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/motor/voltage',
      messageType: 'std_msgs/msg/Float32',
    });

    this.voltageSubscriber.subscribe((message: unknown) => {
      const msg = message as { data: number };
      this.voltageCallback?.(msg.data);
    });
  }

  unsubscribeFromVoltage(): void {
    this.voltageCallback = null;
    if (this.voltageSubscriber) {
      this.voltageSubscriber.unsubscribe();
      this.voltageSubscriber = null;
    }
  }

  // Subscribe to motor currents
  subscribeToCurrents(callback: (currentA: number, currentB: number) => void): void {
    this.currentsCallback = callback;
    if (this.ros && this.connected) {
      this.doSubscribeCurrents();
    }
  }

  private doSubscribeCurrents(): void {
    if (!this.ros || !this.currentsCallback) return;

    if (this.currentASubscriber) {
      this.currentASubscriber.unsubscribe();
    }
    if (this.currentBSubscriber) {
      this.currentBSubscriber.unsubscribe();
    }

    let currentA = 0;
    let currentB = 0;

    this.currentASubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/motor/current_a',
      messageType: 'std_msgs/msg/Float32',
    });

    this.currentBSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/motor/current_b',
      messageType: 'std_msgs/msg/Float32',
    });

    this.currentASubscriber.subscribe((message: unknown) => {
      const msg = message as { data: number };
      currentA = msg.data;
      this.currentsCallback?.(currentA, currentB);
    });

    this.currentBSubscriber.subscribe((message: unknown) => {
      const msg = message as { data: number };
      currentB = msg.data;
      this.currentsCallback?.(currentA, currentB);
    });
  }

  unsubscribeFromCurrents(): void {
    this.currentsCallback = null;
    if (this.currentASubscriber) {
      this.currentASubscriber.unsubscribe();
      this.currentASubscriber = null;
    }
    if (this.currentBSubscriber) {
      this.currentBSubscriber.unsubscribe();
      this.currentBSubscriber = null;
    }
  }
}

// Type definitions
export interface OccupancyGridData {
  info: {
    resolution: number;
    width: number;
    height: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: number[];
}

export interface OdomData {
  pose: {
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
}

// Robot pose in map coordinate frame (used for correct display during SLAM and navigation)
export interface RobotPoseInMap {
  x: number;
  y: number;
  yaw: number;
}

export const rosbridgeService = new RosbridgeService();
