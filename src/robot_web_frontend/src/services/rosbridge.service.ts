import ROSLIB from 'roslib';
import { ROBOT_CONFIG } from '../config/robot.config';

type ConnectionCallback = (connected: boolean) => void;

class RosbridgeService {
  private ros: ROSLIB.Ros | null = null;
  private cmdVelPublisher: ROSLIB.Topic | null = null;
  private mapSubscriber: ROSLIB.Topic | null = null;
  private odomSubscriber: ROSLIB.Topic | null = null;
  private voltageSubscriber: ROSLIB.Topic | null = null;
  private currentASubscriber: ROSLIB.Topic | null = null;
  private currentBSubscriber: ROSLIB.Topic | null = null;
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private tfClient: any = null;
  private tfSubscriber: ROSLIB.Topic | null = null;
  private robotPoseCallback: ((pose: RobotPoseInMap) => void) | null = null;
  // TF 累積資料 (用於計算 map->base_footprint)
  private tfData: { [key: string]: { translation: {x: number, y: number, z: number}, rotation: {x: number, y: number, z: number, w: number} } } = {};
  private connected = false;
  private connectionCallbacks: Set<ConnectionCallback> = new Set();

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ros = new ROSLIB.Ros({ url: ROBOT_CONFIG.ROSBRIDGE_URL });

      this.ros.on('connection', () => {
        console.log('Connected to rosbridge');
        this.setConnected(true);
        this.setupTopics();
        resolve();
      });

      this.ros.on('error', (error) => {
        console.error('Rosbridge error:', error);
        reject(error);
      });

      this.ros.on('close', () => {
        console.log('Rosbridge connection closed');
        this.setConnected(false);
      });
    });
  }

  disconnect(): void {
    if (this.ros) {
      this.ros.close();
      this.ros = null;
      this.setConnected(false);
    }
  }

  isConnected(): boolean {
    return this.connected;
  }

  onConnectionChange(callback: ConnectionCallback): () => void {
    this.connectionCallbacks.add(callback);
    callback(this.connected);
    return () => this.connectionCallbacks.delete(callback);
  }

  private setConnected(value: boolean): void {
    if (this.connected !== value) {
      this.connected = value;
      this.connectionCallbacks.forEach((cb) => cb(value));
    }
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
    if (!this.ros || !this.connected) return;

    // 取消現有訂閱
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
      callback(message as OccupancyGridData);
    });
  }

  unsubscribeFromMap(): void {
    if (this.mapSubscriber) {
      this.mapSubscriber.unsubscribe();
      this.mapSubscriber = null;
    }
  }

  // Subscribe to odometry
  subscribeToOdom(callback: (odom: OdomData) => void): void {
    if (!this.ros) return;

    this.odomSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: ROBOT_CONFIG.TOPICS.ODOM,
      messageType: 'nav_msgs/msg/Odometry',
    });

    this.odomSubscriber.subscribe((message: unknown) => {
      callback(message as OdomData);
    });
  }

  unsubscribeFromOdom(): void {
    if (this.odomSubscriber) {
      this.odomSubscriber.unsubscribe();
      this.odomSubscriber = null;
    }
  }

  // Subscribe to robot pose in map frame via TF
  // Directly subscribe to /tf topic and compute map->base_footprint transform
  subscribeToRobotPoseInMap(callback: (pose: RobotPoseInMap) => void): void {
    if (!this.ros || !this.connected) {
      console.warn('[TF] Cannot subscribe: ros not connected');
      return;
    }

    this.robotPoseCallback = callback;
    this.tfData = {};

    console.log('[TF] Subscribing to /tf topic directly');

    // eslint-disable-next-line @typescript-eslint/no-explicit-any
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
      const mapToOdom = this.tfData['map->odom'];
      const odomToBase = this.tfData['odom->base_footprint'];

      if (mapToOdom && odomToBase && this.robotPoseCallback) {
        // 組合兩個變換
        const pose = this.composeTF(mapToOdom, odomToBase);
        this.robotPoseCallback(pose);
      }
    });
  }

  // 組合兩個 TF 變換
  private composeTF(
    tf1: { translation: {x: number, y: number, z: number}, rotation: {x: number, y: number, z: number, w: number} },
    tf2: { translation: {x: number, y: number, z: number}, rotation: {x: number, y: number, z: number, w: number} }
  ): RobotPoseInMap {
    // 簡化計算：假設只有 yaw 旋轉 (2D 導航)
    const q1 = tf1.rotation;
    const yaw1 = Math.atan2(2.0 * (q1.w * q1.z + q1.x * q1.y), 1.0 - 2.0 * (q1.y * q1.y + q1.z * q1.z));

    const q2 = tf2.rotation;
    const yaw2 = Math.atan2(2.0 * (q2.w * q2.z + q2.x * q2.y), 1.0 - 2.0 * (q2.y * q2.y + q2.z * q2.z));

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
    if (this.tfSubscriber) {
      this.tfSubscriber.unsubscribe();
      this.tfSubscriber = null;
    }
    if (this.tfClient) {
      this.tfClient.unsubscribe('base_footprint');
      this.tfClient = null;
    }
    this.robotPoseCallback = null;
    this.tfData = {};
  }

  // Subscribe to motor voltage
  subscribeToVoltage(callback: (voltage: number) => void): void {
    if (!this.ros) return;

    this.voltageSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/motor/voltage',
      messageType: 'std_msgs/msg/Float32',
    });

    this.voltageSubscriber.subscribe((message: unknown) => {
      const msg = message as { data: number };
      callback(msg.data);
    });
  }

  unsubscribeFromVoltage(): void {
    if (this.voltageSubscriber) {
      this.voltageSubscriber.unsubscribe();
      this.voltageSubscriber = null;
    }
  }

  // Subscribe to motor currents
  subscribeToCurrents(callback: (currentA: number, currentB: number) => void): void {
    if (!this.ros) return;

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
      callback(currentA, currentB);
    });

    this.currentBSubscriber.subscribe((message: unknown) => {
      const msg = message as { data: number };
      currentB = msg.data;
      callback(currentA, currentB);
    });
  }

  unsubscribeFromCurrents(): void {
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
