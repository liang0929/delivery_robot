import ROSLIB from 'roslib';
import { ROBOT_CONFIG } from '../config/robot.config';

class RosbridgeService {
  private ros: ROSLIB.Ros | null = null;
  private cmdVelPublisher: ROSLIB.Topic | null = null;
  private mapSubscriber: ROSLIB.Topic | null = null;
  private odomSubscriber: ROSLIB.Topic | null = null;
  private voltageSubscriber: ROSLIB.Topic | null = null;
  private currentASubscriber: ROSLIB.Topic | null = null;
  private currentBSubscriber: ROSLIB.Topic | null = null;
  private connected = false;

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ros = new ROSLIB.Ros({ url: ROBOT_CONFIG.ROSBRIDGE_URL });

      this.ros.on('connection', () => {
        console.log('Connected to rosbridge');
        this.connected = true;
        this.setupTopics();
        resolve();
      });

      this.ros.on('error', (error) => {
        console.error('Rosbridge error:', error);
        reject(error);
      });

      this.ros.on('close', () => {
        console.log('Rosbridge connection closed');
        this.connected = false;
      });
    });
  }

  disconnect(): void {
    if (this.ros) {
      this.ros.close();
      this.ros = null;
      this.connected = false;
    }
  }

  isConnected(): boolean {
    return this.connected;
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

export const rosbridgeService = new RosbridgeService();
