# Robot Web Frontend

Web-based control interface for the robot.

## Features

1. **Remote Control** - Virtual joystick for manual robot control
2. **SLAM Mapping** - Create and save maps using SLAM
3. **Navigation** - Click on map to navigate to goals

## Setup

```bash
cd ~/base_dev/src/robot_web_frontend
npm install
```

## Running

### Recommended: Use bringup.launch.py (includes all services)

```bash
ros2 launch motor_control bringup.launch.py
```

This will start:
- Motor controller, LiDAR, IMU, EKF
- rosbridge WebSocket server
- API server

### Start the web frontend (development)

```bash
cd ~/base_dev/src/robot_web_frontend
npm run dev
```

Access the web interface at: http://<robot-ip>:3000

### For SLAM Mapping

Use the web interface "Start Mapping" button, or manually:
```bash
ros2 launch nav2 mapping.launch.py
```

### For Navigation

Use the web interface "Start Navigation" button, or manually:
```bash
ros2 launch nav2 autonomous_navigation.launch.py
```

## Configuration

Edit `src/config/robot.config.ts` to change:
- API port
- rosbridge port
- Velocity limits

Robot IP is automatically detected from the browser's hostname.
To override, set `VITE_ROBOT_IP` environment variable when building:
```bash
VITE_ROBOT_IP=192.168.1.100 npm run build
```
