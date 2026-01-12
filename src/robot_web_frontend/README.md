# Robot Web Frontend

Web-based control interface for the robot.

## Features

1. **Remote Control** - Virtual joystick for manual robot control
2. **SLAM Mapping** - Create and save maps using SLAM
3. **Navigation** - Click on map to navigate to goals

## Setup

```bash
cd /home/jetson/base_dev/src/robot_web_frontend
npm install
```

## Running

### 1. Start rosbridge (required for all features)

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Or use the included launch file:

```bash
ros2 launch robot_web_frontend web_system.launch.py
```

### 2. Start the web frontend

```bash
cd /home/jetson/base_dev/src/robot_web_frontend
npm run dev
```

Access the web interface at: http://192.168.0.100:3000

### 3. For SLAM Mapping

Make sure the robot core is running:
```bash
ros2 launch motor_control full_system.launch.py
```

### 4. For Navigation

Start autonomous navigation:
```bash
ros2 launch nav2 autonomous_navigation.launch.py
```

## Configuration

Edit `src/config/robot.config.ts` to change:
- Robot IP address
- API port
- rosbridge port
- Velocity limits
