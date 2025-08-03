# Stage 1: Quadcopter Simulation

## 🚁 Overview

This stage implements a fully functional quadcopter simulation in Gazebo with a PX4-style flight controller module. The system is designed to run on M1 MacBook with realistic physics and provides a modular interface for future AI integration.

## 🏗️ Architecture

### Components
- **Gazebo Quadcopter Model**: Realistic quadcopter with 4 rotors, IMU, and sensors
- **PX4-Style Flight Controller**: Software-in-the-loop flight controller
- **Control Interface**: ROS 2 topics for joystick and programmatic control
- **Physics Engine**: Realistic aerodynamics and motor dynamics

### Key Features
- ✅ M1 MacBook compatibility
- ✅ Docker containerization
- ✅ Realistic physics simulation
- ✅ Modular flight controller interface
- ✅ Joystick and topic-based control
- ✅ Extensible sensor framework

## 🚀 Quick Start

### Prerequisites
- Docker Desktop for Mac
- Git

### Installation & Running

```bash
# Navigate to Stage 1
cd stages/s1_quadcopter_simulation

# Build and run with Docker
docker-compose up --build

# Or run individual components
./scripts/launch_simulation.sh
./scripts/launch_controller.sh
./scripts/test_joystick.sh
```

### Manual Setup (Alternative)

```bash
# Install ROS 2 Humble
brew install ros/humble/ros

# Install Gazebo Fortress
brew install gazebo

# Clone and build
git clone <repository>
cd stages/s1_quadcopter_simulation
colcon build
source install/setup.bash
```

## 📁 File Structure

```
s1_quadcopter_simulation/
├── README.md
├── docker-compose.yml
├── Dockerfile
├── src/
│   ├── quadcopter_controller/
│   ├── flight_controller/
│   └── joystick_interface/
├── launch/
│   ├── simulation.launch.py
│   ├── controller.launch.py
│   └── joystick.launch.py
├── worlds/
│   └── empty_world.world
├── models/
│   └── quadcopter/
│       ├── model.sdf
│       └── quadcopter.urdf
├── config/
│   ├── controller_params.yaml
│   └── joystick_mapping.yaml
├── scripts/
│   ├── launch_simulation.sh
│   ├── launch_controller.sh
│   ├── test_joystick.sh
│   └── test_flight.sh
└── tests/
    ├── test_physics.py
    ├── test_controller.py
    └── test_joystick.py
```

## 🎮 Control Interface

### Joystick Control
- **Left Stick**: Pitch/Roll control
- **Right Stick**: Yaw control
- **Triggers**: Throttle control
- **Buttons**: Takeoff/Land/Emergency Stop

### Topic-Based Control
```bash
# Manual control via topics
ros2 topic pub /quadcopter/control/velocity geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Set waypoints
ros2 topic pub /quadcopter/control/waypoint geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 2.0}}}"
```

### Flight Controller Interface
The flight controller provides these interfaces for future AI integration:

```python
# Available topics for AI integration
/quadcopter/sensors/imu          # IMU data
/quadcopter/sensors/gps          # GPS position
/quadcopter/sensors/altitude     # Altitude sensor
/quadcopter/state/pose           # Current pose
/quadcopter/state/velocity       # Current velocity
/quadcopter/control/velocity     # Velocity commands
/quadcopter/control/waypoint     # Waypoint commands
/quadcopter/control/mode         # Flight mode (MANUAL/AUTO)
```

## 🧪 Testing

### Run All Tests
```bash
./scripts/run_tests.sh
```

### Individual Tests
```bash
# Test physics simulation
python3 tests/test_physics.py

# Test flight controller
python3 tests/test_controller.py

# Test joystick interface
python3 tests/test_joystick.py

# Test complete system
python3 tests/test_integration.py
```

### Manual Testing
```bash
# Launch simulation
./scripts/launch_simulation.sh

# In another terminal, test joystick
./scripts/test_joystick.sh

# Test autonomous flight
./scripts/test_flight.sh
```

## 🔧 Configuration

### Controller Parameters
Edit `config/controller_params.yaml`:
```yaml
flight_controller:
  pid_gains:
    roll_p: 1.0
    roll_i: 0.1
    roll_d: 0.05
    pitch_p: 1.0
    pitch_i: 0.1
    pitch_d: 0.05
    yaw_p: 2.0
    yaw_i: 0.1
    yaw_d: 0.1
    throttle_p: 1.0
    throttle_i: 0.1
    throttle_d: 0.05
  
  limits:
    max_velocity: 10.0  # m/s
    max_angular_velocity: 2.0  # rad/s
    max_altitude: 100.0  # m
```

### Joystick Mapping
Edit `config/joystick_mapping.yaml`:
```yaml
joystick:
  device: "/dev/input/js0"
  deadzone: 0.1
  scaling:
    pitch: 1.0
    roll: 1.0
    yaw: 1.0
    throttle: 1.0
```

## 🐛 Troubleshooting

### Common Issues

1. **Gazebo not starting**
   ```bash
   # Check if Gazebo is installed
   gazebo --version
   
   # Reset Gazebo
   rm -rf ~/.gazebo
   ```

2. **Joystick not detected**
   ```bash
   # List input devices
   ls /dev/input/
   
   # Test joystick
   jstest /dev/input/js0
   ```

3. **Docker issues on M1**
   ```bash
   # Ensure Docker Desktop is running
   docker --version
   
   # Rebuild with platform specification
   docker-compose build --platform linux/arm64
   ```

### Performance Optimization
- Use `--disable-gpu` flag for Gazebo if GPU issues occur
- Adjust physics timestep in world file for better performance
- Use simplified models for faster simulation

## 📊 Performance Metrics

### Target Performance (M1 MacBook)
- **Simulation Rate**: 60+ Hz
- **Controller Update Rate**: 100+ Hz
- **Memory Usage**: < 2GB
- **CPU Usage**: < 50%

### Monitoring
```bash
# Monitor system resources
./scripts/monitor_performance.sh

# Check simulation performance
ros2 topic echo /gazebo/performance
```

## 🔄 Next Stage Integration

This stage provides the foundation for:
- **Stage 2**: 3D map creation and environment simulation
- **Stage 3**: Sensor simulation and data fusion
- **Stage 4**: SLAM engine integration
- **Stage 5**: AI perception system

The flight controller interface is designed to be modular and extensible for future AI integration.

## 📝 API Documentation

### Flight Controller API
```python
class FlightController:
    def set_velocity(self, linear, angular):
        """Set target velocity"""
        pass
    
    def set_waypoint(self, pose):
        """Set target waypoint"""
        pass
    
    def set_mode(self, mode):
        """Set flight mode (MANUAL/AUTO)"""
        pass
    
    def get_state(self):
        """Get current state"""
        pass
```

### Sensor Interface
```python
class SensorInterface:
    def get_imu_data(self):
        """Get IMU sensor data"""
        pass
    
    def get_gps_data(self):
        """Get GPS position data"""
        pass
    
    def get_altitude(self):
        """Get altitude sensor data"""
        pass
```

## 🤝 Contributing

1. Follow the modular design principles
2. Add tests for new features
3. Update documentation
4. Ensure M1 compatibility
5. Test with Docker containerization 