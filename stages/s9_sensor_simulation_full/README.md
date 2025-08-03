# Stage 9: Full Sensor Simulation

## 📡 Overview

This stage implements comprehensive sensor simulation with advanced noise profiles, failure modes, and edge case handling. The system provides realistic sensor behavior including GPS multipath, lidar occlusion, camera motion blur, and environmental interference.

## 🏗️ Architecture

### Components
- **Advanced GPS Simulation**: Multipath, satellite geometry, atmospheric effects
- **Enhanced Lidar Modeling**: Occlusion, reflection, atmospheric scattering
- **Realistic Camera Simulation**: Motion blur, lens distortion, lighting effects
- **Comprehensive IMU**: Bias drift, temperature effects, vibration
- **Environmental Sensors**: Temperature, humidity, pressure, wind
- **Failure Simulation**: Sensor degradation, complete failure, intermittent issues

### Key Features
- ✅ Realistic sensor noise and latency profiles
- ✅ Environmental interference simulation
- ✅ Sensor failure and degradation modeling
- ✅ Edge case handling and validation
- ✅ Comprehensive sensor fusion
- ✅ Performance optimization for edge devices

## 🚀 Quick Start

### Prerequisites
- Stage 1: Quadcopter Simulation
- Stage 2: 3D Map Creation
- Stage 3: Sensor Simulation
- Stage 4: SLAM Engine
- Stage 5: Perception Language Model (PLM)
- Stage 6: LLM Decision Engine
- Stage 7: Navigation Control Loop
- Stage 8: Visual Prompt System
- Docker Desktop for Mac

### Installation & Running

```bash
# Navigate to Stage 9
cd stages/s9_sensor_simulation_full

# Build and run with Docker
docker-compose up --build

# Or run individual components
./scripts/launch_gps_simulation.sh
./scripts/launch_lidar_simulation.sh
./scripts/launch_camera_simulation.sh
```

## 📁 File Structure

```
s9_sensor_simulation_full/
├── README.md
├── docker-compose.yml
├── Dockerfile
├── src/
│   ├── gps_simulator/
│   ├── lidar_simulator/
│   ├── camera_simulator/
│   ├── imu_simulator/
│   └── environmental_sensors/
├── launch/
│   ├── gps_simulation.launch.py
│   ├── lidar_simulation.launch.py
│   ├── camera_simulation.launch.py
│   └── all_sensors.launch.py
├── config/
│   ├── gps_params.yaml
│   ├── lidar_params.yaml
│   ├── camera_params.yaml
│   └── environmental_params.yaml
├── scripts/
│   ├── launch_gps_simulation.sh
│   ├── launch_lidar_simulation.sh
│   └── test_sensors.sh
└── tests/
    ├── test_gps_simulation.py
    ├── test_lidar_simulation.py
    └── test_camera_simulation.py
```

## 📡 Sensor Components

### Advanced GPS Simulator
- **Input**: Position, time, environment, satellite constellation
- **Features**: Multipath effects, atmospheric delay, satellite geometry
- **Output**: Realistic GPS measurements with errors
- **Performance**: 10 Hz, < 50ms latency
- **Topics**: `/sensors/gps/fix`, `/sensors/gps/vel`, `/sensors/gps/status`

### Enhanced Lidar Simulator
- **Input**: 3D environment, drone position, atmospheric conditions
- **Features**: Occlusion, reflection, atmospheric scattering, noise
- **Output**: Realistic point clouds with artifacts
- **Performance**: 10 Hz, < 100ms latency
- **Topics**: `/sensors/lidar/points`, `/sensors/lidar/scan`, `/sensors/lidar/status`

### Realistic Camera Simulator
- **Input**: Environment, drone motion, lighting conditions
- **Features**: Motion blur, lens distortion, noise, lighting effects
- **Output**: Realistic camera images with artifacts
- **Performance**: 30 FPS, < 100ms latency
- **Topics**: `/sensors/camera/image_raw`, `/sensors/camera/camera_info`

### Comprehensive IMU Simulator
- **Input**: Drone motion, temperature, vibration
- **Features**: Bias drift, temperature effects, vibration noise
- **Output**: Realistic IMU measurements
- **Performance**: 100 Hz, < 10ms latency
- **Topics**: `/sensors/imu/data`, `/sensors/imu/temperature`

## 🧪 Testing

### Run All Tests
```bash
./scripts/run_tests.sh
```

### Individual Tests
```bash
# Test GPS simulation
python3 tests/test_gps_simulation.py

# Test lidar simulation
python3 tests/test_lidar_simulation.py

# Test camera simulation
python3 tests/test_camera_simulation.py

# Test IMU simulation
python3 tests/test_imu_simulation.py
```

## 🔧 Configuration

### GPS Parameters
Edit `config/gps_params.yaml`:
```yaml
gps_simulator:
  accuracy:
    horizontal_std: 2.0  # meters
    vertical_std: 5.0  # meters
    velocity_std: 0.1  # m/s
  
  multipath:
    enabled: true
    probability: 0.2
    delay_std: 0.5  # meters
    amplitude_factor: 0.3
  
  atmospheric:
    ionospheric_delay: true
    tropospheric_delay: true
    delay_std: 1.0  # meters
  
  satellites:
    min_visible: 4
    max_visible: 12
    geometry_dilution: true
    signal_strength: true
```

### Lidar Parameters
Edit `config/lidar_params.yaml`:
```yaml
lidar_simulator:
  occlusion:
    enabled: true
    transparency_threshold: 0.1
    reflection_probability: 0.05
  
  atmospheric:
    scattering: true
    absorption: true
    visibility_range: 100.0  # meters
  
  noise:
    range_std: 0.02  # meters
    angle_std: 0.001  # radians
    intensity_noise: 0.1
  
  artifacts:
    dropout_probability: 0.001
    ghost_echoes: true
    multiple_returns: true
```

### Camera Parameters
Edit `config/camera_params.yaml`:
```yaml
camera_simulator:
  motion_blur:
    enabled: true
    exposure_time: 0.033  # seconds
    blur_factor: 0.1
  
  lens_distortion:
    radial_k1: 0.1
    radial_k2: 0.05
    tangential_p1: 0.001
    tangential_p2: 0.001
  
  noise:
    gaussian_std: 0.02
    salt_pepper_prob: 0.001
    dark_current: 0.01
  
  lighting:
    auto_exposure: true
    dynamic_range: 70.0  # dB
    low_light_noise: true
```

## 🔄 Integration with Previous Stages

### Input from Stage 8
- Visual prompt system data
- Camera simulation requirements
- Performance monitoring data
- Debug and validation requirements

### Output for Stage 10
- Comprehensive sensor data with realistic artifacts
- Failure mode simulation data
- Edge case handling validation
- Performance optimization data

## 📊 Performance Metrics

### Target Performance
- **GPS Simulation**: 10 Hz, < 50ms latency
- **Lidar Simulation**: 10 Hz, < 100ms latency
- **Camera Simulation**: 30 FPS, < 100ms latency
- **IMU Simulation**: 100 Hz, < 10ms latency
- **Memory Usage**: < 2GB total
- **CPU Usage**: < 40% total

### Accuracy Metrics
- **GPS Accuracy**: ±2m horizontal, ±5m vertical
- **Lidar Accuracy**: ±2cm range, ±0.1° angle
- **Camera Realism**: 90% human rating
- **IMU Accuracy**: ±0.1° orientation, ±0.1 m/s velocity

## 🎯 Success Criteria

1. **✅ Realistic Sensor Behavior**: Accurate noise and artifact simulation
2. **✅ Environmental Effects**: Atmospheric, lighting, interference modeling
3. **✅ Failure Simulation**: Degradation and complete failure modes
4. **✅ Edge Case Handling**: Comprehensive validation and testing
5. **✅ Integration**: Works with Stages 1-8
6. **✅ Testing**: Comprehensive sensor validation
7. **✅ Documentation**: Complete sensor API documentation
8. **✅ Performance**: Optimized for real-time operation

## 🚀 Next Stage Preparation

This stage provides:
- **Comprehensive sensor simulation** with realistic artifacts
- **Environmental interference** modeling
- **Failure mode simulation** for robustness testing
- **Edge case handling** for validation

Ready for **Stage 10: Sensor Failure Robustness** where we'll implement systems to handle sensor failures gracefully and maintain operation with degraded sensor input. 