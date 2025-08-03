# Stage 3: Sensor Simulation

## 📡 Overview

This stage implements comprehensive sensor simulation for the quadcopter, including RGB camera, lidar, GPS, ultrasonic sensors, and other advanced sensors with realistic noise profiles, latency simulation, and failure modes.

## 🏗️ Architecture

### Components
- **RGB Camera**: High-resolution camera with realistic optics and noise
- **Lidar Sensor**: 3D point cloud generation with range and intensity
- **GPS Module**: Position and velocity with realistic accuracy
- **Ultrasonic Sensors**: Short-range obstacle detection
- **IMU Enhancement**: Improved inertial measurement with bias and drift
- **Depth Camera**: RGB-D sensor for 3D perception
- **Thermal Camera**: Infrared imaging for search and rescue

### Key Features
- ✅ Realistic sensor noise and latency profiles
- ✅ Sensor failure simulation and recovery
- ✅ Multi-sensor fusion capabilities
- ✅ Configurable sensor parameters
- ✅ Real-time data processing
- ✅ Sensor calibration and validation

## 🚀 Quick Start

### Prerequisites
- Stage 1: Quadcopter Simulation
- Stage 2: 3D Map Creation
- Docker Desktop for Mac

### Installation & Running

```bash
# Navigate to Stage 3
cd stages/s3_sensor_simulation

# Build and run with Docker
docker-compose up --build

# Or run individual components
./scripts/launch_camera_sensor.sh
./scripts/launch_lidar_sensor.sh
./scripts/launch_gps_sensor.sh
```

## 📁 File Structure

```
s3_sensor_simulation/
├── README.md
├── docker-compose.yml
├── Dockerfile
├── src/
│   ├── camera_sensor/
│   ├── lidar_sensor/
│   ├── gps_sensor/
│   ├── ultrasonic_sensor/
│   └── sensor_fusion/
├── launch/
│   ├── camera.launch.py
│   ├── lidar.launch.py
│   ├── gps.launch.py
│   └── all_sensors.launch.py
├── config/
│   ├── camera_params.yaml
│   ├── lidar_params.yaml
│   ├── gps_params.yaml
│   └── sensor_fusion.yaml
├── scripts/
│   ├── launch_camera_sensor.sh
│   ├── launch_lidar_sensor.sh
│   └── test_sensors.sh
└── tests/
    ├── test_camera.py
    ├── test_lidar.py
    └── test_sensor_fusion.py
```

## 📷 Sensor Types

### RGB Camera
- **Resolution**: 1920x1080 (configurable)
- **Frame Rate**: 30 FPS
- **Field of View**: 90° horizontal
- **Features**: Auto-exposure, noise simulation, lens distortion
- **Topics**: `/quadcopter/sensors/camera/image_raw`, `/quadcopter/sensors/camera/camera_info`

### Lidar Sensor
- **Type**: 3D Velodyne-style lidar
- **Range**: 0.5m - 100m
- **Points**: 64 channels, 360° horizontal
- **Features**: Intensity data, range accuracy simulation
- **Topics**: `/quadcopter/sensors/lidar/points`, `/quadcopter/sensors/lidar/scan`

### GPS Module
- **Accuracy**: ±2m horizontal, ±5m vertical
- **Update Rate**: 10 Hz
- **Features**: Satellite simulation, multipath effects
- **Topics**: `/quadcopter/sensors/gps/fix`, `/quadcopter/sensors/gps/vel`

### Ultrasonic Sensors
- **Range**: 0.1m - 4m
- **Accuracy**: ±1cm
- **Update Rate**: 20 Hz
- **Features**: Multi-echo detection, temperature compensation
- **Topics**: `/quadcopter/sensors/ultrasonic/range`

### Depth Camera
- **Resolution**: 640x480
- **Range**: 0.5m - 10m
- **Features**: RGB + depth fusion, noise simulation
- **Topics**: `/quadcopter/sensors/depth/image_raw`, `/quadcopter/sensors/depth/points`

### Thermal Camera
- **Resolution**: 320x240
- **Temperature Range**: -40°C to +150°C
- **Features**: Heat signature detection, emissivity simulation
- **Topics**: `/quadcopter/sensors/thermal/image_raw`

## 🧪 Testing

### Run All Tests
```bash
./scripts/run_tests.sh
```

### Individual Tests
```bash
# Test camera sensor
python3 tests/test_camera.py

# Test lidar sensor
python3 tests/test_lidar.py

# Test GPS sensor
python3 tests/test_gps.py

# Test sensor fusion
python3 tests/test_sensor_fusion.py
```

## 🔧 Configuration

### Camera Parameters
Edit `config/camera_params.yaml`:
```yaml
camera_sensor:
  resolution:
    width: 1920
    height: 1080
  frame_rate: 30.0
  field_of_view: 90.0
  noise:
    gaussian_std: 0.02
    salt_pepper_prob: 0.001
  distortion:
    k1: 0.1
    k2: 0.05
    p1: 0.001
    p2: 0.001
  exposure:
    auto: true
    gain: 1.0
    exposure_time: 0.033
```

### Lidar Parameters
Edit `config/lidar_params.yaml`:
```yaml
lidar_sensor:
  channels: 64
  horizontal_resolution: 0.1
  vertical_fov: [-15, 15]
  range:
    min: 0.5
    max: 100.0
  accuracy:
    range_std: 0.02
    angle_std: 0.001
  noise:
    gaussian_std: 0.01
    dropout_prob: 0.001
```

### GPS Parameters
Edit `config/gps_params.yaml`:
```yaml
gps_sensor:
  accuracy:
    horizontal_std: 2.0
    vertical_std: 5.0
    velocity_std: 0.1
  update_rate: 10.0
  satellites:
    min_visible: 4
    max_visible: 12
  multipath:
    enabled: true
    probability: 0.1
    delay_std: 0.5
```

## 🔄 Integration with Previous Stages

### Input from Stage 1 & 2
- Quadcopter model and physics
- 3D environment and obstacles
- Flight controller interface
- Basic sensor framework

### Output for Stage 4
- Rich sensor data streams
- Multi-modal perception data
- Sensor fusion capabilities
- Realistic noise and failure modes

## 📊 Performance Metrics

### Target Performance
- **Camera Processing**: 30 FPS, < 100ms latency
- **Lidar Processing**: 10 Hz, < 50ms latency
- **GPS Processing**: 10 Hz, < 20ms latency
- **Sensor Fusion**: 20 Hz, < 100ms latency
- **Memory Usage**: < 1GB per sensor
- **CPU Usage**: < 30% per sensor

### Sensor Accuracy
- **Camera**: ±2% intensity, ±1° angle
- **Lidar**: ±2cm range, ±0.1° angle
- **GPS**: ±2m horizontal, ±5m vertical
- **Ultrasonic**: ±1cm range
- **Depth**: ±2cm range, ±1° angle

## 🎯 Success Criteria

1. **✅ Realistic Sensor Data**: Accurate noise and latency profiles
2. **✅ Multi-Sensor Support**: Camera, lidar, GPS, ultrasonic, depth, thermal
3. **✅ Sensor Fusion**: Multi-modal data integration
4. **✅ Failure Simulation**: Realistic sensor failure modes
5. **✅ Performance**: Real-time processing on M1 MacBook
6. **✅ Integration**: Works with Stages 1 & 2
7. **✅ Testing**: Comprehensive sensor validation
8. **✅ Documentation**: Complete sensor API documentation

## 🚀 Next Stage Preparation

This stage provides:
- **Rich sensor data streams** for perception
- **Multi-modal sensor fusion** capabilities
- **Realistic noise and failure** simulation
- **Configurable sensor parameters** for different scenarios

Ready for **Stage 4: SLAM Engine** where we'll implement simultaneous localization and mapping using the sensor data. 