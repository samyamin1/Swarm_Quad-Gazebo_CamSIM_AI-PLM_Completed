# Stage 4: Modular SLAM Engine

## 🗺️ Overview

This stage implements a modular SLAM (Simultaneous Localization and Mapping) engine that can work with degraded sensor input and provides fallback options for missing sensors. The system generates dynamic maps and accurate pose estimation for autonomous navigation.

## 🏗️ Architecture

### Components
- **Visual SLAM**: ORB-SLAM3 integration for camera-based mapping
- **Lidar SLAM**: LOAM/LeGO-LOAM for point cloud mapping
- **Multi-Sensor Fusion**: Kalman filter for sensor fusion
- **Map Management**: Dynamic map generation and storage
- **Pose Estimation**: Real-time position and orientation tracking
- **Loop Closure**: Global consistency and drift correction

### Key Features
- ✅ Modular design with plug-and-play sensors
- ✅ Fallback options for missing sensors
- ✅ Real-time map generation and updates
- ✅ Loop closure detection and correction
- ✅ Multi-sensor fusion capabilities
- ✅ Dynamic obstacle detection and mapping

## 🚀 Quick Start

### Prerequisites
- Stage 1: Quadcopter Simulation
- Stage 2: 3D Map Creation
- Stage 3: Sensor Simulation
- Docker Desktop for Mac

### Installation & Running

```bash
# Navigate to Stage 4
cd stages/s4_slam_engine

# Build and run with Docker
docker-compose up --build

# Or run individual components
./scripts/launch_visual_slam.sh
./scripts/launch_lidar_slam.sh
./scripts/launch_fusion_slam.sh
```

## 📁 File Structure

```
s4_slam_engine/
├── README.md
├── docker-compose.yml
├── Dockerfile
├── src/
│   ├── visual_slam/
│   ├── lidar_slam/
│   ├── sensor_fusion/
│   ├── map_manager/
│   └── pose_estimator/
├── launch/
│   ├── visual_slam.launch.py
│   ├── lidar_slam.launch.py
│   ├── fusion_slam.launch.py
│   └── all_slam.launch.py
├── config/
│   ├── slam_params.yaml
│   ├── fusion_params.yaml
│   ├── map_params.yaml
│   └── loop_closure.yaml
├── scripts/
│   ├── launch_visual_slam.sh
│   ├── launch_lidar_slam.sh
│   └── test_slam.sh
└── tests/
    ├── test_visual_slam.py
    ├── test_lidar_slam.py
    └── test_fusion.py
```

## 🗺️ SLAM Types

### Visual SLAM (ORB-SLAM3)
- **Input**: RGB camera images
- **Features**: ORB features, keyframe tracking
- **Output**: Camera trajectory, sparse map
- **Performance**: 30 FPS, real-time tracking
- **Topics**: `/slam/visual/trajectory`, `/slam/visual/map`

### Lidar SLAM (LOAM)
- **Input**: 3D point clouds
- **Features**: Edge and planar features
- **Output**: Point cloud map, odometry
- **Performance**: 10 Hz, high accuracy
- **Topics**: `/slam/lidar/map`, `/slam/lidar/odometry`

### Multi-Sensor Fusion
- **Input**: Camera, lidar, IMU, GPS
- **Features**: Extended Kalman filter
- **Output**: Fused pose, uncertainty
- **Performance**: 20 Hz, robust estimation
- **Topics**: `/slam/fusion/pose`, `/slam/fusion/map`

## 🧪 Testing

### Run All Tests
```bash
./scripts/run_tests.sh
```

### Individual Tests
```bash
# Test visual SLAM
python3 tests/test_visual_slam.py

# Test lidar SLAM
python3 tests/test_lidar_slam.py

# Test sensor fusion
python3 tests/test_fusion.py

# Test loop closure
python3 tests/test_loop_closure.py
```

## 🔧 Configuration

### SLAM Parameters
Edit `config/slam_params.yaml`:
```yaml
visual_slam:
  orb_slam3:
    vocabulary_path: "/opt/orb_slam3/Vocabulary/ORBvoc.txt"
    settings_path: "/opt/orb_slam3/Examples/Monocular/TUM1.yaml"
    camera_topic: "/quadcopter/sensors/camera/image_raw"
    camera_info_topic: "/quadcopter/sensors/camera/camera_info"
  
  features:
    max_features: 2000
    scale_factor: 1.2
    min_levels: 8
    max_levels: 8

lidar_slam:
  loam:
    scan_period: 0.1
    num_regions: 6
    curvature_threshold: 0.1
  
  features:
    edge_threshold: 0.1
    planar_threshold: 0.1
    edge_min_valid: 10
    planar_min_valid: 5

sensor_fusion:
  ekf:
    process_noise:
      position: 0.1
      velocity: 0.1
      orientation: 0.01
    
    measurement_noise:
      gps: 2.0
      imu: 0.1
      camera: 0.5
      lidar: 0.1
```

### Map Parameters
Edit `config/map_params.yaml`:
```yaml
map_manager:
  resolution: 0.1  # meters per pixel
  size: [100, 100, 10]  # meters
  update_rate: 5.0  # Hz
  
  visualization:
    enabled: true
    color_mode: "height"
    point_size: 2
  
  storage:
    save_format: "pcd"
    auto_save: true
    save_interval: 60.0  # seconds
```

## 🔄 Integration with Previous Stages

### Input from Stage 3
- RGB camera images
- 3D lidar point clouds
- IMU and GPS data
- Multi-sensor timestamps

### Output for Stage 5
- Dynamic environment maps
- Accurate pose estimation
- Feature-rich environment data
- Navigation waypoints

## 📊 Performance Metrics

### Target Performance
- **Visual SLAM**: 30 FPS, < 50ms latency
- **Lidar SLAM**: 10 Hz, < 100ms latency
- **Sensor Fusion**: 20 Hz, < 100ms latency
- **Map Generation**: 5 Hz, < 200ms latency
- **Memory Usage**: < 2GB total
- **CPU Usage**: < 50% total

### Accuracy Metrics
- **Position Accuracy**: ±10cm in 10m trajectory
- **Orientation Accuracy**: ±1° in 360° rotation
- **Map Accuracy**: ±5cm feature localization
- **Loop Closure**: 95% detection rate

## 🎯 Success Criteria

1. **✅ Modular Design**: Works with any sensor combination
2. **✅ Real-time Performance**: Sub-100ms latency
3. **✅ High Accuracy**: Centimeter-level precision
4. **✅ Robust Operation**: Handles sensor failures
5. **✅ Loop Closure**: Global consistency
6. **✅ Dynamic Mapping**: Real-time map updates
7. **✅ Integration**: Works with Stages 1-3
8. **✅ Testing**: Comprehensive validation

## 🚀 Next Stage Preparation

This stage provides:
- **Dynamic environment maps** for navigation
- **Accurate pose estimation** for autonomous flight
- **Feature-rich environment** data for AI perception
- **Robust localization** under various conditions

Ready for **Stage 5: Perception Language Model (PLM)** where we'll convert sensor and SLAM data into natural language descriptions. 