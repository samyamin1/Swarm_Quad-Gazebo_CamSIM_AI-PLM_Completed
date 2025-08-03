# 🧠 AI-Powered Swarm Quadcopter Simulation Project

A fully modular, AI-powered swarm simulation system for quadcopters using **Gazebo**, **PX4**, **ROS 2**, and **LLMs/PLMs**, designed to run realistically on **M1 MacBook** and be **portable via Docker**.

## 🏗️ Project Architecture

This project implements a 14-stage modular system for autonomous swarm quadcopter simulation:

### ✅ Stage 1: Quadcopter Simulation
- Gazebo quadcopter model with realistic physics
- PX4-style flight controller module
- Joystick/input topic control interface
- M1 MacBook compatibility

### ✅ Stage 2: 3D Map Creation
- Indoor test environment with obstacles
- Navigable routes and realistic geometry
- Lighting and surface types

### ✅ Stage 3: Sensor Simulation
- RGB Camera, Lidar, GPS, Ultrasonic sensors
- Realistic noise and latency profiles

### ✅ Stage 4: Modular SLAM Engine
- Dynamic map generation and pose estimation
- Fallback options for missing sensors

### ✅ Stage 5: Perception Language Model (PLM)
- Sensor data to descriptive output conversion
- Real-time environment understanding

### ✅ Stage 6: LLM Decision Engine
- AI-powered navigation decisions
- Waypoint generation and behavior control

### ✅ Stage 7: Navigation Control Loop
- Flight controller interface integration
- Obstacle avoidance and velocity control

### ✅ Stage 8: Simulated Visual Prompt System
- 360-degree camera simulation
- Real-time perception input generation

### ✅ Stage 9: Other Sensor Simulation
- GPS, Lidar, Ultrasonic full simulation
- Map geometry and noise profiles

### ✅ Stage 10: Sensor-Failure Robustness
- Partial sensor failure handling
- Best-available perception selection

### ✅ Stage 11: Edge Deployment Constraints
- Low-latency, low-compute optimization
- Micro edge computer compatibility

### ✅ Stage 12: Search and Rescue Mission
- Building simulation with lost target
- Autonomous AI-driven mission execution

### ✅ Stage 13: Swarm System
- 3-6 quadcopter swarm
- Inter-agent communication via Swarm Packets

### ✅ Stage 14: Swarm AI Algorithm
- Fully distributed swarm intelligence
- Leader election and task assignment
- Consensus-based decision making

## 🚀 Quick Start

### Prerequisites
- macOS with M1 chip
- Docker Desktop
- Git

### Installation
```bash
# Clone the repository
git clone git@github.com:samyamin1/Swarm_Quad-Gazebo_CamSIM_AI-PLM.git
cd Swarm_Quad-Gazebo_CamSIM_AI-PLM

# Build and run Stage 1
cd stages/s1_quadcopter_simulation
docker-compose up --build
```

## 📁 Project Structure

```
Swarm_Gazebo_CAMSIM_PLM-Quad/
├── stages/
│   ├── s1_quadcopter_simulation/
│   ├── s2_3d_map_creation/
│   ├── s3_sensor_simulation/
│   ├── s4_slam_engine/
│   ├── s5_perception_plm/
│   ├── s6_llm_decision_engine/
│   ├── s7_navigation_control/
│   ├── s8_visual_prompt_system/
│   ├── s9_sensor_simulation_full/
│   ├── s10_sensor_failure_robustness/
│   ├── s11_edge_deployment/
│   ├── s12_search_rescue_mission/
│   ├── s13_swarm_system/
│   └── s14_swarm_ai_algorithm/
├── shared/
│   ├── models/
│   ├── worlds/
│   ├── launch/
│   └── config/
├── docker/
├── docs/
└── tests/
```

## 🔧 Technology Stack

- **Simulation**: Gazebo Fortress
- **Flight Controller**: PX4 SITL
- **Middleware**: ROS 2 Humble
- **AI/ML**: Python, PyTorch, Transformers
- **Containerization**: Docker
- **Platform**: macOS M1

## 📊 Current Status

- [x] Project structure and documentation
- [ ] Stage 1: Quadcopter Simulation
- [ ] Stage 2: 3D Map Creation
- [ ] Stage 3: Sensor Simulation
- [ ] Stage 4: SLAM Engine
- [ ] Stage 5: Perception PLM
- [ ] Stage 6: LLM Decision Engine
- [ ] Stage 7: Navigation Control
- [ ] Stage 8: Visual Prompt System
- [ ] Stage 9: Full Sensor Simulation
- [ ] Stage 10: Sensor Failure Robustness
- [ ] Stage 11: Edge Deployment
- [ ] Stage 12: Search & Rescue Mission
- [ ] Stage 13: Swarm System
- [ ] Stage 14: Swarm AI Algorithm

## 🤝 Contributing

Each stage is fully independent with its own:
- README with setup instructions
- Test cases and validation
- Docker configuration
- CLI launch scripts

## 📝 License

MIT License - see LICENSE file for details. 