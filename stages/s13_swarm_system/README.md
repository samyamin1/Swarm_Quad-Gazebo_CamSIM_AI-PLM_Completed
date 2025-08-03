# Stage 13: Swarm System

## 🐝 Overview

This stage implements a multi-quadcopter swarm system with 3-6 quadcopters featuring inter-agent communication using swarm packets. The system provides distributed coordination, collision avoidance, and cooperative mission execution.

## 🏗️ Architecture

### Components
- **Multi-Agent System**: 3-6 quadcopter swarm with individual autonomy
- **Swarm Communication**: Inter-agent communication via swarm packets
- **Distributed Coordination**: Decentralized decision making and task allocation
- **Collision Avoidance**: Multi-agent collision prevention and safety
- **Cooperative Mission**: Shared mission objectives and resource allocation
- **Swarm Intelligence**: Emergent behaviors and collective decision making

### Key Features
- ✅ Multi-quadcopter swarm (3-6 agents)
- ✅ Inter-agent communication via swarm packets
- ✅ Distributed coordination and decision making
- ✅ Collision avoidance and safety protocols
- ✅ Cooperative mission execution
- ✅ Emergent swarm behaviors

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
- Stage 9: Full Sensor Simulation
- Stage 10: Sensor Failure Robustness
- Stage 11: Edge Deployment
- Stage 12: Search & Rescue Mission
- Docker Desktop for Mac

### Installation & Running

```bash
# Navigate to Stage 13
cd stages/s13_swarm_system

# Build and run with Docker
docker-compose up --build

# Or run individual components
./scripts/launch_swarm_system.sh
./scripts/launch_swarm_communication.sh
./scripts/launch_cooperative_mission.sh
```

## 📁 File Structure

```
s13_swarm_system/
├── README.md
├── docker-compose.yml
├── Dockerfile
├── src/
│   ├── swarm_agents/
│   ├── swarm_communication/
│   ├── distributed_coordination/
│   ├── collision_avoidance/
│   └── cooperative_mission/
├── launch/
│   ├── swarm_system.launch.py
│   ├── swarm_communication.launch.py
│   ├── cooperative_mission.launch.py
│   └── all_swarm.launch.py
├── config/
│   ├── swarm_params.yaml
│   ├── communication_params.yaml
│   ├── coordination_params.yaml
│   └── collision_params.yaml
├── scripts/
│   ├── launch_swarm_system.sh
│   ├── launch_swarm_communication.sh
│   └── test_swarm.sh
└── tests/
    ├── test_swarm_agents.py
    ├── test_swarm_communication.py
    └── test_cooperative_mission.py
```

## 🐝 Swarm Components

### Swarm Agents
- **Input**: Individual agent state, swarm communication, mission objectives
- **Features**: Autonomous behavior, swarm packet generation, local decision making
- **Output**: Individual actions, swarm packets, mission progress
- **Performance**: Real-time agent behavior
- **Topics**: `/swarm/agent/{id}/state`, `/swarm/agent/{id}/actions`

### Swarm Communication
- **Input**: Agent states, mission data, coordination messages
- **Features**: Swarm packet generation, network protocols, message routing
- **Output**: Swarm packets, communication status, network health
- **Performance**: Real-time communication
- **Topics**: `/swarm/communication/packets`, `/swarm/communication/status`

### Distributed Coordination
- **Input**: Swarm packets, mission objectives, agent capabilities
- **Features**: Decentralized decision making, task allocation, resource sharing
- **Output**: Coordinated actions, task assignments, resource allocation
- **Performance**: Real-time coordination
- **Topics**: `/swarm/coordination/decisions`, `/swarm/coordination/tasks`

### Collision Avoidance
- **Input**: Agent positions, velocities, trajectories
- **Features**: Multi-agent collision detection, avoidance maneuvers, safety protocols
- **Output**: Collision-free trajectories, safety alerts, emergency stops
- **Performance**: Real-time collision avoidance
- **Topics**: `/swarm/collision/alerts`, `/swarm/collision/trajectories`

## 🧪 Testing

### Run All Tests
```bash
./scripts/run_tests.sh
```

### Individual Tests
```bash
# Test swarm agents
python3 tests/test_swarm_agents.py

# Test swarm communication
python3 tests/test_swarm_communication.py

# Test distributed coordination
python3 tests/test_distributed_coordination.py

# Test collision avoidance
python3 tests/test_collision_avoidance.py
```

## 🔧 Configuration

### Swarm Parameters
Edit `config/swarm_params.yaml`:
```yaml
swarm_system:
  agents:
    count: 4  # 3-6 quadcopters
    initial_positions:
      - [0, 0, 2]
      - [5, 0, 2]
      - [0, 5, 2]
      - [5, 5, 2]
  
  communication:
    packet_size: 1024  # bytes
    update_rate: 10.0  # Hz
    range: 50.0  # meters
    protocol: "udp"
  
  coordination:
    decision_rate: 5.0  # Hz
    consensus_threshold: 0.7
    task_allocation: "auction"
    resource_sharing: true
  
  safety:
    collision_distance: 3.0  # meters
    emergency_stop_distance: 1.0  # meters
    max_velocity: 5.0  # m/s
    formation_spacing: 5.0  # meters
```

### Communication Parameters
Edit `config/communication_params.yaml`:
```yaml
swarm_communication:
  packet_structure:
    header:
      agent_id: "uint8"
      timestamp: "float64"
      packet_type: "uint8"
    
    payload:
      position: [3, "float32"]
      velocity: [3, "float32"]
      orientation: [4, "float32"]
      mission_status: "uint8"
      sensor_data: "variable"
  
  protocols:
    udp:
      enabled: true
      port: 5000
      broadcast: true
    
    tcp:
      enabled: false
      port: 5001
      reliable: true
  
  routing:
    flooding: true
    directed: false
    multicast: true
    max_hops: 3
```

### Coordination Parameters
Edit `config/coordination_params.yaml`:
```yaml
distributed_coordination:
  decision_making:
    algorithm: "consensus"
    voting_threshold: 0.6
    timeout: 5.0  # seconds
    
  task_allocation:
    method: "auction"
    bidding_timeout: 2.0  # seconds
    cost_function: "distance"
    
  resource_sharing:
    shared_map: true
    shared_sensor_data: true
    shared_computational_resources: false
    
  formation_control:
    enabled: true
    formation_type: "grid"
    spacing: 5.0  # meters
    alignment_tolerance: 0.5  # meters
```

### Collision Parameters
Edit `config/collision_params.yaml`:
```yaml
collision_avoidance:
  detection:
    range: 10.0  # meters
    update_rate: 20.0  # Hz
    prediction_horizon: 2.0  # seconds
    
  avoidance:
    algorithm: "potential_field"
    safety_margin: 2.0  # meters
    max_acceleration: 2.0  # m/s²
    
  emergency:
    stop_distance: 1.0  # meters
    emergency_maneuver: true
    communication_priority: true
    
  formation:
    maintain_formation: true
    formation_tolerance: 1.0  # meters
    formation_priority: 0.8
```

## 🔄 Integration with Previous Stages

### Input from Stage 12
- Mission execution data and performance metrics
- Search strategy validation and optimization
- Target detection accuracy and reliability
- Mission control and safety protocol validation

### Output for Stage 14
- Multi-agent swarm behavior data
- Inter-agent communication patterns
- Distributed coordination strategies
- Cooperative mission performance metrics

## 📊 Performance Metrics

### Target Performance
- **Swarm Size**: 3-6 quadcopters
- **Communication Rate**: 10 Hz swarm packets
- **Coordination Rate**: 5 Hz distributed decisions
- **Collision Avoidance**: 20 Hz safety monitoring
- **Mission Success**: 85% cooperative completion

### Swarm Metrics
- **Communication Latency**: < 100ms inter-agent
- **Coordination Accuracy**: 90% consensus achievement
- **Collision Avoidance**: 99% collision-free operation
- **Formation Control**: 95% formation maintenance

## 🎯 Success Criteria

1. **✅ Multi-Agent System**: 3-6 quadcopter swarm operation
2. **✅ Swarm Communication**: Real-time inter-agent communication
3. **✅ Distributed Coordination**: Decentralized decision making
4. **✅ Collision Avoidance**: Multi-agent safety protocols
5. **✅ Cooperative Mission**: Shared mission execution
6. **✅ Integration**: Works with Stages 1-12
7. **✅ Testing**: Comprehensive swarm validation
8. **✅ Documentation**: Complete swarm API documentation

## 🚀 Next Stage Preparation

This stage provides:
- **Multi-agent swarm system** with 3-6 quadcopters
- **Inter-agent communication** via swarm packets
- **Distributed coordination** and decision making
- **Cooperative mission** execution capabilities

Ready for **Stage 14: Swarm AI Algorithm** where we'll implement fully distributed swarm intelligence with leader election, task assignment, and consensus-based decision making. 