# Stage 6: LLM Decision Engine

## 🤖 Overview

This stage implements an LLM (Large Language Model) decision engine that processes PLM output to make intelligent navigation and mission decisions. The system provides autonomous decision-making capabilities for search-and-rescue missions.

## 🏗️ Architecture

### Components
- **LLM Integration**: GPT, Mistral, or local model integration
- **Decision Framework**: Structured decision-making pipeline
- **Mission Planning**: Goal-oriented mission execution
- **Navigation Logic**: Path planning and obstacle avoidance
- **Behavior Management**: Adaptive behavior selection
- **Safety Systems**: Emergency protocols and fail-safes

### Key Features
- ✅ AI-powered decision making
- ✅ Mission planning and execution
- ✅ Adaptive navigation strategies
- ✅ Safety and emergency protocols
- ✅ Real-time decision processing
- ✅ Integration with flight controller

## 🚀 Quick Start

### Prerequisites
- Stage 1: Quadcopter Simulation
- Stage 2: 3D Map Creation
- Stage 3: Sensor Simulation
- Stage 4: SLAM Engine
- Stage 5: Perception Language Model (PLM)
- Docker Desktop for Mac

### Installation & Running

```bash
# Navigate to Stage 6
cd stages/s6_llm_decision_engine

# Build and run with Docker
docker-compose up --build

# Or run individual components
./scripts/launch_llm_engine.sh
./scripts/launch_mission_planner.sh
./scripts/launch_navigation_ai.sh
```

## 📁 File Structure

```
s6_llm_decision_engine/
├── README.md
├── docker-compose.yml
├── Dockerfile
├── src/
│   ├── llm_engine/
│   ├── mission_planner/
│   ├── navigation_ai/
│   ├── behavior_manager/
│   └── safety_system/
├── launch/
│   ├── llm_engine.launch.py
│   ├── mission_planner.launch.py
│   ├── navigation_ai.launch.py
│   └── all_ai.launch.py
├── config/
│   ├── llm_params.yaml
│   ├── mission_params.yaml
│   ├── navigation_params.yaml
│   └── behavior_params.yaml
├── scripts/
│   ├── launch_llm_engine.sh
│   ├── launch_mission_planner.sh
│   └── test_ai.sh
└── tests/
    ├── test_llm_engine.py
    ├── test_mission_planner.py
    └── test_navigation_ai.py
```

## 🤖 AI Components

### LLM Decision Engine
- **Input**: PLM descriptions, sensor data, mission goals
- **Model**: GPT-4, Mistral, or local LLM
- **Output**: Navigation decisions, behavior commands
- **Performance**: 5 Hz, < 500ms latency
- **Topics**: `/ai/llm/decisions`, `/ai/llm/reasoning`

### Mission Planner
- **Input**: Mission objectives, environment data
- **Features**: Goal decomposition, task planning
- **Output**: Mission plans, waypoint sequences
- **Performance**: 1 Hz, < 1s latency
- **Topics**: `/ai/mission/plan`, `/ai/mission/status`

### Navigation AI
- **Input**: Current pose, obstacles, goals
- **Features**: Path planning, obstacle avoidance
- **Output**: Waypoints, velocity commands
- **Performance**: 10 Hz, < 100ms latency
- **Topics**: `/ai/navigation/waypoints`, `/ai/navigation/commands`

## 🧪 Testing

### Run All Tests
```bash
./scripts/run_tests.sh
```

### Individual Tests
```bash
# Test LLM engine
python3 tests/test_llm_engine.py

# Test mission planner
python3 tests/test_mission_planner.py

# Test navigation AI
python3 tests/test_navigation_ai.py

# Test behavior management
python3 tests/test_behavior_manager.py
```

## 🔧 Configuration

### LLM Parameters
Edit `config/llm_params.yaml`:
```yaml
llm_engine:
  model:
    type: "gpt4"  # or "mistral", "local"
    api_key: "${OPENAI_API_KEY}"
    max_tokens: 500
    temperature: 0.7
  
  prompts:
    navigation: "Given the environment description: {env_desc}, current position: {pos}, and goal: {goal}, what navigation action should the drone take?"
    obstacle_avoidance: "The drone detects {obstacle} at {distance}. What is the safest avoidance maneuver?"
    mission_planning: "For mission objective: {objective}, in environment: {env}, what is the optimal search strategy?"
  
  decision_framework:
    confidence_threshold: 0.8
    fallback_behavior: "hover"
    max_decision_time: 1.0
```

### Mission Parameters
Edit `config/mission_params.yaml`:
```yaml
mission_planner:
  search_patterns:
    grid_search:
      spacing: 2.0
      altitude: 3.0
      coverage: 0.9
    
    spiral_search:
      radius_increment: 1.0
      angle_increment: 45.0
    
    random_search:
      max_distance: 10.0
      min_distance: 1.0
  
  objectives:
    search_rescue:
      priority: "high"
      search_area: "entire_building"
      target_type: "person"
    
    mapping:
      priority: "medium"
      coverage: "complete"
      detail_level: "high"
```

### Navigation Parameters
Edit `config/navigation_params.yaml`:
```yaml
navigation_ai:
  path_planning:
    algorithm: "a_star"
    resolution: 0.5
    safety_margin: 1.0
  
  obstacle_avoidance:
    detection_range: 5.0
    avoidance_distance: 2.0
    emergency_stop_distance: 1.0
  
  behavior:
    max_velocity: 5.0
    max_acceleration: 2.0
    turn_radius: 2.0
```

## 🔄 Integration with Previous Stages

### Input from Stage 5
- Natural language environment descriptions
- Object and obstacle information
- Navigation context and suggestions
- Mission objectives and constraints

### Output for Stage 7
- Navigation waypoints and commands
- Mission execution plans
- Behavior and safety decisions
- Real-time decision reasoning

## 📊 Performance Metrics

### Target Performance
- **LLM Processing**: 5 Hz, < 500ms latency
- **Mission Planning**: 1 Hz, < 1s latency
- **Navigation AI**: 10 Hz, < 100ms latency
- **Decision Making**: < 200ms per decision
- **Memory Usage**: < 1GB total
- **CPU Usage**: < 30% total

### AI Accuracy Metrics
- **Decision Quality**: 90% optimal choices
- **Path Planning**: 95% successful navigation
- **Obstacle Avoidance**: 99% collision-free
- **Mission Success**: 85% objective completion

## 🎯 Success Criteria

1. **✅ AI Decision Making**: Intelligent navigation choices
2. **✅ Mission Planning**: Goal-oriented mission execution
3. **✅ Safety Systems**: Robust emergency protocols
4. **✅ Real-time Performance**: Sub-second decision times
5. **✅ Integration**: Works with Stages 1-5
6. **✅ Testing**: Comprehensive AI validation
7. **✅ Documentation**: Complete AI API documentation
8. **✅ Reliability**: 99% uptime and error recovery

## 🚀 Next Stage Preparation

This stage provides:
- **AI-powered decision making** for autonomous flight
- **Mission planning and execution** capabilities
- **Intelligent navigation** and obstacle avoidance
- **Safety and emergency** protocols

Ready for **Stage 7: Navigation Control Loop** where we'll connect the AI decisions to the flight controller for autonomous execution. 