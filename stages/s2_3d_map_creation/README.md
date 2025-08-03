# Stage 2: 3D Map Creation

## 🏢 Overview

This stage creates realistic 3D indoor environments for quadcopter navigation and search-and-rescue missions. The maps are designed to represent real-world buildings with multiple rooms, corridors, obstacles, and navigable routes.

## 🏗️ Architecture

### Components
- **Indoor Building Models**: Multi-story buildings with realistic geometry
- **Obstacle System**: Furniture, walls, doors, and dynamic obstacles
- **Navigation Routes**: Predefined paths and waypoints
- **Lighting System**: Realistic indoor lighting and shadows
- **Surface Types**: Different materials (carpet, tile, concrete)

### Key Features
- ✅ Realistic building layouts based on actual structures
- ✅ Multiple room types (offices, corridors, stairwells)
- ✅ Dynamic obstacle placement
- ✅ Navigable routes and waypoints
- ✅ Realistic lighting and materials
- ✅ Search-and-rescue mission environments

## 🚀 Quick Start

### Prerequisites
- Stage 1 completed and working
- Docker Desktop for Mac

### Installation & Running

```bash
# Navigate to Stage 2
cd stages/s2_3d_map_creation

# Build and run with Docker
docker-compose up --build

# Or run individual components
./scripts/launch_office_building.sh
./scripts/launch_hospital.sh
./scripts/launch_warehouse.sh
```

## 📁 File Structure

```
s2_3d_map_creation/
├── README.md
├── docker-compose.yml
├── Dockerfile
├── worlds/
│   ├── office_building.world
│   ├── hospital.world
│   ├── warehouse.world
│   └── search_rescue.world
├── models/
│   ├── building/
│   ├── furniture/
│   ├── obstacles/
│   └── lighting/
├── launch/
│   ├── office_building.launch.py
│   ├── hospital.launch.py
│   └── warehouse.launch.py
├── config/
│   ├── building_params.yaml
│   └── navigation_routes.yaml
└── scripts/
    ├── launch_office_building.sh
    ├── launch_hospital.sh
    └── launch_warehouse.sh
```

## 🏢 Building Types

### Office Building
- **Floors**: 3 stories
- **Rooms**: 15+ offices, conference rooms, lobby
- **Features**: Elevators, stairwells, corridors
- **Mission**: Search for missing person

### Hospital
- **Floors**: 2 stories
- **Rooms**: Emergency rooms, patient rooms, corridors
- **Features**: Medical equipment, emergency exits
- **Mission**: Locate medical equipment

### Warehouse
- **Floors**: 1 story
- **Rooms**: Storage areas, loading docks, offices
- **Features**: Shelving units, forklifts, pallets
- **Mission**: Inventory search and rescue

## 🧪 Testing

### Run All Tests
```bash
./scripts/run_tests.sh
```

### Individual Tests
```bash
# Test office building
python3 tests/test_office_building.py

# Test hospital
python3 tests/test_hospital.py

# Test warehouse
python3 tests/test_warehouse.py

# Test navigation routes
python3 tests/test_navigation.py
```

## 🔧 Configuration

### Building Parameters
Edit `config/building_params.yaml`:
```yaml
office_building:
  floors: 3
  rooms_per_floor: 5
  corridor_width: 2.0
  room_height: 3.0
  lighting: true
  obstacles: true

hospital:
  floors: 2
  emergency_rooms: 4
  patient_rooms: 8
  corridor_width: 2.5
  room_height: 3.5
  medical_equipment: true

warehouse:
  floors: 1
  storage_areas: 6
  loading_docks: 2
  corridor_width: 3.0
  room_height: 4.0
  shelving_units: true
```

### Navigation Routes
Edit `config/navigation_routes.yaml`:
```yaml
office_building:
  waypoints:
    - {x: 0, y: 0, z: 2, name: "entrance"}
    - {x: 10, y: 5, z: 2, name: "conference_room"}
    - {x: 20, y: 10, z: 2, name: "office_area"}
  
  search_areas:
    - {x: 5, y: 5, z: 2, radius: 3, name: "lobby"}
    - {x: 15, y: 15, z: 2, radius: 5, name: "office_wing"}
```

## 🔄 Integration with Stage 1

### Input from Stage 1
- Quadcopter model and physics
- Sensor framework (IMU, GPS, altitude)
- Flight controller interface
- Control topics and commands

### Output for Stage 3
- Realistic indoor environments
- Navigation waypoints and routes
- Obstacle detection scenarios
- Search-and-rescue mission areas

## 📊 Performance Metrics

### Target Performance
- **Map Loading Time**: < 10 seconds
- **Rendering Performance**: 60+ FPS
- **Memory Usage**: < 3GB
- **Collision Detection**: Real-time

### Building Complexity
- **Office Building**: 50+ objects, 3 floors
- **Hospital**: 30+ objects, 2 floors
- **Warehouse**: 40+ objects, 1 floor

## 🎯 Success Criteria

1. **✅ Realistic Geometry**: Buildings match real-world layouts
2. **✅ Navigation Routes**: Predefined paths for autonomous flight
3. **✅ Obstacle System**: Dynamic and static obstacles
4. **✅ Lighting**: Realistic indoor lighting
5. **✅ Performance**: Smooth rendering on M1 MacBook
6. **✅ Integration**: Works with Stage 1 quadcopter
7. **✅ Testing**: Comprehensive test suite
8. **✅ Documentation**: Complete setup instructions

## 🚀 Next Stage Preparation

This stage provides:
- **Realistic indoor environments** for navigation
- **Search-and-rescue scenarios** for mission testing
- **Obstacle systems** for collision avoidance
- **Navigation waypoints** for autonomous flight

Ready for **Stage 3: Sensor Simulation** where we'll add camera, lidar, and other sensors to the quadcopter. 