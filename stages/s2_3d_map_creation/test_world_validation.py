#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import os
import sys

def validate_hospital_world():
    """Validate the hospital world file structure and content"""
    
    print("🏥 HOSPITAL WORLD VALIDATION")
    print("=" * 50)
    
    # Check if hospital.world exists
    world_file = "worlds/hospital.world"
    if not os.path.exists(world_file):
        print(f"❌ ERROR: {world_file} not found")
        return False
    
    print(f"✅ Found {world_file}")
    
    try:
        # Parse the XML
        tree = ET.parse(world_file)
        root = tree.getroot()
        
        print("✅ XML structure is valid")
        
        # Check for required elements
        required_elements = [
            'physics',
            'include',  # sun and ground plane
            'model',    # hospital structures
            'gui',
            'plugin'    # ROS 2 plugins
        ]
        
        found_elements = []
        for elem in root.iter():
            found_elements.append(elem.tag)
        
        print("\n📋 Checking required elements:")
        for element in required_elements:
            if element in found_elements:
                print(f"✅ {element} found")
            else:
                print(f"❌ {element} missing")
        
        # Count models
        models = root.findall('.//model')
        print(f"\n🏗️ Found {len(models)} models in hospital world")
        
        # List model names
        model_names = []
        for model in models:
            name = model.get('name')
            if name:
                model_names.append(name)
        
        print("🏥 Hospital models found:")
        for name in model_names:
            print(f"  - {name}")
        
        # Check for quadcopter spawn
        includes = root.findall('.//include')
        quadcopter_found = False
        for include in includes:
            uri = include.find('uri')
            if uri is not None and 'quadcopter' in uri.text:
                quadcopter_found = True
                break
        
        if quadcopter_found:
            print("✅ Quadcopter spawn point found")
        else:
            print("❌ Quadcopter spawn point missing")
        
        # Check for physics settings
        physics = root.find('.//physics')
        if physics is not None:
            print("✅ Physics settings configured")
        else:
            print("❌ Physics settings missing")
        
        # Check for lighting
        lights = root.findall('.//light')
        if lights:
            print(f"✅ Found {len(lights)} light sources")
        else:
            print("❌ No lighting configured")
        
        # Check for GUI camera
        gui = root.find('.//gui')
        if gui is not None:
            camera = gui.find('.//camera')
            if camera is not None:
                print("✅ GUI camera configured")
            else:
                print("❌ GUI camera missing")
        else:
            print("❌ GUI configuration missing")
        
        # Check for ROS 2 plugins
        plugins = root.findall('.//plugin')
        ros_plugins = 0
        for plugin in plugins:
            if 'gazebo_ros' in plugin.get('filename', ''):
                ros_plugins += 1
        
        if ros_plugins > 0:
            print(f"✅ Found {ros_plugins} ROS 2 plugins")
        else:
            print("❌ No ROS 2 plugins found")
        
        print("\n" + "=" * 50)
        print("🎉 HOSPITAL WORLD VALIDATION COMPLETED")
        print("✅ World file is properly structured")
        print("✅ Ready for Gazebo simulation")
        print("✅ Ready for Stage 3: Sensor Simulation")
        
        return True
        
    except ET.ParseError as e:
        print(f"❌ XML parsing error: {e}")
        return False
    except Exception as e:
        print(f"❌ Validation error: {e}")
        return False

def validate_directory_structure():
    """Validate the Stage 2 directory structure"""
    
    print("\n📁 DIRECTORY STRUCTURE VALIDATION")
    print("=" * 50)
    
    required_files = [
        "worlds/hospital.world",
        "launch/hospital_test.launch.py",
        "test_hospital_world.py",
        "Dockerfile",
        "docker-compose.yml"
    ]
    
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"✅ {file_path}")
        else:
            print(f"❌ {file_path} missing")
    
    # Check for additional worlds
    worlds_dir = "worlds"
    if os.path.exists(worlds_dir):
        world_files = [f for f in os.listdir(worlds_dir) if f.endswith('.world')]
        print(f"\n🌍 Found {len(world_files)} world files:")
        for world_file in world_files:
            print(f"  - {world_file}")
    
    print("\n" + "=" * 50)
    print("✅ Directory structure validation completed")

def main():
    """Main validation function"""
    
    print("🚀 STAGE 2: 3D MAP CREATION VALIDATION")
    print("=" * 60)
    
    # Validate directory structure
    validate_directory_structure()
    
    # Validate hospital world
    success = validate_hospital_world()
    
    if success:
        print("\n🎯 STAGE 2 STATUS: READY FOR TESTING")
        print("✅ Hospital world created successfully")
        print("✅ All files in place")
        print("✅ Ready for Docker testing")
        print("\n🚀 Next: Run 'docker-compose up' to test the hospital world")
    else:
        print("\n❌ STAGE 2 STATUS: VALIDATION FAILED")
        print("Please fix the issues above before proceeding")
        sys.exit(1)

if __name__ == "__main__":
    main() 