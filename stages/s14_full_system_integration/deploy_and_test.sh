#!/bin/bash

# 🚁 AI-Powered Swarm Quadcopter Simulation System
# Complete System Deployment and Testing Script

set -e  # Exit on any error

echo "🚁 ================================================"
echo "🚁 AI-POWERED SWARM QUADCOPTER SIMULATION SYSTEM"
echo "🚁 COMPLETE SYSTEM DEPLOYMENT AND TESTING"
echo "🚁 ================================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check prerequisites
check_prerequisites() {
    print_status "Checking prerequisites..."
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed. Please install Docker first."
        exit 1
    fi
    
    # Check Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        print_error "Docker Compose is not installed. Please install Docker Compose first."
        exit 1
    fi
    
    # Check if Docker is running
    if ! docker info &> /dev/null; then
        print_error "Docker is not running. Please start Docker first."
        exit 1
    fi
    
    print_success "Prerequisites check passed"
}

# Build the complete system
build_system() {
    print_status "Building complete AI-powered swarm system..."
    
    # Build all containers
    docker-compose build --no-cache
    
    if [ $? -eq 0 ]; then
        print_success "System build completed successfully"
    else
        print_error "System build failed"
        exit 1
    fi
}

# Start the complete system
start_system() {
    print_status "Starting complete AI-powered swarm system..."
    
    # Start all services
    docker-compose up -d
    
    if [ $? -eq 0 ]; then
        print_success "System started successfully"
    else
        print_error "System startup failed"
        exit 1
    fi
    
    # Wait for system to be ready
    print_status "Waiting for system to be ready..."
    sleep 30
}

# Check system health
check_system_health() {
    print_status "Checking system health..."
    
    # Check if all containers are running
    running_containers=$(docker-compose ps -q | wc -l)
    total_services=$(docker-compose config --services | wc -l)
    
    if [ "$running_containers" -eq "$total_services" ]; then
        print_success "All system components are running"
    else
        print_warning "Some system components may not be running properly"
        docker-compose ps
    fi
    
    # Check system status via ROS 2 topics
    print_status "Checking ROS 2 system status..."
    
    # Wait a bit more for ROS 2 to be ready
    sleep 10
    
    # Check if we can access the system
    if docker exec swarm_system_integration ros2 topic list | grep -q "quadcopter"; then
        print_success "ROS 2 communication is working"
    else
        print_warning "ROS 2 communication may not be ready yet"
    fi
}

# Run system tests
run_system_tests() {
    print_status "Running complete system tests..."
    
    # Run the test script
    docker exec swarm_system_integration python3 /opt/ros/workspace/src/full_system_integration/full_system_integration/test_complete_system.py &
    
    # Wait for test to complete
    sleep 70
    
    # Check test results
    if docker exec swarm_system_integration test -f /tmp/complete_system_test_results.json; then
        print_success "System tests completed"
        print_status "Test results:"
        docker exec swarm_system_integration cat /tmp/complete_system_test_results.json | python3 -m json.tool
    else
        print_warning "System tests may not have completed properly"
    fi
}

# Monitor system performance
monitor_system() {
    print_status "Monitoring system performance..."
    
    echo "🚁 ================================================"
    echo "🚁 SYSTEM MONITORING - Press Ctrl+C to stop"
    echo "🚁 ================================================"
    
    # Monitor system resources
    docker stats --format "table {{.Container}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.BlockIO}}"
}

# Show system status
show_system_status() {
    print_status "System Status Summary:"
    echo "🚁 ================================================"
    
    # Show running containers
    echo "📦 Running Containers:"
    docker-compose ps
    
    echo ""
    
    # Show system logs
    echo "📋 Recent System Logs:"
    docker-compose logs --tail=20 system_integration
    
    echo ""
    
    # Show ROS 2 topics
    echo "🔗 Active ROS 2 Topics:"
    docker exec swarm_system_integration ros2 topic list | grep quadcopter | head -10
    
    echo ""
    
    # Show system health
    echo "❤️  System Health:"
    docker exec swarm_system_integration ros2 topic echo /quadcopter/system/health --once 2>/dev/null || echo "Health data not available yet"
}

# Stop the system
stop_system() {
    print_status "Stopping AI-powered swarm system..."
    
    docker-compose down
    
    if [ $? -eq 0 ]; then
        print_success "System stopped successfully"
    else
        print_error "System shutdown failed"
    fi
}

# Main deployment flow
main() {
    case "${1:-deploy}" in
        "deploy")
            check_prerequisites
            build_system
            start_system
            check_system_health
            show_system_status
            print_success "🚁 AI-Powered Swarm System deployed successfully!"
            print_status "Run './deploy_and_test.sh test' to run system tests"
            print_status "Run './deploy_and_test.sh monitor' to monitor system performance"
            print_status "Run './deploy_and_test.sh stop' to stop the system"
            ;;
        "test")
            run_system_tests
            ;;
        "monitor")
            monitor_system
            ;;
        "status")
            show_system_status
            ;;
        "stop")
            stop_system
            ;;
        "restart")
            stop_system
            sleep 5
            start_system
            check_system_health
            show_system_status
            ;;
        *)
            echo "Usage: $0 {deploy|test|monitor|status|stop|restart}"
            echo ""
            echo "Commands:"
            echo "  deploy   - Build and start the complete system"
            echo "  test     - Run system tests"
            echo "  monitor  - Monitor system performance"
            echo "  status   - Show system status"
            echo "  stop     - Stop the system"
            echo "  restart  - Restart the system"
            exit 1
            ;;
    esac
}

# Handle script interruption
trap 'echo -e "\n${YELLOW}[WARNING]${NC} Script interrupted. Use './deploy_and_test.sh stop' to stop the system."' INT

# Run main function
main "$@" 