#!/bin/bash
# Script to help compare Pure Pursuit and MPC controllers for thesis

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_header() {
    echo -e "${BLUE}$1${NC}"
}

# Default values
TEST_NAME="campus_test"
DURATION=300  # 5 minutes default
OUTPUT_DIR="${ROOT_DIR}/thesis_data"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --name)
            TEST_NAME="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Controller comparison helper for thesis data collection."
            echo ""
            echo "Options:"
            echo "  --name <name>       Test name prefix (default: campus_test)"
            echo "  --duration <sec>    Recording duration in seconds (default: 300)"
            echo "  --output <dir>      Output directory (default: thesis_data/)"
            echo "  --help              Show this help message"
            echo ""
            echo "This script will guide you through:"
            echo "1. Recording data with Pure Pursuit controller"
            echo "2. Recording data with MPC controller"
            echo "3. Organizing data for analysis"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Create output directories
mkdir -p "${OUTPUT_DIR}/pure_pursuit_tests"
mkdir -p "${OUTPUT_DIR}/mpc_tests"
mkdir -p "${OUTPUT_DIR}/analysis"

print_header "======================================================"
print_header "Controller Comparison Test Setup"
print_header "======================================================"
echo ""
print_info "Test name: ${TEST_NAME}"
print_info "Duration: ${DURATION} seconds"
print_info "Output directory: ${OUTPUT_DIR}"
echo ""

# Topics to record
TOPICS=(
    "/control/trajectory_follower/lateral/control_cmd"
    "/control/trajectory_follower/lateral/diagnostic"
    "/control/command/control_cmd"
    "/localization/kinematic_state"
    "/planning/scenario_planning/trajectory"
    "/perception/object_recognition/objects"
    "/vehicle/status/velocity_status"
    "/vehicle/status/steering_status"
    "/diagnostics"
    "/tf"
)

print_header "Recording Topics:"
for topic in "${TOPICS[@]}"; do
    echo "  - $topic"
done
echo ""

# Function to record data
record_data() {
    local controller=$1
    local test_num=$2
    local output_file="${OUTPUT_DIR}/${controller}_tests/${TEST_NAME}_${controller}_${test_num}"

    print_info "Recording ${controller} test #${test_num}..."
    print_info "Output: ${output_file}"
    print_info "Duration: ${DURATION} seconds"
    print_info ""
    print_info "Recording will start in 5 seconds..."
    print_info "Make sure Autoware is running with ${controller} controller!"
    sleep 5

    # Record rosbag
    ros2 bag record -o "${output_file}" \
        --max-cache-size 0 \
        --storage sqlite3 \
        "${TOPICS[@]}" &

    local bag_pid=$!

    # Wait for duration
    print_info "Recording in progress..."
    for ((i=${DURATION}; i>0; i-=10)); do
        if [ $i -gt 10 ]; then
            echo "  Time remaining: ${i} seconds..."
            sleep 10
        else
            echo "  Time remaining: ${i} seconds..."
            sleep $i
        fi
    done

    # Stop recording
    print_info "Stopping recording..."
    kill -SIGINT ${bag_pid}
    wait ${bag_pid} 2>/dev/null || true

    print_info "✓ Recording complete: ${output_file}"
    echo ""
}

# Main comparison workflow
print_header "======================================================"
print_header "STEP 1: Pure Pursuit Controller Test"
print_header "======================================================"
echo ""
print_info "1. Apply Pure Pursuit campus config:"
echo "   ${SCRIPT_DIR}/apply_campus_config.sh --controller pure_pursuit"
echo ""
print_info "2. Launch Autoware with Pure Pursuit:"
echo "   ros2 launch autoware_launch autoware.launch.xml lateral_controller_mode:=pure_pursuit"
echo ""
read -p "Press ENTER when Autoware is running with Pure Pursuit..."

# Record Pure Pursuit tests
record_data "pure_pursuit" "01"

print_info "Do you want to record another Pure Pursuit test? (y/n)"
read -r response
if [[ "$response" =~ ^[Yy]$ ]]; then
    record_data "pure_pursuit" "02"
fi

echo ""
print_header "======================================================"
print_header "STEP 2: MPC Controller Test"
print_header "======================================================"
echo ""
print_info "1. Stop current Autoware instance (Ctrl+C)"
echo ""
print_info "2. Apply MPC campus config:"
echo "   ${SCRIPT_DIR}/apply_campus_config.sh --controller mpc"
echo ""
print_info "3. Rebuild workspace:"
echo "   colcon build --packages-select autoware_launch"
echo "   source install/setup.bash"
echo ""
print_info "4. Launch Autoware with MPC:"
echo "   ros2 launch autoware_launch autoware.launch.xml lateral_controller_mode:=mpc"
echo ""
read -p "Press ENTER when Autoware is running with MPC..."

# Record MPC tests
record_data "mpc" "01"

print_info "Do you want to record another MPC test? (y/n)"
read -r response
if [[ "$response" =~ ^[Yy]$ ]]; then
    record_data "mpc" "02"
fi

# Create summary
SUMMARY_FILE="${OUTPUT_DIR}/analysis/${TEST_NAME}_summary.txt"
cat > "${SUMMARY_FILE}" << EOF
Controller Comparison Test Summary
==================================

Test Name: ${TEST_NAME}
Date: $(date)
Duration per test: ${DURATION} seconds

Pure Pursuit Tests:
-------------------
$(ls -lh ${OUTPUT_DIR}/pure_pursuit_tests/${TEST_NAME}_pure_pursuit_* 2>/dev/null | tail -n +2 || echo "No tests recorded")

MPC Tests:
----------
$(ls -lh ${OUTPUT_DIR}/mpc_tests/${TEST_NAME}_mpc_* 2>/dev/null | tail -n +2 || echo "No tests recorded")

Next Steps for Analysis:
------------------------
1. Extract data from rosbags using:
   ros2 bag info <bag_file>
   ros2 bag play <bag_file>

2. Analyze tracking error:
   - Topic: /control/trajectory_follower/lateral/diagnostic
   - Metrics: lateral_error, heading_error

3. Analyze comfort:
   - Topic: /vehicle/status/velocity_status
   - Compute: lateral acceleration, jerk

4. Analyze pedestrian safety:
   - Topic: /perception/object_recognition/objects
   - Compute: minimum distances, TTC values

5. Compare metrics between controllers using Python/MATLAB:
   - Load rosbag data
   - Extract time series
   - Compute statistics
   - Generate plots

Recommended Python packages:
- rosbags (pip install rosbags)
- pandas, numpy, matplotlib
- scipy for statistics

Example analysis script location:
${ROOT_DIR}/scripts/analyze_controller_data.py (TODO)
EOF

print_header ""
print_header "======================================================"
print_header "✓ Data Collection Complete!"
print_header "======================================================"
echo ""
print_info "Summary saved to: ${SUMMARY_FILE}"
echo ""
print_info "Data locations:"
echo "  Pure Pursuit: ${OUTPUT_DIR}/pure_pursuit_tests/"
echo "  MPC:          ${OUTPUT_DIR}/mpc_tests/"
echo "  Analysis:     ${OUTPUT_DIR}/analysis/"
echo ""
print_info "Next steps:"
echo "1. Review recorded data"
echo "2. Analyze rosbags"
echo "3. Compare controller performance"
echo "4. Generate thesis plots"
echo ""
print_info "See ${SUMMARY_FILE} for detailed analysis instructions."
