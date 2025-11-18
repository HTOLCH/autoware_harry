#!/bin/bash
# Manual rosbag recording script for campus AV thesis
# Usage: ./record_rosbag.sh [test_name] [duration_seconds]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Default values
TEST_NAME="${1:-campus_test}"
DURATION="${2:-300}"
OUTPUT_DIR="${ROOT_DIR}/thesis_data/manual_recordings"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_header() {
    echo -e "${BLUE}$1${NC}"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# Get timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_FILE="${OUTPUT_DIR}/${TEST_NAME}_${TIMESTAMP}"

print_header "======================================================"
print_header "ROS 2 Bag Recording for Campus AV Thesis"
print_header "======================================================"
echo ""
print_info "Test name: ${TEST_NAME}"
print_info "Duration: ${DURATION} seconds"
print_info "Output: ${OUTPUT_FILE}"
echo ""

# Topics to record
declare -a TOPICS=(
    # Control topics - Controller performance
    "/control/trajectory_follower/lateral/control_cmd"
    "/control/trajectory_follower/lateral/diagnostic"
    "/control/command/control_cmd"

    # Localization - Vehicle state
    "/localization/kinematic_state"
    "/localization/pose_estimator/pose"
    "/localization/pose_estimator/pose_with_covariance"

    # Planning - Trajectory tracking
    "/planning/scenario_planning/trajectory"
    "/planning/scenario_planning/lane_driving/behavior_planning/path"
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id"

    # Perception - Pedestrian detection
    "/perception/object_recognition/detection/objects"
    "/perception/object_recognition/tracking/objects"
    "/perception/object_recognition/objects"

    # Vehicle status
    "/vehicle/status/velocity_status"
    "/vehicle/status/steering_status"
    "/vehicle/status/control_mode"

    # Diagnostics
    "/diagnostics"
    "/diagnostics_agg"

    # TF for coordinate transforms
    "/tf"
    "/tf_static"
)

print_info "Recording topics:"
printf '%s\n' "${TOPICS[@]}" | sed 's/^/  - /'
echo ""

# Recording options
print_header "Recording Options:"
echo "  --storage sqlite3         : Use SQLite3 storage (default)"
echo "  --max-cache-size 0        : Unlimited cache (prevents data loss)"
echo "  --compression-mode file   : Compress entire bag file"
echo "  --include-hidden-topics   : Record hidden topics"
echo ""

print_info "Press Ctrl+C to stop recording early"
print_info "Starting recording in 3 seconds..."
sleep 3

# Start recording
print_info "Recording started..."
ros2 bag record \
    -o "${OUTPUT_FILE}" \
    --storage sqlite3 \
    --max-cache-size 0 \
    --compression-mode file \
    --qos-profile-overrides-path "${SCRIPT_DIR}/qos_overrides.yaml" \
    "${TOPICS[@]}" &

RECORD_PID=$!

# Wait for duration
print_info "Recording in progress (PID: ${RECORD_PID})..."
for ((i=${DURATION}; i>0; i-=10)); do
    if ! kill -0 ${RECORD_PID} 2>/dev/null; then
        print_warning "Recording process ended unexpectedly"
        break
    fi

    if [ $i -gt 10 ]; then
        echo "  Time remaining: ${i} seconds..."
        sleep 10
    else
        echo "  Time remaining: ${i} seconds..."
        sleep $i
    fi
done

# Stop recording gracefully
if kill -0 ${RECORD_PID} 2>/dev/null; then
    print_info "Stopping recording..."
    kill -SIGINT ${RECORD_PID}
    wait ${RECORD_PID} 2>/dev/null || true
fi

# Get bag info
print_header ""
print_header "======================================================"
print_header "Recording Complete!"
print_header "======================================================"
print_info "Output location: ${OUTPUT_FILE}"
echo ""

# Display bag info
if [ -d "${OUTPUT_FILE}" ]; then
    print_info "Bag information:"
    ros2 bag info "${OUTPUT_FILE}"
else
    print_warning "Bag directory not found. Recording may have failed."
fi

print_header ""
print_info "Next steps:"
echo "  1. View bag info: ros2 bag info ${OUTPUT_FILE}"
echo "  2. Play bag:      ros2 bag play ${OUTPUT_FILE}"
echo "  3. Analyze data:  See scripts/analyze_rosbag.py"
echo ""
