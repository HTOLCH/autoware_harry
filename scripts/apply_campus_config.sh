#!/bin/bash
# Script to apply campus-optimized configurations to Autoware launch

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LAUNCH_CONFIG_DIR="${ROOT_DIR}/src/launcher/autoware_launch/autoware_launch/config"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running from correct directory
if [ ! -d "${ROOT_DIR}/campus_config" ]; then
    print_error "Campus config directory not found. Run this script from the repository root."
    exit 1
fi

# Parse command line arguments
CONTROLLER="pure_pursuit"
BACKUP=true
DRY_RUN=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --controller)
            CONTROLLER="$2"
            shift 2
            ;;
        --no-backup)
            BACKUP=false
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Apply campus-optimized configurations to Autoware."
            echo ""
            echo "Options:"
            echo "  --controller <type>    Controller type: 'pure_pursuit' or 'mpc' (default: pure_pursuit)"
            echo "  --no-backup           Don't create backup of original files"
            echo "  --dry-run             Show what would be done without making changes"
            echo "  --help                Show this help message"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Validate controller type
if [ "${CONTROLLER}" != "pure_pursuit" ] && [ "${CONTROLLER}" != "mpc" ]; then
    print_error "Invalid controller type: ${CONTROLLER}"
    echo "Must be 'pure_pursuit' or 'mpc'"
    exit 1
fi

print_info "========================================"
print_info "Campus Configuration Application"
print_info "========================================"
print_info "Controller: ${CONTROLLER}"
print_info "Backup: ${BACKUP}"
print_info "Dry run: ${DRY_RUN}"
print_info ""

# Function to backup file if it exists
backup_file() {
    local file=$1
    if [ -f "${file}" ] && [ "${BACKUP}" = true ]; then
        local backup_file="${file}.backup_$(date +%Y%m%d_%H%M%S)"
        if [ "${DRY_RUN}" = false ]; then
            cp "${file}" "${backup_file}"
            print_info "Backed up: ${file} -> ${backup_file##*/}"
        else
            print_info "[DRY RUN] Would backup: ${file}"
        fi
    fi
}

# Function to copy config file
copy_config() {
    local source=$1
    local dest=$2
    local description=$3

    if [ ! -f "${source}" ]; then
        print_error "Source file not found: ${source}"
        return 1
    fi

    # Create destination directory if needed
    local dest_dir=$(dirname "${dest}")
    if [ ! -d "${dest_dir}" ]; then
        if [ "${DRY_RUN}" = false ]; then
            mkdir -p "${dest_dir}"
        else
            print_info "[DRY RUN] Would create directory: ${dest_dir}"
        fi
    fi

    backup_file "${dest}"

    if [ "${DRY_RUN}" = false ]; then
        cp "${source}" "${dest}"
        print_info "✓ Applied ${description}"
    else
        print_info "[DRY RUN] Would copy: ${source} -> ${dest}"
    fi
}

# Apply control configurations
print_info ""
print_info "Applying control configurations..."
print_info "-----------------------------------"

# Lateral controller (Pure Pursuit or MPC)
copy_config \
    "${ROOT_DIR}/campus_config/control/${CONTROLLER}_campus.param.yaml" \
    "${LAUNCH_CONFIG_DIR}/control/trajectory_follower/lateral/${CONTROLLER}.param.yaml" \
    "${CONTROLLER} lateral controller"

# Longitudinal controller (PID)
copy_config \
    "${ROOT_DIR}/campus_config/control/pid_longitudinal_campus.param.yaml" \
    "${LAUNCH_CONFIG_DIR}/control/trajectory_follower/longitudinal/pid.param.yaml" \
    "PID longitudinal controller"

# Apply planning configurations
print_info ""
print_info "Applying planning configurations..."
print_info "------------------------------------"

# Crosswalk parameters
copy_config \
    "${ROOT_DIR}/campus_config/planning/crosswalk_campus.param.yaml" \
    "${LAUNCH_CONFIG_DIR}/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml" \
    "crosswalk module"

# Obstacle stop parameters
copy_config \
    "${ROOT_DIR}/campus_config/planning/obstacle_stop_campus.param.yaml" \
    "${LAUNCH_CONFIG_DIR}/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml" \
    "obstacle stop module"

# Summary
print_info ""
print_info "========================================"
if [ "${DRY_RUN}" = false ]; then
    print_info "✓ Campus configuration applied successfully!"
    print_info ""
    print_info "Next steps:"
    print_info "1. Build the workspace: colcon build --packages-select autoware_launch"
    print_info "2. Source the workspace: source install/setup.bash"
    print_info "3. Launch Autoware with: ros2 launch autoware_launch autoware.launch.xml lateral_controller_mode:=${CONTROLLER}"
    print_info ""
    if [ "${BACKUP}" = true ]; then
        print_info "Note: Original files backed up with .backup_YYYYMMDD_HHMMSS extension"
    fi
else
    print_info "Dry run complete. No files were modified."
    print_info "Run without --dry-run to apply changes."
fi
print_info "========================================"
