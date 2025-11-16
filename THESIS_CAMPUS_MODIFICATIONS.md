# Campus Autonomous Vehicle Thesis - Modifications Documentation

## Project Overview

**Thesis Focus**: Optimizing autonomous vehicle control for campus environments with emphasis on:
- Pedestrian-dense areas
- Smooth ride quality
- Path maintenance
- Enhanced pedestrian yielding logic
- Fleet status monitoring

**Target Environment**: Campus operation at 5-20 km/h with frequent pedestrian interactions

---

## 1. Controller Comparison Framework

### 1.1 Controllers Under Test

#### Pure Pursuit Controller
- **Type**: Geometric path following controller
- **Advantages**:
  - Simple, computationally efficient
  - Predictable behavior
  - Well-suited for low speeds
  - Easy to tune
- **Campus Configuration**: `campus_config/control/pure_pursuit_campus.param.yaml`

#### Model Predictive Control (MPC)
- **Type**: Optimization-based controller
- **Advantages**:
  - Optimal trajectory tracking
  - Handles constraints explicitly
  - Better at low speeds
  - Smoother control outputs
- **Campus Configuration**: `campus_config/control/mpc_campus.param.yaml`

### 1.2 Comparison Metrics

Track and compare the following metrics for your thesis:

#### Tracking Performance
- **Lateral Error**: Distance from reference path
- **Heading Error**: Angular deviation from reference
- **Tracking RMS Error**: Overall tracking quality
- **Maximum Deviation**: Worst-case tracking error

#### Comfort Metrics
- **Lateral Acceleration**: Passenger comfort indicator
- **Lateral Jerk**: Smoothness of steering
- **Steering Rate**: Rate of steering wheel movement
- **Ride Smoothness Score**: Combined comfort metric

#### Safety Metrics
- **Pedestrian Clearance**: Minimum distance to pedestrians
- **Stop Distance**: Braking performance
- **Reaction Time**: Time to respond to obstacles
- **Safety Margin Violations**: Number of unsafe events

#### Efficiency Metrics
- **Path Length**: Total distance traveled
- **Travel Time**: Time to complete route
- **Number of Stops**: Efficiency indicator
- **Average Speed**: Operational efficiency

### 1.3 Test Scenarios

Design campus-specific test scenarios:

1. **Straight Path Following**
   - Long straight sections (library to cafeteria)
   - Measure: tracking error, comfort

2. **Tight Turn Navigation**
   - 90-degree intersections
   - Roundabouts
   - Measure: tracking accuracy, steering smoothness

3. **Pedestrian Yielding**
   - Crosswalk scenarios
   - Sudden pedestrian appearances
   - Measure: stop distance, reaction time, passenger comfort

4. **Dense Pedestrian Areas**
   - Peak hour campus traffic
   - Multiple crossing pedestrians
   - Measure: safety margins, navigation efficiency

5. **Mixed Speed Zones**
   - Transitions between zones (15 km/h → 10 km/h)
   - Measure: speed tracking, comfort during transitions

---

## 2. Campus-Specific Modifications

### 2.1 Control Parameters

#### Pure Pursuit - Key Campus Changes

```yaml
# Lookahead distances reduced for tight spaces
min_lookahead_distance: 2.5 m      # (was 4.35 m)
max_lookahead_distance: 8.0 m      # (was 15.0 m)

# Tighter curve following
ld_curvature_ratio: 80.0           # (was 120.0)

# Smoothing enabled for comfort
enable_path_smoothing: true        # (was false)
path_filter_moving_ave_num: 35     # (was 25)
```

#### MPC - Key Campus Changes

```yaml
# Increased tracking precision
mpc_weight_lat_error: 1.5          # (was 1.0)

# Enhanced smoothness
mpc_weight_lat_jerk: 0.2           # (was 0.1)
enable_path_smoothing: true        # (was false)

# Comfort-oriented steering limits
steer_rate_lim_dps_list_by_velocity: [50, 40, 30]  # (was [60, 50, 40])

# Lower acceleration limits
acceleration_limit: 1.2 m/s²       # (was 2.0)
```

#### Longitudinal Control - Key Campus Changes

```yaml
# Comfortable acceleration limits
max_acc: 1.2 m/s²                  # (was 3.0 m/s²)
min_acc: -2.0 m/s²                 # (was -5.0 m/s²)

# Smooth jerk limits
max_jerk: 1.0 m/s³                 # (was 2.0 m/s³)
min_jerk: -2.0 m/s³                # (was -5.0 m/s³)

# Gentler stopping
smooth_stop_max_strong_acc: -0.4   # (was -0.5)
```

### 2.2 Pedestrian Yielding Enhancements

#### Crosswalk Module Improvements

```yaml
# Increased safety distances
stop_distance_from_crosswalk: 4.0 m        # (was 3.5 m)
stop_distance_from_object_preferred: 3.5 m # (was 3.0 m)

# More patient waiting
timeout_ego_stop_for_yield: 2.0 s          # (was 1.0 s)
timeout_no_intention_to_walk: [3.0, 0.0]   # (was [1.0, 0.0])

# Conservative TTC margins
ego_pass_first_margin_y: [5.0] s           # (was [4.0] s)
ego_pass_later_margin_y: [15.0] s          # (was [13.0] s)

# Enhanced occlusion handling
occlusion.occluded_object_velocity: 1.5 m/s  # (was 1.0)
occlusion.min_size: 0.5 m                    # (was 1.0) - detect smaller obstacles
```

**Rationale**: Campus environments have:
- Building corners creating blind spots
- Trees and vegetation occluding pedestrians
- Students often distracted (phones, conversations)
- Need for conservative, predictable behavior

#### Obstacle Stop Module Improvements

```yaml
# Larger safety margins
stop_margin: 5.5 m                 # (was 5.0 m)

# Wider detection zones
max_lat_margin: 0.4 m              # (was 0.3 m)

# More conservative time margins
collision_time_margin: 1.5 s       # (was 1.0 s)

# Smoother velocity planning
max_ego_accel: 1.0 m/s²            # (comfortable acceleration)
max_ego_jerk: 0.8 m/s³             # (smooth jerk)
```

---

## 3. Implementation Guide

### 3.1 Testing Pure Pursuit

```bash
# Launch with campus-tuned pure pursuit
ros2 launch autoware_launch autoware.launch.xml \
  lateral_controller_mode:=pure_pursuit \
  vehicle_param_file:=$(pwd)/campus_config/vehicle_info.param.yaml \
  lateral_controller_param_file:=$(pwd)/campus_config/control/pure_pursuit_campus.param.yaml \
  longitudinal_controller_param_file:=$(pwd)/campus_config/control/pid_longitudinal_campus.param.yaml
```

### 3.2 Testing MPC

```bash
# Launch with campus-tuned MPC
ros2 launch autoware_launch autoware.launch.xml \
  lateral_controller_mode:=mpc \
  vehicle_param_file:=$(pwd)/campus_config/vehicle_info.param.yaml \
  lateral_controller_param_file:=$(pwd)/campus_config/control/mpc_campus.param.yaml \
  longitudinal_controller_param_file:=$(pwd)/campus_config/control/pid_longitudinal_campus.param.yaml
```

### 3.3 Applying Campus Pedestrian Parameters

You'll need to copy the campus parameter files to the launch config directory:

```bash
# Backup originals
cp src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml \
   src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml.backup

# Apply campus config
cp campus_config/planning/crosswalk_campus.param.yaml \
   src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml

# Same for obstacle stop
cp campus_config/planning/obstacle_stop_campus.param.yaml \
   src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml
```

### 3.4 Data Collection for Thesis

Record the following ROS 2 topics for analysis:

```bash
# Create rosbag for thesis analysis
ros2 bag record -o campus_test_pure_pursuit \
  /control/trajectory_follower/lateral/control_cmd \
  /control/trajectory_follower/lateral/diagnostic \
  /localization/kinematic_state \
  /planning/scenario_planning/trajectory \
  /perception/object_recognition/objects \
  /control/command/control_cmd \
  /vehicle/status/velocity_status \
  /diagnostics
```

**Key Topics for Analysis**:
- `/control/trajectory_follower/lateral/diagnostic` - Tracking errors
- `/vehicle/status/velocity_status` - Actual vehicle motion
- `/planning/scenario_planning/trajectory` - Reference trajectory
- `/perception/object_recognition/objects` - Pedestrian detections

---

## 4. Fleet Monitoring System

### 4.1 Monitoring Dashboard Requirements

For robotics lab visibility, set up monitoring for:

#### Real-Time Vehicle Status
- **Vehicle Position**: GPS coordinates, map overlay
- **Current State**: Autonomous/Manual, Speed, Battery
- **Active Controller**: Pure Pursuit / MPC
- **Current Route**: Origin, Destination, Progress

#### Safety Metrics
- **Pedestrian Detections**: Count, locations
- **Safety Violations**: Near-misses, hard braking events
- **Emergency Stops**: Count, reasons
- **System Health**: All nodes operational

#### Performance Metrics
- **Tracking Error**: Real-time lateral/heading error
- **Comfort Score**: Acceleration/jerk levels
- **Ride Quality**: Vibration, smoothness metrics
- **Controller Performance**: CPU usage, loop time

### 4.2 Visualization Options

#### Option 1: RViz Dashboard (Simple, Quick Setup)
```bash
# Launch RViz with custom config
rviz2 -d campus_config/rviz/fleet_monitor.rviz
```

**Advantages**:
- Native ROS 2 integration
- Easy to set up
- Real-time 3D visualization

**Limitations**:
- Not web-accessible
- Limited customization

#### Option 2: Foxglove Studio (Recommended for Thesis)
```bash
# Install Foxglove Studio (one-time)
# Download from https://foxglove.dev/download

# Run Foxglove Bridge
ros2 run foxglove_bridge foxglove_bridge
```

**Advantages**:
- Web-based (accessible in lab)
- Beautiful, customizable dashboards
- Data playback for analysis
- Export capabilities for thesis

**Setup**:
1. Install Foxglove Studio on lab monitor
2. Create custom layout for campus fleet
3. Configure panels for metrics above

#### Option 3: Custom Web Dashboard (Advanced)

Create custom web dashboard using:
- **rosbridge_suite**: WebSocket connection to ROS 2
- **roslibjs**: JavaScript ROS library
- **Leaflet.js**: Map visualization
- **Chart.js**: Real-time plots

Example stack:
```
ROS 2 ← rosbridge_suite ← WebSocket ← Web Browser Dashboard
```

### 4.3 Fleet Monitoring Setup

**File**: `campus_config/monitoring/fleet_dashboard.launch.xml`

```xml
<launch>
  <!-- Foxglove Bridge for web access -->
  <node pkg="foxglove_bridge" exec="foxglove_bridge" name="foxglove_bridge">
    <param name="port" value="8765"/>
    <param name="address" value="0.0.0.0"/>
  </node>

  <!-- Diagnostics aggregator for system health -->
  <node pkg="diagnostic_aggregator" exec="aggregator_node" name="diagnostic_aggregator">
    <rosparam file="$(find-pkg-share campus_config)/config/diagnostic_aggregator.param.yaml"/>
  </node>

  <!-- Status publisher for fleet info -->
  <node pkg="campus_monitoring" exec="fleet_status_publisher" name="fleet_status">
    <param name="update_rate" value="10.0"/>
    <param name="vehicle_id" value="campus_av_01"/>
  </node>
</launch>
```

---

## 5. Expected Results & Thesis Analysis

### 5.1 Hypotheses

**H1**: MPC will provide better tracking accuracy than Pure Pursuit at campus speeds (5-20 km/h)

**H2**: MPC will provide smoother ride quality (lower jerk) than Pure Pursuit

**H3**: Pure Pursuit will be computationally more efficient (lower CPU, faster loop times)

**H4**: Campus-tuned parameters will improve pedestrian safety metrics compared to default parameters

### 5.2 Analysis Methods

#### Statistical Comparison
- Paired t-tests for controller performance
- ANOVA for multi-scenario comparison
- Effect size calculations (Cohen's d)

#### Visualizations for Thesis
- Box plots of tracking errors by controller
- Time-series plots of comfort metrics
- Heat maps of pedestrian interactions
- Comparison tables of safety margins

#### Performance Profiles
- CPU usage vs tracking accuracy
- Comfort vs efficiency trade-offs
- Safety margin vs travel time

### 5.3 Thesis Chapters Suggestion

1. **Introduction**
   - Campus AV challenges
   - Research questions
   - Thesis objectives

2. **Literature Review**
   - Path following controllers
   - Pedestrian detection & avoidance
   - Campus autonomous vehicles

3. **Methodology**
   - Controller implementations
   - Parameter tuning approach
   - Test scenarios
   - Metrics definition

4. **System Architecture**
   - Autoware platform
   - Controller integration
   - Pedestrian yielding system
   - Fleet monitoring

5. **Experimental Results**
   - Pure Pursuit performance
   - MPC performance
   - Comparative analysis
   - Pedestrian safety evaluation

6. **Discussion**
   - Controller trade-offs
   - Campus-specific insights
   - Practical recommendations

7. **Conclusion**
   - Key findings
   - Contributions
   - Future work

---

## 6. Thesis Contribution Summary

### 6.1 Technical Contributions

1. **Campus-Optimized Control Parameters**
   - Tuned Pure Pursuit for low-speed campus operation
   - Tuned MPC for campus environment
   - Validated parameter sets for 5-20 km/h operation

2. **Enhanced Pedestrian Safety**
   - Extended safety margins for campus density
   - Improved occlusion handling
   - More conservative time-to-collision margins
   - Patient yielding behavior

3. **Controller Comparison Framework**
   - Metrics for campus-specific evaluation
   - Test scenarios for campus AVs
   - Methodology for controller selection

4. **Fleet Monitoring System**
   - Real-time visibility for operations
   - Safety monitoring dashboard
   - Performance tracking infrastructure

### 6.2 Practical Impact

- **Safer Campus Operation**: Enhanced pedestrian detection and yielding
- **Better Passenger Experience**: Reduced jerk, smooth acceleration
- **Operational Insights**: Fleet monitoring for continuous improvement
- **Validated Approach**: Data-driven controller selection

---

## 7. Next Steps

### Immediate Actions
1. ✅ Create campus-tuned parameter files
2. ⬜ Test parameters in simulation
3. ⬜ Set up data collection framework
4. ⬜ Create Foxglove dashboard layout
5. ⬜ Design test routes on campus

### Testing Phase
1. Simulate both controllers on test routes
2. Collect performance metrics
3. Refine parameters based on results
4. Validate pedestrian yielding logic

### Analysis Phase
1. Process rosbag data
2. Generate comparative statistics
3. Create thesis visualizations
4. Write results chapter

### Deployment Phase
1. Field test on campus (if applicable)
2. Gather real-world data
3. Validate simulation results
4. Final parameter refinement

---

## 8. File Organization

```
autoware_harry/
├── campus_config/
│   ├── control/
│   │   ├── pure_pursuit_campus.param.yaml
│   │   ├── mpc_campus.param.yaml
│   │   └── pid_longitudinal_campus.param.yaml
│   ├── planning/
│   │   ├── crosswalk_campus.param.yaml
│   │   └── obstacle_stop_campus.param.yaml
│   ├── monitoring/
│   │   ├── fleet_dashboard.launch.xml (TODO)
│   │   └── diagnostic_aggregator.param.yaml (TODO)
│   ├── rviz/
│   │   └── fleet_monitor.rviz (TODO)
│   └── test_scenarios/
│       └── campus_routes.yaml (TODO)
├── thesis_data/
│   ├── pure_pursuit_tests/
│   ├── mpc_tests/
│   └── analysis/
└── THESIS_CAMPUS_MODIFICATIONS.md (this file)
```

---

## 9. Resources & References

### Autoware Documentation
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)
- [Control Component Guide](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/control/)
- [Planning Component Guide](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/)

### Relevant Papers
- Pure Pursuit: Coulter, R. C. (1992). Implementation of the pure pursuit path tracking algorithm.
- MPC: Falcone, P., et al. (2007). Predictive active steering control for autonomous vehicle systems.
- Campus AVs: Your literature review sources

### Support
- Autoware Discord: https://discord.gg/Q94UsPvReQ
- GitHub Issues: https://github.com/autowarefoundation/autoware/issues

---

**Last Updated**: 2025-11-16
**Thesis Student**: [Your Name]
**Supervisor**: [Supervisor Name]
**Institution**: [University Name]
