# Campus Configuration Files

This directory contains campus-optimized configuration files for Autoware autonomous vehicle operation in a university environment.

## Directory Structure

```
campus_config/
├── control/          # Controller parameter files
├── planning/         # Planning and pedestrian yielding parameters
├── monitoring/       # Fleet monitoring configurations (TODO)
├── rviz/            # Visualization configs (TODO)
└── test_scenarios/  # Test route definitions (TODO)
```

## Quick Start

### 1. Testing Pure Pursuit Controller

```bash
# Copy campus config to autoware_launch
cp campus_config/control/pure_pursuit_campus.param.yaml \
   src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral/pure_pursuit.param.yaml

cp campus_config/control/pid_longitudinal_campus.param.yaml \
   src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml

# Launch with pure pursuit
ros2 launch autoware_launch autoware.launch.xml \
  lateral_controller_mode:=pure_pursuit
```

### 2. Testing MPC Controller

```bash
# Copy campus config to autoware_launch
cp campus_config/control/mpc_campus.param.yaml \
   src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml

# Launch with MPC
ros2 launch autoware_launch autoware.launch.xml \
  lateral_controller_mode:=mpc
```

### 3. Applying Campus Pedestrian Safety Parameters

```bash
# Apply crosswalk parameters
cp campus_config/planning/crosswalk_campus.param.yaml \
   src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml

# Apply obstacle stop parameters
cp campus_config/planning/obstacle_stop_campus.param.yaml \
   src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml
```

## Configuration Files

### Control Parameters

#### `control/pure_pursuit_campus.param.yaml`
Campus-optimized Pure Pursuit lateral controller:
- **Reduced lookahead**: 2.5-8.0 m (vs 4.35-15.0 m default)
- **Tighter curve following**: curvature_ratio 80.0 (vs 120.0)
- **Smoothing enabled**: for passenger comfort
- **Target speed**: 5-20 km/h campus operation

#### `control/mpc_campus.param.yaml`
Campus-optimized Model Predictive Control:
- **Enhanced tracking**: lateral error weight 1.5 (vs 1.0)
- **Increased smoothness**: jerk weight 0.2 (vs 0.1)
- **Gentler steering**: reduced rate limits
- **Comfort limits**: accel 1.2 m/s² (vs 2.0)

#### `control/pid_longitudinal_campus.param.yaml`
Campus-optimized longitudinal (speed) control:
- **Comfortable acceleration**: max 1.2 m/s² (vs 3.0)
- **Gentle braking**: min -2.0 m/s² (vs -5.0)
- **Smooth jerk**: max 1.0 m/s³ (vs 2.0)
- **Softer stopping**: reduced stopping accelerations

### Planning Parameters

#### `planning/crosswalk_campus.param.yaml`
Enhanced pedestrian safety at crosswalks:
- **Larger safety margins**: 4.0 m stop distance (vs 3.5 m)
- **More patient**: 2.0 s yield timeout (vs 1.0 s)
- **Conservative TTC**: 5.0 s ego-first margin (vs 4.0 s)
- **Better occlusion handling**: detect smaller obstacles (0.5 m vs 1.0 m)

#### `planning/obstacle_stop_campus.param.yaml`
Enhanced obstacle avoidance:
- **Increased margins**: 5.5 m stop distance (vs 5.0 m)
- **Wider detection**: 0.4 m lateral margin (vs 0.3 m)
- **More conservative TTC**: 1.5 s collision margin (vs 1.0 s)
- **Smoother velocity planning**: comfort-focused

## Key Campus Modifications

### Speed Range
- **Target**: 5-20 km/h (vs default 0-60 km/h)
- **Typical**: 10-15 km/h in pedestrian areas

### Safety Margins
- **Pedestrian clearance**: +0.5 m wider than default
- **Stop distances**: +0.5-1.0 m beyond default
- **Time margins**: +1.0 s more conservative

### Comfort Settings
- **Max acceleration**: 1.2 m/s² (gentle)
- **Max jerk**: 1.0 m/s³ (smooth)
- **Braking**: -2.0 m/s² (comfortable)

### Pedestrian Handling
- **Detection range**: Extended for occlusions
- **Yield patience**: 2-3 seconds
- **Stop distance from crosswalk**: 4.0 m

## Performance Comparison

### Pure Pursuit
**Pros:**
- Simple, reliable
- Low computational cost
- Predictable behavior
- Easy to tune

**Cons:**
- Less optimal at very low speeds
- Fixed lookahead strategy
- No explicit constraint handling

**Best for:**
- Higher campus speeds (15-20 km/h)
- Straight paths with gentle curves
- Computational efficiency needed

### MPC
**Pros:**
- Optimal trajectory tracking
- Handles constraints explicitly
- Better at very low speeds
- Smoother control

**Cons:**
- Higher computational cost
- More parameters to tune
- Requires accurate vehicle model

**Best for:**
- Low speeds (5-10 km/h)
- Tight maneuvers
- Maximum comfort needed

## Data Collection

### Record Data for Analysis

```bash
# Pure Pursuit test
ros2 bag record -o campus_pure_pursuit \
  /control/trajectory_follower/lateral/control_cmd \
  /control/trajectory_follower/lateral/diagnostic \
  /localization/kinematic_state \
  /planning/scenario_planning/trajectory \
  /perception/object_recognition/objects \
  /vehicle/status/velocity_status

# MPC test
ros2 bag record -o campus_mpc \
  /control/trajectory_follower/lateral/control_cmd \
  /control/trajectory_follower/lateral/diagnostic \
  /localization/kinematic_state \
  /planning/scenario_planning/trajectory \
  /perception/object_recognition/objects \
  /vehicle/status/velocity_status
```

## Metrics to Track

### Tracking Performance
- Lateral error (m)
- Heading error (rad)
- Maximum deviation (m)

### Comfort
- Lateral acceleration (m/s²)
- Lateral jerk (m/s³)
- Steering rate (rad/s)

### Safety
- Pedestrian clearance (m)
- Stop distance (m)
- Emergency stop count

### Efficiency
- Average speed (m/s)
- Travel time (s)
- Number of stops

## Troubleshooting

### Controller Not Loading
```bash
# Check parameter file syntax
ros2 param list /control/trajectory_follower_node
ros2 param get /control/trajectory_follower_node lateral_controller_mode
```

### Poor Tracking Performance
1. Check lookahead distances (might be too large/small)
2. Verify vehicle model parameters match actual vehicle
3. Check trajectory quality from planner

### Excessive Stopping
1. Review pedestrian detection parameters
2. Adjust TTC margins if too conservative
3. Check occlusion detection settings

### Uncomfortable Ride
1. Increase path smoothing
2. Reduce acceleration/jerk limits
3. Lower PID gains

## Contributing

When modifying parameters:
1. Document changes in this README
2. Test in simulation first
3. Record data before/after
4. Update THESIS_CAMPUS_MODIFICATIONS.md

## References

- Main thesis documentation: `../THESIS_CAMPUS_MODIFICATIONS.md`
- Autoware documentation: https://autowarefoundation.github.io/autoware-documentation/
- Pure Pursuit theory: Coulter, R.C. (1992)
- MPC theory: Falcone, P. et al. (2007)
