# Rosbag Recording and Analysis Guide

Complete guide for using ROS 2 bags to record and analyze campus AV controller performance.

## 📦 What are Rosbags?

Rosbags are **the standard way to record ROS 2 data**. They capture all published messages on specified topics, allowing you to:
- Record real-world test runs
- Play back data for analysis
- Share data with advisors/colleagues
- Debug issues offline
- Generate thesis plots and statistics

## 🎯 Quick Start

### Option 1: Use the Automated Scripts (Recommended)

```bash
# Compare both controllers with automated recording
./scripts/compare_controllers.sh --name my_campus_test --duration 300

# This automatically:
# - Guides you through testing both controllers
# - Records rosbags for each test
# - Organizes data into thesis_data/
```

### Option 2: Manual Recording

```bash
# Record a custom rosbag
./scripts/record_rosbag.sh my_test_name 300

# Records 300 seconds (5 minutes) of data
# Saves to thesis_data/manual_recordings/
```

### Option 3: Direct ros2 bag Command

```bash
# Most basic usage
ros2 bag record /topic1 /topic2

# Record all topics (not recommended - huge files!)
ros2 bag record -a

# Record specific topics for thesis
ros2 bag record \
  /control/trajectory_follower/lateral/diagnostic \
  /vehicle/status/velocity_status \
  /perception/object_recognition/objects
```

## 📊 Topics to Record for Your Thesis

### Essential Topics (Always Record These)

#### Controller Performance
```bash
/control/trajectory_follower/lateral/control_cmd        # Steering commands
/control/trajectory_follower/lateral/diagnostic         # Tracking errors
/control/command/control_cmd                            # Final control output
```

#### Vehicle State
```bash
/localization/kinematic_state                           # Position, velocity
/vehicle/status/velocity_status                         # Speed
/vehicle/status/steering_status                         # Steering angle
```

#### Planning
```bash
/planning/scenario_planning/trajectory                  # Reference trajectory
```

#### Pedestrian Detection
```bash
/perception/object_recognition/objects                  # Detected pedestrians
```

### Optional Topics (For Deep Analysis)

```bash
/diagnostics                                            # System health
/tf                                                     # Coordinate transforms
/planning/scenario_planning/lane_driving/behavior_planning/path  # Behavior planning
```

## 🔧 Recording Commands

### Basic Recording

```bash
# Record for 5 minutes
ros2 bag record -d 300 /control/trajectory_follower/lateral/diagnostic

# Record with custom output name
ros2 bag record -o my_test_pure_pursuit /control/trajectory_follower/lateral/diagnostic

# Record multiple topics
ros2 bag record \
  /control/trajectory_follower/lateral/diagnostic \
  /vehicle/status/velocity_status
```

### Advanced Recording Options

```bash
# Use compression to save disk space
ros2 bag record \
  --compression-mode file \
  /control/trajectory_follower/lateral/diagnostic

# Set storage format (sqlite3 is default and recommended)
ros2 bag record \
  --storage sqlite3 \
  /control/trajectory_follower/lateral/diagnostic

# Unlimited cache (prevents message loss)
ros2 bag record \
  --max-cache-size 0 \
  /control/trajectory_follower/lateral/diagnostic

# Full campus recording command
ros2 bag record \
  -o campus_test \
  --storage sqlite3 \
  --compression-mode file \
  --max-cache-size 0 \
  /control/trajectory_follower/lateral/diagnostic \
  /control/command/control_cmd \
  /localization/kinematic_state \
  /vehicle/status/velocity_status \
  /perception/object_recognition/objects \
  /planning/scenario_planning/trajectory
```

### Recording with QoS Overrides

Some topics use "best effort" QoS which can cause recording issues:

```bash
# Use QoS overrides file
ros2 bag record \
  --qos-profile-overrides-path scripts/qos_overrides.yaml \
  /perception/object_recognition/objects
```

## 📖 Viewing Rosbag Data

### Get Bag Information

```bash
# Show bag metadata
ros2 bag info thesis_data/pure_pursuit_tests/my_test

# Example output:
# Files:             my_test.db3
# Bag size:          45.2 MB
# Storage id:        sqlite3
# Duration:          300.5s
# Start:             Jan 15 2025 10:30:00.123
# End:               Jan 15 2025 10:35:00.623
# Messages:          15234
# Topic information: Topic: /control/trajectory_follower/lateral/diagnostic | Type: ... | Count: 9000 | ...
```

### Play Back Rosbag

```bash
# Play at normal speed
ros2 bag play thesis_data/pure_pursuit_tests/my_test

# Play at 2x speed
ros2 bag play -r 2.0 thesis_data/pure_pursuit_tests/my_test

# Play at 0.5x speed (slow motion)
ros2 bag play -r 0.5 thesis_data/pure_pursuit_tests/my_test

# Start from specific time (10 seconds in)
ros2 bag play -s 10 thesis_data/pure_pursuit_tests/my_test

# Play specific duration (30 seconds)
ros2 bag play -d 30 thesis_data/pure_pursuit_tests/my_test

# Play in loop
ros2 bag play -l thesis_data/pure_pursuit_tests/my_test

# Play with pausing capability
ros2 bag play --pause thesis_data/pure_pursuit_tests/my_test
# Then press SPACE to pause/resume
```

### Echo Messages from Bag

```bash
# While playing, echo a topic in another terminal
ros2 bag play thesis_data/pure_pursuit_tests/my_test

# In another terminal:
ros2 topic echo /control/trajectory_follower/lateral/diagnostic
```

## 🔬 Analyzing Rosbag Data

### Method 1: Use the Provided Analysis Script

```bash
# Install dependencies first
pip install rosbags numpy matplotlib pandas scipy

# Basic analysis
python3 scripts/analyze_rosbag.py \
  thesis_data/pure_pursuit_tests/my_test \
  --controller pure_pursuit

# With plots
python3 scripts/analyze_rosbag.py \
  thesis_data/pure_pursuit_tests/my_test \
  --controller pure_pursuit \
  --plot

# Save plots for thesis
python3 scripts/analyze_rosbag.py \
  thesis_data/pure_pursuit_tests/my_test \
  --controller pure_pursuit \
  --save-plots thesis_data/analysis/plots

# Export to CSV for Excel/MATLAB
python3 scripts/analyze_rosbag.py \
  thesis_data/pure_pursuit_tests/my_test \
  --controller pure_pursuit \
  --export-csv thesis_data/analysis/csv
```

**Output includes**:
- ✅ Tracking performance (lateral error, heading error, RMS)
- ✅ Comfort metrics (acceleration, jerk)
- ✅ Safety metrics (pedestrian distances)
- ✅ Efficiency metrics (average speed)
- ✅ Publication-quality plots
- ✅ CSV data export

### Method 2: Custom Python Analysis

```python
#!/usr/bin/env python3
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np

# Open bag
bag_path = 'thesis_data/pure_pursuit_tests/my_test'
lateral_errors = []

with Reader(bag_path) as reader:
    # Iterate through messages
    for connection, timestamp, rawdata in reader.messages():
        # Filter for specific topic
        if connection.topic == '/control/trajectory_follower/lateral/diagnostic':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            lateral_errors.append(msg.lateral_error)

# Analyze
print(f"Mean lateral error: {np.mean(np.abs(lateral_errors)):.4f} m")
print(f"Max lateral error: {np.max(np.abs(lateral_errors)):.4f} m")
print(f"RMS lateral error: {np.sqrt(np.mean(np.square(lateral_errors))):.4f} m")
```

### Method 3: Export to CSV Then Use Excel/MATLAB

```bash
# Export specific topics to CSV
ros2 bag export \
  thesis_data/pure_pursuit_tests/my_test \
  --output thesis_data/analysis/exported_data

# Or use the analysis script
python3 scripts/analyze_rosbag.py \
  thesis_data/pure_pursuit_tests/my_test \
  --export-csv thesis_data/analysis/csv
```

Then open in Excel, MATLAB, or Python for custom analysis.

## 📈 Comparing Controllers

### Step 1: Record Both Controllers

```bash
# Automated way (recommended)
./scripts/compare_controllers.sh --name campus_route1 --duration 300
```

### Step 2: Analyze Each

```bash
# Pure Pursuit
python3 scripts/analyze_rosbag.py \
  thesis_data/pure_pursuit_tests/campus_route1_pure_pursuit_01 \
  --controller pure_pursuit \
  --save-plots thesis_data/analysis/plots \
  --export-csv thesis_data/analysis/csv

# MPC
python3 scripts/analyze_rosbag.py \
  thesis_data/mpc_tests/campus_route1_mpc_01 \
  --controller mpc \
  --save-plots thesis_data/analysis/plots \
  --export-csv thesis_data/analysis/csv
```

### Step 3: Compare Statistics

Create a comparison script `scripts/compare_stats.py`:

```python
#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# Load CSV data from both controllers
pp_data = pd.read_csv('thesis_data/analysis/csv/pure_pursuit_data.csv')
mpc_data = pd.read_csv('thesis_data/analysis/csv/mpc_data.csv')

# Compare lateral errors
pp_error = pp_data['lateral_error'].abs().mean()
mpc_error = mpc_data['lateral_error'].abs().mean()

print(f"Pure Pursuit mean lateral error: {pp_error:.4f} m")
print(f"MPC mean lateral error: {mpc_error:.4f} m")
print(f"Improvement: {((pp_error - mpc_error) / pp_error * 100):.1f}%")

# Create comparison bar chart
controllers = ['Pure Pursuit', 'MPC']
errors = [pp_error, mpc_error]

plt.bar(controllers, errors)
plt.ylabel('Mean Lateral Error (m)')
plt.title('Controller Comparison: Tracking Error')
plt.savefig('thesis_data/analysis/plots/comparison.png', dpi=300)
```

## 💾 Managing Rosbag Storage

### Disk Space Considerations

Rosbags can get **very large**:
- **5 minutes** of essential topics: ~100-500 MB
- **5 minutes** with cameras: ~5-20 GB
- **5 minutes** with all topics (-a): ~50+ GB

**For your thesis**, 100-500 MB per test is reasonable.

### Check Bag Size

```bash
# Check size
du -sh thesis_data/pure_pursuit_tests/my_test

# Check all bags
du -sh thesis_data/*_tests/*
```

### Compress Bags

```bash
# Already compressed during recording with --compression-mode file

# Or compress after recording
ros2 bag compress thesis_data/pure_pursuit_tests/my_test
```

### Delete Old Bags

```bash
# Delete specific bag
rm -rf thesis_data/manual_recordings/old_test

# Clean up old bags (be careful!)
find thesis_data/ -name "*.db3" -mtime +30 -delete  # Delete bags older than 30 days
```

## 🎓 Thesis Workflow with Rosbags

### 1. Planning Phase
```bash
# Create test plan
mkdir -p thesis_data/test_plan
echo "Route 1: Library to Cafeteria - straight path" > thesis_data/test_plan/routes.txt
echo "Route 2: Main Circle - tight turns" >> thesis_data/test_plan/routes.txt
```

### 2. Data Collection Phase
```bash
# For each route, test both controllers
for route in route1 route2 route3; do
    # Pure Pursuit
    ./scripts/compare_controllers.sh --name ${route} --duration 300
done
```

### 3. Analysis Phase
```bash
# Analyze all bags
for bag in thesis_data/pure_pursuit_tests/*; do
    python3 scripts/analyze_rosbag.py $bag --controller pure_pursuit --save-plots thesis_data/analysis/plots
done

for bag in thesis_data/mpc_tests/*; do
    python3 scripts/analyze_rosbag.py $bag --controller mpc --save-plots thesis_data/analysis/plots
done
```

### 4. Visualization Phase
```bash
# Generate thesis plots
python3 scripts/compare_stats.py

# Result: Beautiful plots in thesis_data/analysis/plots/
```

## 🔍 Troubleshooting

### Problem: "No messages recorded"

**Solution**: Check QoS compatibility
```bash
# Use QoS overrides
ros2 bag record --qos-profile-overrides-path scripts/qos_overrides.yaml /topic
```

### Problem: "Bag file too large"

**Solutions**:
1. Don't record camera topics unless needed
2. Use compression: `--compression-mode file`
3. Record only essential topics
4. Reduce recording duration

### Problem: "Can't play bag - clock issues"

**Solution**: Use sim time flag
```bash
# If bag was recorded in simulation
ros2 bag play --clock thesis_data/my_test
```

### Problem: "Topic not in bag"

**Check what's actually in the bag**:
```bash
ros2 bag info thesis_data/my_test
```

### Problem: "Python library errors"

**Install dependencies**:
```bash
pip install rosbags numpy matplotlib pandas scipy
```

## 📚 Additional Resources

### Official ROS 2 Docs
- https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html

### Python Libraries
- **rosbags**: https://pypi.org/project/rosbags/ (reading bags in Python)
- **pandas**: Data analysis
- **matplotlib**: Plotting
- **scipy**: Statistics

### Useful Commands Cheatsheet

```bash
# Record
ros2 bag record -o name /topic1 /topic2

# Info
ros2 bag info path/to/bag

# Play
ros2 bag play path/to/bag

# Play at 2x speed
ros2 bag play -r 2.0 path/to/bag

# Compress
ros2 bag compress path/to/bag

# List topics while recording
ros2 topic list

# Check topic rate
ros2 topic hz /topic_name

# Echo topic while playing
ros2 topic echo /topic_name
```

## ✅ Best Practices for Thesis

1. **Organize by test scenario**
   ```
   thesis_data/
   ├── route1_straight/
   ├── route2_turns/
   └── route3_pedestrians/
   ```

2. **Use descriptive names**
   ```bash
   # Good
   campus_library_route_pure_pursuit_20250116_1430

   # Bad
   test1
   ```

3. **Record metadata**
   ```bash
   # Create notes file
   echo "Date: 2025-01-16" > thesis_data/route1/notes.txt
   echo "Weather: Sunny" >> thesis_data/route1/notes.txt
   echo "Pedestrian density: Low" >> thesis_data/route1/notes.txt
   ```

4. **Backup important data**
   ```bash
   # Backup to external drive
   rsync -av thesis_data/ /media/backup/thesis_rosbags/
   ```

5. **Keep raw data**
   - Don't delete rosbags until thesis is submitted
   - Advisor might want to verify results
   - You might need to re-analyze

---

**For complete thesis framework, see**: `../THESIS_CAMPUS_MODIFICATIONS.md`
