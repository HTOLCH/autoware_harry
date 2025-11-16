# Fleet Monitoring Setup Guide

This guide helps you set up real-time fleet monitoring for the robotics lab.

## Option 1: Foxglove Studio (Recommended)

### Advantages
- Beautiful web-based interface
- Accessible from multiple monitors
- Built-in layouts for autonomous vehicles
- Easy data recording and playback
- Export capabilities for thesis

### Setup Steps

#### 1. Install Foxglove Bridge

```bash
# Install via apt (if available)
sudo apt install ros-humble-foxglove-bridge

# Or build from source
cd ~/autoware_harry
mkdir -p src/monitoring
cd src/monitoring
git clone https://github.com/foxglove/ros-foxglove-bridge.git
cd ~/autoware_harry
colcon build --packages-select foxglove_bridge
```

#### 2. Launch Foxglove Bridge

```bash
# Source workspace
source ~/autoware_harry/install/setup.bash

# Launch bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# Or run directly
ros2 run foxglove_bridge foxglove_bridge
```

Default port: `8765`

#### 3. Install Foxglove Studio on Lab Monitor

**Option A: Web version**
- Open browser to: https://app.foxglove.dev
- Click "Open connection"
- Enter WebSocket URL: `ws://<vehicle-ip>:8765`

**Option B: Desktop app**
- Download from: https://foxglove.dev/download
- Install on Ubuntu lab monitor
- Connect to WebSocket: `ws://<vehicle-ip>:8765`

#### 4. Create Campus Fleet Layout

In Foxglove Studio:

1. **Add Map Panel**
   - Shows vehicle position on campus map
   - Subscribe to `/map` and `/vehicle/pose`

2. **Add 3D Panel**
   - Visualize vehicle, pedestrians, trajectory
   - Subscribe to:
     - `/perception/object_recognition/objects`
     - `/planning/scenario_planning/trajectory`
     - `/tf`

3. **Add Plot Panel** for tracking error
   - X-axis: time
   - Y-axis: `/control/trajectory_follower/lateral/diagnostic/lateral_error`

4. **Add Plot Panel** for speed
   - Y-axis: `/vehicle/status/velocity_status/longitudinal_velocity`

5. **Add State Transitions Panel**
   - Show vehicle state (autonomous/manual)
   - Subscribe to `/autoware/state`

6. **Add Diagnostics Panel**
   - System health monitoring
   - Subscribe to `/diagnostics`

7. **Add Table Panel** for pedestrian count
   - Show detected pedestrians
   - Subscribe to `/perception/object_recognition/objects`

8. **Save layout as** `campus_fleet_monitor.json`

#### 5. Full-Screen Display

For robotics lab monitor:
```bash
# Launch Foxglove in fullscreen
foxglove-studio --fullscreen --layout=/path/to/campus_fleet_monitor.json
```

---

## Option 2: RViz2 (Simple, ROS Native)

### Advantages
- Built into ROS 2
- No additional installation
- Good for development/debugging

### Setup Steps

#### 1. Create RViz Config

Save to `campus_config/rviz/fleet_monitor.rviz`:

```yaml
# Basic RViz config for campus monitoring
# Load with: rviz2 -d campus_config/rviz/fleet_monitor.rviz
```

#### 2. Configure Panels

Add these displays in RViz:

1. **Map** - Campus map background
2. **TF** - Vehicle frame transforms
3. **PointCloud2** - Lidar data (`/sensing/lidar/concatenated/pointcloud`)
4. **Path** - Trajectory (`/planning/scenario_planning/trajectory`)
5. **MarkerArray** - Detected objects (`/perception/object_recognition/objects/markers`)
6. **PoseStamped** - Vehicle pose (`/localization/pose_estimator/pose`)

#### 3. Launch on Lab Monitor

```bash
rviz2 -d ~/autoware_harry/campus_config/rviz/fleet_monitor.rviz --fullscreen
```

### Limitations
- Not web-accessible
- Less customizable than Foxglove
- No built-in data recording

---

## Option 3: Custom Web Dashboard (Advanced)

### Architecture

```
[ROS 2] ← rosbridge_suite ← WebSocket ← [Web Browser]
```

### Setup Steps

#### 1. Install rosbridge

```bash
sudo apt install ros-humble-rosbridge-suite
```

#### 2. Launch rosbridge

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Default port: `9090`

#### 3. Create Web Dashboard

Create `campus_config/monitoring/dashboard.html`:

```html
<!DOCTYPE html>
<html>
<head>
    <title>Campus Fleet Monitor</title>
    <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        body { font-family: Arial; margin: 0; padding: 20px; background: #1a1a1a; color: #fff; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .panel { background: #2a2a2a; padding: 20px; border-radius: 8px; }
        #map { height: 400px; }
        .metric { font-size: 2em; font-weight: bold; color: #4CAF50; }
        .status { padding: 10px; border-radius: 4px; margin: 5px 0; }
        .status.ok { background: #4CAF50; }
        .status.warning { background: #FF9800; }
        .status.error { background: #f44336; }
    </style>
</head>
<body>
    <h1>🚗 Campus Fleet Monitor</h1>

    <div class="grid">
        <div class="panel">
            <h2>Vehicle Status</h2>
            <div>Speed: <span id="speed" class="metric">0.0</span> m/s</div>
            <div>Controller: <span id="controller">-</span></div>
            <div>Mode: <span id="mode">-</span></div>
            <div class="status ok" id="status">System OK</div>
        </div>

        <div class="panel">
            <h2>Safety Metrics</h2>
            <div>Pedestrians Detected: <span id="ped-count" class="metric">0</span></div>
            <div>Min Distance: <span id="min-dist">-</span> m</div>
            <div>Emergency Stops: <span id="estops">0</span></div>
        </div>

        <div class="panel">
            <h2>Tracking Performance</h2>
            <canvas id="error-chart"></canvas>
        </div>

        <div class="panel">
            <h2>Campus Map</h2>
            <div id="map"></div>
        </div>
    </div>

    <script>
        // Connect to ROS
        var ros = new ROSLIB.Ros({
            url : 'ws://localhost:9090'
        });

        ros.on('connection', function() {
            document.getElementById('status').textContent = 'Connected to ROS';
            document.getElementById('status').className = 'status ok';
        });

        ros.on('error', function(error) {
            document.getElementById('status').textContent = 'ROS Error';
            document.getElementById('status').className = 'status error';
        });

        // Subscribe to vehicle velocity
        var velocityListener = new ROSLIB.Topic({
            ros : ros,
            name : '/vehicle/status/velocity_status',
            messageType : 'autoware_vehicle_msgs/VelocityReport'
        });

        velocityListener.subscribe(function(message) {
            document.getElementById('speed').textContent =
                message.longitudinal_velocity.toFixed(2);
        });

        // Subscribe to detected objects (pedestrians)
        var objectsListener = new ROSLIB.Topic({
            ros : ros,
            name : '/perception/object_recognition/objects',
            messageType : 'autoware_perception_msgs/PredictedObjects'
        });

        objectsListener.subscribe(function(message) {
            var pedCount = message.objects.filter(obj =>
                obj.classification[0].label === 'PEDESTRIAN'
            ).length;
            document.getElementById('ped-count').textContent = pedCount;
        });

        // Subscribe to tracking error
        var diagnosticListener = new ROSLIB.Topic({
            ros : ros,
            name : '/control/trajectory_follower/lateral/diagnostic',
            messageType : 'autoware_control_msgs/LateralControllerDiagnostic'
        });

        var errorData = [];
        var maxPoints = 100;

        diagnosticListener.subscribe(function(message) {
            errorData.push(message.lateral_error);
            if (errorData.length > maxPoints) errorData.shift();
            updateChart();
        });

        // Initialize chart
        var ctx = document.getElementById('error-chart').getContext('2d');
        var chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: Array(maxPoints).fill(''),
                datasets: [{
                    label: 'Lateral Error (m)',
                    data: errorData,
                    borderColor: '#4CAF50',
                    backgroundColor: 'rgba(76, 175, 80, 0.1)',
                    borderWidth: 2
                }]
            },
            options: {
                responsive: true,
                scales: {
                    y: { beginAtZero: true }
                },
                animation: false
            }
        });

        function updateChart() {
            chart.data.datasets[0].data = errorData;
            chart.update('none');
        }

        // Initialize map
        var map = L.map('map').setView([0, 0], 18); // Set to your campus coords
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
        var vehicleMarker = L.marker([0, 0]).addTo(map);

        // Subscribe to vehicle pose
        var poseListener = new ROSLIB.Topic({
            ros : ros,
            name : '/localization/pose_estimator/pose',
            messageType : 'geometry_msgs/PoseStamped'
        });

        poseListener.subscribe(function(message) {
            // Convert to lat/lon (you'll need proper conversion)
            var lat = message.pose.position.y; // Simplified
            var lon = message.pose.position.x;
            vehicleMarker.setLatLng([lat, lon]);
            map.panTo([lat, lon]);
        });
    </script>
</body>
</html>
```

#### 4. Serve Dashboard

```bash
# Simple HTTP server
cd ~/autoware_harry/campus_config/monitoring
python3 -m http.server 8000

# Open in browser
firefox http://localhost:8000/dashboard.html
```

---

## Multi-Vehicle Fleet Monitoring

For multiple vehicles on campus:

### 1. Namespace Each Vehicle

```bash
# Vehicle 1
ros2 launch autoware_launch autoware.launch.xml \
  use_sim_time:=false \
  namespace:=/av1

# Vehicle 2
ros2 launch autoware_launch autoware.launch.xml \
  use_sim_time:=false \
  namespace:=/av2
```

### 2. Aggregate Data

Create aggregator node to combine data from all vehicles:

```python
#!/usr/bin/env python3
# scripts/fleet_aggregator.py

import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport

class FleetAggregator(Node):
    def __init__(self):
        super().__init__('fleet_aggregator')

        # Subscribe to each vehicle
        self.create_subscription(
            VelocityReport,
            '/av1/vehicle/status/velocity_status',
            lambda msg: self.velocity_callback(msg, 'av1'),
            10
        )

        self.create_subscription(
            VelocityReport,
            '/av2/vehicle/status/velocity_status',
            lambda msg: self.velocity_callback(msg, 'av2'),
            10
        )

        self.vehicle_status = {}

    def velocity_callback(self, msg, vehicle_id):
        self.vehicle_status[vehicle_id] = {
            'speed': msg.longitudinal_velocity,
            'timestamp': self.get_clock().now()
        }
        self.publish_fleet_status()

    def publish_fleet_status(self):
        # Publish aggregated status
        self.get_logger().info(f'Fleet status: {self.vehicle_status}')

def main():
    rclpy.init()
    node = FleetAggregator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## Recommended Setup for Thesis

**For your thesis, I recommend:**

1. **Development**: RViz2
   - Quick debugging
   - Test parameter changes

2. **Data Collection**: Foxglove Studio
   - Record all test runs
   - Beautiful visualizations for thesis
   - Easy data export

3. **Lab Display**: Foxglove Studio (fullscreen)
   - Large monitor in robotics lab
   - Real-time fleet status
   - Impress visitors!

4. **Analysis**: Python + rosbags
   - Extract data from Foxglove recordings
   - Generate thesis plots
   - Statistical analysis

---

## Next Steps

1. Choose monitoring option (recommend Foxglove)
2. Install and test
3. Create campus-specific layout
4. Set up lab monitor
5. Test with both controllers
6. Record data for thesis

## Troubleshooting

### Foxglove Bridge Won't Connect
```bash
# Check if bridge is running
ros2 node list | grep foxglove

# Check port
netstat -an | grep 8765

# Restart bridge
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

### No Data in Dashboard
```bash
# Check topics are publishing
ros2 topic list
ros2 topic echo /vehicle/status/velocity_status

# Check Foxglove bridge topics
ros2 topic list | grep foxglove
```

### Performance Issues
- Reduce update rate in Foxglove
- Limit number of panels
- Use compression for WebSocket
- Check network latency

---

**For questions, see main thesis documentation:** `../THESIS_CAMPUS_MODIFICATIONS.md`
