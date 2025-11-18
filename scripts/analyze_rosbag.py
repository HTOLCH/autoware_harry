#!/usr/bin/env python3
"""
Rosbag Analysis Script for Campus AV Thesis
Extracts and analyzes controller performance metrics from ROS 2 bags

Usage:
    python3 analyze_rosbag.py /path/to/rosbag --controller pure_pursuit
    python3 analyze_rosbag.py /path/to/rosbag --controller mpc --plot
"""

import argparse
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

try:
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
except ImportError:
    print("ERROR: rosbags library not installed")
    print("Install with: pip install rosbags")
    sys.exit(1)

# Optional imports
try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False
    print("WARNING: pandas not available. Install with: pip install pandas")

try:
    from scipy import stats
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("WARNING: scipy not available. Install with: pip install scipy")


class RosbagAnalyzer:
    """Analyze controller performance from rosbag data"""

    def __init__(self, bag_path, controller_name="unknown"):
        self.bag_path = Path(bag_path)
        self.controller_name = controller_name

        # Data storage
        self.lateral_errors = []
        self.heading_errors = []
        self.velocities = []
        self.steering_angles = []
        self.timestamps = []
        self.lateral_accels = []
        self.lateral_jerks = []
        self.pedestrian_counts = []
        self.min_pedestrian_distances = []

    def extract_data(self):
        """Extract relevant data from rosbag"""
        print(f"Opening rosbag: {self.bag_path}")

        with Reader(self.bag_path) as reader:
            # Get topic info
            connections = [c for c in reader.connections]
            print(f"Found {len(connections)} topics")

            # Process messages
            for connection, timestamp, rawdata in reader.messages():
                msg = deserialize_cdr(rawdata, connection.msgtype)

                # Lateral diagnostic (tracking errors)
                if connection.topic == '/control/trajectory_follower/lateral/diagnostic':
                    self.lateral_errors.append(msg.lateral_error)
                    self.heading_errors.append(msg.heading_error)
                    self.timestamps.append(timestamp / 1e9)  # Convert to seconds

                # Velocity status
                elif connection.topic == '/vehicle/status/velocity_status':
                    self.velocities.append(msg.longitudinal_velocity)

                # Steering status
                elif connection.topic == '/vehicle/status/steering_status':
                    self.steering_angles.append(msg.steering_tire_angle)

                # Pedestrian objects
                elif connection.topic == '/perception/object_recognition/objects':
                    pedestrians = [obj for obj in msg.objects
                                 if obj.classification[0].label == 4]  # 4 = PEDESTRIAN
                    self.pedestrian_counts.append(len(pedestrians))

                    # Calculate minimum distance to pedestrians
                    if pedestrians:
                        # Simplified - assumes ego at origin
                        distances = [np.sqrt(p.kinematics.initial_pose_with_covariance.pose.position.x**2 +
                                           p.kinematics.initial_pose_with_covariance.pose.position.y**2)
                                   for p in pedestrians]
                        self.min_pedestrian_distances.append(min(distances))
                    else:
                        self.min_pedestrian_distances.append(float('inf'))

        print(f"Extracted {len(self.lateral_errors)} tracking error samples")
        print(f"Extracted {len(self.velocities)} velocity samples")
        print(f"Extracted {len(self.pedestrian_counts)} pedestrian detection samples")

        # Compute derived metrics
        self._compute_acceleration_jerk()

    def _compute_acceleration_jerk(self):
        """Compute lateral acceleration and jerk from velocity and steering"""
        if len(self.velocities) < 2 or len(self.steering_angles) < 2:
            return

        # Simplified calculation - assumes 2.79m wheelbase (from vehicle_info)
        wheelbase = 2.79

        for i in range(min(len(self.velocities), len(self.steering_angles))):
            v = self.velocities[i]
            delta = self.steering_angles[i]

            # Lateral acceleration: a_lat = v^2 * tan(delta) / L
            lat_accel = (v**2 * np.tan(delta)) / wheelbase
            self.lateral_accels.append(lat_accel)

        # Jerk is derivative of acceleration
        if len(self.lateral_accels) >= 2:
            dt = 0.03  # Assume 30 Hz control rate
            for i in range(1, len(self.lateral_accels)):
                jerk = (self.lateral_accels[i] - self.lateral_accels[i-1]) / dt
                self.lateral_jerks.append(jerk)

    def compute_statistics(self):
        """Compute statistical metrics"""
        stats_dict = {}

        # Tracking performance
        if self.lateral_errors:
            stats_dict['lateral_error_mean'] = np.mean(np.abs(self.lateral_errors))
            stats_dict['lateral_error_std'] = np.std(self.lateral_errors)
            stats_dict['lateral_error_max'] = np.max(np.abs(self.lateral_errors))
            stats_dict['lateral_error_rms'] = np.sqrt(np.mean(np.square(self.lateral_errors)))

        if self.heading_errors:
            stats_dict['heading_error_mean'] = np.mean(np.abs(self.heading_errors))
            stats_dict['heading_error_std'] = np.std(self.heading_errors)
            stats_dict['heading_error_max'] = np.max(np.abs(self.heading_errors))

        # Comfort metrics
        if self.lateral_accels:
            stats_dict['lateral_accel_mean'] = np.mean(np.abs(self.lateral_accels))
            stats_dict['lateral_accel_max'] = np.max(np.abs(self.lateral_accels))
            stats_dict['lateral_accel_std'] = np.std(self.lateral_accels)

        if self.lateral_jerks:
            stats_dict['lateral_jerk_mean'] = np.mean(np.abs(self.lateral_jerks))
            stats_dict['lateral_jerk_max'] = np.max(np.abs(self.lateral_jerks))
            stats_dict['lateral_jerk_std'] = np.std(self.lateral_jerks)

        # Safety metrics
        if self.pedestrian_counts:
            stats_dict['avg_pedestrians_detected'] = np.mean(self.pedestrian_counts)
            stats_dict['max_pedestrians_detected'] = np.max(self.pedestrian_counts)

        if self.min_pedestrian_distances:
            valid_distances = [d for d in self.min_pedestrian_distances if d != float('inf')]
            if valid_distances:
                stats_dict['min_pedestrian_distance'] = np.min(valid_distances)
                stats_dict['avg_pedestrian_distance'] = np.mean(valid_distances)

        # Efficiency metrics
        if self.velocities:
            stats_dict['avg_velocity'] = np.mean(self.velocities)
            stats_dict['max_velocity'] = np.max(self.velocities)

        return stats_dict

    def print_statistics(self):
        """Print analysis results"""
        stats = self.compute_statistics()

        print("\n" + "="*60)
        print(f"Analysis Results: {self.controller_name.upper()}")
        print("="*60)

        print("\n📊 TRACKING PERFORMANCE")
        print("-" * 60)
        if 'lateral_error_mean' in stats:
            print(f"  Lateral Error (Mean):    {stats['lateral_error_mean']:.4f} m")
            print(f"  Lateral Error (RMS):     {stats['lateral_error_rms']:.4f} m")
            print(f"  Lateral Error (Max):     {stats['lateral_error_max']:.4f} m")
            print(f"  Lateral Error (Std Dev): {stats['lateral_error_std']:.4f} m")

        if 'heading_error_mean' in stats:
            print(f"  Heading Error (Mean):    {stats['heading_error_mean']:.4f} rad ({np.rad2deg(stats['heading_error_mean']):.2f}°)")
            print(f"  Heading Error (Max):     {stats['heading_error_max']:.4f} rad ({np.rad2deg(stats['heading_error_max']):.2f}°)")

        print("\n🎢 COMFORT METRICS")
        print("-" * 60)
        if 'lateral_accel_mean' in stats:
            print(f"  Lateral Accel (Mean):    {stats['lateral_accel_mean']:.4f} m/s²")
            print(f"  Lateral Accel (Max):     {stats['lateral_accel_max']:.4f} m/s²")
            print(f"  Lateral Accel (Std Dev): {stats['lateral_accel_std']:.4f} m/s²")

        if 'lateral_jerk_mean' in stats:
            print(f"  Lateral Jerk (Mean):     {stats['lateral_jerk_mean']:.4f} m/s³")
            print(f"  Lateral Jerk (Max):      {stats['lateral_jerk_max']:.4f} m/s³")

        print("\n🚶 SAFETY METRICS")
        print("-" * 60)
        if 'avg_pedestrians_detected' in stats:
            print(f"  Avg Pedestrians Detected: {stats['avg_pedestrians_detected']:.2f}")
            print(f"  Max Pedestrians Detected: {stats['max_pedestrians_detected']:.0f}")

        if 'min_pedestrian_distance' in stats:
            print(f"  Min Pedestrian Distance:  {stats['min_pedestrian_distance']:.2f} m")
            print(f"  Avg Pedestrian Distance:  {stats['avg_pedestrian_distance']:.2f} m")

        print("\n⚡ EFFICIENCY METRICS")
        print("-" * 60)
        if 'avg_velocity' in stats:
            print(f"  Average Velocity:        {stats['avg_velocity']:.2f} m/s ({stats['avg_velocity']*3.6:.2f} km/h)")
            print(f"  Maximum Velocity:        {stats['max_velocity']:.2f} m/s ({stats['max_velocity']*3.6:.2f} km/h)")

        print("\n" + "="*60)

        return stats

    def plot_results(self, output_dir=None):
        """Generate plots for thesis"""
        if not self.lateral_errors:
            print("No data to plot")
            return

        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f'Controller Performance Analysis: {self.controller_name.upper()}',
                     fontsize=16, fontweight='bold')

        # Convert timestamps to relative time
        if self.timestamps:
            t = np.array(self.timestamps) - self.timestamps[0]
        else:
            t = np.arange(len(self.lateral_errors)) * 0.03  # Assume 30 Hz

        # Plot 1: Lateral Error
        axes[0, 0].plot(t[:len(self.lateral_errors)], self.lateral_errors, 'b-', linewidth=0.5)
        axes[0, 0].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Lateral Error (m)')
        axes[0, 0].set_title('Lateral Tracking Error')
        axes[0, 0].grid(True, alpha=0.3)

        # Plot 2: Heading Error
        if self.heading_errors:
            axes[0, 1].plot(t[:len(self.heading_errors)], np.rad2deg(self.heading_errors),
                           'r-', linewidth=0.5)
            axes[0, 1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('Heading Error (°)')
            axes[0, 1].set_title('Heading Tracking Error')
            axes[0, 1].grid(True, alpha=0.3)

        # Plot 3: Lateral Acceleration
        if self.lateral_accels:
            t_accel = np.arange(len(self.lateral_accels)) * 0.03
            axes[1, 0].plot(t_accel, self.lateral_accels, 'g-', linewidth=0.5)
            axes[1, 0].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
            axes[1, 0].axhline(y=1.2, color='r', linestyle=':', linewidth=1, label='Campus limit')
            axes[1, 0].axhline(y=-1.2, color='r', linestyle=':', linewidth=1)
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Lateral Acceleration (m/s²)')
            axes[1, 0].set_title('Lateral Acceleration (Comfort Metric)')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)

        # Plot 4: Velocity
        if self.velocities:
            t_vel = np.arange(len(self.velocities)) * 0.03
            axes[1, 1].plot(t_vel, np.array(self.velocities) * 3.6, 'm-', linewidth=1)
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Velocity (km/h)')
            axes[1, 1].set_title('Vehicle Velocity')
            axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()

        # Save or show
        if output_dir:
            output_path = Path(output_dir) / f'{self.controller_name}_analysis.png'
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"\nPlot saved to: {output_path}")
        else:
            plt.show()

    def export_to_csv(self, output_dir):
        """Export data to CSV for further analysis"""
        if not PANDAS_AVAILABLE:
            print("pandas not available, cannot export to CSV")
            return

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Create DataFrame
        max_len = max(len(self.lateral_errors), len(self.velocities), len(self.steering_angles))

        data = {
            'timestamp': self.timestamps[:max_len] if self.timestamps else list(range(max_len)),
            'lateral_error': self.lateral_errors + [None] * (max_len - len(self.lateral_errors)),
            'heading_error': self.heading_errors + [None] * (max_len - len(self.heading_errors)),
            'velocity': self.velocities + [None] * (max_len - len(self.velocities)),
            'steering_angle': self.steering_angles + [None] * (max_len - len(self.steering_angles)),
        }

        df = pd.DataFrame(data)
        csv_path = output_path / f'{self.controller_name}_data.csv'
        df.to_csv(csv_path, index=False)
        print(f"Data exported to: {csv_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze rosbag data for campus AV controller comparison'
    )
    parser.add_argument('bag_path', type=str, help='Path to rosbag directory')
    parser.add_argument('--controller', type=str, default='unknown',
                       help='Controller name (pure_pursuit or mpc)')
    parser.add_argument('--plot', action='store_true',
                       help='Generate plots')
    parser.add_argument('--save-plots', type=str,
                       help='Directory to save plots')
    parser.add_argument('--export-csv', type=str,
                       help='Directory to export CSV data')

    args = parser.parse_args()

    # Validate bag path
    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"ERROR: Bag path does not exist: {bag_path}")
        sys.exit(1)

    # Create analyzer
    analyzer = RosbagAnalyzer(bag_path, args.controller)

    # Extract data
    print(f"\nAnalyzing {args.controller} controller performance...")
    analyzer.extract_data()

    # Compute and print statistics
    stats = analyzer.print_statistics()

    # Plot if requested
    if args.plot or args.save_plots:
        analyzer.plot_results(output_dir=args.save_plots)

    # Export CSV if requested
    if args.export_csv:
        analyzer.export_to_csv(args.export_csv)

    print("\n✓ Analysis complete!")


if __name__ == '__main__':
    main()
