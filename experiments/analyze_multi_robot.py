#!/usr/bin/env python3
"""
Multi-Robot NS3-Gazebo Network Analysis Tool

This script analyzes network performance for multi-robot systems where robots
are progressively added to the network. It provides comprehensive visualization
of communication quality metrics across multiple robots over time.

Features:
- Per-robot communication quality analysis
- Temporal analysis showing network degradation as robots join
- Network-wide aggregate metrics
- Spatial heatmaps of signal quality
- Comparative analysis across robots
- Scalability metrics
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import sys
import os
from pathlib import Path
from typing import Dict, List, Tuple
import warnings
warnings.filterwarnings('ignore')

# Set publication-quality plot style
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")


class MultiRobotNetworkAnalyzer:
    """Comprehensive analyzer for multi-robot network experiments"""

    def __init__(self, csv_path: str, base_position: Tuple[float, float] = (0.0, 0.0)):
        """
        Initialize analyzer with multi-robot CSV data

        Args:
            csv_path: Path to CSV log file with columns:
                     timestamp_sec, model_name, ns3_node_id, robot_x, robot_y, robot_z,
                     distance_to_base, rssi_dbm, snr_db, packets_sent, packets_received,
                     packets_lost, packet_loss_rate
            base_position: Base station position (x, y)
        """
        self.csv_path = csv_path
        self.base_position = base_position
        self.data = None
        self.robots = []
        self.robot_join_times = {}
        self.load_and_preprocess_data()

    def load_and_preprocess_data(self):
        """Load and preprocess multi-robot CSV data"""
        try:
            self.data = pd.read_csv(self.csv_path)
            original_count = len(self.data)

            # Validate required columns
            required_cols = ['timestamp_sec', 'model_name', 'ns3_node_id', 'robot_x',
                           'robot_y', 'rssi_dbm', 'snr_db', 'packets_sent',
                           'packets_received', 'packet_loss_rate']
            missing = [col for col in required_cols if col not in self.data.columns]
            if missing:
                raise ValueError(f"Missing required columns: {missing}")

            # Remove invalid data (RSSI = -100 indicates no signal yet)
            self.data = self.data[self.data['rssi_dbm'] > -100].copy()
            self.data = self.data[self.data['packet_loss_rate'] <= 100].copy()

            # Get unique robots
            self.robots = sorted(self.data['model_name'].unique())

            # Determine when each robot joined the network
            for robot in self.robots:
                robot_data = self.data[self.data['model_name'] == robot]
                self.robot_join_times[robot] = robot_data['timestamp_sec'].min()

            removed = original_count - len(self.data)
            print(f"\n{'='*70}")
            print(f"Multi-Robot Network Analysis")
            print(f"{'='*70}")
            print(f"Data file: {self.csv_path}")
            print(f"Total data points: {len(self.data):,}")
            print(f"Removed invalid samples: {removed:,} ({removed/original_count*100:.1f}%)")
            print(f"Number of robots: {len(self.robots)}")
            print(f"Robots: {', '.join(self.robots)}")
            print(f"Duration: {self.data['timestamp_sec'].max():.1f} seconds")
            print(f"{'='*70}\n")

        except Exception as e:
            print(f"Error loading CSV file: {e}")
            sys.exit(1)

    def get_robot_statistics(self, robot_name: str = None) -> pd.DataFrame:
        """
        Get statistics for a specific robot or all robots

        Args:
            robot_name: Name of robot (None for all robots)

        Returns:
            DataFrame with statistics
        """
        if robot_name:
            data = self.data[self.data['model_name'] == robot_name]
            robots = [robot_name]
        else:
            data = self.data
            robots = self.robots

        stats = []
        for robot in robots:
            robot_data = self.data[self.data['model_name'] == robot]

            # Get final packet counts
            final_row = robot_data.iloc[-1]

            stats.append({
                'Robot': robot,
                'Node_ID': int(robot_data['ns3_node_id'].iloc[0]),
                'Join_Time': self.robot_join_times[robot],
                'Data_Points': len(robot_data),
                'Avg_Distance': robot_data['distance_to_base'].mean(),
                'Avg_RSSI': robot_data['rssi_dbm'].mean(),
                'Min_RSSI': robot_data['rssi_dbm'].min(),
                'Max_RSSI': robot_data['rssi_dbm'].max(),
                'Std_RSSI': robot_data['rssi_dbm'].std(),
                'Avg_SNR': robot_data['snr_db'].mean(),
                'Min_SNR': robot_data['snr_db'].min(),
                'Packets_Sent': int(final_row['packets_sent']),
                'Packets_Received': int(final_row['packets_received']),
                'Packets_Lost': int(final_row['packets_lost']),
                'Loss_Rate': final_row['packet_loss_rate']
            })

        return pd.DataFrame(stats)

    def plot_per_robot_quality(self, output_path: str = None):
        """
        Plot 1: Individual robot communication quality over time
        Shows RSSI, SNR, and packet loss for each robot
        """
        n_robots = len(self.robots)
        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

        colors = plt.cm.tab10(np.linspace(0, 1, n_robots))

        # Plot RSSI for each robot
        for idx, robot in enumerate(self.robots):
            robot_data = self.data[self.data['model_name'] == robot]
            axes[0].plot(robot_data['timestamp_sec'], robot_data['rssi_dbm'],
                        label=robot, linewidth=1.5, alpha=0.8, color=colors[idx])

        axes[0].set_ylabel('RSSI (dBm)', fontsize=12, fontweight='bold')
        axes[0].set_title('Per-Robot Communication Quality Over Time',
                         fontsize=14, fontweight='bold', pad=20)
        axes[0].legend(loc='upper right', ncol=min(4, n_robots), framealpha=0.9)
        axes[0].grid(True, alpha=0.3)
        axes[0].axhline(y=-65, color='r', linestyle='--', alpha=0.5, label='Poor signal')

        # Plot SNR for each robot
        for idx, robot in enumerate(self.robots):
            robot_data = self.data[self.data['model_name'] == robot]
            axes[1].plot(robot_data['timestamp_sec'], robot_data['snr_db'],
                        label=robot, linewidth=1.5, alpha=0.8, color=colors[idx])

        axes[1].set_ylabel('SNR (dB)', fontsize=12, fontweight='bold')
        axes[1].legend(loc='upper right', ncol=min(4, n_robots), framealpha=0.9)
        axes[1].grid(True, alpha=0.3)

        # Plot packet loss rate for each robot
        for idx, robot in enumerate(self.robots):
            robot_data = self.data[self.data['model_name'] == robot]
            axes[2].plot(robot_data['timestamp_sec'], robot_data['packet_loss_rate'],
                        label=robot, linewidth=1.5, alpha=0.8, color=colors[idx])

        axes[2].set_ylabel('Packet Loss Rate (%)', fontsize=12, fontweight='bold')
        axes[2].set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
        axes[2].legend(loc='upper right', ncol=min(4, n_robots), framealpha=0.9)
        axes[2].grid(True, alpha=0.3)

        # Add vertical lines for robot join times
        for ax in axes:
            for robot, join_time in self.robot_join_times.items():
                ax.axvline(x=join_time, color='gray', linestyle=':', alpha=0.4)

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {output_path}")
        else:
            plt.show()
        plt.close()

    def plot_network_degradation(self, output_path: str = None):
        """
        Plot 2: Network degradation as robots join
        Shows how average network quality changes with robot count
        """
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Calculate metrics vs active robot count
        time_points = np.sort(self.data['timestamp_sec'].unique())
        robot_counts = []
        avg_rssi = []
        avg_snr = []
        avg_loss = []
        throughput = []

        for t in time_points[::10]:  # Sample every 10 time points for performance
            # Count active robots at this time
            active_robots = sum(1 for join_time in self.robot_join_times.values()
                              if join_time <= t)

            # Get data at this time
            time_data = self.data[
                (self.data['timestamp_sec'] >= t) &
                (self.data['timestamp_sec'] < t + 0.1)
            ]

            if len(time_data) > 0:
                robot_counts.append(active_robots)
                avg_rssi.append(time_data['rssi_dbm'].mean())
                avg_snr.append(time_data['snr_db'].mean())
                avg_loss.append(time_data['packet_loss_rate'].mean())

                # Estimate throughput (packets received per second per robot)
                if active_robots > 0:
                    total_rx = time_data.groupby('model_name')['packets_received'].max().sum()
                    throughput.append(total_rx / active_robots if t > 0 else 0)
                else:
                    throughput.append(0)

        # Plot 1: RSSI vs Robot Count
        axes[0, 0].scatter(robot_counts, avg_rssi, alpha=0.5, s=20)
        axes[0, 0].set_xlabel('Active Robot Count', fontsize=11, fontweight='bold')
        axes[0, 0].set_ylabel('Average RSSI (dBm)', fontsize=11, fontweight='bold')
        axes[0, 0].set_title('Signal Strength Degradation', fontsize=12, fontweight='bold')
        axes[0, 0].grid(True, alpha=0.3)

        # Add trend line
        if len(robot_counts) > 1:
            z = np.polyfit(robot_counts, avg_rssi, 2)
            p = np.poly1d(z)
            rc_range = np.linspace(min(robot_counts), max(robot_counts), 50)
            axes[0, 0].plot(rc_range, p(rc_range), 'r--', alpha=0.8, linewidth=2)

        # Plot 2: SNR vs Robot Count
        axes[0, 1].scatter(robot_counts, avg_snr, alpha=0.5, s=20, color='orange')
        axes[0, 1].set_xlabel('Active Robot Count', fontsize=11, fontweight='bold')
        axes[0, 1].set_ylabel('Average SNR (dB)', fontsize=11, fontweight='bold')
        axes[0, 1].set_title('SNR Degradation', fontsize=12, fontweight='bold')
        axes[0, 1].grid(True, alpha=0.3)

        if len(robot_counts) > 1:
            z = np.polyfit(robot_counts, avg_snr, 2)
            p = np.poly1d(z)
            axes[0, 1].plot(rc_range, p(rc_range), 'r--', alpha=0.8, linewidth=2)

        # Plot 3: Packet Loss vs Robot Count
        axes[1, 0].scatter(robot_counts, avg_loss, alpha=0.5, s=20, color='red')
        axes[1, 0].set_xlabel('Active Robot Count', fontsize=11, fontweight='bold')
        axes[1, 0].set_ylabel('Average Packet Loss (%)', fontsize=11, fontweight='bold')
        axes[1, 0].set_title('Network Congestion', fontsize=12, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3)

        if len(robot_counts) > 1:
            z = np.polyfit(robot_counts, avg_loss, 2)
            p = np.poly1d(z)
            axes[1, 0].plot(rc_range, p(rc_range), 'r--', alpha=0.8, linewidth=2)

        # Plot 4: Throughput vs Robot Count
        axes[1, 1].scatter(robot_counts, throughput, alpha=0.5, s=20, color='green')
        axes[1, 1].set_xlabel('Active Robot Count', fontsize=11, fontweight='bold')
        axes[1, 1].set_ylabel('Avg Throughput\n(packets/robot)', fontsize=11, fontweight='bold')
        axes[1, 1].set_title('Per-Robot Throughput', fontsize=12, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3)

        fig.suptitle('Network Performance Degradation as Robots Join',
                    fontsize=14, fontweight='bold', y=0.995)
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {output_path}")
        else:
            plt.show()
        plt.close()

    def plot_spatial_heatmap(self, output_path: str = None):
        """
        Plot 3: Spatial heatmap of signal quality
        Shows RSSI distribution across physical space
        """
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        # Prepare data for heatmap
        x = self.data['robot_x'].values
        y = self.data['robot_y'].values
        rssi = self.data['rssi_dbm'].values
        snr = self.data['snr_db'].values

        # Create grid for interpolation
        grid_resolution = 50
        xi = np.linspace(x.min(), x.max(), grid_resolution)
        yi = np.linspace(y.min(), y.max(), grid_resolution)
        xi_grid, yi_grid = np.meshgrid(xi, yi)

        # Interpolate RSSI
        from scipy.interpolate import griddata
        rssi_grid = griddata((x, y), rssi, (xi_grid, yi_grid), method='cubic')

        # Plot RSSI heatmap
        im1 = axes[0].contourf(xi_grid, yi_grid, rssi_grid, levels=20, cmap='RdYlGn')
        axes[0].scatter([self.base_position[0]], [self.base_position[1]],
                       c='blue', s=200, marker='*', edgecolors='black', linewidths=2,
                       label='Base Station', zorder=10)
        axes[0].set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
        axes[0].set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
        axes[0].set_title('RSSI Spatial Distribution', fontsize=12, fontweight='bold')
        axes[0].legend(loc='upper right')
        axes[0].set_aspect('equal')
        plt.colorbar(im1, ax=axes[0], label='RSSI (dBm)')

        # Interpolate SNR
        snr_grid = griddata((x, y), snr, (xi_grid, yi_grid), method='cubic')

        # Plot SNR heatmap
        im2 = axes[1].contourf(xi_grid, yi_grid, snr_grid, levels=20, cmap='RdYlGn')
        axes[1].scatter([self.base_position[0]], [self.base_position[1]],
                       c='blue', s=200, marker='*', edgecolors='black', linewidths=2,
                       label='Base Station', zorder=10)
        axes[1].set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
        axes[1].set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
        axes[1].set_title('SNR Spatial Distribution', fontsize=12, fontweight='bold')
        axes[1].legend(loc='upper right')
        axes[1].set_aspect('equal')
        plt.colorbar(im2, ax=axes[1], label='SNR (dB)')

        fig.suptitle('Spatial Signal Quality Heatmap',
                    fontsize=14, fontweight='bold', y=0.98)
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {output_path}")
        else:
            plt.show()
        plt.close()

    def plot_robot_comparison_boxplot(self, output_path: str = None):
        """
        Plot 4: Box plot comparison of metrics across robots
        Shows distribution and outliers for each robot
        """
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # RSSI comparison
        rssi_data = [self.data[self.data['model_name'] == robot]['rssi_dbm'].values
                     for robot in self.robots]
        bp1 = axes[0, 0].boxplot(rssi_data, labels=self.robots, patch_artist=True)
        axes[0, 0].set_ylabel('RSSI (dBm)', fontsize=11, fontweight='bold')
        axes[0, 0].set_title('RSSI Distribution per Robot', fontsize=12, fontweight='bold')
        axes[0, 0].grid(True, alpha=0.3, axis='y')
        axes[0, 0].tick_params(axis='x', rotation=45)

        # Color boxes
        for patch, color in zip(bp1['boxes'], plt.cm.tab10(np.linspace(0, 1, len(self.robots)))):
            patch.set_facecolor(color)

        # SNR comparison
        snr_data = [self.data[self.data['model_name'] == robot]['snr_db'].values
                    for robot in self.robots]
        bp2 = axes[0, 1].boxplot(snr_data, labels=self.robots, patch_artist=True)
        axes[0, 1].set_ylabel('SNR (dB)', fontsize=11, fontweight='bold')
        axes[0, 1].set_title('SNR Distribution per Robot', fontsize=12, fontweight='bold')
        axes[0, 1].grid(True, alpha=0.3, axis='y')
        axes[0, 1].tick_params(axis='x', rotation=45)

        for patch, color in zip(bp2['boxes'], plt.cm.tab10(np.linspace(0, 1, len(self.robots)))):
            patch.set_facecolor(color)

        # Distance comparison
        dist_data = [self.data[self.data['model_name'] == robot]['distance_to_base'].values
                     for robot in self.robots]
        bp3 = axes[1, 0].boxplot(dist_data, labels=self.robots, patch_artist=True)
        axes[1, 0].set_ylabel('Distance to Base (m)', fontsize=11, fontweight='bold')
        axes[1, 0].set_title('Distance Distribution per Robot', fontsize=12, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3, axis='y')
        axes[1, 0].tick_params(axis='x', rotation=45)

        for patch, color in zip(bp3['boxes'], plt.cm.tab10(np.linspace(0, 1, len(self.robots)))):
            patch.set_facecolor(color)

        # Packet loss comparison
        loss_data = [self.data[self.data['model_name'] == robot]['packet_loss_rate'].values
                     for robot in self.robots]
        bp4 = axes[1, 1].boxplot(loss_data, labels=self.robots, patch_artist=True)
        axes[1, 1].set_ylabel('Packet Loss Rate (%)', fontsize=11, fontweight='bold')
        axes[1, 1].set_title('Packet Loss Distribution per Robot', fontsize=12, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3, axis='y')
        axes[1, 1].tick_params(axis='x', rotation=45)

        for patch, color in zip(bp4['boxes'], plt.cm.tab10(np.linspace(0, 1, len(self.robots)))):
            patch.set_facecolor(color)

        fig.suptitle('Robot-by-Robot Performance Comparison',
                    fontsize=14, fontweight='bold', y=0.995)
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {output_path}")
        else:
            plt.show()
        plt.close()

    def plot_network_fairness(self, output_path: str = None):
        """
        Plot 5: Network fairness analysis
        Shows how fairly network resources are distributed among robots
        """
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Calculate per-robot cumulative metrics
        robot_metrics = []
        for robot in self.robots:
            robot_data = self.data[self.data['model_name'] == robot]
            final = robot_data.iloc[-1]
            robot_metrics.append({
                'robot': robot,
                'packets_received': final['packets_received'],
                'avg_rssi': robot_data['rssi_dbm'].mean(),
                'avg_snr': robot_data['snr_db'].mean(),
                'join_time': self.robot_join_times[robot]
            })

        metrics_df = pd.DataFrame(robot_metrics)

        # Plot 1: Packets received per robot
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.robots)))
        axes[0, 0].bar(metrics_df['robot'], metrics_df['packets_received'],
                      color=colors, alpha=0.8)
        axes[0, 0].set_ylabel('Packets Received', fontsize=11, fontweight='bold')
        axes[0, 0].set_title('Total Packets Received per Robot', fontsize=12, fontweight='bold')
        axes[0, 0].tick_params(axis='x', rotation=45)
        axes[0, 0].grid(True, alpha=0.3, axis='y')

        # Plot 2: Average RSSI per robot
        axes[0, 1].bar(metrics_df['robot'], metrics_df['avg_rssi'],
                      color=colors, alpha=0.8)
        axes[0, 1].set_ylabel('Average RSSI (dBm)', fontsize=11, fontweight='bold')
        axes[0, 1].set_title('Average RSSI per Robot', fontsize=12, fontweight='bold')
        axes[0, 1].tick_params(axis='x', rotation=45)
        axes[0, 1].grid(True, alpha=0.3, axis='y')

        # Plot 3: Jain's Fairness Index over time
        time_points = np.sort(self.data['timestamp_sec'].unique())[::50]
        fairness_indices = []

        for t in time_points:
            time_data = self.data[self.data['timestamp_sec'] <= t]
            robot_rx = []
            for robot in self.robots:
                if self.robot_join_times[robot] <= t:
                    robot_time_data = time_data[time_data['model_name'] == robot]
                    if len(robot_time_data) > 0:
                        robot_rx.append(robot_time_data['packets_received'].iloc[-1])

            if len(robot_rx) > 1:
                # Jain's Fairness Index: (sum xi)^2 / (n * sum xi^2)
                robot_rx = np.array(robot_rx)
                fairness = (np.sum(robot_rx) ** 2) / (len(robot_rx) * np.sum(robot_rx ** 2))
                fairness_indices.append(fairness)
            else:
                fairness_indices.append(1.0)

        axes[1, 0].plot(time_points, fairness_indices, linewidth=2, color='purple')
        axes[1, 0].set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
        axes[1, 0].set_ylabel("Jain's Fairness Index", fontsize=11, fontweight='bold')
        axes[1, 0].set_title('Network Fairness Over Time', fontsize=12, fontweight='bold')
        axes[1, 0].set_ylim([0, 1.05])
        axes[1, 0].axhline(y=1.0, color='g', linestyle='--', alpha=0.5, label='Perfect fairness')
        axes[1, 0].axhline(y=0.8, color='orange', linestyle='--', alpha=0.5, label='Good fairness')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # Plot 4: Cumulative packets over time for each robot
        for idx, robot in enumerate(self.robots):
            robot_data = self.data[self.data['model_name'] == robot]
            axes[1, 1].plot(robot_data['timestamp_sec'],
                          robot_data['packets_received'],
                          label=robot, linewidth=2, color=colors[idx])

        axes[1, 1].set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
        axes[1, 1].set_ylabel('Cumulative Packets Received', fontsize=11, fontweight='bold')
        axes[1, 1].set_title('Packet Reception Timeline', fontsize=12, fontweight='bold')
        axes[1, 1].legend(loc='upper left')
        axes[1, 1].grid(True, alpha=0.3)

        fig.suptitle('Network Fairness Analysis',
                    fontsize=14, fontweight='bold', y=0.995)
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {output_path}")
        else:
            plt.show()
        plt.close()

    def plot_scalability_analysis(self, output_path: str = None):
        """
        Plot 6: Network scalability analysis
        Shows how network performance scales with robot count
        """
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Analyze network state at different robot counts
        scalability_data = []

        for n_robots in range(1, len(self.robots) + 1):
            # Get time window when exactly n_robots were active
            active_robots = sorted(self.robots, key=lambda r: self.robot_join_times[r])[:n_robots]

            if n_robots < len(self.robots):
                start_time = self.robot_join_times[active_robots[-1]]
                next_robot = sorted(self.robots, key=lambda r: self.robot_join_times[r])[n_robots]
                end_time = self.robot_join_times[next_robot]
            else:
                start_time = self.robot_join_times[active_robots[-1]]
                end_time = self.data['timestamp_sec'].max()

            # Get data in this window
            window_data = self.data[
                (self.data['timestamp_sec'] >= start_time) &
                (self.data['timestamp_sec'] < end_time) &
                (self.data['model_name'].isin(active_robots))
            ]

            if len(window_data) > 0:
                scalability_data.append({
                    'n_robots': n_robots,
                    'avg_rssi': window_data['rssi_dbm'].mean(),
                    'std_rssi': window_data['rssi_dbm'].std(),
                    'avg_snr': window_data['snr_db'].mean(),
                    'avg_loss': window_data['packet_loss_rate'].mean(),
                    'total_throughput': window_data.groupby('model_name')['packets_received'].max().sum() / (end_time - start_time)
                })

        scale_df = pd.DataFrame(scalability_data)

        # Plot 1: RSSI degradation with confidence interval
        axes[0, 0].errorbar(scale_df['n_robots'], scale_df['avg_rssi'],
                          yerr=scale_df['std_rssi'], marker='o', linewidth=2,
                          capsize=5, capthick=2)
        axes[0, 0].set_xlabel('Number of Active Robots', fontsize=11, fontweight='bold')
        axes[0, 0].set_ylabel('Average RSSI (dBm)', fontsize=11, fontweight='bold')
        axes[0, 0].set_title('RSSI vs Network Size', fontsize=12, fontweight='bold')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].set_xticks(range(1, len(self.robots) + 1))

        # Plot 2: SNR degradation
        axes[0, 1].plot(scale_df['n_robots'], scale_df['avg_snr'],
                       marker='s', linewidth=2, markersize=8, color='orange')
        axes[0, 1].set_xlabel('Number of Active Robots', fontsize=11, fontweight='bold')
        axes[0, 1].set_ylabel('Average SNR (dB)', fontsize=11, fontweight='bold')
        axes[0, 1].set_title('SNR vs Network Size', fontsize=12, fontweight='bold')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].set_xticks(range(1, len(self.robots) + 1))

        # Plot 3: Packet loss increase
        axes[1, 0].plot(scale_df['n_robots'], scale_df['avg_loss'],
                       marker='^', linewidth=2, markersize=8, color='red')
        axes[1, 0].set_xlabel('Number of Active Robots', fontsize=11, fontweight='bold')
        axes[1, 0].set_ylabel('Average Packet Loss (%)', fontsize=11, fontweight='bold')
        axes[1, 0].set_title('Packet Loss vs Network Size', fontsize=12, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].set_xticks(range(1, len(self.robots) + 1))

        # Plot 4: Total network throughput
        axes[1, 1].plot(scale_df['n_robots'], scale_df['total_throughput'],
                       marker='D', linewidth=2, markersize=8, color='green')
        axes[1, 1].set_xlabel('Number of Active Robots', fontsize=11, fontweight='bold')
        axes[1, 1].set_ylabel('Total Network Throughput\n(packets/sec)', fontsize=11, fontweight='bold')
        axes[1, 1].set_title('Aggregate Throughput vs Network Size', fontsize=12, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].set_xticks(range(1, len(self.robots) + 1))

        fig.suptitle('Network Scalability Analysis',
                    fontsize=14, fontweight='bold', y=0.995)
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"✓ Saved: {output_path}")
        else:
            plt.show()
        plt.close()

    def generate_comprehensive_report(self, output_path: str = None):
        """
        Generate comprehensive text report
        """
        report = []
        report.append("=" * 80)
        report.append("MULTI-ROBOT NETWORK PERFORMANCE ANALYSIS REPORT")
        report.append("=" * 80)
        report.append(f"\nData Source: {self.csv_path}")
        report.append(f"Analysis Date: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"Total Data Points: {len(self.data):,}")
        report.append(f"Experiment Duration: {self.data['timestamp_sec'].max():.1f} seconds")
        report.append(f"Number of Robots: {len(self.robots)}")

        report.append("\n" + "-" * 80)
        report.append("ROBOT JOIN SEQUENCE")
        report.append("-" * 80)
        for robot in sorted(self.robots, key=lambda r: self.robot_join_times[r]):
            report.append(f"  {robot}: joined at t={self.robot_join_times[robot]:.2f}s")

        report.append("\n" + "-" * 80)
        report.append("PER-ROBOT STATISTICS")
        report.append("-" * 80)
        stats_df = self.get_robot_statistics()
        report.append(stats_df.to_string(index=False))

        report.append("\n" + "-" * 80)
        report.append("NETWORK-WIDE SUMMARY")
        report.append("-" * 80)
        report.append(f"  Overall Average RSSI: {self.data['rssi_dbm'].mean():.2f} dBm")
        report.append(f"  Overall Average SNR: {self.data['snr_db'].mean():.2f} dB")
        report.append(f"  RSSI Range: [{self.data['rssi_dbm'].min():.2f}, {self.data['rssi_dbm'].max():.2f}] dBm")
        report.append(f"  SNR Range: [{self.data['snr_db'].min():.2f}, {self.data['snr_db'].max():.2f}] dB")

        total_sent = stats_df['Packets_Sent'].sum()
        total_received = stats_df['Packets_Received'].sum()
        total_lost = stats_df['Packets_Lost'].sum()
        overall_loss = (total_lost / total_sent * 100) if total_sent > 0 else 0

        report.append(f"\n  Total Packets Sent: {total_sent:,}")
        report.append(f"  Total Packets Received: {total_received:,}")
        report.append(f"  Total Packets Lost: {total_lost:,}")
        report.append(f"  Overall Loss Rate: {overall_loss:.2f}%")

        report.append("\n" + "-" * 80)
        report.append("FAIRNESS ANALYSIS")
        report.append("-" * 80)
        rx_values = stats_df['Packets_Received'].values
        if len(rx_values) > 1:
            fairness = (np.sum(rx_values) ** 2) / (len(rx_values) * np.sum(rx_values ** 2))
            report.append(f"  Jain's Fairness Index: {fairness:.4f}")
            report.append(f"  Interpretation: ", end="")
            if fairness > 0.95:
                report.append("Excellent - Very fair distribution")
            elif fairness > 0.85:
                report.append("Good - Fair distribution")
            elif fairness > 0.7:
                report.append("Moderate - Some unfairness present")
            else:
                report.append("Poor - Significant unfairness")

        report.append("\n" + "=" * 80)

        report_text = "\n".join(report)
        print("\n" + report_text)

        if output_path:
            with open(output_path, 'w') as f:
                f.write(report_text)
            print(f"\n✓ Saved report: {output_path}")

    def generate_all_plots(self, output_dir: str):
        """
        Generate all analysis plots and report

        Args:
            output_dir: Directory to save all outputs
        """
        os.makedirs(output_dir, exist_ok=True)
        base_name = Path(self.csv_path).stem

        print(f"\n{'='*70}")
        print(f"Generating comprehensive multi-robot network analysis...")
        print(f"Output directory: {output_dir}")
        print(f"{'='*70}\n")

        # Generate all plots
        self.plot_per_robot_quality(
            os.path.join(output_dir, f"{base_name}_per_robot_quality.png"))

        self.plot_network_degradation(
            os.path.join(output_dir, f"{base_name}_network_degradation.png"))

        self.plot_spatial_heatmap(
            os.path.join(output_dir, f"{base_name}_spatial_heatmap.png"))

        self.plot_robot_comparison_boxplot(
            os.path.join(output_dir, f"{base_name}_robot_comparison.png"))

        self.plot_network_fairness(
            os.path.join(output_dir, f"{base_name}_network_fairness.png"))

        self.plot_scalability_analysis(
            os.path.join(output_dir, f"{base_name}_scalability.png"))

        # Generate report
        self.generate_comprehensive_report(
            os.path.join(output_dir, f"{base_name}_analysis_report.txt"))

        print(f"\n{'='*70}")
        print(f"✓ Analysis complete! All outputs saved to: {output_dir}")
        print(f"{'='*70}\n")


def main():
    """Main function with CLI interface"""
    if len(sys.argv) < 2:
        print("="*70)
        print("Multi-Robot NS3-Gazebo Network Analysis Tool")
        print("="*70)
        print("\nUsage: python3 analyze_multi_robot.py <csv_file> [output_dir]")
        print("\nArguments:")
        print("  csv_file    : Path to multi-robot network log CSV file")
        print("  output_dir  : Directory to save plots and reports (optional)")
        print("\nCSV Format Required:")
        print("  timestamp_sec, model_name, ns3_node_id, robot_x, robot_y, robot_z,")
        print("  distance_to_base, rssi_dbm, snr_db, packets_sent, packets_received,")
        print("  packets_lost, packet_loss_rate")
        print("\nExamples:")
        print("  # Display plots interactively")
        print("  python3 analyze_multi_robot.py ns3_network_log.csv")
        print("\n  # Save all plots and report to directory")
        print("  python3 analyze_multi_robot.py ns3_network_log.csv results/")
        print("="*70)
        sys.exit(1)

    csv_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.exists(csv_file):
        print(f"ERROR: CSV file not found: {csv_file}")
        sys.exit(1)

    # Initialize analyzer
    analyzer = MultiRobotNetworkAnalyzer(csv_file)

    # Generate outputs
    if output_dir:
        analyzer.generate_all_plots(output_dir)
    else:
        print("\n" + "="*70)
        print("Interactive Mode - Plots will be displayed sequentially")
        print("Close each plot window to see the next plot")
        print("="*70 + "\n")

        analyzer.plot_per_robot_quality()
        analyzer.plot_network_degradation()
        analyzer.plot_spatial_heatmap()
        analyzer.plot_robot_comparison_boxplot()
        analyzer.plot_network_fairness()
        analyzer.plot_scalability_analysis()
        analyzer.generate_comprehensive_report()


if __name__ == "__main__":
    main()
