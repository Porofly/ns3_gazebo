#!/usr/bin/env python3
"""
NS3-Gazebo Experiment Data Analysis Tool

This script provides generic data analysis functionality for NS3-Gazebo experiments.
It can load CSV logs, perform basic statistical analysis, and generate plots.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from pathlib import Path


class NS3GazeboDataAnalyzer:
    """Analyzer for NS3-Gazebo experiment data"""

    def __init__(self, csv_path):
        """
        Initialize analyzer with CSV data file

        Args:
            csv_path: Path to CSV log file
        """
        self.csv_path = csv_path
        self.data = None
        self.load_data()

    def load_data(self):
        """Load CSV data into pandas DataFrame"""
        try:
            self.data = pd.read_csv(self.csv_path)
            original_count = len(self.data)

            # Remove initial unstable period (first 3 seconds or until valid data)
            # Filter out rows with RSSI = -100 (no signal data yet)
            self.data = self.data[self.data['rssi_dbm'] > -100].copy()

            # Also filter out anomalous packet loss values
            self.data = self.data[self.data['packet_loss_rate'] <= 100].copy()

            removed = original_count - len(self.data)
            print(f"Loaded {len(self.data)} data points from {self.csv_path}")
            print(f"Removed {removed} unstable/invalid samples ({removed/original_count*100:.1f}%)")
            print(f"Columns: {list(self.data.columns)}")
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            sys.exit(1)

    def get_statistics(self):
        """Print basic statistics of the data"""
        print("\n=== Data Statistics ===")
        print(self.data.describe())

    def plot_distance_vs_rssi(self, output_path=None):
        """
        Plot distance vs RSSI

        Args:
            output_path: Path to save the plot (optional)
        """
        plt.figure(figsize=(10, 6))
        plt.scatter(self.data['distance_to_base'], self.data['rssi_dbm'],
                   alpha=0.6, s=20)
        plt.xlabel('Distance to Base Station (m)', fontsize=12)
        plt.ylabel('RSSI (dBm)', fontsize=12)
        plt.title('RSSI vs Distance', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)

        # Add trend line
        z = np.polyfit(self.data['distance_to_base'], self.data['rssi_dbm'], 2)
        p = np.poly1d(z)
        x_trend = np.linspace(self.data['distance_to_base'].min(),
                              self.data['distance_to_base'].max(), 100)
        plt.plot(x_trend, p(x_trend), "r--", alpha=0.8, linewidth=2,
                label='Trend (2nd order)')
        plt.legend()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved plot to {output_path}")
        else:
            plt.show()
        plt.close()

    def plot_distance_vs_loss(self, output_path=None):
        """
        Plot distance vs packet loss rate

        Args:
            output_path: Path to save the plot (optional)
        """
        plt.figure(figsize=(10, 6))
        plt.scatter(self.data['distance_to_base'], self.data['packet_loss_rate'],
                   alpha=0.6, s=20, color='red')
        plt.xlabel('Distance to Base Station (m)', fontsize=12)
        plt.ylabel('Packet Loss Rate (%)', fontsize=12)
        plt.title('Packet Loss Rate vs Distance', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.ylim(-5, max(100, self.data['packet_loss_rate'].max() + 5))

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved plot to {output_path}")
        else:
            plt.show()
        plt.close()

    def plot_time_series(self, output_path=None):
        """
        Plot time series of key metrics

        Args:
            output_path: Path to save the plot (optional)
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

        # Distance over time
        axes[0].plot(self.data['timestamp_sec'], self.data['distance_to_base'],
                    linewidth=2)
        axes[0].set_ylabel('Distance (m)', fontsize=11)
        axes[0].set_title('Network Metrics Over Time', fontsize=14, fontweight='bold')
        axes[0].grid(True, alpha=0.3)

        # RSSI over time
        axes[1].plot(self.data['timestamp_sec'], self.data['rssi_dbm'],
                    linewidth=2, color='orange')
        axes[1].set_ylabel('RSSI (dBm)', fontsize=11)
        axes[1].grid(True, alpha=0.3)

        # Packet loss over time
        axes[2].plot(self.data['timestamp_sec'], self.data['packet_loss_rate'],
                    linewidth=2, color='red')
        axes[2].set_xlabel('Time (seconds)', fontsize=11)
        axes[2].set_ylabel('Packet Loss (%)', fontsize=11)
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved plot to {output_path}")
        else:
            plt.show()
        plt.close()

    def plot_rssi_snr_correlation(self, output_path=None):
        """
        Plot RSSI vs SNR correlation

        Args:
            output_path: Path to save the plot (optional)
        """
        plt.figure(figsize=(10, 6))
        plt.scatter(self.data['rssi_dbm'], self.data['snr_db'],
                   alpha=0.6, s=20, c=self.data['distance_to_base'],
                   cmap='viridis')
        plt.colorbar(label='Distance (m)')
        plt.xlabel('RSSI (dBm)', fontsize=12)
        plt.ylabel('SNR (dB)', fontsize=12)
        plt.title('RSSI vs SNR (colored by distance)', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved plot to {output_path}")
        else:
            plt.show()
        plt.close()

    def generate_summary_report(self, output_path=None):
        """
        Generate a text summary report

        Args:
            output_path: Path to save the report (optional)
        """
        report = []
        report.append("=" * 60)
        report.append("NS3-Gazebo Experiment Summary Report")
        report.append("=" * 60)
        report.append(f"\nData file: {self.csv_path}")
        report.append(f"Total data points: {len(self.data)}")
        report.append(f"Duration: {self.data['timestamp_sec'].max():.1f} seconds")

        report.append("\n--- Distance Statistics ---")
        report.append(f"Min distance: {self.data['distance_to_base'].min():.2f} m")
        report.append(f"Max distance: {self.data['distance_to_base'].max():.2f} m")
        report.append(f"Avg distance: {self.data['distance_to_base'].mean():.2f} m")

        report.append("\n--- RSSI Statistics ---")
        report.append(f"Min RSSI: {self.data['rssi_dbm'].min():.2f} dBm")
        report.append(f"Max RSSI: {self.data['rssi_dbm'].max():.2f} dBm")
        report.append(f"Avg RSSI: {self.data['rssi_dbm'].mean():.2f} dBm")

        report.append("\n--- SNR Statistics ---")
        report.append(f"Min SNR: {self.data['snr_db'].min():.2f} dB")
        report.append(f"Max SNR: {self.data['snr_db'].max():.2f} dB")
        report.append(f"Avg SNR: {self.data['snr_db'].mean():.2f} dB")

        report.append("\n--- Packet Statistics ---")
        final_sent = self.data['packets_sent'].iloc[-1]
        final_received = self.data['packets_received'].iloc[-1]
        final_loss = self.data['packet_loss_rate'].iloc[-1]
        report.append(f"Total packets sent: {final_sent}")
        report.append(f"Total packets received: {final_received}")
        report.append(f"Overall loss rate: {final_loss:.2f}%")

        report.append("\n" + "=" * 60)

        report_text = "\n".join(report)
        print(report_text)

        if output_path:
            with open(output_path, 'w') as f:
                f.write(report_text)
            print(f"\nSaved report to {output_path}")


def main():
    """Main function"""
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_data.py <csv_file> [output_dir]")
        print("\nExample:")
        print("  python3 analyze_data.py ns3_gazebo_log.csv")
        print("  python3 analyze_data.py ns3_gazebo_log.csv results/")
        sys.exit(1)

    csv_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.exists(csv_file):
        print(f"Error: CSV file not found: {csv_file}")
        sys.exit(1)

    # Create output directory if specified
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        print(f"Output directory: {output_dir}")

    # Initialize analyzer
    analyzer = NS3GazeboDataAnalyzer(csv_file)

    # Generate statistics
    analyzer.get_statistics()

    # Generate plots
    base_name = Path(csv_file).stem

    if output_dir:
        analyzer.plot_distance_vs_rssi(
            os.path.join(output_dir, f"{base_name}_distance_vs_rssi.png"))
        analyzer.plot_distance_vs_loss(
            os.path.join(output_dir, f"{base_name}_distance_vs_loss.png"))
        analyzer.plot_time_series(
            os.path.join(output_dir, f"{base_name}_time_series.png"))
        analyzer.plot_rssi_snr_correlation(
            os.path.join(output_dir, f"{base_name}_rssi_snr.png"))
        analyzer.generate_summary_report(
            os.path.join(output_dir, f"{base_name}_report.txt"))
    else:
        print("\nGenerating plots...")
        analyzer.plot_distance_vs_rssi()
        analyzer.plot_distance_vs_loss()
        analyzer.plot_time_series()
        analyzer.plot_rssi_snr_correlation()
        analyzer.generate_summary_report()


if __name__ == "__main__":
    main()
