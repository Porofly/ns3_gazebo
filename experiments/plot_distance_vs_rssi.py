#!/usr/bin/env python3
"""
Quick plot: Distance vs RSSI for Experiment 1
"""

import sys
import os
from analyze_data import NS3GazeboDataAnalyzer


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_distance_vs_rssi.py <csv_file> [output_png]")
        sys.exit(1)

    csv_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    analyzer = NS3GazeboDataAnalyzer(csv_file)
    analyzer.plot_distance_vs_rssi(output_file)
    analyzer.plot_distance_vs_loss(
        output_file.replace('.png', '_loss.png') if output_file else None)


if __name__ == "__main__":
    main()
