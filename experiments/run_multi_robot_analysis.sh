#!/bin/bash
# Multi-Robot Network Analysis Runner
# This script analyzes multi-robot network simulation data

set -e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Multi-Robot Network Analysis${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Default paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DEFAULT_CSV="${SCRIPT_DIR}/../ns3_gazebo_plugin/ns3_network_log.csv"
OUTPUT_DIR="${SCRIPT_DIR}/results_multi_robot"

# Parse arguments
CSV_FILE="${1:-$DEFAULT_CSV}"
OUTPUT_DIR="${2:-$OUTPUT_DIR}"

# Check if CSV file exists
if [ ! -f "$CSV_FILE" ]; then
    echo -e "${RED}ERROR: CSV file not found: $CSV_FILE${NC}"
    echo ""
    echo "Usage: $0 [csv_file] [output_dir]"
    echo ""
    echo "Examples:"
    echo "  $0"
    echo "  $0 my_experiment.csv"
    echo "  $0 my_experiment.csv custom_results/"
    exit 1
fi

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}ERROR: python3 not found${NC}"
    exit 1
fi

# Check for required Python packages
echo -e "${YELLOW}Checking Python dependencies...${NC}"
python3 -c "import pandas, numpy, matplotlib, seaborn, scipy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}Installing required Python packages...${NC}"
    pip3 install pandas numpy matplotlib seaborn scipy --user
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Run analysis
echo -e "${GREEN}Analyzing: $CSV_FILE${NC}"
echo -e "${GREEN}Output directory: $OUTPUT_DIR${NC}"
echo ""

python3 "${SCRIPT_DIR}/analyze_multi_robot.py" "$CSV_FILE" "$OUTPUT_DIR"

# Check if analysis succeeded
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ Analysis complete!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "Generated files:"
    ls -lh "$OUTPUT_DIR" | tail -n +2 | awk '{print "  " $9 " (" $5 ")"}'
    echo ""
    echo -e "${BLUE}View results:${NC}"
    echo "  Report: $OUTPUT_DIR/*_analysis_report.txt"
    echo "  Plots:  $OUTPUT_DIR/*.png"
else
    echo -e "${RED}ERROR: Analysis failed${NC}"
    exit 1
fi
