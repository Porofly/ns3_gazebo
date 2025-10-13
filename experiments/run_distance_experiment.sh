#!/bin/bash
# Distance Experiment - Automated Runner
# Measures RSSI and packet loss as vehicle moves away from base station

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../ns3_gazebo_plugin" && pwd)"
SCENARIO_DIR="$(cd "${SCRIPT_DIR}/../scenarios" && pwd)"

# Global variables for cleanup
GZ_PID=""
CTRL_PID=""

# Cleanup function
cleanup() {
    local exit_code=$?
    echo ""
    echo "Cleaning up processes..."

    # Kill controller
    if [ ! -z "$CTRL_PID" ] && kill -0 $CTRL_PID 2>/dev/null; then
        kill $CTRL_PID 2>/dev/null || true
        wait $CTRL_PID 2>/dev/null || true
        echo "  ✓ Controller stopped (PID: $CTRL_PID)"
    fi

    # Kill Gazebo
    if [ ! -z "$GZ_PID" ] && kill -0 $GZ_PID 2>/dev/null; then
        kill $GZ_PID 2>/dev/null || true
        sleep 1
        kill -9 $GZ_PID 2>/dev/null || true
        echo "  ✓ Gazebo stopped (PID: $GZ_PID)"
    fi

    # Kill any remaining gz sim processes
    pkill -f "gz sim.*distance_experiment" 2>/dev/null && echo "  ✓ Cleaned up stray Gazebo processes" || true

    # Kill any remaining python controllers
    pkill -f "distance_controller.py" 2>/dev/null && echo "  ✓ Cleaned up stray controller processes" || true

    echo "  ✓ Cleanup complete"

    if [ $exit_code -ne 0 ]; then
        echo ""
        echo "Experiment interrupted or failed (exit code: $exit_code)"
    fi
}

# Register cleanup on exit (normal exit, interrupt, terminate)
trap cleanup EXIT INT TERM

echo "======================================"
echo "Distance Experiment - Automated Run"
echo "======================================"
echo ""
echo "Press Ctrl+C to stop at any time"
echo ""

# Configuration
DURATION=120  # seconds
CSV_FILE="distance_experiment.csv"
RESULTS_DIR="${SCRIPT_DIR}/results_distance"

# Step 1: Clean old data
echo "[1/6] Cleaning old data..."
cd "${NS3_DIR}"
rm -f "${CSV_FILE}"
rm -rf "${RESULTS_DIR}"
mkdir -p "${RESULTS_DIR}"
echo "✓ Clean"
echo ""

# Step 2: Set environment
echo "[2/6] Setting environment..."
export GZ_SIM_SYSTEM_PLUGIN_PATH="${NS3_DIR}/build"
echo "✓ GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo ""

# Step 3: Start Gazebo in background
echo "[3/6] Starting Gazebo simulation..."
cd "${NS3_DIR}"
gz sim "${SCENARIO_DIR}/distance_experiment.sdf" > /tmp/gz_distance.log 2>&1 &
GZ_PID=$!
echo "✓ Gazebo started (PID: ${GZ_PID})"
echo ""

# Wait for initialization
echo "Waiting for Gazebo initialization..."
sleep 5

# Verify Gazebo is still running
if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "ERROR: Gazebo failed to start"
    echo "Check log: tail -50 /tmp/gz_distance.log"
    exit 1
fi

# Step 4: Start controller
echo "[4/6] Starting movement controller..."
echo "  Speed: 0.5 m/s"
echo "  Duration: ${DURATION} seconds"
echo ""

python3 "${SCRIPT_DIR}/distance_controller.py" &
CTRL_PID=$!

# Wait for controller to finish
wait $CTRL_PID 2>/dev/null || true

echo ""
echo "[5/6] Stopping Gazebo..."
# Cleanup will be handled by trap
echo "✓ Simulation stopped"
echo ""

# Step 6: Analyze data
echo "[6/6] Analyzing data..."

if [ ! -f "${CSV_FILE}" ]; then
    echo "ERROR: CSV file not found: ${CSV_FILE}"
    exit 1
fi

LINE_COUNT=$(wc -l < "${CSV_FILE}")
echo "✓ CSV file: ${CSV_FILE}"
echo "  Records: ${LINE_COUNT}"

# Run analysis
cd "${SCRIPT_DIR}"
python3 analyze_data.py "${NS3_DIR}/${CSV_FILE}" "${RESULTS_DIR}/"

echo ""
echo "======================================"
echo "Experiment Complete!"
echo "======================================"
echo ""
echo "Results saved to: ${RESULTS_DIR}"
echo ""
echo "Generated files:"
ls -lh "${RESULTS_DIR}/" | grep -v "^total" | awk '{print "  - " $9 " (" $5 ")"}'
echo ""
echo "View results:"
echo "  cat ${RESULTS_DIR}/distance_experiment_report.txt"
echo "  xdg-open ${RESULTS_DIR}/distance_experiment_distance_vs_rssi.png"
echo ""
