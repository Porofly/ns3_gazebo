#!/bin/bash

# Network Configuration Verification Tool
#
# This script verifies that the dual network (WiFi + Direct) is properly configured
# for the multi-robot NS-3 simulation.
#
# It checks:
# 1. Network namespace existence
# 2. WiFi network interfaces (for NS-3)
# 3. Direct network interfaces (for host communication)
# 4. Routing tables
# 5. Connectivity between namespaces and host
#
# Usage:
#   sudo ./verify_network.sh [num_namespaces]
#
# Example:
#   sudo ./verify_network.sh 2

NUM_NS=${1:-2}

echo "=========================================="
echo " Network Configuration Verification"
echo "=========================================="
echo ""
echo "Checking configuration for $NUM_NS network namespaces"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run with sudo"
    exit 1
fi

ERRORS=0
WARNINGS=0

# Function to print status
print_status() {
    local status=$1
    local message=$2

    case $status in
        OK)
            echo "  ✓ $message"
            ;;
        WARN)
            echo "  ! $message"
            ((WARNINGS++))
            ;;
        ERROR)
            echo "  ✗ $message"
            ((ERRORS++))
            ;;
        INFO)
            echo "    $message"
            ;;
    esac
}

# Check 1: Network namespaces
echo "1. Checking network namespaces..."
for i in $(seq 1 $NUM_NS); do
    if ip netns list | grep -q "nns$i"; then
        print_status OK "nns$i exists"
    else
        print_status ERROR "nns$i not found"
    fi
done
echo ""

# Check 2: WiFi network interfaces
echo "2. Checking WiFi network (NS-3 simulation path)..."
for i in $(seq 1 $NUM_NS); do
    # Check namespace side
    if ip netns exec nns$i ip addr show wifi_veth$i &> /dev/null; then
        WIFI_IP=$(ip netns exec nns$i ip addr show wifi_veth$i | grep 'inet ' | awk '{print $2}')
        print_status OK "nns$i: wifi_veth$i configured ($WIFI_IP)"
    else
        print_status ERROR "nns$i: wifi_veth$i not found"
    fi

    # Check host side
    if ip addr show wifi_vethb$i &> /dev/null; then
        print_status OK "Host: wifi_vethb$i configured"
    else
        print_status ERROR "Host: wifi_vethb$i not found"
    fi

    # Check TAP device
    if ip addr show wifi_tap$i &> /dev/null; then
        print_status OK "Host: wifi_tap$i configured (NS-3 endpoint)"
    else
        print_status ERROR "Host: wifi_tap$i not found"
    fi
done
echo ""

# Check 3: Direct network interfaces
echo "3. Checking Direct network (host communication path)..."
for i in $(seq 1 $NUM_NS); do
    # Check namespace side
    if ip netns exec nns$i ip addr show direct_vethn$i &> /dev/null; then
        DIRECT_IP=$(ip netns exec nns$i ip addr show direct_vethn$i | grep 'inet ' | awk '{print $2}')
        print_status OK "nns$i: direct_vethn$i configured ($DIRECT_IP)"
    else
        print_status WARN "nns$i: direct_vethn$i not found (use --include_direct)"
    fi

    # Check host bridge
    if ip addr show direct_br$i &> /dev/null; then
        HOST_IP=$(ip addr show direct_br$i | grep 'inet ' | awk '{print $2}')
        print_status OK "Host: direct_br$i configured ($HOST_IP)"
    else
        print_status WARN "Host: direct_br$i not found (use --include_direct)"
    fi
done
echo ""

# Check 4: Routing tables
echo "4. Checking routing tables..."
for i in $(seq 1 $NUM_NS); do
    # WiFi route
    if ip netns exec nns$i ip route | grep -q "10.0.0.0/9 dev wifi_veth$i"; then
        print_status OK "nns$i: WiFi route configured (10.0.0.0/9)"
    else
        print_status ERROR "nns$i: WiFi route missing"
    fi

    # Direct route and default gateway
    if ip netns exec nns$i ip addr show direct_vethn$i &> /dev/null; then
        if ip netns exec nns$i ip route | grep -q "default"; then
            GW=$(ip netns exec nns$i ip route | grep default | awk '{print $3}')
            print_status OK "nns$i: Default gateway configured ($GW)"
        else
            print_status WARN "nns$i: No default gateway"
        fi
    fi
done
echo ""

# Check 5: Network isolation
echo "5. Checking network isolation..."
if [ $NUM_NS -ge 2 ]; then
    # Test WiFi network isolation (should NOT be able to ping directly)
    echo "  Testing WiFi network isolation (nns1 → nns2)..."
    if ip netns exec nns1 timeout 1 ping -c 1 -W 1 10.0.0.4 &> /dev/null; then
        print_status WARN "nns1 can ping nns2 directly (expected: only via NS-3)"
    else
        print_status OK "nns1 cannot ping nns2 directly (correct: requires NS-3)"
    fi

    # Test Direct network isolation (should NOT reach other namespace)
    if ip netns exec nns1 ip addr show direct_vethn1 &> /dev/null; then
        echo "  Testing Direct network isolation..."
        NS2_DIRECT_IP=$(ip netns exec nns2 ip addr show direct_vethn2 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)
        if [ -n "$NS2_DIRECT_IP" ]; then
            if ip netns exec nns1 timeout 1 ping -c 1 -W 1 $NS2_DIRECT_IP &> /dev/null; then
                print_status ERROR "nns1 can reach nns2 via Direct network (should be isolated)"
            else
                print_status OK "nns1 cannot reach nns2 via Direct (correct: separate /29 subnets)"
            fi
        fi
    fi
fi
echo ""

# Check 6: Host connectivity via Direct network
echo "6. Checking host connectivity (via Direct network)..."
for i in $(seq 1 $NUM_NS); do
    if ip netns exec nns$i ip addr show direct_vethn$i &> /dev/null; then
        # Get host bridge IP
        HOST_DIRECT_IP=$(ip addr show direct_br$i 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)
        if [ -n "$HOST_DIRECT_IP" ]; then
            if ip netns exec nns$i timeout 2 ping -c 1 -W 1 $HOST_DIRECT_IP &> /dev/null; then
                print_status OK "nns$i can reach host ($HOST_DIRECT_IP)"
            else
                print_status ERROR "nns$i cannot reach host ($HOST_DIRECT_IP)"
            fi
        fi
    else
        print_status WARN "nns$i: Direct network not configured"
    fi
done
echo ""

# Summary
echo "=========================================="
echo " Verification Summary"
echo "=========================================="
echo ""

if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo "✓ All checks passed!"
    echo ""
    echo "Network is properly configured for:"
    echo "  - Robot-to-robot communication (via NS-3 WiFi: 10.0.0.0/9)"
    echo "  - Robot control from namespaces (via Direct: 10.128.0.x/29)"
    echo ""
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo "! Configuration has warnings: $WARNINGS warning(s)"
    echo ""
    echo "The system may still work, but some features might be limited."
    echo "To enable Direct network:"
    echo "  sudo python3 ../scripts/nns_setup.py teardown -c $NUM_NS"
    echo "  sudo python3 ../scripts/nns_setup.py setup -c $NUM_NS --include_direct"
    echo ""
    exit 1
else
    echo "✗ Configuration has issues: $ERRORS error(s), $WARNINGS warning(s)"
    echo ""
    echo "Please reconfigure the network:"
    echo "  cd ../scripts"
    echo "  sudo python3 nns_setup.py teardown -c $NUM_NS --include_direct"
    echo "  sudo python3 nns_setup.py setup -c $NUM_NS --include_direct"
    echo ""
    exit 2
fi
