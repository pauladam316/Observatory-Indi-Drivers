#!/bin/bash

# Test script for Telescope Controller INDI Driver
# This script starts the INDI server and tests basic functionality

echo "========================================="
echo "Telescope Controller Driver Test Script"
echo "========================================="
echo ""

# Check if driver exists
DRIVER_PATH=$(which TelescopeControllerDriver 2>/dev/null)
if [ -z "$DRIVER_PATH" ]; then
    echo "ERROR: TelescopeControllerDriver not found in PATH"
    echo "Make sure the driver is installed: sudo make install"
    exit 1
fi

echo "Found driver at: $DRIVER_PATH"
echo ""

# Start INDI server in background with verbose output
echo "Starting INDI server with verbose output..."
echo "Server PID will be shown below"
echo ""
indiserver -v -v -v TelescopeControllerDriver &
SERVER_PID=$!

# Wait for server to start
sleep 2

# Check if server is still running
if ! kill -0 $SERVER_PID 2>/dev/null; then
    echo "ERROR: INDI server failed to start"
    exit 1
fi

echo "INDI server started (PID: $SERVER_PID)"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Cleaning up..."
    indi_setprop "Telescope Controller.CONNECTION.DISCONNECT=On" 2>/dev/null
    sleep 1
    kill $SERVER_PID 2>/dev/null
    echo "Test complete"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Connect to device
echo "Attempting to connect to device..."
indi_setprop "Telescope Controller.CONNECTION.CONNECT=On"
sleep 3

# Check connection status
echo ""
echo "Connection status:"
indi_getprop "Telescope Controller.CONNECTION.*" 2>/dev/null || echo "Failed to get connection status"

# Get initial telemetry
echo ""
echo "Initial telemetry values:"
indi_getprop "Telescope Controller.TELEMETRY.*" 2>/dev/null || echo "No telemetry data yet"

# Test heater 1
echo ""
echo "Testing Heater 1 control..."
echo "Enabling Heater 1..."
indi_setprop "Telescope Controller.HEATER1.HEATER1_ON=On" 2>/dev/null
sleep 2
echo "Heater 1 state:"
indi_getprop "Telescope Controller.HEATER1.*" 2>/dev/null

sleep 1
echo "Disabling Heater 1..."
indi_setprop "Telescope Controller.HEATER1.HEATER1_OFF=On" 2>/dev/null
sleep 2
echo "Heater 1 state:"
indi_getprop "Telescope Controller.HEATER1.*" 2>/dev/null

# Monitor telemetry for a few seconds
echo ""
echo "Monitoring telemetry for 5 seconds (watch for updates)..."
for i in {1..5}; do
    echo "--- Telemetry update $i ---"
    indi_getprop "Telescope Controller.TELEMETRY.*" 2>/dev/null
    sleep 1
done

echo ""
echo "========================================="
echo "Basic test complete!"
echo "========================================="
echo ""
echo "The INDI server is still running with verbose output."
echo "Check the server output above for detailed communication logs."
echo ""
echo "Press Ctrl+C to stop the server and exit."
echo ""

# Keep script running
wait $SERVER_PID

