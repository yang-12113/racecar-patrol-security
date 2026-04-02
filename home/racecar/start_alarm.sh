#!/bin/bash

echo "Starting car alarm system..."
cd /home/racecar
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start alarm node in background
echo "Starting alarm node..."
python3 install/racecar/lib/racecar/alarm_node_simple.py &
ALARM_PID=$!

sleep 2

# Start alarm monitor in background  
echo "Starting alarm monitor..."
python3 install/racecar/lib/racecar/alarm_monitor.py &
MONITOR_PID=$!

echo "Alarm system started!"
echo "Alarm node PID: $ALARM_PID"
echo "Monitor node PID: $MONITOR_PID"
echo "Press Ctrl+C to stop..."

# Wait for Ctrl+C
trap 'echo "Stopping alarm system..."; kill $ALARM_PID $MONITOR_PID; echo "System stopped"; exit' INT
wait
