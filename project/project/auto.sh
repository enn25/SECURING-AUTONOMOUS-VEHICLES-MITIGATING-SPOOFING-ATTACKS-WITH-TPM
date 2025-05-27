#!/bin/bash
# Script to launch multiple ROS2 commands in separate terminals

# Create a temporary file to store PIDs
PID_FILE="/tmp/ros2_terminals_$$.pid"
touch $PID_FILE

# Function to launch a command in a new terminal and store its PID
launch_command() {
    gnome-terminal -- bash -c "$1; exec bash" &
    PID=$!
    echo $PID >> $PID_FILE
    sleep 1  # Small delay to ensure terminal launches properly
}

# Function to kill all terminals
cleanup() {
    echo "Shutting down all terminals..."
    if [ -f "$PID_FILE" ]; then
        for PID in $(cat $PID_FILE); do
            kill -9 $PID 2>/dev/null
        done
        rm $PID_FILE
    fi
    exit 0
}

# Set up trap to handle script termination
trap cleanup SIGINT SIGTERM

# Launch each command in a separate terminal
launch_command "ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py"
launch_command "ros2 launch my_cartographer carto.launch.py"
launch_command "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
launch_command "python3 encoder_odometry_v1.py"
launch_command "python3 script_runner.py"
launch_command "python3 diagnostic.py"
launch_command "python3 motor_controller.py"

echo "All commands launched in separate terminals"
echo "Press Ctrl+C to close all terminals at once"

# Keep the script running to allow for cleanup
while true; do
    sleep 1
done
