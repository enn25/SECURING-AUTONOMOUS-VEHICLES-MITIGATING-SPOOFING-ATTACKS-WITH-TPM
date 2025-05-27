#!/bin/bash

# system_up - Script to launch multiple ROS2 and Python processes in separate terminals

# Function to check if gnome-terminal is available
use_gnome_terminal() {
    command -v gnome-terminal > /dev/null 2>&1
}

# Function to check if xterm is available
use_xterm() {
    command -v xterm > /dev/null 2>&1
}

# Function to check if konsole is available
use_konsole() {
    command -v konsole > /dev/null 2>&1
}

# Check which terminal is available and launch commands
if use_gnome_terminal; then
    echo "Using gnome-terminal to launch processes..."
    
    gnome-terminal -- bash -c "echo 'Starting rosbridge websocket server...'; ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash"
    sleep 1
    
    gnome-terminal -- bash -c "echo 'Starting YDLidar...'; ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py; exec bash"
    sleep 1
    
    gnome-terminal -- bash -c "echo 'Starting motor controller...'; python3 motor_controller.py; exec bash"
    sleep 1
    
    gnome-terminal -- bash -c "echo 'Starting diagnostics...'; python3 diagnostic.py; exec bash"
    sleep 1
    
    gnome-terminal -- bash -c "echo 'Starting script runner...'; python3 script_runner.py; exec bash"
    
elif use_konsole; then
    echo "Using konsole to launch processes..."
    
    konsole --new-tab -e bash -c "echo 'Starting rosbridge websocket server...'; ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash"
    sleep 1
    
    konsole --new-tab -e bash -c "echo 'Starting YDLidar...'; ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py; exec bash"
    sleep 1
    
    konsole --new-tab -e bash -c "echo 'Starting motor controller...'; python3 motor_controller.py; exec bash"
    sleep 1
    
    konsole --new-tab -e bash -c "echo 'Starting diagnostics...'; python3 diagnostic.py; exec bash"
    sleep 1
    
    konsole --new-tab -e bash -c "echo 'Starting script runner...'; python3 script_runner.py; exec bash"
    
elif use_xterm; then
    echo "Using xterm to launch processes..."
    
    xterm -T "Rosbridge Websocket" -e "echo 'Starting rosbridge websocket server...'; ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash" &
    sleep 1
    
    xterm -T "YDLidar" -e "echo 'Starting YDLidar...'; ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py; exec bash" &
    sleep 1
    
    xterm -T "Motor Controller" -e "echo 'Starting motor controller...'; python3 motor_controller.py; exec bash" &
    sleep 1
    
    xterm -T "Diagnostics" -e "echo 'Starting diagnostics...'; python3 diagnostic.py; exec bash" &
    sleep 1
    
    xterm -T "Script Runner" -e "echo 'Starting script runner...'; python3 script_runner.py; exec bash" &
    
else
    echo "Error: No suitable terminal emulator found (tried gnome-terminal, konsole, and xterm)."
    echo "Please install one of these terminals or modify this script to use your preferred terminal."
    exit 1
fi

echo "All processes launched. Use Ctrl+C in each terminal to stop individual processes."
