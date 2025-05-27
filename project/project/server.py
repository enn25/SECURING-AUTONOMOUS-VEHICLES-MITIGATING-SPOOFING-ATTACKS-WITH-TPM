from flask import Flask, jsonify, request
from flask_cors import CORS
import subprocess
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

app = Flask(__name__)
CORS(app)

# Process tracking
spoofing_process = None
detection_process = None
ros_node = None

# Initialize ROS 2
def init_ros_node():
    global ros_node
    rclpy.init()
    ros_node = Node('flask_ros2_node')
    ros_node.get_logger().info("ROS 2 Node initialized.")
    ros_node.publisher = ros_node.create_publisher(Twist, '/cmd_vel', 10)

# Start ROS Node in a separate thread
ros_thread = threading.Thread(target=init_ros_node)
ros_thread.start()

@app.route('/api/spoofing/start', methods=['POST'])
def start_spoofing():
    global spoofing_process
    try:
        if spoofing_process is None:
            spoofing_process = subprocess.Popen(['python3', 'spoofing_script.py'])
            return jsonify({"status": "success", "message": "Spoofing injection started"})
        return jsonify({"status": "warning", "message": "Spoofing injection already running"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/spoofing/stop', methods=['POST'])
def stop_spoofing():
    global spoofing_process
    try:
        if spoofing_process is not None:
            spoofing_process.terminate()
            spoofing_process = None
            return jsonify({"status": "success", "message": "Spoofing injection stopped"})
        return jsonify({"status": "warning", "message": "Spoofing injection not running"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/detection/start', methods=['POST'])
def start_detection():
    global detection_process
    try:
        if detection_process is None:
            detection_process = subprocess.Popen(['python3', 'detection_script.py'])
            return jsonify({"status": "success", "message": "Spoof detection started"})
        return jsonify({"status": "warning", "message": "Spoof detection already running"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/detection/stop', methods=['POST'])
def stop_detection():
    global detection_process
    try:
        if detection_process is not None:
            detection_process.terminate()
            detection_process = None
            return jsonify({"status": "success", "message": "Spoof detection stopped"})
        return jsonify({"status": "warning", "message": "Spoof detection not running"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/control', methods=['POST'])
def control():
    global ros_node
    data = request.json
    direction = data.get('direction')
    action = data.get('action')
    
    if ros_node is None:
        return jsonify({"status": "error", "message": "ROS node not initialized"})
    
    twist = Twist()

    # Basic direction mapping
    if direction == 'forward':
        twist.linear.x = 0.5
    elif direction == 'backward':
        twist.linear.x = -0.5
    elif direction == 'left':
        twist.angular.z = 0.5
    elif direction == 'right':
        twist.angular.z = -0.5

    if action == 'stop':
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    ros_node.publisher.publish(twist)
    ros_node.get_logger().info(f"Published: {action} {direction}")

    return jsonify({"status": "success", "message": f"{action} {direction}"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
