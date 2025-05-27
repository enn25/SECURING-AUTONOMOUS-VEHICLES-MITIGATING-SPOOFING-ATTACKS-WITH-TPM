# 🚗 Securing Autonomous Vehicles: Mitigating Spoofing Attacks with TPM


This repository contains the ROS 2 (Jazzy) implementation of a secure, miniature autonomous vehicle system using YDLIDAR X2, OE-28 rotary encoders, a smartphone IMU, and spoof detection mechanisms powered by a Trusted Platform Module (TPM).

---

## 📌 Features

- Real-time 2D SLAM using YDLIDAR X2 and Cartographer
- Wheel odometry from OE-28 encoders
- IMU integration via smartphone app
- LiDAR spoofing detection using machine learning
- Secure sensor communication using TPM
- Real-time visualization using Foxglove Studio
- Modular ROS 2 nodes (Jazzy)

---

## 🧰 Hardware Requirements

- Raspberry Pi 5 (8GB RAM)
- YDLIDAR X2
- Raspberry Pi Camera
- OE-28 Rotary Encoders (x2)
- TPM 2.0 Module
- CAN Module
- Smartphone (for IMU streaming)
- Power Bank (≥10,000mAh)
- 2WD/4WD Robotic Chassis

---

## 🧪 Software Stack

- **OS**: Ubuntu Server 24.04 (64-bit)
- **ROS 2**: Jazzy
- **SLAM**: Google Cartographer
- **Machine Learning**: PyTorch (Random Forest Classifier for spoof detection)
- **Dashboard**: Foxglove Studio (for live monitoring)
- **Languages**: Python 3.10+
- **Visualization**: RViz, Foxglove Studio

---

## 📂 Project Structure

```
├── encoder_odometry/       # OE-28 encoder ROS2 node
├── imu_streamer/           # Smartphone IMU data to ROS2
├── ydlidar_driver/         # YDLIDAR X2 ROS2 driver
├── spoof_detection/        # LiDAR spoof detection module
├── combined_odometry/      # Fused odometry (encoder + IMU)
├── launch/                 # ROS2 launch files
├── models/                 # ML models for spoof detection
├── images/                 # Project images (add yours here)
└── README.md               # You are here!
```

---

## 🧭 System Architecture

- Sensor data from LiDAR, camera, and encoders is processed by ROS nodes.
- IMU data is streamed from smartphone.
- Data fusion via EKF (Extended Kalman Filter).
- TPM ensures encrypted data integrity.
- LiDAR spoofing detection uses a Random Forest classifier trained on statistical features.

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/secure-autonomous-vehicle.git
cd secure-autonomous-vehicle
```

### 2. Set up ROS 2 Jazzy

Install dependencies:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions ros-jazzy-cartographer
```

Source the ROS environment:
```bash
source /opt/ros/jazzy/setup.bash
```

### 3. Build the Workspace

```bash
colcon build
source install/setup.bash
```

### 4. Launch the Vehicle Stack

```bash
ros2 launch launch/vehicle_bringup.launch.py
```

---

## 🛡️ Spoof Detection

### Train the Model

Use the training data from `spoof_detection/data/` and run:

```bash
python3 train_model.py
```

### Start Detection

```bash
ros2 run spoof_detection spoof_detector_node.py
```

Spoofed LiDAR scans will trigger alerts on the `/lidar_spoofing_alert` topic.

---

## 🖥️ Dashboard Visualization

Launch Foxglove Studio and connect it to:

- `/scan`, `/map`, `/odom`, `/imu/data`
- `/lidar_spoofing_alert` (alerts)
- Custom control buttons for spoof injection

---

## 🧪 Testing Summary

| Component             | Result                       |
|----------------------|------------------------------|
| Encoder Accuracy      | < 5cm error in 1m travel     |
| IMU Streaming         | Real-time updates            |
| SLAM Mapping          | Stable 2D map with loop closure |
| Spoof Detection       | 92.5% accuracy, 0.27s latency |
| Dashboard             | Real-time & interactive      |


## 📸 Images
---![DFD level2](https://github.com/user-attachments/assets/24ee62dd-a378-4d43-80be-b8e560399370)

![model_side](https://github.com/user-attachments/assets/2911b7e3-fa3c-4c27-b911-87a889d5f7ce)

![model_up](https://github.com/user-attachments/assets/74d5970b-8dd8-43c0-bfb1-6c71b4545199)

![cartographer](https://github.com/user-attachments/assets/e3ccf230-5f5c-4d1d-b6a4-76bc9d470168)

![Fox_dashboard](https://github.com/user-attachments/assets/7c58dfcf-87f3-4379-b7ff-9b731a8e641e)

![spoof_detection](https://github.com/user-attachments/assets/5ca67086-a7b3-427c-9813-4f2ccb3913fe)

