#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import pickle
import os
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import threading

class LidarSpoofingDetector(Node):
    def __init__(self):
        super().__init__('lidar_spoofing_detector')
        
        # Create subscriptions to both the original and spoofed scan topics
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Change from default RELIABLE
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
            )

        self.original_subscription = self.create_subscription(
            LaserScan,
            '/scan_original',
            self.original_scan_callback,
            qos)  # Use the custom QoS profile
    
        self.spoofed_subscription = self.create_subscription(
            LaserScan,
            '/scan_spoofed',
            self.spoofed_scan_callback,
            qos) 
        
        # Buffer to store recent scans for comparison
        self.original_scans_buffer = []
        self.spoofed_scans_buffer = []
        self.buffer_size = 5  # Store the last 5 scans for temporal analysis
        
        # Create publisher for alerts
        self.alert_publisher = self.create_publisher(
            LaserScan,  # You might want to create a custom message type for alerts
            '/lidar_spoofing_alert',
            10)
        
        # Model parameters
        self.model_path = os.path.expanduser('~/lidar_spoofing_model.pkl')
        self.model = None
        self.load_or_train_model()
        
        # Feature extraction parameters
        self.num_features = 15  # Number of features we'll extract from each scan
        
        # Tracking variables
        self.last_original_timestamp = None
        self.last_spoofed_timestamp = None
        self.detection_count = 0
        self.total_scans = 0
        self.is_collecting_data = False
        self.collected_data = []
        self.collected_labels = []
        
        # For evaluation purposes
        self.true_positives = 0
        self.false_positives = 0
        self.true_negatives = 0
        self.false_negatives = 0
        
        # Create timers
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        self.detection_timer = self.create_timer(0.1, self.run_detection)
        
        # Initialize detection lock
        self.detection_lock = threading.Lock()
        
        self.get_logger().info('LiDAR Spoofing Detector initialized!')
        
    def load_or_train_model(self):
        """Load the existing model or train a new one if no model exists"""
        if os.path.exists(self.model_path):
            try:
                with open(self.model_path, 'rb') as f:
                    self.model = pickle.load(f)
                self.get_logger().info(f'Loaded existing model from {self.model_path}')
                return
            except Exception as e:
                self.get_logger().error(f'Failed to load model: {e}')
                
        # If we reached here, we need to train a new model
        self.get_logger().warn('No model found. Starting with default model.')
        # Initialize a basic model - will be replaced by properly trained one
        self.model = RandomForestClassifier(n_estimators=50, random_state=42)
        self.get_logger().info('Initialized default detection model')
        
    def original_scan_callback(self, msg):
        """Process original LiDAR scan data"""
        self.last_original_timestamp = time.time()
        
        # Store the scan in our buffer
        self.original_scans_buffer.append(msg)
        if len(self.original_scans_buffer) > self.buffer_size:
            self.original_scans_buffer.pop(0)
            
        # If we're collecting data for training, add this as a non-spoofed example
        if self.is_collecting_data and len(self.original_scans_buffer) >= 2:
            features = self.extract_features(self.original_scans_buffer)
            self.collected_data.append(features)
            self.collected_labels.append(0)  # 0 = not spoofed
            
    def spoofed_scan_callback(self, msg):
        """Process spoofed LiDAR scan data"""
        self.last_spoofed_timestamp = time.time()
        
        # Store the scan in our buffer
        self.spoofed_scans_buffer.append(msg)
        if len(self.spoofed_scans_buffer) > self.buffer_size:
            self.spoofed_scans_buffer.pop(0)
            
        # If we're collecting data for training, add this as a spoofed example
        if self.is_collecting_data and len(self.spoofed_scans_buffer) >= 2:
            features = self.extract_features(self.spoofed_scans_buffer)
            self.collected_data.append(features)
            self.collected_labels.append(1)  # 1 = spoofed
            
    def extract_features(self, scan_buffer):
        """Extract features from a scan or scan buffer for detection"""
        # Use the most recent scan
        current_scan = scan_buffer[-1]
        
        # Convert LaserScan.ranges to numpy array for analysis
        ranges = np.array(current_scan.ranges)
        
        # Remove invalid readings (0.0 or inf)
        valid_ranges = ranges[(ranges > current_scan.range_min) & (ranges < current_scan.range_max)]
        
        if len(valid_ranges) == 0:
            # Handle case with no valid ranges
            return np.zeros(self.num_features)
        
        # Calculate basic statistical features
        features = []
        
        # 1. Basic statistics of the current scan
        features.append(np.mean(valid_ranges))
        features.append(np.std(valid_ranges))
        features.append(np.median(valid_ranges))
        features.append(np.max(valid_ranges) if len(valid_ranges) > 0 else 0)
        features.append(np.min(valid_ranges) if len(valid_ranges) > 0 else 0)
        
        # 2. Distribution metrics
        hist, _ = np.histogram(valid_ranges, bins=5, range=(0, current_scan.range_max))
        features.extend(hist / (np.sum(hist) if np.sum(hist) > 0 else 1))  # Normalized histogram
        
        # 3. Scan consistency features (if we have previous scans)
        if len(scan_buffer) >= 2:
            prev_scan = scan_buffer[-2]
            prev_ranges = np.array(prev_scan.ranges)
            valid_prev = prev_ranges[(prev_ranges > prev_scan.range_min) & (prev_ranges < prev_scan.range_max)]
            
            if len(valid_prev) > 0 and len(valid_ranges) > 0:
                # Compare overall statistics between scans
                features.append(abs(np.mean(valid_ranges) - np.mean(valid_prev)))
                features.append(abs(np.std(valid_ranges) - np.std(valid_prev)))
                
                # Calculate average change
                min_len = min(len(ranges), len(prev_ranges))
                if min_len > 0:
                    diff = np.abs(ranges[:min_len] - prev_ranges[:min_len])
                    valid_diff = diff[(ranges[:min_len] > 0) & (prev_ranges[:min_len] > 0)]
                    features.append(np.mean(valid_diff) if len(valid_diff) > 0 else 0)
                else:
                    features.append(0)
            else:
                features.extend([0, 0, 0])  # Padding if we don't have valid previous scan
        else:
            features.extend([0, 0, 0])  # Padding if we don't have previous scan
        
        # Ensure we have the right number of features
        while len(features) < self.num_features:
            features.append(0)
            
        # Trim if we somehow have too many
        features = features[:self.num_features]
            
        return features
        
    def detect_spoofing(self, scan_buffer):
        """Apply the trained model to detect spoofing"""
        if not scan_buffer or len(scan_buffer) < 2 or self.model is None:
            return False, 0.0
            
        features = self.extract_features(scan_buffer)
        features = np.array(features).reshape(1, -1)  # Reshape for prediction
        
        # Get the prediction and probability
        prediction = self.model.predict(features)[0]
        probabilities = self.model.predict_proba(features)[0]
        confidence = probabilities[1] if prediction == 1 else probabilities[0]
        
        return prediction == 1, confidence
        
    def run_detection(self):
        """Run spoofing detection on latest data"""
        with self.detection_lock:
            # Skip if we don't have enough data
            if not self.original_scans_buffer or not self.spoofed_scans_buffer:
                return
                
            # Analyze the original scan (should be clean)
            is_original_spoofed, orig_confidence = self.detect_spoofing(self.original_scans_buffer)
            
            # Analyze the spoofed scan (should detect spoofing)
            is_spoofed_detected, spoof_confidence = self.detect_spoofing(self.spoofed_scans_buffer)
            
            # Update metrics (for training evaluation)
            if is_original_spoofed:
                self.false_positives += 1
            else:
                self.true_negatives += 1
                
            if is_spoofed_detected:
                self.true_positives += 1
            else:
                self.false_negatives += 1
                
            self.total_scans += 1
            
            # If we detected spoofing in the spoofed scan, count it and raise an alert
            if is_spoofed_detected:
                self.detection_count += 1
                self.publish_alert(spoof_confidence)
                
    def publish_alert(self, confidence):
        """Publish an alert when spoofing is detected"""
        alert_msg = LaserScan()  # Using LaserScan as a placeholder
        alert_msg.header.stamp = self.get_clock().now().to_msg()
        alert_msg.header.frame_id = "spoofing_alert"
        # Add the confidence as a field - in a real system you'd use a custom message type
        alert_msg.range_min = confidence
        alert_msg.range_max = 1.0
        self.alert_publisher.publish(alert_msg)
        
        self.get_logger().warn(f'SPOOFING DETECTED! Confidence: {confidence:.2f}')
                
    def print_stats(self):
        """Print detection statistics"""
        if self.total_scans == 0:
            self.get_logger().info('No scans processed yet')
            return
            
        detection_rate = (self.detection_count / self.total_scans) * 100
        
        # Calculate accuracy metrics
        total_evaluated = self.true_positives + self.true_negatives + self.false_positives + self.false_negatives
        if total_evaluated > 0:
            accuracy = (self.true_positives + self.true_negatives) / total_evaluated * 100
            precision = self.true_positives / (self.true_positives + self.false_positives) if (self.true_positives + self.false_positives) > 0 else 0
            recall = self.true_positives / (self.true_positives + self.false_negatives) if (self.true_positives + self.false_negatives) > 0 else 0
            f1_score = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0
            
            self.get_logger().info(f'Detection Stats: {self.detection_count}/{self.total_scans} scans flagged ({detection_rate:.1f}%)')
            self.get_logger().info(f'Accuracy: {accuracy:.1f}%, Precision: {precision:.2f}, Recall: {recall:.2f}, F1: {f1_score:.2f}')
            self.get_logger().info(f'TP: {self.true_positives}, TN: {self.true_negatives}, FP: {self.false_positives}, FN: {self.false_negatives}')
        else:
            self.get_logger().info(f'Detection Stats: {self.detection_count}/{self.total_scans} scans flagged ({detection_rate:.1f}%)')
            
    def start_data_collection(self, duration=60):
        """Start collecting data for model training"""
        self.is_collecting_data = True
        self.collected_data = []
        self.collected_labels = []
        self.get_logger().info(f'Started collecting training data for {duration} seconds')
        
        # Stop collection after the specified duration
        timer = threading.Timer(duration, self.stop_data_collection)
        timer.start()
        
    def stop_data_collection(self):
        """Stop data collection and train the model"""
        self.is_collecting_data = False
        
        if len(self.collected_data) < 10 or len(self.collected_labels) < 10:
            self.get_logger().error('Not enough data collected for training!')
            return
            
        self.get_logger().info(f'Data collection complete. Collected {len(self.collected_data)} samples')
        self.train_model()
        
    def train_model(self):
        """Train the spoofing detection model using collected data"""
        if not self.collected_data or not self.collected_labels:
            self.get_logger().error('No data available for training!')
            return
            
        # Convert to numpy arrays
        X = np.array(self.collected_data)
        y = np.array(self.collected_labels)
        
        self.get_logger().info(f'Training model with {len(X)} samples (Spoofed: {sum(y)}, Normal: {len(y) - sum(y)})')
        
        # Split data for training and validation
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)
        
        # Train the model
        self.model = RandomForestClassifier(n_estimators=100, random_state=42)
        self.model.fit(X_train, y_train)
        
        # Evaluate the model
        y_pred = self.model.predict(X_test)
        accuracy = accuracy_score(y_test, y_pred)
        
        self.get_logger().info(f'Model trained with validation accuracy: {accuracy:.4f}')
        
        # Print detailed metrics
        report = classification_report(y_test, y_pred)
        self.get_logger().info(f'Classification Report:\n{report}')
        
        # Save the model
        try:
            with open(self.model_path, 'wb') as f:
                pickle.dump(self.model, f)
            self.get_logger().info(f'Model saved to {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save model: {e}')

def main(args=None):
    rclpy.init(args=args)
    detector = LidarSpoofingDetector()
    
    # Start data collection for 60 seconds after initialization
    detector.start_data_collection(60)
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
