import cv2
import json
import numpy as np
import lebai_sdk
from time import sleep as wait
import http.server
import http.server
from http.server import HTTPServer
import socket
from typing import Dict, Any, Optional, Tuple


# =============================================================================
# Configuration Constants
# =============================================================================
DEFAULT_STORE_PATH = "taught_positions.json"  # Default file containing taught button positions
DEFAULT_LEBAI_IP = "192.168.10.200"          # Default IP address of the Lebai robot
DEFAULT_PORT = 2000                           # Default HTTP server port
LEARNED_THRESHOLDS_FILE = "learned_thresholds.json"  # File to store learned detection values

# Pressure sensor configuration
PRESSURE_SENSOR_ENABLED = True                # Enable/disable pressure sensor detection
PRESSURE_THRESHOLD = 0.5                     # Pressure threshold in N (adjust based on your sensor)
PRESSURE_CHECK_INTERVAL = 0.1                # Pressure check interval in seconds
PRESSURE_STABLE_COUNT = 3                    # Number of consecutive readings to confirm pressure


# =============================================================================
# Button Press Server Class
# =============================================================================

class ButtonPressServer:
    """
    Main server class that handles button pressing with dual verification.
    
    This class manages:
    - Robot arm connection and control
    - Camera capture and image processing
    - ROI-based button state detection
    - Pressure sensor integration
    - Dual-mode button state verification
    - Button press execution with verification
    - Data storage and retrieval
    - Learning-based button detection thresholds
    """
    
    def __init__(self, store_path: str = DEFAULT_STORE_PATH, lebai_ip: str = DEFAULT_LEBAI_IP):
        """
        Initialize the ButtonPressServer.
        
        Args:
            store_path: Path to JSON file containing taught button positions
            lebai_ip: IP address of the Lebai robot
        """
        self.store_path = store_path
        self.lebai_ip = lebai_ip
        self.lebai = None          # Robot arm connection (initialized on demand)
        self.store = self.load_store()  # Load button data from JSON
        self.cap = None            # Camera capture object (initialized on demand)
        self.learned_thresholds = self.load_learned_thresholds()  # Load learned detection values
        
        # Pressure sensor configuration
        self.pressure_enabled = PRESSURE_SENSOR_ENABLED
        self.pressure_threshold = PRESSURE_THRESHOLD
        self.pressure_check_interval = PRESSURE_CHECK_INTERVAL
        self.pressure_stable_count = PRESSURE_STABLE_COUNT
        
        # Pressure sensor calibration data
        self.pressure_baseline = None
        self.pressure_calibrated = False
    
    def load_store(self) -> Dict[str, Any]:
        """
        Load button data from the JSON storage file.
        
        Returns:
            Dictionary containing button positions and metadata, or empty dict if file not found
        """
        try:
            with open(self.store_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            return {}
    
    def load_learned_thresholds(self) -> Dict[str, Any]:
        """
        Load learned button detection thresholds from JSON file.
        
        Returns:
            Dictionary containing learned thresholds for each button, or empty dict if file not found
        """
        try:
            with open(LEARNED_THRESHOLDS_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            return {}
    
    def save_learned_thresholds(self) -> None:
        """
        Save learned button detection thresholds to JSON file.
        """
        with open(LEARNED_THRESHOLDS_FILE, "w", encoding="utf-8") as f:
            json.dump(self.learned_thresholds, f, ensure_ascii=False, indent=2)
    
    def extract_roi(self, button_name: str, frame: np.ndarray) -> Optional[np.ndarray]:
        """
        Extract ROI (Region of Interest) for a specific button from a frame.
        
        Args:
            button_name: Name of the button
            frame: Camera frame as numpy array
            
        Returns:
            ROI image as numpy array, or None if ROI not configured
        """
        if button_name not in self.store:
            return None
        
        roi_data = self.store[button_name].get("roi")
        if not roi_data:
            return None
        
        x, y, w, h = roi_data["x"], roi_data["y"], roi_data["w"], roi_data["h"]
        
        # Check bounds
        if y + h > frame.shape[0] or x + w > frame.shape[1]:
            return None
        
        return frame[y:y+h, x:x+w]
    
    def learn_button_detection(self, button_name: str) -> bool:
        """
        Interactive learning for a specific button's detection thresholds.
        
        Args:
            button_name: Name of the button to learn
            
        Returns:
            True if learning was successful, False otherwise
            
        Note: This function guides the user through capturing OFF and ON states
        of the button to learn reliable detection thresholds.
        """
        if button_name not in self.store:
            print(f"Error: Button '{button_name}' not found in store")
            return False
        
        roi_data = self.store[button_name].get("roi")
        if not roi_data:
            print(f"Error: No ROI configured for button '{button_name}'")
            print("Use 'teach.py roi set' to configure ROI first")
            return False
        
        print(f"\n=== Learning Mode for Button {button_name} ===")
        print(f"ROI: x={roi_data['x']}, y={roi_data['y']}, w={roi_data['w']}, h={roi_data['h']}")
        
        try:
            # Step 1: Capture OFF state
            print("\n1. Ensure button is NOT pressed (LED off)")
            print("2. Position camera to see button clearly")
            print("3. Press Enter when ready...")
            input()
            
            off_frame = self.capture_frame()
            if off_frame is None:
                print("Error: Could not capture frame")
                return False
            
            off_roi = self.extract_roi(button_name, off_frame)
            if off_roi is None:
                print("Error: Could not extract ROI from frame")
                return False
            
            off_hsv = cv2.cvtColor(off_roi, cv2.COLOR_BGR2HSV)
            off_brightness = np.mean(off_hsv[:, :, 2]) / 255.0
            off_saturation = np.mean(off_hsv[:, :, 1]) / 255.0
            
            print(f"   OFF state captured: brightness={off_brightness:.3f}, saturation={off_saturation:.3f}")
            
            # Step 2: Capture ON state
            print("\n4. Now press button (turn on the LED)")
            print("5. Press Enter when LED is lit...")
            input()
            
            on_frame = self.capture_frame()
            if on_frame is None:
                print("Error: Could not capture frame")
                return False
            
            on_roi = self.extract_roi(button_name, on_frame)
            if on_roi is None:
                print("Error: Could not extract ROI from frame")
                return False
            
            on_hsv = cv2.cvtColor(on_roi, cv2.COLOR_BGR2HSV)
            on_brightness = np.mean(on_hsv[:, :, 2]) / 255.0
            on_saturation = np.mean(on_hsv[:, :, 1]) / 255.0
            
            print(f"   ON state captured: brightness={on_brightness:.3f}, saturation={on_saturation:.3f}")
            
            # Calculate differences
            brightness_diff = on_brightness - off_brightness
            saturation_diff = on_saturation - off_saturation
            
            # Store learned values
            self.learned_thresholds[button_name] = {
                'brightness_diff': brightness_diff,
                'saturation_diff': saturation_diff,
                'off_baseline': {
                    'brightness': off_brightness,
                    'saturation': off_saturation
                },
                'on_baseline': {
                    'brightness': on_brightness,
                    'saturation': on_saturation
                }
            }
            
            # Save to file
            self.save_learned_thresholds()
            
            print(f"\n6. Learning complete for {button_name}!")
            print(f"   - Brightness difference: {brightness_diff:.3f}")
            print(f"   - Saturation difference: {saturation_diff:.3f}")
            print(f"   - OFF baseline: brightness={off_brightness:.3f}, saturation={off_saturation:.3f}")
            print(f"   - ON baseline: brightness={on_brightness:.3f}, saturation={on_saturation:.3f}")
            
            return True
            
        except Exception as e:
            print(f"Error during learning: {e}")
            return False
    
    def learn_all_buttons(self) -> None:
        """
        Interactive learning for all buttons that have ROI configured.
        """
        buttons_with_roi = []
        for name in self.store.keys():
            if self.store[name].get("roi"):
                buttons_with_roi.append(name)
        
        if not buttons_with_roi:
            print("No buttons with ROI configured found.")
            print("Use 'teach.py roi set' to configure ROIs first.")
            return
        
        print(f"Found {len(buttons_with_roi)} buttons with ROI configured:")
        for name in buttons_with_roi:
            print(f"  - {name}")
        
        print("\nStarting learning process...")
        for name in buttons_with_roi:
            if not self.learn_button_detection(name):
                print(f"Learning failed for {name}, skipping...")
                continue
        
        print("\nLearning process complete!")
        print(f"Learned thresholds saved to {LEARNED_THRESHOLDS_FILE}")
    
    def check_button_state_learned(self, button_name: str, frame: np.ndarray) -> bool:
        """
        Check button state using learned detection thresholds.
        
        Args:
            button_name: Name of the button to check
            frame: Camera frame as numpy array
            
        Returns:
            True if button is pressed (ON), False if not pressed (OFF)
            
        Note: This function uses learned thresholds for more reliable detection
        that works with any LED color and lighting conditions.
        """
        if button_name not in self.learned_thresholds:
            # Fall back to basic brightness detection
            return self.check_button_state(button_name, frame)
        
        # Extract ROI
        roi = self.extract_roi(button_name, frame)
        if roi is None:
            return False
        
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        current_brightness = np.mean(hsv[:, :, 2]) / 255.0
        current_saturation = np.mean(hsv[:, :, 1]) / 255.0
        
        # Get learned values
        learned = self.learned_thresholds[button_name]
        baseline = learned['off_baseline']
        
        # Calculate changes from baseline
        brightness_change = current_brightness - baseline['brightness']
        saturation_change = current_saturation - baseline['saturation']
        
        # Button is ON if changes exceed learned thresholds (with 70% tolerance)
        brightness_threshold = learned['brightness_diff'] * 0.7
        saturation_threshold = learned['saturation_diff'] * 0.7
        
        brightness_ok = brightness_change > brightness_threshold
        saturation_ok = abs(saturation_change) > saturation_threshold
        
        # Debug output
        if hasattr(self, 'debug_mode') and self.debug_mode:
            print(f"DEBUG {button_name}: brightness_change={brightness_change:.3f} (threshold={brightness_threshold:.3f}), "
                  f"saturation_change={saturation_change:.3f} (threshold={saturation_threshold:.3f})")
        
        return brightness_ok or saturation_ok
    
    def calibrate_pressure_sensor(self) -> bool:
        """
        Calibrate the pressure sensor to establish baseline readings.
        
        Returns:
            True if calibration successful, False otherwise
            
        Note: This function should be called when the robot is in a neutral position
        with no external forces applied to the end-effector.
        """
        if not self.pressure_enabled:
            print("Pressure sensor is disabled")
            return False
        
        try:
            print("=== Pressure Sensor Calibration ===")
            print("1. Ensure robot is in neutral position (no external forces)")
            print("2. Press Enter when ready...")
            input()
            
            # Take multiple readings to establish baseline
            readings = []
            for i in range(10):
                pressure = self.read_pressure_sensor()
                if pressure is not None:
                    readings.append(pressure)
                    print(f"   Reading {i+1}: {pressure:.3f} N")
                wait(self.pressure_check_interval)
            
            if len(readings) >= 5:
                self.pressure_baseline = sum(readings) / len(readings)
                self.pressure_calibrated = True
                print(f"\n✓ Calibration complete!")
                print(f"   Baseline pressure: {self.pressure_baseline:.3f} N")
                print(f"   Detection threshold: {self.pressure_threshold:.3f} N")
                return True
            else:
                print("✗ Calibration failed: insufficient readings")
                return False
                
        except Exception as e:
            print(f"Error during pressure calibration: {e}")
            return False
    
    def read_pressure_sensor(self) -> Optional[float]:
        """
        Read current pressure from the robot's force sensor.
        
        Returns:
            Current pressure reading in Newtons, or None if reading failed
            
        Note: This function attempts to read from common force sensor interfaces.
        You may need to modify this based on your specific Lebai robot model.
        """
        if not self.pressure_enabled:
            return None
        
        try:
            # Try to get force data from the robot
            if self.lebai is None:
                return None
            
            # Method 1: Try to get force/torque data (common in modern robots)
            try:
                # This is a common interface - adjust based on your SDK
                force_data = self.lebai.get_force_data()
                if force_data and 'force_z' in force_data:
                    return abs(force_data['force_z'])  # Use Z-axis force
            except:
                pass
            
            # Method 2: Try to get joint torques and estimate contact
            try:
                kin_data = self.lebai.get_kin_data()
                if kin_data and 'actual_joint_torque' in kin_data:
                    # Sum torques to estimate contact force
                    torques = kin_data['actual_joint_torque']
                    if torques:
                        # Simple heuristic: sum of absolute torques
                        total_torque = sum(abs(t) for t in torques)
                        # Convert to approximate force (this is a rough estimate)
                        return total_torque * 0.1  # Scaling factor
            except:
                pass
            
            # Method 3: Try to get external force (if available)
            try:
                external_force = self.lebai.get_external_force()
                if external_force and 'magnitude' in external_force:
                    return external_force['magnitude']
            except:
                pass
            
            # If all methods fail, return None
            return None
            
        except Exception as e:
            if hasattr(self, 'debug_mode') and self.debug_mode:
                print(f"DEBUG: Pressure reading error: {e}")
            return None
    
    def check_pressure_contact(self, button_name: str) -> bool:
        """
        Check if the robot is in contact with a surface using pressure sensor.
        
        Args:
            button_name: Name of the button (for logging purposes)
            
        Returns:
            True if contact detected, False otherwise
            
        Note: This function checks for stable pressure readings above the threshold.
        """
        if not self.pressure_enabled or not self.pressure_calibrated:
            return False
        
        try:
            # Take multiple readings to ensure stable contact
            contact_readings = 0
            for _ in range(self.pressure_stable_count):
                pressure = self.read_pressure_sensor()
                if pressure is not None:
                    # Check if pressure is above threshold (indicating contact)
                    if pressure > (self.pressure_baseline + self.pressure_threshold):
                        contact_readings += 1
                
                wait(self.pressure_check_interval)
            
            # Contact confirmed if majority of readings indicate contact
            is_contact = contact_readings >= (self.pressure_stable_count // 2 + 1)
            
            if hasattr(self, 'debug_mode') and self.debug_mode:
                print(f"DEBUG {button_name}: pressure_contact={is_contact}, readings={contact_readings}/{self.pressure_stable_count}")
            
            return is_contact
            
        except Exception as e:
            if hasattr(self, 'debug_mode') and self.debug_mode:
                print(f"DEBUG: Pressure contact check error: {e}")
            return False
    
    def check_button_state_dual(self, button_name: str, frame: np.ndarray) -> Tuple[bool, Dict[str, Any]]:
        """
        Check button state using both vision and pressure detection.
        
        Args:
            button_name: Name of the button to check
            frame: Camera frame as numpy array
            
        Returns:
            Tuple of (is_pressed, detection_details)
            
        Note: This function combines vision and pressure detection for maximum reliability.
        """
        detection_details = {
            'vision_detection': False,
            'pressure_detection': False,
            'vision_confidence': 0.0,
            'pressure_confidence': 0.0,
            'final_result': False
        }
        
        # Vision detection
        vision_result = self.check_button_state_smart(button_name, frame)
        detection_details['vision_detection'] = vision_result
        
        # Pressure detection (if enabled and calibrated)
        pressure_result = False
        if self.pressure_enabled and self.pressure_calibrated:
            pressure_result = self.check_pressure_contact(button_name)
            detection_details['pressure_detection'] = pressure_result
        
        # Calculate confidence scores
        if vision_result:
            detection_details['vision_confidence'] = 1.0
        
        if pressure_result:
            detection_details['pressure_confidence'] = 1.0
        
        # Decision logic: Button is pressed if either detection method confirms it
        # This provides redundancy and improves reliability
        final_result = vision_result or pressure_result
        detection_details['final_result'] = final_result
        
        # Debug output
        if hasattr(self, 'debug_mode') and self.debug_mode:
            print(f"DEBUG {button_name}: vision={vision_result}, pressure={pressure_result}, final={final_result}")
        
        return final_result, detection_details
    
    def connect_arm(self):
        """
        Establish connection to the Lebai robot arm.
        
        Note: This function initializes the SDK, connects to the robot, and starts the system.
        The connection is established only when needed (lazy initialization).
        """
        if self.lebai is None:
            lebai_sdk.init()
            self.lebai = lebai_sdk.connect(self.lebai_ip, False)
            self.lebai.start_sys()
    
    def shutdown_arm(self):
        """
        Safely shut down the robot arm connection.
        
        Note: Gracefully handles shutdown even if errors occur during the process.
        """
        if self.lebai:
            try:
                self.lebai.stop_sys()
            except Exception:
                pass
            self.lebai = None
    
    def get_camera(self):
        """
        Initialize and return camera capture object.
        
        Returns:
            OpenCV VideoCapture object
            
        Raises:
            RuntimeError: If no camera can be opened
            
        Note: Tries /dev/video0 first, then falls back to /dev/video1.
        The camera is initialized only when needed (lazy initialization).
        """
        if self.cap is None:
            self.cap = cv2.VideoCapture("/dev/video0")
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture("/dev/video1")
            if not self.cap.isOpened():
                raise RuntimeError("Cannot open camera")
        return self.cap
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """
        Capture a single frame from the camera.
        
        Returns:
            Captured image as numpy array, or None if capture failed
            
        Note: This function handles camera initialization and frame capture in one call.
        """
        cap = self.get_camera()
        ret, frame = cap.read()
        if not ret:
            return None
        return frame
    
    def check_button_state(self, name: str, frame: np.ndarray) -> bool:
        """
        Check if a button is pressed (LED is on) based on ROI brightness analysis.
        
        Args:
            name: Name of the button to check
            frame: Camera frame as numpy array
            
        Returns:
            True if button is pressed (brightness above threshold), False otherwise
            
        Note: This function:
        1. Extracts the ROI (Region of Interest) from the frame
        2. Converts the ROI to HSV color space
        3. Calculates mean brightness in the V (Value) channel
        4. Compares brightness to the configured threshold
        
        The ROI must be configured using 'teach.py roi set' before this function can work.
        """
        if name not in self.store:
            return False
        
        roi_data = self.store[name].get("roi")
        if not roi_data:
            return False
        
        x, y, w, h = roi_data["x"], roi_data["y"], roi_data["w"], roi_data["h"]
        threshold = roi_data["threshold"]
        
        # Extract ROI from the frame
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0:
            return False
        
        # Convert to HSV and get V channel (brightness)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        
        # Calculate mean brightness (normalized to 0.0-1.0)
        mean_brightness = np.mean(v_channel) / 255.0
        
        # Button is pressed if brightness above threshold
        return mean_brightness > threshold
    
    def check_button_state_smart(self, name: str, frame: np.ndarray) -> bool:
        """
        Smart button state detection that automatically chooses the best method.
        
        Args:
            name: Name of the button to check
            frame: Camera frame as numpy array
            
        Returns:
            True if button is pressed, False otherwise
            
        Note: This function automatically uses learned detection if available,
        otherwise falls back to basic brightness detection.
        """
        # Try learned detection first
        if name in self.learned_thresholds:
            return self.check_button_state_learned(name, frame)
        else:
            # Fall back to basic brightness detection
            return self.check_button_state(name, frame)
    
    def precheck_button(self, name: str) -> bool:
        """
        Check if button is already pressed before attempting to press it.
        
        Args:
            name: Name of the button to check
            
        Returns:
            True if button is already pressed, False if not pressed
            
        Note: This function is called before executing a button press to avoid
        unnecessary double-pressing. It uses dual detection when available.
        """
        frame = self.capture_frame()
        if frame is None:
            print(f"Warning: Could not capture frame for precheck of {name}")
            # Fall back to pressure-only detection if vision fails
            if self.pressure_enabled and self.pressure_calibrated:
                pressure_result = self.check_pressure_contact(name)
                print(f"Precheck {name} (pressure only): {'Contact detected' if pressure_result else 'No contact'}")
                return pressure_result
            return False
        
        # Use dual detection if available, otherwise fall back to vision-only
        if self.pressure_enabled and self.pressure_calibrated:
            is_pressed, details = self.check_button_state_dual(name, frame)
            print(f"Precheck {name} (dual): {'Already pressed' if is_pressed else 'Not pressed'}")
            print(f"  Vision: {details['vision_detection']}, Pressure: {details['pressure_detection']}")
        else:
            # Use smart detection (learned if available, basic if not)
            is_pressed = self.check_button_state_smart(name, frame)
            print(f"Precheck {name} (vision only): {'Already pressed' if is_pressed else 'Not pressed'}")
        
        return is_pressed
    
    def confirm_button_press(self, name: str) -> bool:
        """
        Confirm that a button was successfully pressed after execution.
        
        Args:
            name: Name of the button to confirm
            
        Returns:
            True if button press was successful, False if it failed
            
        Note: This function is called after executing a button press to verify
        that the button state changed as expected (e.g., LED turned on).
        """
        frame = self.capture_frame()
        if frame is None:
            print(f"Warning: Could not capture frame for confirmation of {name}")
            # Fall back to pressure-only detection if vision fails
            if self.pressure_enabled and self.pressure_calibrated:
                pressure_result = self.check_pressure_contact(name)
                print(f"Confirm {name} (pressure only): {'Contact confirmed' if pressure_result else 'No contact'}")
                return pressure_result
            return False
        
        # Use dual detection if available, otherwise fall back to vision-only
        if self.pressure_enabled and self.pressure_calibrated:
            is_pressed, details = self.check_button_state_dual(name, frame)
            print(f"Confirm {name} (dual): {'Successfully pressed' if is_pressed else 'Press failed'}")
            print(f"  Vision: {details['vision_detection']}, Pressure: {details['pressure_detection']}")
        else:
            # Use smart detection (learned if available, basic if not)
            is_pressed = self.check_button_state_smart(name, frame)
            print(f"Confirm {name} (vision only): {'Successfully pressed' if is_pressed else 'Press failed'}")
        
        return is_pressed
    
    def execute_button_press(self, name: str) -> bool:
        """
        Execute a complete button press sequence with vision verification.
        
        Args:
            name: Name of the button to press
            
        Returns:
            True if button press was successful, False otherwise
            
        Note: This function performs the complete press sequence:
        1. Pre-check: Verify button is not already pressed
        2. Execute: Move through approach → base → press → base → approach positions
        3. Confirm: Verify button state changed after pressing
        4. Report: Return success/failure status
        
        The function uses the taught position data and metadata from the JSON store.
        """
        if name not in self.store:
            print(f"Error: Button '{name}' not found")
            return False
        
        # Pre-check: is button already pressed?
        if self.precheck_button(name):
            print(f"Button {name} is already pressed, skipping")
            return True
        
        # Get button data from store
        entry = self.store[name]
        base_pose = entry["pose"]
        meta = entry.get("meta", {})
        
        # Extract press parameters
        axis = meta.get("axis", "z")
        approach_offset = meta.get("approach_offset_m", 0.03)
        press_depth = meta.get("press_depth_m", 0.005)
        accel = meta.get("accel", np.pi)
        vel = meta.get("vel", np.pi)
        
        # Calculate approach and press poses
        approach_pose = self.with_offset(base_pose, axis, +approach_offset)
        press_pose = self.with_offset(base_pose, axis, -press_depth)
        
        try:
            # Execute the complete press sequence
            print(f"Executing press sequence for {name}...")
            
            # Move to approach position (safe distance from button)
            self.lebai.movej(approach_pose, accel, vel)
            self.lebai.wait_move()
            
            # Move to base position (exact taught pose)
            self.lebai.movej(base_pose, accel, vel)
            self.lebai.wait_move()
            
            # Move to press position (pressed down)
            self.lebai.movej(press_pose, accel, vel)
            self.lebai.wait_move()
            
            # Return to base position
            self.lebai.movej(base_pose, accel, vel)
            self.lebai.wait_move()
            
            # Return to approach position
            self.lebai.movej(approach_pose, accel, vel)
            self.lebai.wait_move()
            
            # Wait for button state to stabilize
            wait(0.5)
            
            # Confirm success by checking button state
            success = self.confirm_button_press(name)
            if not success:
                print(f"Warning: Button {name} press may have failed")
            
            return success
            
        except Exception as e:
            print(f"Error during button press execution: {e}")
            return False
    
    def with_offset(self, pose: Dict[str, float], axis: str, delta_m: float) -> Dict[str, float]:
        """
        Create a new pose by adding an offset along a specific axis.
        
        Args:
            pose: Original 6D pose dictionary
            axis: Axis to offset ('x', 'y', or 'z')
            delta_m: Offset distance in meters (positive or negative)
            
        Returns:
            New pose dictionary with the specified offset
            
        Raises:
            ValueError: If axis is not 'x', 'y', or 'z'
            
        Note: This function is used to create approach and press poses
        from the base taught pose by offsetting along the specified axis.
        """
        if axis not in {"x", "y", "z"}:
            raise ValueError("axis must be one of {'x','y','z'}")
        
        new_pose = pose.copy()
        new_pose[axis] = float(new_pose[axis] + delta_m)
        return new_pose
    
    def get_button_list(self) -> list:
        """
        Get list of all available button names from the store.
        
        Returns:
            List of button names that can be pressed
            
        Note: This function returns the keys from the loaded JSON store,
        representing all buttons that have been taught and saved.
        """
        return list(self.store.keys())


# =============================================================================
# HTTP Request Handler Class
# =============================================================================

class PressHTTPHandler(http.server.BaseHTTPRequestHandler):
    """
    HTTP request handler for the button press server.
    
    This class handles:
    - GET requests for button status and server information
    - POST requests for executing button presses
    - JSON responses for all endpoints
    
    Note: This handler is instantiated with a reference to the ButtonPressServer
    instance to access its methods and data.
    """
    
    def __init__(self, *args, press_server: ButtonPressServer, **kwargs):
        """
        Initialize the HTTP handler with a reference to the press server.
        
        Args:
            press_server: Instance of ButtonPressServer to handle requests
        """
        self.press_server = press_server
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """
        Handle HTTP GET requests.
        
        Supported endpoints:
        - GET /: List available buttons and usage information
        - GET /status/<button>: Check current state of a specific button
        """
        if self.path == '/':
            # Return list of available buttons and usage information
            buttons = self.press_server.get_button_list()
            response = {
                "available_buttons": buttons,
                "usage": {
                    "press": "POST /press/<button_name>",
                    "list": "GET /",
                    "status": "GET /status/<button_name>"
                }
            }
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response, indent=2).encode())
            
        elif self.path.startswith('/status/'):
            # Check button status by analyzing ROI in current camera frame
            button_name = self.path.split('/')[-1]
            frame = self.press_server.capture_frame()
            if frame is None:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(b'Camera error')
                return
            
            # Analyze button state using ROI detection
            is_pressed = self.press_server.check_button_state_smart(button_name, frame)
            response = {
                "button": button_name,
                "is_pressed": is_pressed,
                "timestamp": np.datetime64('now').isoformat()
            }
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response, indent=2).encode())
            
        else:
            # Return 404 for unknown endpoints
            self.send_error(404)
    
    def do_POST(self):
        """
        Handle HTTP POST requests.
        
        Supported endpoints:
        - POST /press/<button>: Execute button press with full verification
        """
        if self.path.startswith('/press/'):
            # Extract button name from URL path
            button_name = self.path.split('/')[-1]
            
            # Verify button exists in the store
            if button_name not in self.press_server.store:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(f'Button {button_name} not found'.encode())
                return
            
            # Execute button press with vision verification
            success = self.press_server.execute_button_press(button_name)
            
            # Return execution result
            response = {
                "button": button_name,
                "success": success,
                "timestamp": np.datetime64('now').isoformat()
            }
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response, indent=2).encode())
            
        else:
            # Return 404 for unknown endpoints
            self.send_error(404)


# =============================================================================
# Main Server Functions
# =============================================================================

def main():
    """
    Main entry point for the button press server.
    
    This function:
    1. Parses command line arguments
    2. Creates ButtonPressServer instance
    3. Connects to the robot arm
    4. Starts HTTP server with custom handler
    5. Handles graceful shutdown on Ctrl+C
    6. Supports learning mode for button detection
    
    Note: The server runs indefinitely until interrupted by keyboard or error.
    """
    import argparse
    parser = argparse.ArgumentParser(description="Button press server with vision verification")
    parser.add_argument("--store", default=DEFAULT_STORE_PATH, help="Path to taught positions JSON")
    parser.add_argument("--ip", default=DEFAULT_LEBAI_IP, help="Lebai robot IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP server port")
    parser.add_argument("--learn", action="store_true", help="Enter learning mode for button detection")
    parser.add_argument("--learn-button", type=str, help="Learn detection for specific button")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode for detection")
    parser.add_argument("--calibrate-pressure", action="store_true", help="Calibrate pressure sensor")
    parser.add_argument("--pressure-threshold", type=float, default=PRESSURE_THRESHOLD, help="Pressure detection threshold in N")
    parser.add_argument("--disable-pressure", action="store_true", help="Disable pressure sensor detection")
    
    args = parser.parse_args()
    
    # Create press server instance
    press_server = ButtonPressServer(args.store, args.ip)
    
    # Configure pressure sensor
    if args.disable_pressure:
        press_server.pressure_enabled = False
        print("Pressure sensor detection disabled")
    
    if args.pressure_threshold != PRESSURE_THRESHOLD:
        press_server.pressure_threshold = args.pressure_threshold
        print(f"Pressure threshold set to {args.pressure_threshold} N")
    
    # Set debug mode if requested
    if args.debug:
        press_server.debug_mode = True
        print("Debug mode enabled")
    
    # Handle pressure sensor calibration
    if args.calibrate_pressure:
        print("=== Pressure Sensor Calibration Mode ===")
        if not press_server.pressure_enabled:
            print("Error: Pressure sensor is disabled")
            return 1
        
        # Connect to robot for calibration
        try:
            press_server.connect_arm()
            print(f"Connected to robot at {args.ip}")
            
            if press_server.calibrate_pressure_sensor():
                print("Pressure sensor calibration completed successfully!")
            else:
                print("Pressure sensor calibration failed!")
                return 1
        except Exception as e:
            print(f"Error during pressure calibration: {e}")
            return 1
        finally:
            press_server.shutdown_arm()
        
        print("Calibration mode complete.")
        return 0
    
    # Handle learning mode
    if args.learn or args.learn_button:
        print("=== Button Detection Learning Mode ===")
        
        if args.learn_button:
            # Learn specific button
            if args.learn_button not in press_server.store:
                print(f"Error: Button '{args.learn_button}' not found")
                return 1
            
            if not press_server.store[args.learn_button].get("roi"):
                print(f"Error: Button '{args.learn_button}' has no ROI configured")
                print("Use 'teach.py roi set' to configure ROI first")
                return 1
            
            print(f"Learning detection for button: {args.learn_button}")
            if press_server.learn_button_detection(args.learn_button):
                print("Learning completed successfully!")
            else:
                print("Learning failed!")
                return 1
        else:
            # Learn all buttons
            print("Learning detection for all buttons with ROI configured...")
            press_server.learn_all_buttons()
        
        print("Learning mode complete. You can now run the server normally.")
        return 0
    
    try:
        # Connect to robot arm
        press_server.connect_arm()
        print(f"Connected to robot at {args.ip}")
        
        # Display learning status
        if press_server.learned_thresholds:
            print(f"Using learned detection for {len(press_server.learned_thresholds)} buttons:")
            for name in press_server.learned_thresholds.keys():
                print(f"  - {name}")
        else:
            print("No learned thresholds found. Using basic brightness detection.")
            print("Run with --learn to improve detection accuracy.")
        
        # Display pressure sensor status
        if press_server.pressure_enabled:
            if press_server.pressure_calibrated:
                print(f"Pressure sensor: ENABLED (calibrated, threshold: {press_server.pressure_threshold} N)")
                print("Dual detection mode: Vision + Pressure")
            else:
                print("Pressure sensor: ENABLED (not calibrated)")
                print("Run with --calibrate-pressure to calibrate")
                print("Dual detection mode: Vision only (pressure disabled)")
        else:
            print("Pressure sensor: DISABLED")
            print("Detection mode: Vision only")
        
        # Display server information
        print(f"\nPress server started on port {args.port}")
        print("Available endpoints:")
        print("  GET  /                    - List available buttons")
        print("  GET  /status/<button>     - Check button state")
        print("  POST /press/<button>      - Press button with verification")
        
        # Run server indefinitely
        httpd = HTTPServer(('', args.port), lambda *args, **kwargs: PressHTTPHandler(*args, press_server=press_server, **kwargs))
        httpd.serve_forever()
        
    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        print("\nShutting down...")
    finally:
        # Clean up resources
        press_server.shutdown_arm()
        if press_server.cap:
            press_server.cap.release()


if __name__ == "__main__":
    main()
