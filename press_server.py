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


# =============================================================================
# Button Press Server Class
# =============================================================================

class ButtonPressServer:
    """
    Main server class that handles button pressing with vision verification.
    
    This class manages:
    - Robot arm connection and control
    - Camera capture and image processing
    - ROI-based button state detection
    - Button press execution with verification
    - Data storage and retrieval
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
    
    def precheck_button(self, name: str) -> bool:
        """
        Check if button is already pressed before attempting to press it.
        
        Args:
            name: Name of the button to check
            
        Returns:
            True if button is already pressed, False if not pressed
            
        Note: This function is called before executing a button press to avoid
        unnecessary double-pressing. It captures a frame and analyzes the ROI.
        """
        frame = self.capture_frame()
        if frame is None:
            print(f"Warning: Could not capture frame for precheck of {name}")
            return False
        
        is_pressed = self.check_button_state(name, frame)
        print(f"Precheck {name}: {'Already pressed' if is_pressed else 'Not pressed'}")
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
            return False
        
        is_pressed = self.check_button_state(name, frame)
        print(f"Confirm {name}: {'Successfully pressed' if is_pressed else 'Press failed'}")
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
            is_pressed = self.press_server.check_button_state(button_name, frame)
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
    
    Note: The server runs indefinitely until interrupted by keyboard or error.
    """
    import argparse
    parser = argparse.ArgumentParser(description="Button press server with vision verification")
    parser.add_argument("--store", default=DEFAULT_STORE_PATH, help="Path to taught positions JSON")
    parser.add_argument("--ip", default=DEFAULT_LEBAI_IP, help="Lebai robot IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP server port")
    
    args = parser.parse_args()
    
    # Create press server instance
    press_server = ButtonPressServer(args.store, args.ip)
    
    try:
        # Connect to robot arm
        press_server.connect_arm()
        print(f"Connected to robot at {args.ip}")
        
        # Start HTTP server with custom handler
        server_address = ('', args.port)
        handler = lambda *args, **kwargs: PressHTTPHandler(*args, press_server=press_server, **kwargs)
        httpd = HTTPServer(server_address, handler)
        
        # Display server information
        print(f"Press server started on port {args.port}")
        print("Available endpoints:")
        print("  GET  /                    - List available buttons")
        print("  GET  /status/<button>     - Check button state")
        print("  POST /press/<button>      - Press button with verification")
        
        # Run server indefinitely
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
