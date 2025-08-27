import argparse
import json
import signal
import sys
from copy import deepcopy
from typing import Any, Dict, Tuple

import lebai_sdk
import numpy as np


# =============================================================================
# Configuration Constants
# =============================================================================
DEFAULT_STORE_PATH = "taught_positions.json"  # Default file to store taught button positions
DEFAULT_LEBAI_IP = "192.168.10.200"          # Default IP address of the Lebai robot


# =============================================================================
# Data Storage Functions
# =============================================================================

def load_store(store_path: str) -> Dict[str, Any]:
    """
    Load the JSON file containing taught button positions and metadata.
    
    Args:
        store_path: Path to the JSON file storing button data
        
    Returns:
        Dictionary containing button data, or empty dict if file not found
    """
    try:
        with open(store_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


def save_store(store_path: str, data: Dict[str, Any]) -> None:
    """
    Save button data to JSON file with proper formatting.
    
    Args:
        store_path: Path where to save the JSON file
        data: Dictionary containing button data to save
    """
    with open(store_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


# =============================================================================
# Robot Connection Management
# =============================================================================

def connect_arm(lebai_ip: str):
    """
    Initialize and connect to the Lebai robot arm.
    
    Args:
        lebai_ip: IP address of the Lebai robot
        
    Returns:
        Connected Lebai robot instance
        
    Note: This function initializes the SDK, connects to the robot, and starts the system
    """
    lebai_sdk.init()
    lebai = lebai_sdk.connect(lebai_ip, False)
    lebai.start_sys()
    return lebai


def shutdown_arm(lebai) -> None:
    """
    Safely shut down the robot arm connection.
    
    Args:
        lebai: Lebai robot instance to shut down
        
    Note: Gracefully handles shutdown even if errors occur
    """
    try:
        lebai.stop_sys()
    except Exception:
        pass


# =============================================================================
# Robot State Functions
# =============================================================================

def get_current_tcp_pose(lebai) -> Dict[str, float]:
    """
    Get the current Tool Center Point (TCP) pose from the robot.
    
    Args:
        lebai: Connected Lebai robot instance
        
    Returns:
        Dictionary containing 6D pose: x, y, z (position) and rx, ry, rz (rotation)
        
    Note: TCP pose represents the position and orientation of the robot's end effector
    """
    kin = lebai.get_kin_data()
    return {
        "x": float(kin["actual_tcp_pose"]["x"]),
        "y": float(kin["actual_tcp_pose"]["y"]),
        "z": float(kin["actual_tcp_pose"]["z"]),
        "rx": float(kin["actual_tcp_pose"]["rx"]),
        "ry": float(kin["actual_tcp_pose"]["ry"]),
        "rz": float(kin["actual_tcp_pose"]["rz"]),
    }


# =============================================================================
# Pose Manipulation Functions
# =============================================================================

def with_offset(pose: Dict[str, float], axis: str, delta_m: float) -> Dict[str, float]:
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
        
    Note: This is used to create approach and press poses from the base pose
    """
    if axis not in {"x", "y", "z"}:
        raise ValueError("axis must be one of {'x','y','z'}")
    new_pose = deepcopy(pose)
    new_pose[axis] = float(new_pose[axis] + delta_m)
    return new_pose


def movej_and_wait(lebai, pose: Dict[str, float], accel: float, vel: float) -> None:
    """
    Move the robot to a joint pose and wait for completion.
    
    Args:
        lebai: Connected Lebai robot instance
        pose: Target 6D pose dictionary
        accel: Joint acceleration in rad/s²
        vel: Joint velocity in rad/s
        
    Note: This function blocks until the movement is complete
    """
    lebai.movej(pose, accel, vel)
    lebai.wait_move()


# =============================================================================
# Button Teaching Functions
# =============================================================================

def teach_save(args: argparse.Namespace) -> int:
    """
    Save the current robot TCP pose as a named button position.
    
    Args:
        args: Command line arguments containing button name and parameters
        
    Returns:
        0 on success, 2 if button name already exists
        
    Note: This function saves the current pose without enabling teach mode.
    The robot should already be positioned at the desired button location.
    """
    store = load_store(args.store)
    name = args.name
    if (not args.overwrite) and (name in store):
        print(f"Error: name '{name}' already exists. Use --overwrite to replace.")
        return 2

    lebai = connect_arm(args.ip)
    try:
        pose = get_current_tcp_pose(lebai)
    finally:
        shutdown_arm(lebai)

    store[name] = {
        "pose": pose,
        "meta": {
            "axis": args.axis,
            "approach_offset_m": args.approach,
            "press_depth_m": args.press,
            "accel": args.accel,
            "vel": args.vel,
        },
    }
    save_store(args.store, store)
    print(f"Saved '{name}': {pose}")
    return 0


def teach_save_interactive(args: argparse.Namespace) -> int:
    """
    Save button position interactively by enabling teach mode for manual positioning.
    
    Args:
        args: Command line arguments containing button name and parameters
        
    Returns:
        0 on success, 2 if button name already exists
        
    Note: This function enables teach mode so you can manually move the robot
    to the desired button position, then press Enter to save the current pose.
    """
    store = load_store(args.store)
    name = args.name
    if (not args.overwrite) and (name in store):
        print(f"Error: name '{name}' already exists. Use --overwrite to replace.")
        return 2

    lebai = connect_arm(args.ip)
    try:
        # Enable manual teaching mode so you can move the arm by hand
        try:
            lebai.teach_mode()
        except Exception:
            pass
        print(f"Teaching '{name}'. Move the arm by hand to the desired press pose, then press ENTER to save (or Ctrl+C to cancel)...")
        input()
        pose = get_current_tcp_pose(lebai)
    finally:
        shutdown_arm(lebai)

    store[name] = {
        "pose": pose,
        "meta": {
            "axis": args.axis,
            "approach_offset_m": args.approach,
            "press_depth_m": args.press,
            "accel": args.accel,
            "vel": args.vel,
        },
    }
    save_store(args.store, store)
    print(f"Saved '{name}': {pose}")
    return 0


# =============================================================================
# Button Management Functions
# =============================================================================

def teach_list(args: argparse.Namespace) -> int:
    """
    List all taught button positions and their metadata.
    
    Args:
        args: Command line arguments (unused in this function)
        
    Returns:
        0 on success
        
    Note: Displays each button's name, pose coordinates, and metadata in a readable format
    """
    store = load_store(args.store)
    if not store:
        print("No taught positions.")
        return 0
    for key, val in store.items():
        p = val.get("pose", {})
        m = val.get("meta", {})
        print(f"{key}: pose={{x:{p.get('x'):.4f}, y:{p.get('y'):.4f}, z:{p.get('z'):.4f}, rx:{p.get('rx'):.3f}, ry:{p.get('ry'):.3f}, rz:{p.get('rz'):.3f}}} meta={m}")
    return 0


def teach_delete(args: argparse.Namespace) -> int:
    """
    Delete a taught button position from storage.
    
    Args:
        args: Command line arguments containing the button name to delete
        
    Returns:
        0 on success, 1 if button not found
        
    Note: This permanently removes the button data from the JSON file
    """
    store = load_store(args.store)
    if args.name not in store:
        print(f"Not found: {args.name}")
        return 1
    del store[args.name]
    save_store(args.store, store)
    print(f"Deleted '{args.name}'.")
    return 0


# =============================================================================
# Button Execution Functions
# =============================================================================

def teach_go(args: argparse.Namespace) -> int:
    """
    Execute a taught button press sequence with optional vision verification.
    
    Args:
        args: Command line arguments containing button name and execution parameters
        
    Returns:
        0 on success, 1 if button not found
        
    Note: This function performs the complete press sequence:
    1. Move to approach position (offset from base pose)
    2. Move to base position (exact taught pose)
    3. Move to press position (pressed down)
    4. Return to base position
    5. Return to approach position
    
    If --verify-vision is enabled, it checks if ROI is configured for the button.
    """
    store = load_store(args.store)
    if args.name not in store:
        print(f"Not found: {args.name}")
        return 1

    entry = store[args.name]
    base_pose: Dict[str, float] = entry["pose"]
    meta = entry.get("meta", {})

    axis: str = args.axis or meta.get("axis", "z")
    approach_offset_m: float = args.approach if args.approach is not None else meta.get("approach_offset_m", 0.03)
    press_depth_m: float = args.press if args.press is not None else meta.get("press_depth_m", 0.005)
    accel: float = args.accel if args.accel is not None else meta.get("accel", np.pi)
    vel: float = args.vel if args.vel is not None else meta.get("vel", np.pi)

    approach_pose = with_offset(base_pose, axis, +approach_offset_m)
    press_pose = with_offset(base_pose, axis, -press_depth_m)

    lebai = connect_arm(args.ip)

    def handle_sigint(_sig, _frm):
        """Handle Ctrl+C interruption by safely shutting down the robot"""
        try:
            lebai.stop_sys()
        finally:
            sys.exit(130)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        # Vision verification pre-check
        if args.verify_vision:
            roi_data = entry.get("roi")
            if not roi_data:
                print("Warning: --verify-vision requested but no ROI set for this button")
                print("Use 'teach.py roi set' to configure ROI first")
            else:
                print(f"Vision verification enabled for {args.name}")
                # Note: Full vision verification requires camera access
                # For now, just check if ROI is configured
                print(f"ROI configured: x={roi_data['x']}, y={roi_data['y']}, w={roi_data['w']}, h={roi_data['h']}")

        # Execute press sequence
        movej_and_wait(lebai, approach_pose, accel, vel)
        movej_and_wait(lebai, base_pose, accel, vel)
        movej_and_wait(lebai, press_pose, accel, vel)
        movej_and_wait(lebai, base_pose, accel, vel)
        movej_and_wait(lebai, approach_pose, accel, vel)
        
        print(f"Executed '{args.name}' with axis={axis}, approach={approach_offset_m}m, press={press_depth_m}m")
        
        if args.verify_vision:
            print("Note: Use press_server.py for full vision verification with camera")
            
    finally:
        shutdown_arm(lebai)

    return 0


# =============================================================================
# Teach Mode Functions
# =============================================================================

def teach_mode_cmd(args: argparse.Namespace) -> int:
    """
    Enable teach mode for manual robot positioning.
    
    Args:
        args: Command line arguments (unused in this function)
        
    Returns:
        0 on success
        
    Note: This function enables teach mode so you can manually move the robot
    by hand. Press Enter to exit teach mode and shut down the robot.
    """
    lebai = connect_arm(args.ip)
    try:
        try:
            lebai.teach_mode()
        except Exception:
            # If teach_mode not available, at least keep the system started
            pass
        print("Teach mode enabled. Move the arm by hand. Press ENTER to exit.")
        input()
    finally:
        shutdown_arm(lebai)
    return 0


# =============================================================================
# ROI Management Functions
# =============================================================================

def roi_set(args: argparse.Namespace) -> int:
    """
    Set ROI (Region of Interest) parameters for a button to enable vision-based state detection.
    
    Args:
        args: Command line arguments containing ROI coordinates and threshold
        
    Returns:
        0 on success, 1 if button not found
        
    Note: ROI defines a rectangular area in the camera image where the button is located.
    The system monitors brightness in this area to determine if the button is pressed.
    """
    store = load_store(args.store)
    name = args.name
    if name not in store:
        print(f"Error: name '{name}' not found. Save a position first.")
        return 1

    store[name]["roi"] = {
        "x": args.x,
        "y": args.y,
        "w": args.w,
        "h": args.h,
        "threshold": args.threshold
    }
    save_store(args.store, store)
    print(f"Set ROI for '{name}': x={args.x}, y={args.y}, w={args.w}, h={args.h}, threshold={args.threshold}")
    return 0


def roi_show(args: argparse.Namespace) -> int:
    """
    Display ROI settings for a specific button.
    
    Args:
        args: Command line arguments containing the button name
        
    Returns:
        0 on success, 1 if button not found
        
    Note: Shows the ROI coordinates and brightness threshold for the specified button
    """
    store = load_store(args.store)
    name = args.name
    if name not in store:
        print(f"Error: name '{name}' not found.")
        return 1

    roi = store[name].get("roi")
    if not roi:
        print(f"No ROI set for '{name}'")
        return 0

    print(f"ROI for '{name}': x={roi['x']}, y={roi['y']}, w={roi['w']}, h={roi['h']}, threshold={roi['threshold']}")
    return 0


def roi_list(args: argparse.Namespace) -> int:
    """
    List ROI settings for all buttons.
    
    Args:
        args: Command line arguments (unused in this function)
        
    Returns:
        0 on success
        
    Note: Displays ROI information for all buttons that have ROI configured,
    and indicates which buttons don't have ROI set
    """
    store = load_store(args.store)
    if not store:
        print("No taught positions.")
        return 0

    for key, val in store.items():
        roi = val.get("roi")
        if roi:
            print(f"{key}: ROI(x={roi['x']}, y={roi['y']}, w={roi['w']}, h={roi['h']}, thresh={roi['threshold']:.3f})")
        else:
            print(f"{key}: No ROI set")
    return 0


# =============================================================================
# Command Line Interface Setup
# =============================================================================

def build_parser() -> argparse.ArgumentParser:
    """
    Build the command line argument parser with all available subcommands.
    
    Returns:
        Configured ArgumentParser with all subcommands and their arguments
        
    Note: This function sets up the complete CLI interface including:
    - Button teaching commands (save, savei, list, delete, go)
    - Teach mode command
    - ROI management commands (set, show, list)
    - All necessary command line arguments and help text
    """
    parser = argparse.ArgumentParser(description="Teach-and-Repeat for Lebai robot (no camera)")
    parser.add_argument("--store", default=DEFAULT_STORE_PATH, help="Path to taught positions JSON store")
    parser.add_argument("--ip", default=DEFAULT_LEBAI_IP, help="Lebai robot IP")

    subparsers = parser.add_subparsers(dest="cmd", required=True)

    # Button teaching subcommands
    p_save = subparsers.add_parser("save", help="Save current TCP pose as a named button")
    p_save.add_argument("name", help="Name of this taught position, e.g., BTN_1")
    p_save.add_argument("--axis", default="z", choices=["x", "y", "z"], help="Panel normal axis in base frame")
    p_save.add_argument("--approach", type=float, default=0.03, help="Approach offset along axis (meters)")
    p_save.add_argument("--press", type=float, default=0.005, help="Press depth along -axis (meters)")
    p_save.add_argument("--accel", type=float, default=float(np.pi), help="Joint accel (rad/s^2)")
    p_save.add_argument("--vel", type=float, default=float(np.pi), help="Joint vel (rad/s)")
    p_save.add_argument("--overwrite", action="store_true", help="Overwrite existing entry")
    p_save.set_defaults(func=teach_save)

    p_savei = subparsers.add_parser("savei", help="Interactive save with teach mode enabled (move by hand, then ENTER)")
    p_savei.add_argument("name", help="Name of this taught position, e.g., BTN_1")
    p_savei.add_argument("--axis", default="z", choices=["x", "y", "z"], help="Panel normal axis in base frame")
    p_savei.add_argument("--approach", type=float, default=0.03, help="Approach offset along axis (meters)")
    p_savei.add_argument("--press", type=float, default=0.005, help="Press depth along -axis (meters)")
    p_savei.add_argument("--accel", type=float, default=float(np.pi), help="Joint accel (rad/s^2)")
    p_savei.add_argument("--vel", type=float, default=float(np.pi), help="Joint vel (rad/s)")
    p_savei.add_argument("--overwrite", action="store_true", help="Overwrite existing entry")
    p_savei.set_defaults(func=teach_save_interactive)

    p_list = subparsers.add_parser("list", help="List taught positions")
    p_list.set_defaults(func=teach_list)

    p_del = subparsers.add_parser("delete", help="Delete a taught position")
    p_del.add_argument("name", help="Name to delete")
    p_del.set_defaults(func=teach_delete)

    p_go = subparsers.add_parser("go", help="Execute a taught position (approach-press-return)")
    p_go.add_argument("name", help="Name to execute")
    p_go.add_argument("--axis", choices=["x", "y", "z"], help="Override axis for this run")
    p_go.add_argument("--approach", type=float, help="Override approach offset (m)")
    p_go.add_argument("--press", type=float, help="Override press depth (m)")
    p_go.add_argument("--accel", type=float, help="Override joint accel (rad/s^2)")
    p_go.add_argument("--vel", type=float, help="Override joint vel (rad/s)")
    p_go.add_argument("--verify-vision", action="store_true", help="Enable vision verification (requires ROI)")
    p_go.add_argument("--verify-probe", action="store_true", help="Enable probe verification (experimental)")
    p_go.set_defaults(func=teach_go)

    p_teach = subparsers.add_parser("teach", help="Enable teach mode, wait for ENTER to exit")
    p_teach.set_defaults(func=teach_mode_cmd)

    # ROI management subcommands
    roi_parser = subparsers.add_parser("roi", help="Manage ROI for button state detection")
    roi_subparsers = roi_parser.add_subparsers(dest="roi_cmd", required=True)

    p_roi_set = roi_subparsers.add_parser("set", help="Set ROI for a button")
    p_roi_set.add_argument("name", help="Button name")
    p_roi_set.add_argument("--x", type=int, required=True, help="ROI x coordinate (pixels)")
    p_roi_set.add_argument("--y", type=int, required=True, help="ROI y coordinate (pixels)")
    p_roi_set.add_argument("--w", type=int, required=True, help="ROI width (pixels)")
    p_roi_set.add_argument("--h", type=int, required=True, help="ROI height (pixels)")
    p_roi_set.add_argument("--threshold", type=float, required=True, help="Brightness threshold (0.0-1.0)")
    p_roi_set.set_defaults(func=roi_set)

    p_roi_show = roi_subparsers.add_parser("show", help="Show ROI for a button")
    p_roi_show.add_argument("name", help="Button name")
    p_roi_show.set_defaults(func=roi_show)

    p_roi_list = roi_subparsers.add_parser("list", help="List all ROIs")
    p_roi_list.set_defaults(func=roi_list)

    return parser


# =============================================================================
# Main Entry Point
# =============================================================================

def main() -> int:
    """
    Main entry point for the teach-and-repeat CLI application.
    
    Returns:
        Exit code (0 for success, non-zero for errors)
        
    Note: This function parses command line arguments, calls the appropriate
    function based on the command, and handles keyboard interrupts gracefully
    """
    parser = build_parser()
    args = parser.parse_args()
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())


