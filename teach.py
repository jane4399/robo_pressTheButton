import argparse
import json
import signal
import sys
from copy import deepcopy
from typing import Any, Dict, Tuple

import lebai_sdk
import numpy as np


DEFAULT_STORE_PATH = "taught_positions.json"
DEFAULT_LEBAI_IP = "192.168.10.200"


def load_store(store_path: str) -> Dict[str, Any]:
    try:
        with open(store_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


def save_store(store_path: str, data: Dict[str, Any]) -> None:
    with open(store_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def connect_arm(lebai_ip: str):
    lebai_sdk.init()
    lebai = lebai_sdk.connect(lebai_ip, False)
    lebai.start_sys()
    return lebai


def shutdown_arm(lebai) -> None:
    try:
        lebai.stop_sys()
    except Exception:
        pass


def get_current_tcp_pose(lebai) -> Dict[str, float]:
    kin = lebai.get_kin_data()
    return {
        "x": float(kin["actual_tcp_pose"]["x"]),
        "y": float(kin["actual_tcp_pose"]["y"]),
        "z": float(kin["actual_tcp_pose"]["z"]),
        "rx": float(kin["actual_tcp_pose"]["rx"]),
        "ry": float(kin["actual_tcp_pose"]["ry"]),
        "rz": float(kin["actual_tcp_pose"]["rz"]),
    }


def with_offset(pose: Dict[str, float], axis: str, delta_m: float) -> Dict[str, float]:
    if axis not in {"x", "y", "z"}:
        raise ValueError("axis must be one of {'x','y','z'}")
    new_pose = deepcopy(pose)
    new_pose[axis] = float(new_pose[axis] + delta_m)
    return new_pose


def movej_and_wait(lebai, pose: Dict[str, float], accel: float, vel: float) -> None:
    lebai.movej(pose, accel, vel)
    lebai.wait_move()


def teach_save(args: argparse.Namespace) -> int:
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


def teach_list(args: argparse.Namespace) -> int:
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
    store = load_store(args.store)
    if args.name not in store:
        print(f"Not found: {args.name}")
        return 1
    del store[args.name]
    save_store(args.store, store)
    print(f"Deleted '{args.name}'.")
    return 0


def teach_go(args: argparse.Namespace) -> int:
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
        try:
            lebai.stop_sys()
        finally:
            sys.exit(130)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        movej_and_wait(lebai, approach_pose, accel, vel)
        movej_and_wait(lebai, base_pose, accel, vel)
        movej_and_wait(lebai, press_pose, accel, vel)
        movej_and_wait(lebai, base_pose, accel, vel)
        movej_and_wait(lebai, approach_pose, accel, vel)
        print(f"Executed '{args.name}' with axis={axis}, approach={approach_offset_m}m, press={press_depth_m}m")
    finally:
        shutdown_arm(lebai)

    return 0


def teach_mode_cmd(args: argparse.Namespace) -> int:
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Teach-and-Repeat for Lebai robot (no camera)")
    parser.add_argument("--store", default=DEFAULT_STORE_PATH, help="Path to taught positions JSON store")
    parser.add_argument("--ip", default=DEFAULT_LEBAI_IP, help="Lebai robot IP")

    subparsers = parser.add_subparsers(dest="cmd", required=True)

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
    p_go.set_defaults(func=teach_go)

    p_teach = subparsers.add_parser("teach", help="Enable teach mode, wait for ENTER to exit")
    p_teach.set_defaults(func=teach_mode_cmd)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())


