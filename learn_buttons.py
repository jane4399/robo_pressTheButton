#!/usr/bin/env python3
"""
Button Detection Learning Script

This script helps you set up learned button detection thresholds for more reliable
button state detection that works with any LED color and lighting conditions.

Usage:
    python learn_buttons.py                    # Learn all buttons
    python learn_buttons.py --button F1        # Learn specific button
    python learn_buttons.py --list             # List buttons with ROI
    python learn_buttons.py --status           # Show learning status
"""

import argparse
import json
import sys
from press_server import ButtonPressServer, LEARNED_THRESHOLDS_FILE


def list_buttons_with_roi(press_server):
    """List all buttons that have ROI configured."""
    buttons_with_roi = []
    for name in press_server.store.keys():
        if press_server.store[name].get("roi"):
            buttons_with_roi.append(name)
    
    if not buttons_with_roi:
        print("No buttons with ROI configured found.")
        print("\nTo configure ROI for a button:")
        print("1. First teach the button position:")
        print("   python teach.py savei <BUTTON_NAME> --axis z --approach 0.03 --press 0.005")
        print("2. Then set the ROI:")
        print("   python teach.py roi set <BUTTON_NAME> --x <X> --y <Y> --w <WIDTH> --h <HEIGHT> --threshold 0.6")
        return
    
    print(f"Found {len(buttons_with_roi)} buttons with ROI configured:")
    for i, name in enumerate(buttons_with_roi, 1):
        roi = press_server.store[name]["roi"]
        print(f"  {i}. {name}: ROI({roi['x']}, {roi['y']}, {roi['w']}, {roi['h']})")
    
    print("\nTo learn detection for these buttons:")
    print("  python learn_buttons.py                    # Learn all buttons")
    print("  python learn_buttons.py --button <NAME>    # Learn specific button")


def show_learning_status(press_server):
    """Show current learning status for all buttons."""
    print("=== Button Detection Learning Status ===")
    
    all_buttons = set(press_server.store.keys())
    buttons_with_roi = {name for name in all_buttons if press_server.store[name].get("roi")}
    buttons_learned = set(press_server.learned_thresholds.keys())
    
    print(f"Total buttons: {len(all_buttons)}")
    print(f"Buttons with ROI: {len(buttons_with_roi)}")
    print(f"Buttons with learned detection: {len(buttons_learned)}")
    
    if buttons_with_roi:
        print("\nROI Status:")
        for name in sorted(buttons_with_roi):
            roi = press_server.store[name]["roi"]
            learned = "✓ Learned" if name in buttons_learned else "✗ Not learned"
            print(f"  {name}: ROI({roi['x']}, {roi['y']}, {roi['w']}, {roi['h']}) - {learned}")
    
    if buttons_learned:
        print("\nLearned Detection Details:")
        for name in sorted(buttons_learned):
            learned = press_server.learned_thresholds[name]
            print(f"  {name}:")
            print(f"    Brightness diff: {learned['brightness_diff']:.3f}")
            print(f"    Saturation diff: {learned['saturation_diff']:.3f}")
            print(f"    OFF baseline: brightness={learned['off_baseline']['brightness']:.3f}, saturation={learned['off_baseline']['saturation']:.3f}")
            print(f"    ON baseline: brightness={learned['on_baseline']['brightness']:.3f}, saturation={learned['on_baseline']['saturation']:.3f}")
    
    if not buttons_with_roi:
        print("\nNo buttons configured for learning.")
        print("Follow these steps:")
        print("1. Teach button positions: python teach.py savei <NAME> --axis z --approach 0.03 --press 0.005")
        print("2. Set ROI coordinates: python teach.py roi set <NAME> --x <X> --y <Y> --w <WIDTH> --h <HEIGHT> --threshold 0.6")
        print("3. Run learning: python learn_buttons.py")


def main():
    """Main entry point for the learning script."""
    parser = argparse.ArgumentParser(
        description="Learn button detection thresholds for reliable state detection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python learn_buttons.py                    # Learn all buttons with ROI
  python learn_buttons.py --button F1        # Learn specific button
  python learn_buttons.py --list             # List configured buttons
  python learn_buttons.py --status           # Show learning status

Learning Process:
  1. Ensure button is NOT pressed (LED off)
  2. Press Enter when ready
  3. Press the button (turn on LED)
  4. Press Enter when LED is lit
  5. System learns the difference automatically
        """
    )
    
    parser.add_argument("--button", type=str, help="Learn detection for specific button")
    parser.add_argument("--list", action="store_true", help="List buttons with ROI configured")
    parser.add_argument("--status", action="store_true", help="Show learning status for all buttons")
    parser.add_argument("--store", default="taught_positions.json", help="Path to taught positions JSON")
    parser.add_argument("--ip", default="192.168.10.200", help="Lebai robot IP (not needed for learning)")
    
    args = parser.parse_args()
    
    try:
        # Create press server instance (no robot connection needed for learning)
        press_server = ButtonPressServer(args.store, args.ip)
        
        # Handle different modes
        if args.list:
            list_buttons_with_roi(press_server)
            return 0
        
        if args.status:
            show_learning_status(press_server)
            return 0
        
        if args.button:
            # Learn specific button
            if args.button not in press_server.store:
                print(f"Error: Button '{args.button}' not found")
                return 1
            
            if not press_server.store[args.button].get("roi"):
                print(f"Error: Button '{args.button}' has no ROI configured")
                print("Use 'teach.py roi set' to configure ROI first")
                return 1
            
            print(f"=== Learning Detection for Button: {args.button} ===")
            if press_server.learn_button_detection(args.button):
                print(f"\n✓ Learning completed successfully for {args.button}!")
                print(f"Learned thresholds saved to {LEARNED_THRESHOLDS_FILE}")
            else:
                print(f"\n✗ Learning failed for {args.button}")
                return 1
        else:
            # Learn all buttons
            print("=== Learning Detection for All Buttons ===")
            press_server.learn_all_buttons()
            print(f"\n✓ Learning process complete!")
            print(f"Learned thresholds saved to {LEARNED_THRESHOLDS_FILE}")
        
        print("\nYou can now run the press server with learned detection:")
        print("  python press_server.py")
        
        return 0
        
    except KeyboardInterrupt:
        print("\nLearning interrupted by user")
        return 130
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
