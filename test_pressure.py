#!/usr/bin/env python3
"""
Pressure Sensor Test Script

This script helps you test and calibrate the pressure sensor on your Lebai robot.
It can be used to verify pressure sensor functionality and adjust thresholds.

Usage:
    python test_pressure.py                    # Test pressure sensor readings
    python test_pressure.py --calibrate       # Calibrate pressure sensor
    python test_pressure.py --threshold 0.3   # Test with custom threshold
    python test_pressure.py --continuous      # Continuous pressure monitoring
"""

import argparse
import time
import sys
from press_server import ButtonPressServer, PRESSURE_THRESHOLD


def test_pressure_readings(press_server, duration: int = 10):
    """
    Test pressure sensor readings for a specified duration.
    
    Args:
        press_server: ButtonPressServer instance
        duration: Test duration in seconds
    """
    print(f"=== Testing Pressure Sensor Readings ({duration}s) ===")
    print("Ensure robot is in neutral position with no external forces.")
    print("Press Enter to start...")
    input()
    
    readings = []
    start_time = time.time()
    
    try:
        while time.time() - start_time < duration:
            pressure = press_server.read_pressure_sensor()
            if pressure is not None:
                readings.append(pressure)
                print(f"  {time.time() - start_time:6.1f}s: {pressure:8.3f} N")
            else:
                print(f"  {time.time() - start_time:6.1f}s: No reading")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    if readings:
        print(f"\nTest Results:")
        print(f"  Total readings: {len(readings)}")
        print(f"  Min pressure: {min(readings):.3f} N")
        print(f"  Max pressure: {max(readings):.3f} N")
        print(f"  Mean pressure: {sum(readings)/len(readings):.3f} N")
        print(f"  Std deviation: {calculate_std_deviation(readings):.3f} N")
    else:
        print("No valid readings obtained")


def calculate_std_deviation(values):
    """Calculate standard deviation of a list of values."""
    if len(values) < 2:
        return 0.0
    
    mean = sum(values) / len(values)
    variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
    return variance ** 0.5


def test_pressure_threshold(press_server, threshold: float):
    """
    Test pressure detection with a specific threshold.
    
    Args:
        press_server: ButtonPressServer instance
        threshold: Pressure threshold to test
    """
    print(f"=== Testing Pressure Threshold ({threshold} N) ===")
    print("1. Ensure robot is in neutral position")
    print("2. Press Enter when ready...")
    input()
    
    # Get baseline
    baseline_readings = []
    for i in range(5):
        pressure = press_server.read_pressure_sensor()
        if pressure is not None:
            baseline_readings.append(pressure)
        time.sleep(0.1)
    
    if not baseline_readings:
        print("Error: Could not get baseline readings")
        return
    
    baseline = sum(baseline_readings) / len(baseline_readings)
    print(f"Baseline pressure: {baseline:.3f} N")
    
    print("\n3. Now apply gentle pressure to the end-effector")
    print("4. Press Enter when applying pressure...")
    input()
    
    # Test pressure detection
    contact_detected = 0
    for i in range(10):
        pressure = press_server.read_pressure_sensor()
        if pressure is not None:
            if pressure > (baseline + threshold):
                contact_detected += 1
                print(f"  Reading {i+1}: {pressure:.3f} N - CONTACT DETECTED")
            else:
                print(f"  Reading {i+1}: {pressure:.3f} N - No contact")
        else:
            print(f"  Reading {i+1}: No reading")
        time.sleep(0.1)
    
    print(f"\nThreshold Test Results:")
    print(f"  Threshold: {threshold} N")
    print(f"  Baseline: {baseline:.3f} N")
    print(f"  Contact detections: {contact_detected}/10")
    
    if contact_detected >= 7:
        print("  ✓ Threshold appears suitable")
    elif contact_detected >= 4:
        print("  ⚠ Threshold may need adjustment")
    else:
        print("  ✗ Threshold too high - consider lowering")


def continuous_monitoring(press_server):
    """Continuous pressure monitoring mode."""
    print("=== Continuous Pressure Monitoring ===")
    print("Press Ctrl+C to stop monitoring")
    print("Monitoring pressure readings...")
    
    try:
        while True:
            pressure = press_server.read_pressure_sensor()
            timestamp = time.strftime("%H:%M:%S")
            if pressure is not None:
                print(f"[{timestamp}] Pressure: {pressure:8.3f} N")
            else:
                print(f"[{timestamp}] No reading")
            time.sleep(0.2)
    
    except KeyboardInterrupt:
        print("\nMonitoring stopped")


def main():
    """Main entry point for the pressure sensor test script."""
    parser = argparse.ArgumentParser(
        description="Test and calibrate pressure sensor functionality",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python test_pressure.py                    # Basic pressure test
  python test_pressure.py --calibrate       # Calibrate pressure sensor
  python test_pressure.py --threshold 0.3   # Test specific threshold
  python test_pressure.py --continuous      # Monitor pressure continuously
  python test_pressure.py --duration 20     # Test for 20 seconds

Pressure Sensor Tips:
  - Ensure robot is in neutral position for baseline readings
  - Apply consistent, gentle pressure for testing
  - Adjust threshold based on your specific setup
  - Calibrate in the same conditions as normal operation
        """
    )
    
    parser.add_argument("--calibrate", action="store_true", help="Calibrate pressure sensor")
    parser.add_argument("--threshold", type=float, help="Test specific pressure threshold")
    parser.add_argument("--continuous", action="store_true", help="Continuous pressure monitoring")
    parser.add_argument("--duration", type=int, default=10, help="Test duration in seconds")
    parser.add_argument("--store", default="taught_positions.json", help="Path to taught positions JSON")
    parser.add_argument("--ip", default="192.168.10.200", help="Lebai robot IP")
    
    args = parser.parse_args()
    
    try:
        # Create press server instance
        press_server = ButtonPressServer(args.store, args.ip)
        
        # Handle different modes
        if args.calibrate:
            # Calibrate pressure sensor
            print("=== Pressure Sensor Calibration ===")
            try:
                press_server.connect_arm()
                print(f"Connected to robot at {args.ip}")
                
                if press_server.calibrate_pressure_sensor():
                    print("✓ Pressure sensor calibration completed successfully!")
                else:
                    print("✗ Pressure sensor calibration failed!")
                    return 1
            except Exception as e:
                print(f"Error during calibration: {e}")
                return 1
            finally:
                press_server.shutdown_arm()
        
        elif args.continuous:
            # Continuous monitoring
            try:
                press_server.connect_arm()
                print(f"Connected to robot at {args.ip}")
                continuous_monitoring(press_server)
            except Exception as e:
                print(f"Error during monitoring: {e}")
                return 1
            finally:
                press_server.shutdown_arm()
        
        elif args.threshold:
            # Test specific threshold
            try:
                press_server.connect_arm()
                print(f"Connected to robot at {args.ip}")
                press_server.pressure_threshold = args.threshold
                test_pressure_threshold(press_server, args.threshold)
            except Exception as e:
                print(f"Error during threshold testing: {e}")
                return 1
            finally:
                press_server.shutdown_arm()
        
        else:
            # Basic pressure test
            try:
                press_server.connect_arm()
                print(f"Connected to robot at {args.ip}")
                test_pressure_readings(press_server, args.duration)
            except Exception as e:
                print(f"Error during testing: {e}")
                return 1
            finally:
                press_server.shutdown_arm()
        
        return 0
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        return 130
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
