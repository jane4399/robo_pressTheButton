# Button Detection Learning Guide

This guide explains how to use the new learning-based button detection system that automatically adapts to your specific buttons, LED colors, and lighting conditions.

## Why Use Learning-Based Detection?

### **Problems with Fixed Thresholds**
- ❌ Different LED colors need different thresholds
- ❌ Lighting changes affect detection accuracy
- ❌ Manual threshold tuning is time-consuming
- ❌ Inconsistent results across different conditions

### **Benefits of Learning-Based Detection**
- ✅ **Works with any LED color** (red, orange, green, blue, white)
- ✅ **Adapts to your lighting** conditions automatically
- ✅ **No manual threshold tuning** needed
- ✅ **More reliable detection** in varying conditions
- ✅ **Learns from your specific setup**

## Complete Setup Workflow

### **Step 1: Install Fixed Camera**
Mount a camera in a **fixed position** that always sees your elevator buttons. The camera should NOT move with the robot arm.

**Recommended positions:**
- Above the elevator panel (30-60cm)
- Fixed to the wall or ceiling
- Consistent lighting conditions

### **Step 2: Teach Button Positions**
```bash
# Teach each button position
python teach.py savei F1 --axis z --approach 0.03 --press 0.005
python teach.py savei F2 --axis z --approach 0.03 --press 0.005
python teach.py savei OPEN --axis z --approach 0.03 --press 0.005
```

### **Step 3: Set ROI for Each Button**
```bash
# Set ROI coordinates (adjust x, y, w, h based on your camera view)
python teach.py roi set F1 --x 100 --y 150 --w 50 --h 50 --threshold 0.6
python teach.py roi set F2 --x 200 --y 150 --w 50 --h 50 --threshold 0.6
python teach.py roi set OPEN --x 300 --y 150 --w 50 --h 50 --threshold 0.6
```

**ROI Guidelines:**
- **Size**: Make ROI large enough to capture the entire button LED area
- **Position**: Center ROI on the button
- **Threshold**: Start with 0.6 (will be replaced by learning)

### **Step 4: Learn Button Detection**
```bash
# Learn all buttons at once
python learn_buttons.py

# Or learn specific buttons
python learn_buttons.py --button F1
python learn_buttons.py --button F2
```

**Learning Process:**
1. **Ensure button is OFF** (LED not lit)
2. **Press Enter** when ready
3. **Press the button** (turn on LED)
4. **Press Enter** when LED is lit
5. **System learns automatically**

### **Step 5: Run with Learned Detection**
```bash
# Start the press server
python press_server.py

# The server automatically uses learned detection when available
```

## Learning Script Commands

### **Learn All Buttons**
```bash
python learn_buttons.py
```
- Automatically detects buttons with ROI configured
- Guides you through learning each button
- Saves all learned thresholds to `learned_thresholds.json`

### **Learn Specific Button**
```bash
python learn_buttons.py --button F1
```
- Learn detection for a single button
- Useful for adding new buttons or re-learning existing ones

### **List Configured Buttons**
```bash
python learn_buttons.py --list
```
- Shows all buttons with ROI configured
- Helps verify setup before learning

### **Check Learning Status**
```bash
python learn_buttons.py --status
```
- Shows which buttons have been learned
- Displays learned threshold values
- Identifies buttons that need learning

## How Learning Works

### **What the System Learns**
For each button, the system learns:

1. **OFF State Baseline**
   - Brightness when LED is off
   - Saturation when LED is off

2. **ON State Baseline**
   - Brightness when LED is on
   - Saturation when LED is on

3. **Detection Thresholds**
   - Brightness difference (ON - OFF)
   - Saturation difference (ON - OFF)

### **Detection Algorithm**
```python
# When checking button state:
current_brightness = measure_roi_brightness()
current_saturation = measure_roi_saturation()

# Calculate changes from OFF baseline
brightness_change = current_brightness - off_baseline_brightness
saturation_change = current_saturation - off_baseline_saturation

# Button is ON if changes exceed learned thresholds
brightness_ok = brightness_change > (learned_brightness_diff * 0.7)
saturation_ok = abs(saturation_change) > (learned_saturation_diff * 0.7)

return brightness_ok or saturation_ok
```

### **Why This Works Better**
- **Relative detection**: Compares to baseline, not absolute values
- **Color-aware**: Considers both brightness and saturation changes
- **Adaptive**: Automatically adjusts to your specific setup
- **Robust**: Works with lighting variations

## Troubleshooting

### **Common Issues**

#### **"No ROI configured" Error**
```bash
# Solution: Set ROI first
python teach.py roi set F1 --x 100 --y 150 --w 50 --h 50 --threshold 0.6
```

#### **"Button not found" Error**
```bash
# Solution: Teach button position first
python teach.py savei F1 --axis z --approach 0.03 --press 0.005
```

#### **Learning Fails**
- Ensure camera can see the button clearly
- Check that ROI coordinates are correct
- Verify button LED actually turns on/off
- Try adjusting ROI size or position

#### **Poor Detection After Learning**
- Re-run learning with better lighting conditions
- Ensure ROI covers the entire LED area
- Check for camera movement or focus issues

### **Debug Mode**
```bash
# Enable debug output to see detection values
python press_server.py --debug
```

## Advanced Usage

### **Re-learning Buttons**
```bash
# Re-learn a specific button (overwrites existing learning)
python learn_buttons.py --button F1
```

### **Custom Store File**
```bash
# Use different button position file
python learn_buttons.py --store my_buttons.json
```

### **Learning Without Robot**
The learning script doesn't require robot connection, so you can set up detection before connecting the robot.

## File Structure

After learning, you'll have:

```
project/
├── taught_positions.json      # Button positions and ROIs
├── learned_thresholds.json    # Learned detection values
├── press_server.py            # Enhanced server with learning
├── learn_buttons.py           # Learning script
└── docs/
    └── learning_guide.md      # This guide
```

## Example Complete Session

```bash
# 1. Check what's configured
python learn_buttons.py --status

# 2. Learn all buttons
python learn_buttons.py

# 3. Verify learning status
python learn_buttons.py --status

# 4. Run server with learned detection
python press_server.py

# 5. Test detection
curl http://localhost:2000/status/F1
curl -X POST http://localhost:2000/press/F1
```

## Tips for Best Results

1. **Consistent Setup**: Use the same camera position and lighting for learning and operation
2. **Clear ROI**: Ensure ROI covers the entire button LED area
3. **Good Lighting**: Learn in lighting conditions similar to normal operation
4. **Stable Camera**: Prevent camera movement during learning and operation
5. **Test Thoroughly**: Verify detection works before running the full system

The learning system makes button detection **much more reliable** and **tailored to your specific setup**! 🎯
