## ROI-Based Button State Detection Guide

This guide explains how to use the new ROI (Region of Interest) system to detect button states before and after pressing.

### What is ROI?
ROI (Region of Interest) is a rectangular area in the camera image that contains a button. The system monitors the brightness in this area to determine if the button is pressed (LED is on) or not pressed.

### Setup Workflow

#### 1. Teach Button Position
First, teach the button position as usual:
```bash
python teach.py savei F1 --axis z --approach 0.03 --press 0.005
```

#### 2. Set ROI for Button State Detection
After teaching, set the ROI coordinates and brightness threshold:
```bash
python teach.py roi set F1 --x 100 --y 150 --w 50 --h 50 --threshold 0.6
```
- `--x, --y`: Top-left corner of ROI in pixels
- `--w, --h`: Width and height of ROI in pixels
- `--threshold`: Brightness threshold (0.0-1.0) to determine if button is pressed

#### 3. View ROI Settings
Check ROI configuration:
```bash
python teach.py roi show F1
```

#### 4. List All ROIs
See all configured ROIs:
```bash
python teach.py roi list
```

### Using Vision Verification

#### Option 1: Basic Verification (teach.py)
Check if ROI is configured before pressing:
```bash
python teach.py go F1 --verify-vision
```

#### Option 2: Full Vision Verification (press_server.py)
For complete vision verification with camera:
```bash
python press_server.py --store taught_positions.json --ip 192.168.10.200
```

Then use HTTP endpoints:
- Check button status: `GET /status/F1`
- Press button with verification: `POST /press/F1`

### How ROI Detection Works

1. **Pre-check**: Before pressing, camera captures image and checks ROI brightness
   - If brightness > threshold: Button already pressed, skip
   - If brightness ≤ threshold: Button not pressed, proceed

2. **Press Execution**: Standard approach → press → return sequence

3. **Confirmation**: After pressing, camera checks ROI brightness again
   - If brightness > threshold: Press successful
   - If brightness ≤ threshold: Press may have failed

### Tips for Setting ROI

- **ROI Size**: Make ROI large enough to capture the entire button LED area
- **Threshold**: Start with 0.5-0.7, adjust based on lighting conditions
- **Position**: Use image viewer to find exact pixel coordinates
- **Testing**: Test ROI detection before running full press sequence

### Example Complete Workflow

```bash
# 1. Teach button position
python teach.py savei F1 --axis z --approach 0.03 --press 0.005

# 2. Set ROI (adjust coordinates based on your camera view)
python teach.py roi set F1 --x 100 --y 150 --w 50 --h 50 --threshold 0.6

# 3. Verify ROI configuration
python teach.py roi show F1

# 4. Test with vision verification
python teach.py go F1 --verify-vision

# 5. For full camera verification, use press server
python press_server.py
# Then: curl -X POST http://localhost:2000/press/F1
```

### Troubleshooting

- **"No ROI set"**: Use `teach.py roi set` to configure ROI first
- **Detection unreliable**: Adjust threshold or ROI size/position
- **Camera not working**: Check camera device path in press_server.py
- **Button not found**: Ensure button name matches exactly

### Safety Notes

- Always test ROI detection before running full press sequences
- Start with conservative thresholds and adjust gradually
- Ensure good lighting conditions for reliable detection
- Backup your `taught_positions.json` file regularly
