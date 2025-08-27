## Teach-and-Repeat Quick Guide (Lebai)

This guide shows how to save, list, execute, and re-teach button poses without using the camera.

### Prerequisites
- Python 3.10+
- Lebai SDK installed:
```bash
python -m pip install lebai-sdk
```
- Default robot IP: `192.168.10.200` (override with `--ip <addr>`)

---

### Enable teach mode (move arm by hand)
```bash
python teach.py teach
```
- Starts system and enables teach mode; press ENTER to exit.

### Save a button pose (interactive, recommended)
Move the arm by hand to the desired press pose, then save:
```bash
python teach.py savei <NAME> --axis z --approach 0.03 --press 0.005
# Example:
python teach.py savei F1 --axis z --approach 0.03 --press 0.005
```
- `--axis`: panel normal direction in base frame (x|y|z)
- `--approach`: approach distance (m), the arm backs off by this much for approach
- `--press`: press depth (m), the arm pushes forward by this much to press
- Add `--overwrite` to replace an existing name

### Save current pose (non-interactive)
If the arm is already at the press pose and you just want to record it:
```bash
python teach.py save <NAME> --axis z --approach 0.03 --press 0.005
```

### List saved poses
```bash
python teach.py list
```

### Execute a saved pose (approach → press → return)
```bash
python teach.py go <NAME>
# Override on the fly if needed
python teach.py go <NAME> --axis z --approach 0.04 --press 0.006
```

### Delete a saved pose
```bash
python teach.py delete <NAME>
```

### Re-teach (overwrite)
```bash
# Interactive re-teach
python teach.py savei <NAME> --axis z --approach 0.03 --press 0.005 --overwrite
# Or save current pose and overwrite
python teach.py save <NAME> --axis z --approach 0.03 --press 0.005 --overwrite
```

---

### Tips and Troubleshooting
- Verify base pose first (no press):
```bash
python teach.py go <NAME> --press 0
```
- If the press ends too high/low, the press axis may be wrong. Try x/y/z to find the panel normal, then set `--axis` accordingly.
- If press depth is insufficient, increase `--press` (e.g., `0.010` or `0.015`).
- Keep approach > 0 to ensure a safe approach/retreat path.
- Store file: `taught_positions.json` (in repo root). Back it up as needed.
- Robot IP override: add `--ip <addr>` to any command.

### Safety
- Start with low speeds/accelerations; ensure workspace is clear.
- The robot base must repeatedly stop at the same location/orientation for taught points to remain accurate.
