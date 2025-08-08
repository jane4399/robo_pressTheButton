import cv2
import numpy as np
import glob

# === Step 1: Setup parameters ===
chessboard_size = (9, 6)
square_size = 25.0  # mm

# Prepare known 3D chessboard coordinates
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0],
                       0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Load robot poses (camera mount in robot TCP)
arm_poses = np.load("arm_data.npy", allow_pickle=True)  # shape: (N, 6)
images = sorted(glob.glob('handeye_images/*.jpg'))  # Must be same order as arm poses

assert len(images) == len(arm_poses), "Mismatch between image and arm data count"

camera_matrix = np.array([
    [595.90456391, 0., 304.51916571],
    [0., 596.38629532, 216.41537217],
    [0., 0., 1.],
], dtype=np.float64)

distortion_coefficients = np.array([
     [-1.14468302e-01, 7.69562282e-01, -5.75281533e-03, 4.90092555e-04, -1.83338907e+00]
], dtype=np.float64)

# Load intrinsics (optional, skip if not needed)
# camera_matrix = ...
# dist_coeffs = ...

R_target2cam = []
t_target2cam = []
R_base2gripper = []
t_base2gripper = []

for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if not ret:
        print(f"Skipping image {fname}, corners not found.")
        continue

    # Estimate camera-to-target pose (i.e., chessboard pose in camera coords)
    success, rvec, tvec = cv2.solvePnP(objp, corners, cameraMatrix=camera_matrix,
                                       distCoeffs=distortion_coefficients, flags=cv2.SOLVEPNP_ITERATIVE)

    R_cam, _ = cv2.Rodrigues(rvec)
    R_target2cam.append(R_cam)
    t_target2cam.append(tvec)

    # Convert robot pose to base->gripper transform
    pose = arm_poses[idx]
    x, y, z, rx, ry, rz = pose['x'], pose['y'], pose['z'], pose['rx'], pose['ry'], pose['rz']
    print(x, y, z, rx, ry, rz)
    R_gripper, _ = cv2.Rodrigues(np.array([rx, ry, rz]))
    t_gripper = np.array([[x*1000], [y*1000], [z*1000]])

    R_base2gripper.append(R_gripper)
    t_base2gripper.append(t_gripper)

# === Step 4: Perform hand-eye calibration ===
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_base2gripper, t_base2gripper,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI  # or use other methods like DANIELS, PARK, etc.
)

print("Hand-eye rotation:\n", R_cam2gripper)
print("Hand-eye translation:\n", t_cam2gripper)

# === Optional: Save result ===
np.savez("handeye_result.npz", 
         R=R_cam2gripper, 
         t=t_cam2gripper)
