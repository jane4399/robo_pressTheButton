from time import sleep as wait
import cv2
import lebai_sdk
import numpy as np

import http.server
from http.server import HTTPServer
import socket

LEBAI_IP = "192.168.10.200"
lebai = lebai_sdk.connect(LEBAI_IP, False)

REAL_BUTTON_DIAMETER = 20.0  # mm, adjust to your button

# camera_matrix = np.array([
#     [604.01636706, 0., 304.43305102],
#     [0., 604.16616675, 233.52448302],
#     [0., 0., 1.]
# ], dtype=np.float64)

# distortion_coefficients = np.array([
#     [4.71963742e-02, -5.80544465e-01, 7.13021814e-04, 3.07953427e-04, 1.39332970e+00]
# ], dtype=np.float64)

camera_matrix = np.array([
    [595.90456391, 0., 304.51916571],
    [0., 596.38629532, 216.41537217],
    [0., 0., 1.],
], dtype=np.float64)

distortion_coefficients = np.array([
     [-1.14468302e-01, 7.69562282e-01, -5.75281533e-03, 4.90092555e-04, -1.83338907e+00]
], dtype=np.float64)

R_gripper2camera = np.array([
    [ 0.99261845, -0.09695258, -0.07286154],
    [ 0.10012265, 0.31606229, 0.94344056],
    [-0.06844021, -0.9437716, 0.32343641]
], dtype=np.float64)

T_gripper2camera = np.array([
    [-7.03944204/1000],
    [-195.83453327/1000],
    [-32.45528542/1000]
 ]) 

def reset_arm(lebai):
    lebai.movej([
        -np.pi + np.pi/5,
        np.deg2rad(-165-25),
        np.deg2rad(75+25),
        -np.pi/2,
        -np.pi + (np.pi/2 - np.pi/5),
        -np.pi/2
    ], np.pi, np.pi)

def undistort_frame(frame, camera_matrix, distortion_coefficients):
    w, h = frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, distortion_coefficients, (w, h), 1, (w, h)
    )
    undistorted = cv2.undistort(frame, camera_matrix, distortion_coefficients, None, new_camera_matrix)
    return undistorted

def undistort_pixel(u, v, camera_matrix, distortion_coeffs):
    # Undistort and reproject into pixel space using the original camera matrix
    pts = np.array([[[u, v]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(pts, camera_matrix, distortion_coeffs, P=camera_matrix)
    return undistorted[0][0]

def detect_object(frame):
    # frame = undistort_frame(frame, camera_matrix, distortion_coefficients)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 7)

    detected_circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT_ALT, 1.5, 20,
        param1=50,
        param2=0.75,
        minRadius=25,
        maxRadius=40
    )

    if detected_circles is not None:
        detected_circles = np.uint16(np.around(detected_circles))

        rsum = 0
        num = 0

        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
            rsum += r
            num += 1
            cv2.circle(frame, (a, b), r, (0, 255, 0), 2)
            cv2.circle(frame, (a, b), 1, (0, 0, 255), 3)

        detected_circles[0, :, 2] = rsum/num

        # frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return True, frame, detected_circles

    # frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return False, frame, None

def estimate_distance(observed_diameter_px, axis='x'):
    # Returns distance in mm
    if observed_diameter_px <= 0:
        raise ValueError("object_pixel_size must be > 0")

    if axis == 'x':
        focal_length = camera_matrix[0, 0]  # fx
    elif axis == 'y':
        focal_length = camera_matrix[1, 1]  # fy
    else:
        raise ValueError("axis must be 'x' or 'y'")

    depth = (focal_length * REAL_BUTTON_DIAMETER) / observed_diameter_px
    return depth

def pixel_to_camera_point(u, v, depth, camera_matrix):
    """
    Convert a 2D pixel + depth to a 3D point in camera coordinates.

    Parameters:
        u, v            : Pixel coordinates (image space)
        depth           : Depth value at (u, v) in meters or millimeters
        camera_matrix   : 3x3 numpy array with intrinsics [fx, fy, cx, cy]

    Returns:
        np.array([x, y, z]) in the camera coordinate frame
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # Back-project to 3D
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth

    return np.array([x, y, z])

def get_target_position(lebai, u, v, r):
    # Step 1: Undistort pixel
    print(u,v)
    u, v = undistort_pixel(u, v, camera_matrix, distortion_coefficients)
    print(u,v)

    # Step 2: Back-project to 3D in camera frame
    depth = estimate_distance(r * 2) / 1000.0  # Convert mm to meters
    pos_camera = pixel_to_camera_point(u, v, depth, camera_matrix)

    print(pos_camera)

    cur = lebai.get_kin_data()["actual_tcp_pose"]

    target_position = {
        'x': float(cur['x']+pos_camera[2]-0.05),
        'y': float(cur['y']-pos_camera[1]+0.03245528542),
        'z': float(cur['z']+pos_camera[0]),
        'rx': -np.pi/2,  # Reuse current orientation
        'ry': -np.pi/2,
        'rz': -np.pi/2
    }

    print(target_position)

    return target_position



dimensions = [0, 0]

def get_capture():
    cap = cv2.VideoCapture("/dev/video0")

    if cap is None or not cap.isOpened():
        cap = cv2.VideoCapture("/dev/video1")

    ret, img = cap.read()

    cap.release()
    return ret, img, cap

class LebaiHTTPHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/':
            self.send_error(404)
            return
        
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()

        reset_arm(lebai)
        lebai.wait_move()

        wait(1)

        ret, frame, _ = get_capture()
        if not ret:
            print("no video")
            return

        result, frame, circles = detect_object(frame)

        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            print("failed jpg")
            return

        self.wfile.write(b'--frame\r\n')
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg.tobytes())
        self.wfile.write(b'\r\n')

        if result and circles is not None:
            target = circles[0][0] # TODO: implement selector

            x, y, r = target[0], target[1], target[2]
            print(r)
            distance = estimate_distance(r*2)

            print(distance)

            target_position = get_target_position(lebai, x, y, r)
            lebai.movej(target_position, 1 * np.pi, 2 * np.pi)
            lebai.wait_move()
            wait(5)
            # target_position['x'] += 0.0115
            # lebai.movej(target_position, 0.01 * np.pi, 1 * np.pi)
            # lebai.wait_move()
            # target_position['x'] -= 0.025
            # lebai.movej(target_position, 0.5 * np.pi, 1 * np.pi)
            # lebai.wait_move()
            reset_arm(lebai)
            lebai.wait_move()

def main():
    lebai.start_sys()
    # lebai.init_claw()

    client = socket.socket()
    client.connect(('192.168.10.201', 5180))
    client.send("init_claw()".encode())
    client.send("disable_joint_limits()".encode())

    try:
        reset_arm(lebai)
        lebai.set_claw(50, 0)

        server = HTTPServer(('', 2000), LebaiHTTPHandler)
        server.serve_forever()

    except KeyboardInterrupt:
        print("closing")
        # cap.release()
        lebai.stop_sys()
        # cv2.destroyAllWindows()
        # lebai.stop()

if __name__ == "__main__":
    main()