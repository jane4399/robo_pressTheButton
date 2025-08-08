import http.server
import os
import time
import cv2
import lebai_sdk
lebai_sdk.init()

import numpy as np
arm_data = []

lebai = lebai_sdk.connect("192.168.10.200", False)

cap = cv2.VideoCapture("/dev/video0")
if cap is None or not cap.isOpened():
    cap = cv2.VideoCapture("/dev/video1")

os.makedirs("handeye_images", exist_ok=True)



class LebaiHTTPHandler(http.server.BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != '/':
            self.send_error(404)
            return
        ret, frame = cap.read()
        if not ret:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(b'Failed to capture frame.\n')
            print("❌ Failed to capture frame")
            return

        filename = f"handeye_images/capture_{int(time.time())}.jpg"
        cv2.imwrite(filename, frame)
        print(f"✅ Saved image: {filename}")

        self.send_response(200)
        self.end_headers()
        self.wfile.write(b'Image captured and saved.\n')

        arm_data.append(lebai.get_kin_data()["actual_tcp_pose"])

    def do_GET(self):
        if self.path != '/':
            self.send_error(404)
            return
        
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()

        ret, frame = cap.read()
        if not ret:
            print("no video")
            return

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

def main():
    server_address = ('', 2001)  # Serve on all interfaces, port 8000
    httpd = http.server.HTTPServer(server_address, LebaiHTTPHandler)
    httpd.serve_forever()

if __name__ == "__main__":
    try:
        lebai.start_sys()
        lebai.teach_mode()
        main()
    except KeyboardInterrupt:
        print(arm_data)
        arm_data = np.array(arm_data)
        np.save("arm_data.npy", arm_data)