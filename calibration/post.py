import requests

# Replace with the IP address or hostname of the robot/server
ROBOT_IP = '192.168.10.201'
PORT = 2001

def trigger_capture():
    url = f'http://{ROBOT_IP}:{PORT}/'

    try:
        response = requests.post(url)  # Or use .post() if your server expects POST
        if response.status_code == 200:
            print("✅ Image captured successfully!")
        else:
            print(f"⚠️ Failed with status code: {response.status_code}")
            print("Response:", response.text)
    except requests.RequestException as e:
        print("❌ Request failed:", e)

def main():
    print("Press ENTER to send a capture request. Type 'q' and press ENTER to quit.")
    while True:
        user_input = input("> ")
        if user_input.strip().lower() == 'q':
            print("Exiting.")
            break
        trigger_capture()

if __name__ == '__main__':
    main()
