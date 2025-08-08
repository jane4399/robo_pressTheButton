import time
from math import pi, sin, cos
from dynamixel_sdk import * 

import lebai_sdk
lebai_sdk.init()

# ----------------------------------------

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

PROTOCOL_VERSION = 2.0

BAUDRATE = 1000000
DEVICENAME = '/dev/cu.usbmodem1101'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

DXL_IDS = [1, 2, 3, 4, 5, 6] 

LEBAI_IP = "192.168.10.200"

# limits
# servo 1: 0 - 360 deg
# servo 2: 8 - 147 deg
# servo 3: 92 - 280 deg
# servo 4: 69 - 290 deg
# servo 5: -360 - 360 deg
# servo 6: 0 - 93

# ----------------------------------------

def open_port():
    portHandler = PortHandler(DEVICENAME)
    if not portHandler.openPort():
        raise Exception("Failed to open the port")
    if not portHandler.setBaudRate(BAUDRATE):
        raise Exception("Failed to set baudrate")
    return portHandler

def enable_torque(packetHandler, portHandler, dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

def disable_torque(packetHandler, portHandler, dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

def set_goal_position(packetHandler, portHandler, dxl_id, position):
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position)

def get_present_position(packetHandler, portHandler, dxl_id):
    pos, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    return pos

def pos_to_radians(pos, last_pos, resolution=4096, angle_range=360.0):
    center = (resolution - 1) // 2
    pos = pos-0b11111111111111111111111111111111 if abs(last_pos-pos) > 2147483647 else pos
    return ((pos / center) * (angle_range / 2))*(pi/180)

# ----------------------------------------

def main():
    portHandler = open_port()
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    last_pos = [0]*6
    pos = [0]*6

    lebai = lebai_sdk.connect(LEBAI_IP, False)

    for dxl_id in DXL_IDS:
        disable_torque(packetHandler, portHandler, dxl_id)

    try:
        lebai.start_sys()
        lebai.init_claw()

        while True:
            time.sleep(0.05)

            for dxl_id in DXL_IDS:
                pos[dxl_id-1] = pos_to_radians(
                    get_present_position(packetHandler, portHandler, dxl_id),
                    last_pos[dxl_id-1]
                )
                last_pos[dxl_id-1] = pos[dxl_id-1]

            lebai.towardj(
                [ # joint angles (rad)
                    pos[0] - pi, -((3*pi)/4-pos[1]),
                    -(pos[2]-(3*pi)/2), -(pos[3]-pi),
                    pi/2,
                    pos[4]
                ],
                4*pi, # acceleration (rad/s2)
                2*pi # velocity (rad/s)
            )

    finally:
        for dxl_id in DXL_IDS:
            disable_torque(packetHandler, portHandler, dxl_id)
        portHandler.closePort()
        lebai.stop()

if __name__ == "__main__":
    main()