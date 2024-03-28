
import pybullet as p
import time
import numpy as np
import pybullet_data

from utils.go2_sdk_emulator import UnitreeGo2SDKEmulator

try:
    # # Add the path to the robot interface library
    # sys.path.append('./unitree_legged_sdk-go1/lib/python/arm64')
    # import robot_interface as sdk
    from utils.go2_sdk_emulator import UnitreeGo2SDKEmulator as sdk
    has_sdk = True
except ModuleNotFoundError:
    print("No robot interface library found. Running in test mode only, the robot will not do anything.")
    has_sdk = False


def main(reader):
    # Initialize variables
    global dt
    qDes = [0] * 12
    qInit = [0] * 12

    if has_sdk:
        # Initialize UDP and safety modules
        udp = sdk.UDP(0xff, 8080, "192.168.123.10", 8007)
        safe = sdk.Safety(sdk.LeggedType.Go1)

        # Initialize command and state variables
        cmd = sdk.LowCmd()
        state = sdk.LowState()
        udp.InitCmdData(cmd)