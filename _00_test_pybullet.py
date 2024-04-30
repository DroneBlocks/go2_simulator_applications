import time
import numpy as np


try:
    import pybullet_data
    import pybullet as p
except Exception as e:
    print(e)
    print("Need to install pybullet!")


try:
    from scipy.spatial.transform import Rotation
except Exception as e:
    print(e)
    print("Need to install scipy!")


try:
    import cv2
except Exception as e:
    print(e)
    print("Need to install opencv-python!")

try:
    import PIL
except Exception as e:
    print(e)
    print("Need to install pillow!")



from utils.pybullet_utils import Robot


if __name__ == "__main__":

    robot = Robot(robot_name="Go2", fix_base=False)

    # Simulate
    joint_pos_targets = robot.initial_joint_positions
    dt = 0.002
    time_to_simulate = 100 # seconds
    steps_to_simulate = int(time_to_simulate / dt)

    for i in range(steps_to_simulate):
        robot.step_with_pd_targets(joint_pos_targets=joint_pos_targets,
                                   Kp=40.,
                                   Kd=1.,
                                   )
    
    # Disconnect from the simulation
    p.disconnect()
