
import pybullet as p
import time
import numpy as np
import pybullet_data

from utils.pybullet_utils import Robot


if __name__ == "__main__":

    robot = Robot(robot_name="Go2", fix_base=False)

    robot.load_factory_scene()

    

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
