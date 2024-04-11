
import pybullet as p
import time
import numpy as np
import pybullet_data
import cv2

from utils.pybullet_utils import Robot


if __name__ == "__main__":

    robot = Robot(robot_name="Go2", fix_base=False, gui=True, gui_follows_robot=True)

    # Simulate
    joint_pos_targets = robot.initial_joint_positions
    dt = 0.002
    time_to_simulate = 100 # seconds
    steps_to_simulate = int(time_to_simulate / dt)

    # cv2.window

    try:
        ts = time.time()
        for i in range(steps_to_simulate):
            robot.step_with_pd_targets(joint_pos_targets=joint_pos_targets,
                                    Kp=40.,
                                    Kd=1.,
                                    )
            
            if i % 50 == 0:
                # rgb_image, depth_image = robot.get_camera_image(viewpoint="AERIAL")
                following_rgb_image, following_depth_image = robot.get_camera_image(viewpoint="FOLLOWING")
                first_person_rgb_image, first_person_depth_image = robot.get_camera_image(viewpoint="FIRST_PERSON")
                following_rgb_image = cv2.pyrUp(following_rgb_image, 4)
                cv2.imshow("Following Camera", following_rgb_image)
                cv2.imshow("First Person Depth", first_person_depth_image)
                cv2.waitKey(1)

            if i % 50 == 0:
                print(f"fps: {50./(time.time()-ts)}")
                ts = time.time()

    except KeyboardInterrupt:
        print("Quitting")

    cv2.destroyAllWindows()

    
    # Disconnect from the simulation
    p.disconnect()
