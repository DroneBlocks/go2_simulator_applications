
import pybullet as p
import time
import numpy as np
import pybullet_data
import cv2

from utils.pybullet_utils import Robot


if __name__ == "__main__":

    robot = Robot(robot_name="Go2", fix_base=True, gui=True, gui_follows_robot=False)
    robot.load_boxes()

    # Simulate
    joint_pos_targets = robot.initial_joint_positions
    dt = 0.002
    time_to_simulate = 10000 # seconds
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
                # read the depth image
                rgb_image, depth_image = robot.get_camera_image(viewpoint="FIRST_PERSON")
                rgb_image = cv2.resize(rgb_image, (200, 200))  # Adjust the size as needed
                depth_image = cv2.resize(depth_image, (200, 200))  # Adjust the size as needed

                # read the overhead depth image
                # process into a lidar-like format
                following_rgb_image, lidar_image = robot.get_camera_image(viewpoint="FOLLOWING")
                lidar_image[lidar_image<=0.95] = 0.0
                lidar_image[lidar_image>0.95] = 1.0
                lidar_image = cv2.normalize(lidar_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                lidar_image = cv2.cvtColor(lidar_image, cv2.COLOR_BGR2RGB)
                lidar_image = cv2.resize(lidar_image, (400, 400))  # Adjust the size as needed

                # following_rgb_image = cv2.pyrUp(following_rgb_image, 4)
                cv2.imshow("Lidar", lidar_image)
                cv2.imshow("RGB Camera", rgb_image)
                cv2.imshow("Depth Camera", depth_image)
                cv2.waitKey(1)

            if i % 50 == 0:
                print(f"fps: {50./(time.time()-ts)}")
                ts = time.time()

    except KeyboardInterrupt:
        print("Quitting")

    cv2.destroyAllWindows()

    
    # Disconnect from the simulation
    p.disconnect()
