import pybullet as p
import time
import numpy as np
import pybullet_data
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import math
from _02_factory_teleoperation import FactoryScene, get_orientation_from_yaw

if __name__ == "__main__":

    factory_scene = FactoryScene()

    # Simulate
    dt = 0.002
    time_to_simulate = 100000 # seconds
    steps_to_simulate = int(time_to_simulate / dt)


    try:
       
        ts = time.time()

        # initialize
        initial_position = np.array([15.1, -5.2, 0.38])
        initial_yaw = 0
        factory_scene.reset_robot_pose(initial_position, get_orientation_from_yaw(initial_yaw))


        for i in range(steps_to_simulate):

            # step the factory scene by one timestep
            factory_scene.step()

            lidar_data = factory_scene.get_lidar()
            depth_image = factory_scene.get_depth_image()
            
            if i % 50 == 0:
                print(f"fps: {50./(time.time()-ts)}")
                ts = time.time()

    except KeyboardInterrupt:
        print("Quitting")

    cv2.destroyAllWindows()

    
    # Disconnect from the simulation
    p.disconnect()
