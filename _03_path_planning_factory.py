import pybullet as p
import time
import numpy as np
import pybullet_data
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import math
from utils.factory_scene_utils import FactoryScene, get_orientation_from_yaw

from collections import deque

def plan_path_bfs(lidar_data, destination, search_map_dim=64):
    # Define the movement directions (up, down, left, right)
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    # downsize the LIDAR map and destination to search_map_dim
    original_rows, original_cols = lidar_data.shape
    destination = (int(destination[1] * search_map_dim / original_rows), int(destination[0] * search_map_dim / original_cols))
    lidar_data = cv2.resize(lidar_data, (search_map_dim, search_map_dim))

    # Get the dimensions of the LIDAR data
    rows, cols = lidar_data.shape

    # Define the start position as the center of the LIDAR data
    start = (rows // 2, cols // 2)

    # Create a queue to store the nodes to visit
    queue = deque()
    queue.append((start, [start]))

    # Create a set to keep track of visited nodes
    visited = set()

    while queue:
        current_pos, path = queue.popleft()

        # Check if the current position is the destination
        if current_pos == destination:
            # revert downsizing operation
            path = [(int(p[0] * original_rows / search_map_dim), int(p[1] * original_cols / search_map_dim)) for p in path]
            return path

        # Mark the current position as visited
        visited.add(current_pos)

        # Explore the neighboring positions
        for direction in directions:
            new_pos = (current_pos[0] + direction[0], current_pos[1] + direction[1])

            # Check if the new position is within the LIDAR data bounds
            if 0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols:
                # Check if the new position is not an obstacle and hasn't been visited
                if lidar_data[new_pos[0], new_pos[1]] != 0 and new_pos not in visited:
                    new_path = path + [new_pos]
                    queue.append((new_pos, new_path))
                    visited.add(new_pos)

    # If no path is found, return an empty list
    return []

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

            # every 5 seconds, plan a new path
            if i % 250 == 0:
                print("Click the point to navigate to!")
                destination = factory_scene.await_user_click()

                path = plan_path_bfs(lidar_data, destination)

                print(path)

                factory_scene.display_path(path)
                factory_scene.display_popup("Planned path! Click OK to execute.")

                factory_scene.execute_path(path)
            
            if i % 50 == 0:
                print(f"fps: {50./(time.time()-ts)}")
                ts = time.time()

    except KeyboardInterrupt:
        print("Quitting")

    cv2.destroyAllWindows()

    
    # Disconnect from the simulation
    p.disconnect()
