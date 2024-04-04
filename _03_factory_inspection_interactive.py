
import pybullet as p
import time
import numpy as np
import pybullet_data
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import math

from utils.pybullet_utils import Robot

def get_orientation_from_yaw(yaw):
    # Convert yaw to radians
    yaw_rad = math.radians(yaw)
    
    # Calculate the quaternion components
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    
    # Create the quaternion
    orientation_quat = np.array([0, 0, sy, cy])
    
    return orientation_quat

def rotate_by_yaw(vec, yaw):
    # Convert yaw to radians
    yaw_rad = math.radians(yaw)

    rotation_matrix = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                                [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                                [0, 0, 1]])
    
    # Apply the rotation matrix to the vector
    rotated_vec = np.dot(rotation_matrix, vec)
    
    return rotated_vec


if __name__ == "__main__":

    robot = Robot(robot_name="Go2", fix_base=False, gui=True)#, gui_follows_robot=True)
    
    robot.load_factory_scene()

    # Simulate
    joint_pos_targets = robot.initial_joint_positions
    dt = 0.002
    time_to_simulate = 100 # seconds
    steps_to_simulate = int(time_to_simulate / dt)

    # global current_position
    # global current_yaw

    current_position = np.array([0, 0, 0.38])
    current_yaw = 0

    # cv2.window

    try:
       # Create the main Tkinter window
        window = tk.Tk()
        window.title("Robot Control")

        # Create a frame for the images
        image_frame = tk.Frame(window)
        image_frame.grid(row=0, column=0, padx=10, pady=10)

        # Create a frame for the following images
        following_frame = tk.Frame(image_frame)
        following_frame.grid(row=0, column=0, padx=5, pady=5)

        # Create labels for displaying the following images
        following_label_depth = tk.Label(following_frame)
        following_label_depth.grid(row=0, column=0, padx=5, pady=5)

        following_label_rgb = tk.Label(following_frame)
        following_label_rgb.grid(row=0, column=1, padx=5, pady=5)

        # Create labels for the following image text
        following_depth_text = tk.Label(following_frame, text="Following Depth")
        following_depth_text.grid(row=1, column=0, padx=5, pady=5)

        following_rgb_text = tk.Label(following_frame, text="Following RGB")
        following_rgb_text.grid(row=1, column=1, padx=5, pady=5)

        # Create a frame for the first person images
        first_person_frame = tk.Frame(image_frame)
        first_person_frame.grid(row=0, column=1, padx=5, pady=5)

        # Create labels for displaying the first person images
        first_person_label_depth = tk.Label(first_person_frame)
        first_person_label_depth.grid(row=0, column=0, padx=5, pady=5)
        first_person_depth_text = tk.Label(first_person_frame, text="First Person Depth")
        first_person_depth_text.grid(row=1, column=0, padx=5, pady=5)

        first_person_label_rgb = tk.Label(first_person_frame)
        first_person_label_rgb.grid(row=2, column=0, padx=5, pady=5)
        first_person_rgb_text = tk.Label(first_person_frame, text="First Person RGB")
        first_person_rgb_text.grid(row=3, column=0, padx=5, pady=5)

        # Create a frame for the buttons
        button_frame = tk.Frame(window)
        button_frame.grid(row=1, column=0, padx=10, pady=10)

        # Function to move the robot forward
        def move_forward(event=None):
            global current_yaw, current_position
            current_position = current_position + rotate_by_yaw(np.array([0.2, 0.0, 0.0]), current_yaw)
            
        # Function to move the robot backward
        def move_backward(event=None):
            global current_yaw, current_position
            current_position = current_position - rotate_by_yaw(np.array([0.2, 0.0, 0.0]), current_yaw)

        # Function to turn the robot left
        def turn_left(event=None):
            global current_yaw
            current_yaw += 15 # degrees

        # Function to turn the robot right
        def turn_right(event=None):
            global current_yaw
            current_yaw -= 15 # degrees

        # Create buttons for controlling the robot
        forward_button = tk.Button(button_frame, text="Forward", command=move_forward)
        forward_button.pack(side=tk.LEFT)
        forward_button = tk.Button(button_frame, text="Backward", command=move_backward)
        forward_button.pack(side=tk.LEFT)
        left_button = tk.Button(button_frame, text="Left", command=turn_left)
        left_button.pack(side=tk.LEFT)
        right_button = tk.Button(button_frame, text="Right", command=turn_right)
        right_button.pack(side=tk.LEFT)

        # Bind arrow keys to robot control functions
        window.bind("<Up>", move_forward)
        window.bind("<Down>", move_backward)
        window.bind("<Left>", turn_left)
        window.bind("<Right>", turn_right)

        ts = time.time()

        for i in range(steps_to_simulate):
            robot.step_with_pd_targets(joint_pos_targets=joint_pos_targets,
                                    Kp=40.,
                                    Kd=1.)
            
            current_orientation = get_orientation_from_yaw(current_yaw)
            # print(current_yaw[0], current_orientation)
            robot.set_base_position(current_position, current_orientation)

            if i % 50 == 0:
                following_rgb_image, following_depth_image = robot.get_camera_image(viewpoint="FOLLOWING")
                first_person_rgb_image, first_person_depth_image = robot.get_camera_image(viewpoint="FIRST_PERSON")
                following_rgb_image = cv2.pyrUp(following_rgb_image, 4)

                # Convert the images to RGB format
                following_rgb_image = cv2.cvtColor(following_rgb_image, cv2.COLOR_BGR2RGB)
                following_rgb_image = cv2.resize(following_rgb_image, (400, 400))  # Adjust the size as needed
                following_rgb_image = (following_rgb_image * 255).astype(np.uint8)
                following_depth_image[following_depth_image<=15.2] = 0.0
                following_depth_image[following_depth_image>15.2] = 1.0
                following_depth_image = cv2.normalize(following_depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                following_depth_image = cv2.cvtColor(following_depth_image, cv2.COLOR_BGR2RGB)
                following_depth_image = cv2.resize(following_depth_image, (400, 400))  # Adjust the size as needed

                first_person_rgb_image = cv2.cvtColor(first_person_rgb_image, cv2.COLOR_BGR2RGB)
                first_person_rgb_image = (first_person_rgb_image * 255).astype(np.uint8)
                first_person_depth_image = cv2.normalize(first_person_depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                first_person_depth_image = cv2.cvtColor(first_person_depth_image, cv2.COLOR_BGR2RGB)

                # Convert the images to PhotoImage objects
                following_photo_depth = ImageTk.PhotoImage(Image.fromarray(following_depth_image))
                following_photo_rgb = ImageTk.PhotoImage(Image.fromarray(following_rgb_image))
                first_person_photo_depth = ImageTk.PhotoImage(Image.fromarray(first_person_depth_image))
                first_person_photo_rgb = ImageTk.PhotoImage(Image.fromarray(first_person_rgb_image))

                # Update the image labels
                following_label_rgb.configure(image=following_photo_rgb)
                following_label_rgb.image = following_photo_rgb
                following_label_depth.configure(image=following_photo_depth)
                following_label_depth.image = following_photo_depth
                first_person_label_depth.configure(image=first_person_photo_depth)
                first_person_label_depth.image = first_person_photo_depth
                first_person_label_rgb.configure(image=first_person_photo_rgb)
                first_person_label_rgb.image = first_person_photo_rgb

            if i % 50 == 0:
                print(f"fps: {50./(time.time()-ts)}")
                ts = time.time()

            # Update the Tkinter window
            window.update()

        # Run the Tkinter event loop
        window.mainloop()

    except KeyboardInterrupt:
        print("Quitting")

    cv2.destroyAllWindows()

    
    # Disconnect from the simulation
    p.disconnect()
