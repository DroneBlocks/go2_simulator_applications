
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

class FactoryScene:
    def __init__(self):
        self.robot = Robot(robot_name="Go2", fix_base=False, gui=True)#, gui_follows_robot=True)
        self.robot.load_factory_scene()

        self.current_position = np.array([0, 0, 0.38])
        self.current_yaw = 0

        self.timestep = 0

        # initialize sensor data with blank values before first timestep
        self.following_depth_image = np.zeros([128, 128])
        self.first_person_depth_image = np.zeros([128, 128])

        self.initialize_tk_window()

    def reset_robot_pose(self, position, orientation):
        self.robot.set_base_position(position, orientation)

    def step(self):
        joint_pos_targets = self.robot.initial_joint_positions
        Kp, Kd = 40, 1
        self.robot.step_with_pd_targets(joint_pos_targets, Kp, Kd)
        self.reset_robot_pose(self.current_position, get_orientation_from_yaw(self.current_yaw))

        self.timestep += 1
        if self.timestep % 50 == 0:
            self.update_cameras()

        # Update the Tkinter window
        self.window.update()

    def get_depth_image(self):
        return self.first_person_depth_image
    
    def get_lidar(self):
        return self.following_depth_image

    def update_cameras(self):
        following_rgb_image, following_depth_image = self.robot.get_camera_image(viewpoint="FOLLOWING")
        first_person_rgb_image, first_person_depth_image = self.robot.get_camera_image(viewpoint="FIRST_PERSON")
        following_rgb_image = cv2.pyrUp(following_rgb_image, 4)

        # Convert the images to RGB format
        following_rgb_image = cv2.cvtColor(following_rgb_image, cv2.COLOR_BGR2RGB)
        following_rgb_image = cv2.resize(following_rgb_image, (400, 400))  # Adjust the size as needed
        self.following_rgb_image = (following_rgb_image * 255).astype(np.uint8)
        following_depth_image[following_depth_image<=0.95] = 0.0
        following_depth_image[following_depth_image>0.95] = 1.0
        following_depth_image = cv2.normalize(following_depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        following_depth_image = cv2.cvtColor(following_depth_image, cv2.COLOR_BGR2RGB)
        self.following_depth_image = cv2.resize(following_depth_image, (400, 400))  # Adjust the size as needed

        first_person_rgb_image = cv2.cvtColor(first_person_rgb_image, cv2.COLOR_BGR2RGB)
        self.first_person_rgb_image = (first_person_rgb_image * 255).astype(np.uint8)
        first_person_depth_image = cv2.normalize(first_person_depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        self.first_person_depth_image = cv2.cvtColor(first_person_depth_image, cv2.COLOR_BGR2RGB)

        # Convert the images to PhotoImage objects
        following_photo_depth = ImageTk.PhotoImage(Image.fromarray(self.following_depth_image))
        following_photo_rgb = ImageTk.PhotoImage(Image.fromarray(self.following_rgb_image))
        first_person_photo_depth = ImageTk.PhotoImage(Image.fromarray(self.first_person_depth_image))
        first_person_photo_rgb = ImageTk.PhotoImage(Image.fromarray(self.first_person_rgb_image))

        # Update the image labels
        self.following_label_rgb.configure(image=following_photo_rgb)
        self.following_label_rgb.image = following_photo_rgb
        self.following_label_depth.configure(image=following_photo_depth)
        self.following_label_depth.image = following_photo_depth
        self.first_person_label_depth.configure(image=first_person_photo_depth)
        self.first_person_label_depth.image = first_person_photo_depth
        self.first_person_label_rgb.configure(image=first_person_photo_rgb)
        self.first_person_label_rgb.image = first_person_photo_rgb

    # Function to move the robot forward
    def move_forward(self, event=None):
        self.current_position = self.current_position + rotate_by_yaw(np.array([0.2, 0.0, 0.0]), self.current_yaw)
        
    # Function to move the robot backward
    def move_backward(self, event=None):
        self.current_position = self.current_position - rotate_by_yaw(np.array([0.2, 0.0, 0.0]), self.current_yaw)

    # Function to turn the robot left
    def turn_left(self, event=None):
        self.current_yaw += 15 # degrees

    # Function to turn the robot right
    def turn_right(self, event=None):
        self.current_yaw -= 15 # degrees

    def initialize_tk_window(self):
       # Create the main Tkinter window
        self.window = tk.Tk()
        self.window.title("Robot Control")

        # Create a frame for the images
        image_frame = tk.Frame(self.window)
        image_frame.grid(row=0, column=0, padx=10, pady=10)

        # Create a frame for the following images
        following_frame = tk.Frame(image_frame)
        following_frame.grid(row=0, column=0, padx=5, pady=5)

        # Create labels for displaying the following images
        self.following_label_depth = tk.Label(following_frame)
        self.following_label_depth.grid(row=0, column=0, padx=5, pady=5)

        self.following_label_rgb = tk.Label(following_frame)
        self.following_label_rgb.grid(row=0, column=1, padx=5, pady=5)

        # Create labels for the following image text
        following_depth_text = tk.Label(following_frame, text="LIDAR")
        following_depth_text.grid(row=1, column=0, padx=5, pady=5)

        following_rgb_text = tk.Label(following_frame, text="Overhead Camera")
        following_rgb_text.grid(row=1, column=1, padx=5, pady=5)

        # Create a frame for the first person images
        first_person_frame = tk.Frame(image_frame)
        first_person_frame.grid(row=0, column=1, padx=5, pady=5)

        # Create labels for displaying the first person images
        self.first_person_label_depth = tk.Label(first_person_frame)
        self.first_person_label_depth.grid(row=0, column=0, padx=5, pady=5)
        first_person_depth_text = tk.Label(first_person_frame, text="First Person Camera Depth")
        first_person_depth_text.grid(row=1, column=0, padx=5, pady=5)

        self.first_person_label_rgb = tk.Label(first_person_frame)
        self.first_person_label_rgb.grid(row=2, column=0, padx=5, pady=5)
        first_person_rgb_text = tk.Label(first_person_frame, text="First Person Camera RGB")
        first_person_rgb_text.grid(row=3, column=0, padx=5, pady=5)

        # Create a frame for the buttons
        button_frame = tk.Frame(self.window)
        button_frame.grid(row=1, column=0, padx=10, pady=10)

        # Create buttons for controlling the robot
        forward_button = tk.Button(button_frame, text="Forward", command=self.move_forward)
        forward_button.pack(side=tk.LEFT)
        forward_button = tk.Button(button_frame, text="Backward", command=self.move_backward)
        forward_button.pack(side=tk.LEFT)
        left_button = tk.Button(button_frame, text="Left", command=self.turn_left)
        left_button.pack(side=tk.LEFT)
        right_button = tk.Button(button_frame, text="Right", command=self.turn_right)
        right_button.pack(side=tk.LEFT)

        # Bind arrow keys to robot control functions
        self.window.bind("<Up>", self.move_forward)
        self.window.bind("<Down>", self.move_backward)
        self.window.bind("<Left>", self.turn_left)
        self.window.bind("<Right>", self.turn_right)



if __name__ == "__main__":

    factory_scene = FactoryScene()

    # Simulate
    dt = 0.002
    time_to_simulate = 100000 # seconds
    steps_to_simulate = int(time_to_simulate / dt)


    try:
       
        ts = time.time()

        # initialize
        current_position = np.array([15.1, -5.2, 0.38])
        current_yaw = 0
        factory_scene.reset_robot_pose(current_position, get_orientation_from_yaw(current_yaw))


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
