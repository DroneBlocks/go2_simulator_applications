import pybullet as p
import time
import numpy as np
import pybullet_data
import cv2
import tkinter as tk
import tkinter.messagebox as messagebox
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
        self.step()
        self.update_cameras()

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
        return self.following_depth_image[:, :, 0]

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

    def await_user_click(self):
        def on_click(event):
            x = event.x
            y = event.y
            messagebox.showinfo("Click Coordinates", f"Clicked at ({x}, {y})")
            self.clicked_coordinates = (x, y)
            self.window.quit()

        self.following_label_depth.bind("<Button-1>", on_click)
        self.window.mainloop()
        self.following_label_depth.unbind("<Button-1>")
        return self.clicked_coordinates
    
    def display_popup(self, popup_text):
        messagebox.showinfo("Map Information", popup_text)

    def display_path(self, path):
        # Create a copy of the overhead camera image
        image = self.following_rgb_image.copy()

        # Convert the path coordinates to image coordinates
        image_path = [(col, row) for row, col in path]

        # Draw the path on the image
        for i in range(len(image_path) - 1):
            cv2.line(image, image_path[i], image_path[i + 1], (0, 255, 0), 2)

        # Convert the image to a PhotoImage object
        photo = ImageTk.PhotoImage(Image.fromarray(image))

        # Update the overhead camera label with the path
        self.following_label_rgb.configure(image=photo)
        self.following_label_rgb.image = photo

    def execute_path(self, path, meters_per_pixel=0.02):
        if len(path) == 0:
            return

        # Set the desired speed of the robot (in meters per second)
        speed = 1.5

        path = [[-p[1], -p[0]] for p in path]

        # Calculate the total distance of the path in meters
        total_distance = 0
        for i in range(len(path) - 1):
            dx = (path[i + 1][1] - path[i][1]) * meters_per_pixel
            dy = (path[i + 1][0] - path[i][0]) * meters_per_pixel
            total_distance += math.sqrt(dx ** 2 + dy ** 2)

        # Calculate the total time to complete the path
        total_time = total_distance / speed

        # Calculate the number of waypoints based on the desired time step
        time_step = 0.1  # Adjust this value to change the spacing between waypoints
        num_waypoints = int(total_time / time_step)

        # Generate equally spaced waypoints along the path using interpolation
        waypoints = []
        for i in range(num_waypoints + 1):
            t = i * time_step / total_time
            index = int(t * (len(path) - 1))
            fraction = t * (len(path) - 1) - index
            if index < len(path) - 1:
                waypoint = (
                    path[index][0] + fraction * (path[index + 1][0] - path[index][0]),
                    path[index][1] + fraction * (path[index + 1][1] - path[index][1])
                )
            else:
                waypoint = path[-1]
            waypoints.append(waypoint)

        # Move the robot to each waypoint
        for i in range(len(waypoints) - 1):
            start_time = time.time()

            # Calculate the distance and direction to the next waypoint
            dx = (waypoints[i + 1][1] - waypoints[i][1]) * meters_per_pixel
            dy = (waypoints[i + 1][0] - waypoints[i][0]) * meters_per_pixel
            distance = math.sqrt(dx ** 2 + dy ** 2)
            direction = math.atan2(dy, dx)

            # Convert the direction to degrees
            direction_degrees = math.degrees(direction)

            # Calculate the new position of the robot
            new_position = self.current_position + rotate_by_yaw(np.array([distance, 0.0, 0.0]), direction_degrees)

            # Update the robot's position and orientation
            self.current_position = new_position
            self.current_yaw = direction_degrees

            # Wait for the remaining time to achieve the desired time step
            elapsed_time = time.time() - start_time
            if elapsed_time < time_step:
                time.sleep(time_step - elapsed_time)


            # Step the simulation
            self.reset_robot_pose(self.current_position, get_orientation_from_yaw(self.current_yaw))
            # if i % 50 == 0:
            self.update_cameras()
            self.window.update()

        self.current_yaw = 0
        self.reset_robot_pose(self.current_position, get_orientation_from_yaw(self.current_yaw))


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

