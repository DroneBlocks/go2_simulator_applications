import pybullet as p
import time
import numpy as np
print("{REMINDER] WE NEED TO INSTRUCT THE USER TO INSTALL SCIPY")
from scipy.spatial.transform import Rotation


# Let's create a Robot class to contain the generic functionality
class Robot:
    def __init__(self, robot_name, fix_base=False):

        ## Add reset function

        self.load_params(robot_name)

        self.robot = self.load_robot(
            urdf_path=self.urdf_path,
            initial_base_position=self.initial_base_position,
            num_dof=self.num_dof,
            initial_joint_positions=self.initial_joint_positions,
            joint_indices=self.joint_indices,
            camera_distance=self.camera_distance,
            fix_base=fix_base,
        )

        self.time_counter = 0

        
    def get_joint_states(self):
        joint_states = p.getJointStates(self.robot, self.joint_indices)
        joint_pos = np.array([joint_states[n][0] for n in range(self.num_dof)])
        joint_vel = np.array([joint_states[n][1] for n in range(self.num_dof)])
        return joint_pos, joint_vel
    
    def get_base_position(self):
        robot_pos, robot_ori = p.getBasePositionAndOrientation(self.robot)
        return robot_pos, robot_ori

    def step_with_torques(self, torques):
        p.setJointMotorControlArray(bodyIndex=self.robot,
                            jointIndices=self.joint_indices,
                            controlMode=p.TORQUE_CONTROL,
                            forces=torques)
        p.stepSimulation()
        if self.time_counter % 50 == 0:
            self.step_camera()
        time.sleep(1./500.)
        self.time_counter += 1

    def step_with_pd_targets(self, joint_pos_targets, Kp, Kd):
        joint_pos, joint_vel = self.get_joint_states()

        torques = Kp * (np.array(joint_pos_targets) - joint_pos) - Kd * (joint_vel)
        
        self.step_with_torques(torques)

    def load_params(self, robot_name):
        if robot_name == "Go1":
            self.urdf_path="assets/go1_description/urdf/go1.urdf"
            self.initial_base_position=[0, 0, 0.38]
            self.num_dof=12
            self.initial_joint_positions=[0.0, 0.80, -1.5, 0.0, 0.80, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]
            self.joint_indices=[2, 3, 4, 9, 10, 11, 16, 17, 18, 23, 24, 25]
            self.camera_distance = 0.8
        elif robot_name == "Go2":
            self.urdf_path="assets/go2_description/urdf/go2_description.urdf"
            self.initial_base_position=[0, 0, 0.40]
            self.num_dof=12
            self.initial_joint_positions=[0.0, 0.80, -1.5, 0.0, 0.80, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]
            self.joint_indices=[2, 3, 4, 8, 9, 10, 14, 15, 16, 20, 21, 22]
            self.camera_distance = 0.8
        elif robot_name == "H1":
            self.urdf_path="assets/h1_description/urdf/h1.urdf"
            self.initial_base_position=[0, 0, 1.3]
            self.num_dof=18
            self.initial_joint_positions=[0.0] * 18
            self.joint_indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
            self.camera_distance = 1.5
        elif robot_name == "H1_hands":
            self.urdf_path="assets/h1_description/urdf/h1_with_hand.urdf"
            self.initial_base_position=[0, 0, 1.3]
            self.num_dof=19
            self.initial_joint_positions=[0.0] * 19
            self.joint_indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                    10, 11, 12, 13, 14, 29, 30, 31, 32,]
            self.camera_distance = 1.5
        else:
            print(f"Robot {robot_name} not available!")

    def load_robot(
            self,
            urdf_path,
            initial_base_position,
            num_dof,
            initial_joint_positions,
            joint_indices,
            camera_distance,
            fix_base,
    ):

        # Start PyBullet in GUI mode
        physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,1)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)

        p.configureDebugVisualizer(rgbBackground=[0, 0, 0])
        p.setPhysicsEngineParameter(fixedTimeStep=0.002,
                                    numSolverIterations=50, 
                                    solverResidualThreshold=1e-30, 
                                    numSubSteps=1,)


        # Load a plane and a few objects
        plane = p.loadURDF("assets/plane.urdf", useMaximalCoordinates=True)
        robot = p.loadURDF(urdf_path, 
                        basePosition=initial_base_position, 
                        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                        useFixedBase=fix_base,
                        )

        p.setGravity(0, 0, -9.81)

        # Camera parameters
        cameraDistance = camera_distance    # How far the camera is from the target
        cameraYaw = 45        # Yaw angle in degrees
        cameraPitch = -15     # Pitch angle in degrees
        cameraTargetPosition = [initial_base_position[0],
                                initial_base_position[1],
                                initial_base_position[2] - 0.3]  # Focus just below the base position

        # Set the camera view
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

        for j in range(p.getNumJoints(robot)):
            print(p.getJointInfo(robot, j))

        # Set initial position
        for j in range(num_dof):
            p.resetJointState(  robot,
                                joint_indices[j], 
                                targetValue=initial_joint_positions[j],
                                targetVelocity=0,
            )
            
        p.setJointMotorControlArray(bodyIndex=robot,
                        jointIndices=joint_indices,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocities=[0] * num_dof,
                        forces=[0] * num_dof,
        )
        p.setJointMotorControlArray(bodyIndex=robot,
                        jointIndices=joint_indices,
                        controlMode=p.TORQUE_CONTROL,
                        forces=[0] * num_dof,
        )
        p.stepSimulation()

        return robot
    
    def step_camera(self):
        width = 128
        height = 128

        fov = 60
        aspect = width / height
        near = 0.02
        far = 1

        robot_pos, robot_ori = self.get_base_position()
        rot = Rotation.from_quat(robot_ori)
        default_direction = np.array([0.4, 0, -0.2])
        cam_vec = rot.apply(default_direction)

        # camera_pos = [robot_pos[0], robot_pos[1], robot_pos[2]]
        s1 = -1.0
        s2 = 1.3
        camera_pos = [robot_pos[0] + s1*cam_vec[0], 
                         robot_pos[1] + s1*cam_vec[1], 
                         robot_pos[2] + s1*cam_vec[2]]
        camera_lookat = [robot_pos[0] + s2*cam_vec[0], 
                         robot_pos[1] + s2*cam_vec[1], 
                         robot_pos[2] + s2*cam_vec[2]]
        # camera_pos = [0, 0, 0]
        # camera_lookat = [0.5, 0, 0]
        

        self.view_matrix = p.computeViewMatrix(camera_pos, camera_lookat, [0, 0, 1])
        self.projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

        # Get depth values using the OpenGL renderer
        images = p.getCameraImage(width, height, self.view_matrix, self.projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        depth_buffer_opengl = np.reshape(images[3], [width, height])
        depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)

        # # Get depth values using Tiny renderer
        # images = p.getCameraImage(width, height, self.view_matrix, self.projection_matrix, renderer=p.ER_TINY_RENDERER)
        # depth_buffer_tiny = np.reshape(images[3], [width, height])
        # depth_tiny = far * near / (far - (far - near) * depth_buffer_tiny)

    def load_factory_scene(self):
        # name_in = "assets/factory/meshes/factory.obj"
        # name_out = "assets/factory/meshes/factory_vhacd.obj"
        # name_log = "assets/factory/meshes/log.txt"
        # p.vhacd(name_in, name_out, name_log)
        # input("VHACD done")
        factoryId = p.loadURDF("assets/factory/urdf/factory.urdf", 
                               basePosition=[6.0,3.0,-4.71], 
                               baseOrientation=p.getQuaternionFromEuler([1.57, 0, 0]),
                               useFixedBase=True,
                               globalScaling=0.01
                               )
