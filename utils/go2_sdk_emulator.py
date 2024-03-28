
import pybullet as p
import numpy as np
import threading

from .pybullet_utils import Robot

class LeggedType:
    Go1 = 0

class motorCmd:
    q = 0 # joint position
    qd = 0 # joint velocity
    Kp = 0 # proportional gain
    Kd = 0 # derivative gain
    tau = 0 # torque

class LowCmd:
    motorCmd = [motorCmd() for i in range(12)]

class motorState:
    q = 0 # joint position
    qd = 0 # joint velocity

class LowState:
    motorState = [motorState() for i in range(12)]

class HighCmd:
    v = 0

class Safety:
    def PowerProtect(self, cmd, state, level):
        return 1

class UDP:
    def __init__(self):
        self.robot = None
        self.cmd = None
        self.state = None

        self.run_thread = threading.Thread(target=self.loop_sim, daemon=False)
        self.run_thread.start()

    def loop_sim(self):
        while True:
            if self.cmd is not None:
                # apply the command to the simulated robot
                joint_pos_targets = [self.cmd.motorCmd[joint_idx].q for joint_idx in range(12)]
                joint_vel_targets = [self.cmd.motorCmd[joint_idx].qd for joint_idx in range(12)]
                Kps = [self.cmd.motorCmd[joint_idx].Kp for joint_idx in range(12)]
                Kds = [self.cmd.motorCmd[joint_idx].Kd for joint_idx in range(12)]
                joint_ff_torques = [self.cmd.motorCmd[joint_idx].tau for joint_idx in range(12)]

                # convert to numpy
                joint_pos_targets = np.array(joint_pos_targets)
                joint_vel_targets = np.array(joint_vel_targets)
                Kps = np.array(Kps)
                Kds = np.array(Kds)
                joint_ff_torques = np.array(joint_ff_torques)

                # get the joint state
                joint_pos, joint_vel = self.robot.get_joint_states()

                # compute the torque to apply using the PD control formula
                torques = Kps * (joint_pos_targets - joint_pos) + Kds * (joint_vel_targets - joint_vel) + joint_ff_torques

                self.robot.step_with_torques(torques)

    def InitCmdData(self, cmd):
        self.cmd = cmd

    def Recv(self):
        self.assert_init()
        self.state = LowState()

        # read and store joint position and velocity
        joint_pos, joint_vel = self.robot.get_joint_states()

        for joint_idx in range(12):
            self.state.motorState[joint_idx].q = joint_pos[joint_idx]
            self.state.motorState[joint_idx].qd = joint_vel[joint_idx]

    def GetRecv(self, state):
        self.assert_init(require_state=True)
        # update state object passed in
        for joint_idx in range(12):
            state.motorState[joint_idx].q = self.state.motorState[joint_idx].q
            state.motorState[joint_idx].qd = self.state.motorState[joint_idx].qd

    def SetSend(self, cmd):
        self.cmd = cmd

    def Send(self):
        self.assert_init(require_cmd=True)
        # read the command from self.cmd
        
        
    def add_robot(self, robot):
        self.robot = robot

    def assert_init(self, require_cmd=False, require_state=False):
        assert self.robot is not None, "Error: robot not initialized! You have not initialized the UnitreeSDKEmulator."
        if require_cmd:
            assert self.cmd is not None, "Error: cmd not initialized! You have not called UDP.SetSend before UDP.Send"
        if require_state:
            assert self.state is not None, "Error: state not intialized! You have not called UDP.Recv before UDP.GetRecv"


class UnitreeGo2SDKEmulator:
    LeggedType = LeggedType()

    def UDP(mode, port1, ip, port2):
        assert ((port1 == 8080) and (port2 == 8007) and (ip == "192.168.123.10")), "Invalid IP/Port for UDP communication with the robot!"
        if mode == 0xff: # low-level control
            go1_mode = "LOW"
        elif mode == "":
            go1_mode = "HIGH"
        else:
            raise Exception("Invalid mode for UDP communication with the robot!")
        
        udp = UDP()
        udp.add_robot(Robot(robot_name="Go1", fix_base=False))
        return udp

    def Safety(robot_type):
        if robot_type == 0:
            return Safety()
        else:
            raise Exception("Invalid argument for Safety!")
        
    def LowCmd():
        return LowCmd()
    
    def LowState():
        return LowState()