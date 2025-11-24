#import rospy
import numpy as np
import time
from copy import deepcopy as copy
from enum import Enum
import math

import rtde_control
import rtde_receive
from .robotiq import robotiq_gripper_control as rgc
from .robotiq import robotiq_gripper
from typing import List

from openteach.constants import SCALE_FACTOR
from scipy.spatial.transform import Rotation as R
from openteach.constants import *

class RobotControlMode(Enum):
    CARTESIAN_CONTROL = 0
    SERVO_CONTROL = 1

"""
Codes from UR RTDE interface
https://github.com/elpis-lab/UR10_RTDE/blob/60dd19a782e61596a23aa0b1e8bb4a3036abea83/rtde/rtde.py
"""
class RTDE:
    def __init__(self, robot_ip: str = "192.168.1.102"):
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        
        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(robot_ip, 63352)
        self.gripper.activate()

    def get_joint_values(self) -> List[float]:
        """Get the joint positions in radians."""
        return self.rtde_r.getActualQ()

    def get_joint_speed(self) -> List[float]:
        """Get the joint speed in radians per second."""
        return self.rtde_r.getActualQd()

    def get_tool_pose(self) -> List[float]:
        """Get the pose of the Tool Center Point (TCP) in Cartesian space.

        Return [x, y, z, rx, ry, rz] position + rotation vector
        """
        return self.rtde_r.getActualTCPPose()

    def get_tool_speed(self) -> List[float]:
        """Get the speed of the Tool Center Point (TCP) in Cartesian space.

        Return [vx, vy, vz, wx, wy, wz]
        """
        return self.rtde_r.getActualTCPSpeed()

    def set_tool_pose(self, tcp: List[float]):
        """Set the Tool Center Point (TCP) in Cartesian space.
        
        The pose is defined as [x, y, z, rx, ry, rz] position + rotation vector
        """
        self.rtde_c.setTcp(tcp)

    def move_joint(
        self,
        joint_values: List[float],
        speed: float = 0.0,
        acceleration: float = 0.0,
        # a bool specifying if the move command should be asynchronous
        asynchronous: bool = False,
    ):
        """Move the robot to the target joint positions."""
        self.rtde_c.moveJ(joint_values, speed, acceleration, asynchronous)

    def move_joint_trajectory(
        self,
        path: List[List[float]],
        # a bool specifying if the move command should be asynchronous
        asynchronous: bool = False,
    ):
        """Move the robot to follow a given path/trajectory,
        with each waypoint defined as
        [q1, q2, q3, q4, q5, q6, speed, acceleration, blend]
        (angles + others)
        """
        self.rtde_c.moveJ(path, asynchronous)

    def speed_joint(
        self,
        speeds: List[float],
        acceleration: float = 0.0,
        time: float = 0.0,
    ):
        """Accelerate linearly and continue with constant joint speed."""
        self.rtde_c.speedJ(speeds, acceleration, time)

    def move_tool(
        self,
        pose: List[float],
        speed: float = 0.0,
        acceleration: float = 0.0,
        # a bool specifying if the move command should be asynchronous
        asynchronous: bool = False,
    ):
        """Move the robot to the tool position."""
        self.rtde_c.moveL(pose, speed, acceleration, asynchronous)

    def move_tool_trajectory(
        self,
        path: List[List[float]],
        # a bool specifying if the move command should be asynchronous
        asynchronous: bool = False,
    ):
        """Move the robot to follow a given tool path/trajectory,
        with each waypoint defined as
        [x, y, z, rx, ry, rz, speed, acceleration, blend]
        (position + rotation vector + others)
        """
        self.rtde_c.moveL(path, asynchronous)

    def speed_tool(
        self,
        speeds: List[float],
        acceleration: float = 0.0,
        time: float = 0.0,
    ):
        """Accelerate linearly and continue with constant joint speed."""
        self.rtde_c.speedL(speeds, acceleration, time)

    def servo_joint(
        self,
        joint_values: List[float],  # Target joint positions
        speed: float = 0,  # joint velocity
        acceleration: float = 0,  # joint acceleration
        time: float = 0.008,  # time to control the robot
        lookahead_time: float = 0.1,  # project the current position forward
        gain: int = 300,  # P-term as in PID controller
    ):
        """Used to perform online realtime joint control

        The gain parameter works the same way as the P-term of a PID controller
        where it adjusts the current position towards the desired (q).
        The higher the gain, the faster reaction the robot will have.

        The parameter lookahead_time is used to project the current position
        forward in time with the current velocity. A low value gives fast
        reaction, a high value prevents overshoot.

        Note: A high gain or a short lookahead time may cause instability and
        vibrations. Especially if the target positions are noisy or updated
        at a low frequency It is preferred to call this function
        with a new setpoint (q) in each time step
        """
        self.rtde_c.servoJ(
            joint_values, speed, acceleration, time, lookahead_time, gain
        )

    def servo_tool(
        self,
        pose: List[float],  # Target joint positions
        speed: float = 0.0,  # joint velocity
        acceleration: float = 0.0,  # joint acceleration
        time: float = 0.008,  # time to control the robot
        lookahead_time: float = 0.1,  # project the current position forward
        gain: int = 300,  # P-term as in PID controller
    ):
        """Used to perform online realtime tool control

        The gain parameter works the same way as the P-term of a PID controller
        where it adjusts the current position towards the desired (q).
        The higher the gain, the faster reaction the robot will have.

        The parameter lookahead_time is used to project the current position
        forward in time with the current velocity. A low value gives fast
        reaction, a high value prevents overshoot.

        Note: A high gain or a short lookahead time may cause instability and
        vibrations. Especially if the target positions are noisy or updated
        at a low frequency It is preferred to call this function
        with a new setpoint (q) in each time step
        """
        # Tool pose [x, y, z, rx, ry, rz] position + rotation vector
        self.rtde_c.servoL(
            pose, speed, acceleration, time, lookahead_time, gain
        )

    def stop(
        self,
        a: float = 2.0,  # joint acceleration
        # a bool specifying if the move command should be asynchronous
        asynchronous: bool = False,
    ):
        """Stop the robot."""
        self.rtde_c.stopJ(a, asynchronous)

    def stop_script(self):
        """Terminate the script on controller"""
        self.rtde_c.stopScript()


class DexArmControl():
    def __init__(self, ip, record_type=None):
        #self._init_ur_arm_control(record)
        self.robot = RTDE(robot_ip=ip) 

    # Controller initializers
    # def _init_ur_control(self):
    #     self.robot.reset()
        
    #     status, home_pose = self.robot.get_position_aa()
    #     assert status == 0, "Failed to get robot position"
    #     home_affine = self.robot_pose_aa_to_affine(home_pose)
    #     # Initialize timestamp; used to send messages to the robot at a fixed frequency.
    #     last_sent_msg_ts = time.time()

    #     # Initialize the environment state action tuple.

    # Rostopic callback functions
   
    # State information functions
    def _get_pose_aa_mm(self):
        pose = list(self.robot.get_tool_pose()) # [x, y, z, rx, ry, rz] (m, rad)
        pose_mm = pose[:]
        # pose_mm[:3] = [p * SCALE_FACTOR for p in pose[:3]]  # m -> mm
        return pose_mm
    
    def get_arm_cartesian_state(self):
        pose = self._get_pose_aa_mm()
        cartesian_state = dict(
            position = np.array(pose[0:3], dtype=np.float32).flatten(),
            orientation = np.array(pose[3:], dtype=np.float32).flatten(),
            timestamp = time.time()
        )
        return cartesian_state
   
    def get_arm_pose(self):
        pose = np.array(self._get_pose_aa_mm(), dtype=np.float32)
        home_affine = self.robot_pose_aa_to_affine(pose)
        return home_affine

    def get_arm_position(self):
        q = self.robot.get_joint_values()
        return np.array(q, dtype=np.float32)
    
    def get_arm_osc_position(self):
        pose = self._get_pose_aa_mm()
        return np.array(pose, dtype=np.float32)

    def get_arm_velocity(self):
        raise ValueError('get_arm_velocity() is being called - Arm Velocity cannot be collected in UR arms, this method should not be called')

    def get_arm_torque(self):
        raise ValueError('get_arm_torque() is being called - Arm Torques cannot be collected in UR arms, this method should not be called')

    def get_arm_cartesian_coords(self):
        return self._get_pose_aa_mm()

    def get_gripper_state(self):
        gripper_position=self.robot.gripper.get_current_position()
        gripper_position=[gripper_position, [0,0,0]]  # Dummy values
        gripper_pose= dict(
            position = np.array(gripper_position[1], dtype=np.float32).flatten(),
            timestamp = time.time()
        )
        return gripper_pose

    def move_arm_joint(self, joint_angles):
        self.robot.servo_joint(
            joint_values=list(joint_angles),
            speed=0.0, # TODO: tune speed and acceleration
            acceleration=0.0,
            time=0.008,
            lookahead_time=0.1,
            gain=300
        )

    def move_arm_cartesian(self, cartesian_pos, duration=3):
        pose = list(cartesian_pos)  # [x, y, z, rx, ry, rz] (mm, rad)
        # pose[:3] = [p / SCALE_FACTOR for p in pose[:3]]  # mm -> m
        self.robot.move_tool(
            pose=pose,
            speed=0.0,  # 0 = use default
            acceleration=0.0,  # 0 = use default
            asynchronous=False
        )

    def arm_control(self, cartesian_pose):
        pose = list(cartesian_pose)  # [x, y, z, rx, ry, rz] (mm, rad)
        # pose[:3] = [p / SCALE_FACTOR for p in pose[:3]]  # mm -> m
        
        self.robot.servo_tool(
            pose=pose,
            speed=0.0,  # 0 = use default
            acceleration=0.0,  # 0 = use default
            time=0.008,
            lookahead_time=0.2,  # Increased for smoother movement
            gain=100  # Reduced for less aggressive tracking
        )
        
    def get_arm_joint_state(self):
        q = self.robot.get_joint_values()
        joint_state = dict(
            position=np.array(q, dtype=np.float32),
            timestamp=time.time()
        )
        return joint_state
        
    def get_cartesian_state(self):
        return self.get_arm_cartesian_state()
    
    def get_joint_velocity(self):
        qd = self.robot.get_joint_speed()
        return np.array(qd, dtype=np.float32)
    
    def get_joint_torque(self):
        raise NotImplementedError("Joint torque not implemented for UR arms.")

    def home_arm(self):
        self.robot.move_joint(
            joint_values=UR_HOME,
            speed=0.0,  # 0 = use default
            acceleration=0.0,  # 0 = use default
            asynchronous=False
        )

    def reset_arm(self):
        self.home_arm()

    # Full robot commands
    def move_robot(self,arm_angles):
        self.move_arm_joint(arm_angles)
        
    def home_robot(self):
        self.home_arm() # For now we're using cartesian values
        
    def set_gripper_state(self, open):
        goal = self.robot.gripper.get_max_position() if open else self.robot.gripper.get_min_position()
        speed = (self.robot.gripper.get_min_speed() + self.robot.gripper.get_max_speed()) / 2
        force = (self.robot.gripper.get_min_force() + self.robot.gripper.get_max_force()) / 2
        
        self.robot.gripper.move_and_wait_for_pos(position=goal, speed=speed, force=force)
            
    def robot_pose_aa_to_affine(self,pose_aa: np.ndarray) -> np.ndarray:
        """Converts a robot pose in axis-angle format to an affine matrix.
        Args:
            pose_aa (list): [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
            x, y, z are in mm and ax, ay, az are in radians.
        Returns:
            np.ndarray: 4x4 affine matrix [[R, t],[0, 1]]
        """

        rotation = R.from_rotvec(pose_aa[3:]).as_matrix()
        # translation = np.array(pose_aa[:3]) / SCALE_FACTOR
        translation = np.array(pose_aa[:3])  # --- IGNORE ---

        return np.block([[rotation, translation[:, np.newaxis]],
                        [0, 0, 0, 1]])