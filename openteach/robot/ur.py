from openteach.ros_links.ur_control import DexArmControl 
from .robot import RobotWrapper

class URArm(RobotWrapper):
    def __init__(self, ip, record_type=None):
        self._controller = DexArmControl(ip=ip, record_type=record_type)
        self._data_frequency = 50

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state,
            'cartesian_states': self.get_cartesian_state,
            'gripper_state': self.get_gripper_state
        }

    @property
    def name(self):
        return 'ur'

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_joint_state(self):
        return self._controller.get_arm_joint_state()
    
    def get_joint_velocity(self):
        return self._controller.get_arm_velocity()

    def get_joint_torque(self):
        return self._controller.get_arm_torque()

    def get_cartesian_state(self):
        return self._controller.get_arm_cartesian_state()

    def get_joint_position(self):
        return self._controller.get_arm_position()
    
    def get_cartesian_position(self):
        return self._controller.get_arm_cartesian_coords()

    def get_osc_position(self):
        return self._controller.get_arm_osc_position()
    
    def get_pose(self):
        return self._controller.get_arm_pose()
    
    def get_gripper_state(self):
        return self._controller.get_gripper_state()

    # Movement functions
    def home(self):
        return self._controller.home_arm()

    def move(self, input_angles):
        self._controller.move_arm_joint(input_angles)

    def move_coords(self, cartesian_coords, duration=3):
        self._controller.move_arm_cartesian(cartesian_coords, duration=duration)

    def arm_control(self, cartesian_coords):
        self._controller.arm_control(cartesian_coords)

    def move_velocity(self, input_velocity_values, duration):
        pass
    
    def set_gripper_state(self, open):
        self._controller.set_gripper_state(open)