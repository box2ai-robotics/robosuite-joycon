"""
Driver class for Nintendo Switch Joy-Con
"""
import threading
import time
import numpy as np
from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix

from joyconrobotics import JoyconRobotics
import math

from scipy.spatial.transform import Rotation as R

class JoyConController(Device):
    def __init__(
        self,
        env, 
        device="right", # you could use ["right","left"]
        offset_position_m = [-0.100, 0.000, 1.000],
        offset_euler_rad = [0.00, 0.00, 0.00], # [2.184, 2.184, 0] # [0.00, 0.00, 0.00]
        active_robo = 0,
    ):
        # input
        if offset_position_m == []:
            raise("joycon use the absolute eef posture control need the offset_position_m of the robot")
        self.device = device
        self.offset_position_m = offset_position_m
        self.active_robo = active_robo
        # init
        self.joyconrobotics = {}
        self.init_pos = {}
        self.init_raw_rotation = {}
        self.init_rotation_matrix = {}
        
        self.joyconrobotics = JoyconRobotics(
                        device=self.device, 
                        offset_position_m=offset_position_m, 
                        offset_euler_rad=offset_euler_rad,
                        pitch_down_double=False,
                        euler_reverse=[-1, -1, 1],
                        direction_reverse = [1, -1, -1], # watch from the front
                        pure_xz = False,
                        gripper_state = 0,
                    )
        # init
        self.init_pos = offset_position_m
        self.init_raw_rotation = offset_euler_rad # [pitch, roll, yaw]
        r = R.from_euler('xyz', self.init_raw_rotation, degrees=True)
        self.init_rotation_matrix = r.as_matrix()
        
        
        # current
        self.rotation = self.init_rotation_matrix
        self.raw_drotation = np.array(self.init_raw_rotation)  # immediate roll, pitch, yaw delta values from keyboard hits
        self.last_drotation = np.array(self.init_raw_rotation)
        self.pos = self.init_pos  # (x, y, z)
        self.last_pos = np.array(self.init_pos)
        # self.grasp = 0
        self.grasp_states_ = 1
        self._reset_state = 0
        
        # 在不能跟super上函数相同的变量名
        super().__init__(env)
        self._display_controls()
        self._reset_internal_state()
        
        self.active_robot = active_robo
    
    def _display_controls(self):
        """
        Method to pretty print controls.
        """

        def print_command(char, info):
            char += " " * (30 - len(char))
            print("{}\t{}".format(char, info))

        print("")
        print_command("Keys", "Command")
        print_command("+/-", "reset simulation")
        print_command("Joystick press", "Downward in the direction the jaws are pointing")
        print_command("Upper trigger key", "Up in the upward direction")
        print_command("Joystick vertically", "Forward or backward along the front")
        print_command("Joystick laterally", "Move in the left and right directions")
        print_command("Lower trigger key", "toggle gripper (open/close)")
        print_command("Rotating Joycon", "rotate yaw, pitch, roll")
        print("")
        
    def get_controller_state(self): # just use to get the offset_position_m for joycon in device.input2action()
        """
        Grabs the current state of the keyboard.
        Returns:
            dict: A dictionary containing dpos, orn, unmodified orn, grasp, and reset
        """

        self.active_robot = self.active_robo
            
        # get joyconrobotics
        target_pose, gripper_state, button_control = self.joyconrobotics.get_control()
        # print(f'{gripper_state=}')
        self.pos = np.array(target_pose[0:3])
        self.raw_drotation = np.array([target_pose[4], target_pose[3], target_pose[5]])  # (target_pose[3:6]) # # [pitch, roll, yaw]
        
        # rotation
        r = R.from_euler('xyz', self.raw_drotation, degrees=True)
        self.rotation = r.as_matrix()
        raw_drotation = -(self.raw_drotation - self.last_drotation)
        self.last_drotation = np.array(self.raw_drotation)
        
        # pos
        dpos = self.pos - self.last_pos
        self.last_pos = np.array(self.pos)
        
        # others
        self.grasp_states_ = gripper_state
            
        # TODO button_control reset: self._reset_state = 1
        if button_control == 8:
            self._reset_state = 1
            self._enabled = False
            self._reset_internal_state()
        
        return dict(
            dpos=dpos,
            rotation=self.rotation,
            raw_drotation=raw_drotation,
            grasp=int(self.grasp_states_),
            reset=self._reset_state,
            base_mode=int(self.base_mode),
        )
    
    def start_control(self):
        self._reset_state = 0
        pass
    
    def _reset_internal_state(self): 
        """
        Resets internal state of controller, except for the reset signal.
        """
        super()._reset_internal_state()
        self.rotation = self.init_rotation_matrix
        self.raw_drotation =self.init_raw_rotation  # immediate roll, pitch, yaw delta values from keyboard hits
        self.last_drotation = self.init_raw_rotation
        self.pos = self.init_pos  # (x, y, z)
        self.last_pos = self.init_pos
        
        # TODO reset joycon robotics
    
    def _postprocess_device_outputs(self, dpos, drotation):
        drotation = drotation * 10
        dpos = dpos * 125

        dpos = np.clip(dpos, -1, 1)
        drotation = np.clip(drotation, -1, 1)

        return dpos, drotation
