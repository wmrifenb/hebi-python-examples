from enum import Enum, auto
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import time, sleep

import hebi
from hebi.util import create_mobile_io

import typing
if typing.TYPE_CHECKING:
    from typing import Optional
    import numpy.typing as npt
    from hebi._internal.mobile_io import MobileIO
    from hebi.arm import Arm


class ArmControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    TELEOP = auto()
    DISCONNECTED = auto()
    EXIT = auto()


class ArmMobileIOInputs:
    class TrajType(Enum):
        JOINT = auto()
        CARTESIAN = auto()

    def __init__(self,
                 m: 'MobileIO',
                 num_joints: 'int',
                 locked: bool,
                 active: bool,
                 gripper_closed: bool):

        self.mio = m
        
        self.num_joints = num_joints

        self.joint_dt = 0.2
        self.joint_velocity_max = np.pi
        self.joint_velocity = np.zeros(self.num_joints)
        self.prev_joint_velocity = np.zeros(self.num_joints)
        self.joint_displacement = np.zeros(self.num_joints)
        self.joint_direction = 1.0
        self.alpha_joint = 0.1

        self.cartesian_dt = 0.2
        self.cartesian_velocity_max = 1
        self.cartesian_velocity = np.zeros(3)
        self.prev_cartesian_velocity = np.zeros(3)
        self.cartesian_displacement = np.zeros(3)
        self.alpha_cartesian = 0.1

        self.mode = None

        self.locked = locked
        self.active = active
        self.gripper_closed = gripper_closed
    
    def clear_input(self):
        self.joint_velocity = np.zeros(self.num_joints)
        self.cartesian_velocity = np.zeros(3)
    
    def parse_mobile_feedback(self):
        if not self.mio.update(0.0):
            self.clear_input()
        
        self.joint_direction = -(int(self.mio.get_button_state(7)) * 2 - 1)

        for i in range(2, self.num_joints):
            self.joint_velocity[i] = int(self.mio.get_button_state(i-1)) * self.joint_velocity_max * self.joint_direction

        self.cartesian_velocity[0] = self.mio.get_axis_state(8) * self.cartesian_velocity_max
        self.cartesian_velocity[1] = -self.mio.get_axis_state(7) * self.cartesian_velocity_max
        self.cartesian_velocity[2] = self.mio.get_axis_state(5) * self.cartesian_velocity_max

        self.prev_cartesian_velocity = self.cartesian_velocity * self.alpha_cartesian + self.prev_cartesian_velocity * (1.0 - self.alpha_cartesian)    
        self.prev_joint_velocity = self.joint_velocity * self.alpha_joint + self.prev_joint_velocity * (1.0 - self.alpha_joint)
        if np.linalg.norm(self.prev_cartesian_velocity) > 1e-3:
            self.cartesian_displacement = self.prev_cartesian_velocity * self.cartesian_dt
            self.mode = self.TrajType.CARTESIAN
        elif np.linalg.norm(self.prev_joint_velocity) > 1e-2:
            self.joint_displacement = self.prev_joint_velocity * self.joint_dt
            self.mode = self.TrajType.JOINT
        else:
            self.joint_displacement = np.zeros(self.num_joints)
            self.cartesian_displacement = np.zeros(3)
            self.mode = None
        
        self.locked = m.get_button_state(6)
        self.active = m.get_button_state(5)
        self.gripper_closed = m.get_button_state(8)


class ArmControl:
    def __init__(self, arm: 'Arm'):
        self.state = ArmControlState.STARTUP
        self.arm = arm        
        self.arm_home = np.array([0.0, np.pi * 2 / 3, np.pi * 2 / 3, 0.0, np.pi / 2, 0.0])

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def send(self):
        self.arm.send()

    def update(self, t_now: float, arm_input: 'Optional[ArmMobileIOInputs]'):
        self.arm.update()
        self.arm.send()

        if self.state is self.state.EXIT:
            return False

        if not arm_input:
            if t_now - self.last_input_time > 1.0 and self.state is not self.state.DISCONNECTED:
                print("mobileIO timeout, disabling motion")
                self.transition_to(t_now, self.state.DISCONNECTED)
            return True
        else:
            self.last_input_time = t_now

            if self.state is self.state.DISCONNECTED:
                self.last_input_time = t_now
                self.transition_to(t_now, self.state.TELEOP)
                return True

            elif self.state is self.state.HOMING:
                if self.arm.at_goal:
                    self.transition_to(t_now, self.state.TELEOP)
                return True

            elif self.state is self.state.TELEOP:
                if arm_input.locked:
                    self.transition_to(t_now, self.state.HOMING)
                else:
                    arm_goal = self.compute_arm_goal(arm_input)
                    if arm_goal is not None:
                        self.arm.set_goal(arm_goal)

                gripper = self.arm.end_effector
                if gripper is not None:
                    gripper_closed = gripper.state == 1.0
                    if arm_input.gripper_closed and not gripper_closed:
                        gripper.close()
                    elif not arm_input.gripper_closed and gripper_closed:
                        gripper.open()

                return True

            elif self.state is self.state.STARTUP:
                self.transition_to(t_now, self.state.HOMING)
                return True

    def transition_to(self, t_now: float, state: ArmControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            print("TRANSITIONING TO HOMING")
            g = hebi.arm.Goal(self.arm.size)
            g.add_waypoint(position=self.arm_home)
            self.arm.set_goal(g)

        elif state is self.state.TELEOP:
            print("TRANSITIONING TO TELEOP")

        elif state is self.state.DISCONNECTED:
            print("TRANSITIONING TO STOPPED")

        elif state is self.state.EXIT:
            print("TRANSITIONING TO EXIT")

        self.state = state

    def compute_arm_goal(self, arm_inputs: ArmMobileIOInputs):
        arm_goal = hebi.arm.Goal(self.arm.size)

        # Get current joint position
        # We use the last position command for smoother motion
        cur_position = np.zeros(self.arm.size)
        self.arm.last_feedback.get_position_command(cur_position)

        if arm_inputs.mode is None or not arm_inputs.active:
            return None
        elif arm_inputs.mode is arm_inputs.TrajType.CARTESIAN:
            arm_xyz = np.zeros(3)
            arm_rot = np.zeros((3, 3))
            self.arm.FK(cur_position, xyz_out=arm_xyz, orientation_out=arm_rot)
            arm_xyz_target = arm_xyz + arm_inputs.cartesian_displacement

            joint_target = self.arm.ik_target_xyz_so3(
                cur_position,
                arm_xyz_target,
                arm_rot)

            arm_goal.add_waypoint(position=joint_target)
        elif arm_inputs.mode is arm_inputs.TrajType.JOINT:
            arm_goal.add_waypoint(position=cur_position + arm_inputs.joint_displacement)
        
        return arm_goal


def setup_mobile_io(m: 'MobileIO'):
    m.set_button_label(6, '⌂')
    m.set_button_label(5, 'arm')
    m.set_button_mode(5, 1)
    m.set_button_label(8, 'grip')
    m.set_button_mode(8, 1)
    m.set_button_mode(7, 1)

    m.set_axis_label(7, "y")
    m.set_axis_label(8, "x")
    m.set_snap(3, 0.0)
    m.set_snap(5, 0.0)
    m.set_button_label(7, '⟲')


if __name__ == "__main__":
    lookup = hebi.Lookup()
    sleep(2)

    # Arm setup
    arm_family = "Arm"
    module_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']
    hrdf_file = "hrdf/A-2085-06.hrdf"
    gains_file = "gains/A-2085-06.xml"

    # Create Arm object
    arm = hebi.arm.create([arm_family],
                          names=module_names,
                          hrdf_file=hrdf_file,
                          lookup=lookup)
    arm.load_gains(gains_file)

    arm_control = ArmControl(arm)

    # Setup MobileIO
    print('Looking for Mobile IO...')
    m = create_mobile_io(lookup, arm_family)
    while m is None:
        print('Waiting for Mobile IO device to come online...')
        sleep(1)
        m = create_mobile_io(lookup, arm_family)

    m.update()
    setup_mobile_io(m)

    # Arm inputs
    arm_inputs = ArmMobileIOInputs(m, arm.size, False, False, False)

    #######################
    ## Main Control Loop ##
    #######################

    while arm_control.running:
        t = time()
        arm_inputs.parse_mobile_feedback()
        arm_control.update(t, arm_inputs)

        arm_control.send()
