#!/usr/bin/env python3

from time import time
import arm
import numpy as np

import hebi
from util.math_utils import get_grav_comp_efforts
from util.math_utils import gravity_from_quaternion

# Gather feedback from control arm
# Send the feedback as commands to the receive arm
# Track the difference between feedback and commaned efforts on the receive arm
# Send this feedback, scaled down, as added on efforts to the controller arm



# Control Arm setup
control_family_name = "Control"
control_module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
control_hrdf = "hrdf/A-2085-06G_control.hrdf"
control_params = arm.ArmParams(control_family_name, control_module_names, control_hrdf)
control_arm = arm.Arm(control_params)

# Receive Arm setup
receive_family_name = "Receive"
receive_module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
receive_hrdf = "hrdf/A-2085-06G_receive.hrdf"
receive_params = arm.ArmParams(receive_family_name, receive_module_names, receive_hrdf)
receive_arm = arm.Arm(receive_params)


# Main Variables
abort_flag = False
num_joints = receive_arm.group.size

# Startup time tracking
t0 = time()
t = t0
dur = 6 # startup over 6 seconds

# Set starting position
target_joints = [(0, np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0)]
control_arm.update()
receive_arm.update()
control_arm.setGoal(control_arm.createGoal(target_joints, duration=[5]))
receive_arm.setGoal(receive_arm.createGoal(target_joints, duration=[5]))
control_arm.send()
receive_arm.send()



while not abort_flag:

  # ToDo: For precision, adjust the hrdfs to include the weights of the pipe and pen?

  # Gather latest feedback from the arms
  control_arm.update()
  receive_arm.update()

  # Give time for startup
  if (t - t0) < dur:
    t = time()
    control_arm.send()
    receive_arm.send()
    continue

  if control_arm.at_goal and receive_arm.at_goal:
      control_arm.cancelGoal()
      receive_arm.at_goal = False

  # Read the Control Arm feedback
  control_feedback = control_arm.fbk

  # Send the Receive arm its new commands
  # receive_arm.cmd.position = control_feedback.position
  # receive_arm.cmd.velocity = control_feedback.velocity
  
  new_pos = np.empty((num_joints, 2))
  new_vel = np.empty((num_joints, 2))
  new_acc = np.empty((num_joints, 2))


  # Set positions
  new_pos[:,0] = receive_arm.fbk.position[:]
  new_pos[:,1] = control_arm.fbk.position[:]

  # Set velocities
  new_vel[:,0] = receive_arm.fbk.velocity[:]
  new_vel[:,1] = control_arm.fbk.velocity[:]

  # Set accel
  new_acc[:,0] = np.nan
  new_acc[:,1] = np.nan


  receive_arm.trajectory_plan = hebi.trajectory.create_trajectory([0, 0.05], 
                                                                  new_pos, 
                                                                  new_vel, 
                                                                  new_acc)
  receive_arm.setGoal()                                                          
  receive_arm.send()
  
  # Send effort feedback back to the control arm
  effort_diff = receive_arm.cmd.effort - receive_arm.fbk.effort
  grav_comp_effort = get_grav_comp_efforts(receive_arm.model, receive_arm.fbk.position, -receive_arm.gravity_vec)
  scale = 0.5
  control_arm.cmd.effort = grav_comp_effort + (scale * effort_diff)
  control_arm.send()

