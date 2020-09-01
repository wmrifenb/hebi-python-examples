#!/usr/bin/env python3

from time import time
import arm

# Gather feedback from control arm
# Send the feedback as commands to the receiver arm
# Track the difference between feedback and commaned efforts on the receiver arm
# Send this feedback, scaled down, as added on efforts to the controller arm



# Control Arm setup
control_family_name = "Control Arm"]
control_module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
control_hrdf = "hrdf/A-2085-06G_webinar.hrdf"
control_params = arm.ArmParams(control_family_name, control_module_names, control_hrdf)
control_arm = arm.Arm(control_params)

# Receiver Arm setup
receiver_family_name = "Receiver Arm"]
receiver_module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
receiver_hrdf = "hrdf/A-2085-06G_webinar.hrdf"
receiver_params = arm.ArmParams(receiver_family_name, receiver_module_names, receiver_hrdf)
receiver_arm = arm.Arm(receiver_params)


# Main Variables
abort_flag = False


while not abort_flag:

  # TODO: May need to a 'startup' to some home position from where I would start commanding both of them.]

  # Gather latest feedback from the arms
  control_arm.update()
  receiver_arm.update()

  # Read the Control Arm feedback
  control_feedback = control_arm.fbk

  # Send the Receiver arm its new commands
  receiver_arm.createGoal(control_feedback.position, velocity=control_feedback.velocity)
  receiver_arm.send()

  # Send effort feedback back to the control arm
  effort_diff = receiver_arm.cmd.effort - receiver_arm.fbk.effort
  grav_comp_effort = get_grav_comp_efforts(receiver_arm.model, receiver_arm.fbk.position, -receiver_arm.gravity_vec)
  scale = 0.5
  control_arm.cmd.effort = grav_comp_effort + (scale * effort_diff)
  controller_arm.send()
