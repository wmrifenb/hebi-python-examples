#!/usr/bin/env python3

from time import time
import arm
from TeachRepeatFrame import TeachRepeatFrame
from TeachRepeatFrame import HebiThread
import wx
import threading

# Set up arm
family_name  = "Arm"
module_names = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
hrdf = "hrdf/A-2085-06G_custom.hrdf"
gripper_name = "gripperSpool"
p = arm.ArmParams(family_name, module_names, hrdf, hasGripper=True, gripperName=gripper_name)
a = arm.Arm(p)
a.loadGains("gains/A-2085-06.xml")
a.gripper.loadGains("gains/gripper_spool_gains.xml")
a.gripper.open()

# gui_thread = threading.Thread(target=hebi_thread, args=(a, data, ))
# gui_thread.start()


# Set up GUI
app = wx.App()
frm = TeachRepeatFrame(a, None, title='HEBI Teach & Repeat GUI')
frm.Show()
# frm.Layout()
app.MainLoop()




