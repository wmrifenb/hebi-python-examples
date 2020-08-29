#!/usr/bin/env python

from time import time
import threading
import arm
import csv
import numpy as np

import wx


class HebiThread(threading.Thread):
  # Main thread for everything HEBI

  def __init__(self, arm):
    
    # create thread
    threading.Thread.__init__(self)
    self.arm = arm
    self.abort_flag = False

    # teach/repeat data
    self.waypoints = []
    self.grip_states = []
    self.durations = []
    self.run_mode = "training"
    self.playback_waypoint = 0
    
    # Currently these are unused as we are only using stop waypoints
    self.vels = []
    self.accels = []

    # start the thread
    self.start()

  def run(self):
    # Main control loop in this thread

    while not self.abort_flag:      
      # Update arm object
      self.arm.update()

      if (self.run_mode == "playback"):
        
        if self.arm.at_goal:
          print("Reached waypoint number:", self.playback_waypoint)
          # waypoint 0 is the position from where you start the playback
        
          if self.playback_waypoint == len(self.waypoints):
            self.playback_waypoint = 0

          # Set up next waypoint according to waypoint counter
          next_waypoint = self.waypoints[self.playback_waypoint]
          next_duration = self.durations[self.playback_waypoint]

          # Send the new commands
          self.arm.createGoal([next_waypoint], duration = [next_duration])
          self.arm.setGoal()
          self.arm.gripper.setState(self.grip_states[self.playback_waypoint])

          # Iterate waypoint counter
          self.playback_waypoint += 1

      self.arm.send()
      # app.MainLoop()

  def abort(self):
    self.abort_flag = True

  def add_waypoint(self):
    if self.run_mode == "training":
      print("Adding a waypoint")
      self.waypoints.append(self.arm.fbk.position)
      self.durations.append(5)      
      self.grip_states.append(self.arm.gripper.state)
    else:
      print("Did not add waypoint because you're not in training mode!")

  def add_waypoint_toggle(self):
    # Add 2 waypoinst to allow the gripper to open or close
    print("Stop waypoint added and gripper toggled")

    if self.run_mode == "training":
      # First waypoint
      self.waypoints.append(self.arm.fbk.position)
      self.durations.append(5)
      self.grip_states.append(self.arm.gripper.state)

      # Toggle the gripper
      self.arm.gripper.toggle()

      # Second waypoint
      self.waypoints.append(self.arm.fbk.position)
      self.durations.append(2) # time given for gripper toggle is shorter
      self.grip_states.append(self.arm.gripper.state) # this is now the new toggled state
    else:
      print("Did not add waypoint because you're not in training mode!")

  def playback_button(self):

    # This should only work during training mode
    if not self.run_mode == "training":
      print("You are already in playback mode.")
    else:
      # There should always be at least 2 waypoints for playback
      if len(self.waypoints) > 1:
        print("Starting playback of waypoints")
        self.run_mode = "playback"
        self.playback_waypoint = 0 # reset playback_waypoint before starting playback
        self.arm.gripper.open()
        self.arm.send()
      else:
        print("At least two waypoints are needed!")   

  def training_button(self):
    
    # This should only work during playback mode
    if not self.run_mode == "playback":
      print("You are already in training mode.")
    else:
      # Cancel goal and return to training mode
      print("Returning to training")
      self.run_mode = "training"
      self.arm.cancelGoal()

  def clear_waypoints(self):
    print("Cleared waypoints")
    self.waypoints = []
    self.durations = []
    self.grip_states = []

  def load_from_csv(self, file):

    # Read the csv document
    waypoint_reader = file.read()
    rows = waypoint_reader.split('\n')

    # First, we extract the descriptive information:
    # dofs, gripper, # of waypoints
    dof = int(rows[0])
    has_gripper = bool(rows[1])

    # Check that the opened file is compatible with current arm
    # If not, then return False
    curr_dof = self.arm.group.size
    curr_has_gripper = False if (self.arm.gripper is None) else True
    
    if (dof is not curr_dof) or (has_gripper is not curr_has_gripper):
      return False
    else:
      # Clear our current stores values
      self.waypoints = []
      self.grip_states = []
      self.durations = []
      self.run_mode = "training"
      self.playback_waypoint = 0
      self.vels = []
      self.accels = []

      # Disassemble the remaining csv, row by row, to extract desired information
      for row in rows[2:]:

        # Ignore any blank rows
        if not row:
          continue

        row_vals = row.split(',')

        # break the row up and store the appropriate values from
        # [duration, grip_state, positions[0->n], velocities[0->n], accels[0->n]]
        self.durations.append(float(row_vals[0]))
        if has_gripper:
          self.grip_states.append(float(row_vals[1]))
        waypoint_extraction = np.asarray(row_vals[ 2:2+dof ])
        self.waypoints.append(waypoint_extraction.astype(np.float))

        # When they are set to be used, they can accessed here:  
        # self.vels.append(np.asarray(row_vals[ 2+dof:2+2*dof ]))
        # self.accels.append(np.asarray(row_vals[ 2+2*dof: ]))
    
      print ("Durations:", self.durations)
      print ("Grip_states:", self.grip_states)
      print ("Waypoints:", self.waypoints)
      # When they are set to be used, they can be printed here: 
      # print ("Velocities:", self.vels)
      # print ("Accelerations:", self.accels)

  def save_to_csv(self, file):
    with file as save_file:
      waypoint_writer = csv.writer(save_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

      # first line will contain the degrees of freedom involved
      waypoint_writer.writerow([self.arm.group.size])
      
      # second line will say if there is a gripper or not
      grip_states_to_save = []
      if self.arm.gripper is None:
        waypoint_writer.writerow([False])
        grip_states_to_save = [0] * len(self.durations) # 0 also means the gripper remains open
      else:
        waypoint_writer.writerow([True])
        grip_states_to_save = self.grip_states

      # Create empty arrays for vels and accels
      # set to 0 for stop waypoints
      vels = [0] * self.arm.group.size
      accels = [0] * self.arm.group.size

      # Construct the mea of the waypoint saving
      # The structure is:
      # [duration, grip_state, positions[0->n], velocities[0->n], accels[0->n]]
      for i in range (len(self.waypoints)):
        waypoint_writer.writerow([self.durations[i]] +
                                 [grip_states_to_save[i]] +
                                 self.waypoints[i].tolist() + 
                                 vels +
                                 accels)



class TeachRepeatFrame(wx.Frame):

  def __init__(self, arm, *args, **kw):
    # ensure the parent's __init__ is called
    super(TeachRepeatFrame, self).__init__(*args, **kw)

    # Create & start the background thread for the HEBI arm
    # It should pass the arm object that is created
    self.hebi_thread = HebiThread(arm)

    # Save & load variables
    self.contentSaved = False # currently not in use anymore

    # Center the frame on the screen, declare size
    self.SetSize((1200,600))
    self.Centre()
    
    # Create a panel in the frame
    self.panel_main = wx.Panel(self)
    self.panel_main.SetBackgroundColour('#8f8f8f')

    # Set status bar
    self.CreateStatusBar()
    self.SetStatusText("HEBI Teach & Repeat GUI")

    # Create a sizer to manage the layout of child widgets
    self.main_box = wx.BoxSizer(wx.HORIZONTAL)

    # Create the buttons on the left
    self.create_left_bar_buttons()

    # Add spacer to separate left bar from the rest of the conentts 
    self.main_box.AddSpacer(50)

    # Create the waypoints ui
    self.create_waypoints_ui()
    

  def create_left_bar_buttons(self):
    vbox_buttons = wx.BoxSizer(wx.VERTICAL)

    ### Create the left bar of buttons
    vbox_buttons.AddSpacer(30)

    # Button: Load Waypoints
    load_button = wx.Button(self.panel_main, label = "Load Waypoints")
    load_button.Bind(wx.EVT_BUTTON, self.on_load)
    vbox_buttons.Add(load_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Save Waypoints
    save_button = wx.Button(self.panel_main, label = "Save Waypoints")
    save_button.Bind(wx.EVT_BUTTON, self.on_save)
    vbox_buttons.Add(save_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(100)

    # Button: Add Waypoint
    add_button = wx.Button(self.panel_main, label = "Add Waypoint")
    add_button.Bind(wx.EVT_BUTTON, self.on_add_waypoint)
    vbox_buttons.Add(add_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Add Waypoint w/ Gripper Toggle
    add_toggle_button = wx.Button(self.panel_main, label = "Add Waypoint w/ Gripper Toggle")
    add_toggle_button.Bind(wx.EVT_BUTTON, self.on_add_waypoint_toggle)
    vbox_buttons.Add(add_toggle_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Clear Waypoints
    add_toggle_button = wx.Button(self.panel_main, label = "Clear Waypoints")
    add_toggle_button.Bind(wx.EVT_BUTTON, self.on_clear)
    vbox_buttons.Add(add_toggle_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(100)

    # Button: Start Playback
    play_button = wx.Button(self.panel_main, label = "Start Playback")
    play_button.Bind(wx.EVT_BUTTON, self.on_playback_button)
    vbox_buttons.Add(play_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Return to Training
    training_button = wx.Button(self.panel_main, label = "Return to Training")
    training_button.Bind(wx.EVT_BUTTON, self.on_training_button)
    vbox_buttons.Add(training_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Quit the program
    quit_button = wx.Button(self.panel_main, label = "Quit")
    quit_button.Bind(wx.EVT_BUTTON, self.on_exit)
    vbox_buttons.Add(quit_button, flag=wx.EXPAND)

    # Add the left bar sizer into the overall sizer
    self.main_box.Add(vbox_buttons, proportion = 1, flag = wx.EXPAND | wx.ALL)

  def create_waypoints_ui(self):
    self.vbox_waypoints = wx.BoxSizer(wx.VERTICAL)

    ### Add the grid of trajectory things
    self.vbox_waypoints.AddSpacer(50)
    
    columns = self.hebi_thread.arm.group.size + 3 # The 3 are for waypoint number, duration, and grip_state
    rows = len(self.hebi_thread.waypoints) + 2 # The 1 is for headers

    waypoint_grid = wx.GridSizer(rows, columns, 20, 20)

    # Set the headers for the columns
    waypoint_grid.AddMany([ wx.StaticText(self.panel_main, label='Waypoint', style=wx.ALIGN_CENTER),
                            wx.StaticText(self.panel_main, label='Duration', style=wx.ALIGN_CENTER_HORIZONTAL),
                            wx.StaticText(self.panel_main, label='Grip_State', style=wx.ALIGN_CENTRE_HORIZONTAL) ])
  
    for i in range(columns-3):
      # print (i)
      header = "Joint %d" % (i+1)
      # print(header)
      waypoint_grid.Add(wx.StaticText(self.panel_main, label=header))

    # if there are any waypoints, we then also show them on the screen
    if len(self.hebi_thread.waypoints) > 0:
      for i in range(len(self.hebi_thread.waypoints)):

        # TODO: Down the line, this text could be turned into a button, which allows you to retrain this waypoint
        # Waypoint Number
        text_waypoint = wx.TextCtrl(self.panel_main, value = str(i+1), style =  wx.TE_CENTER)
        waypoint_grid.Add(text_waypoint)

        # Duration 
        text_duration = wx.TextCtrl(self.panel_main, value = str(self.hebi_thread.durations[i]), style =  wx.TE_CENTER)
        waypoint_grid.Add(text_duration)
        
        # Grip_State
        if self.hebi_thread.arm.gripper is None:
          grip_val = 0
        else:
          grip_val = self.hebi_thread.grip_states[i]
        text_grip = wx.TextCtrl(self.panel_main, value = str(grip_val), style =  wx.TE_CENTER)
        waypoint_grid.Add(text_grip)

        # Waypoints
        joints = self.hebi_thread.waypoints[i].tolist()
        # joints_list = joints.tolist()
        for joint in joints:
          text_joint = wx.TextCtrl(self.panel_main, value = str(round(joint, 2)), style =  wx.TE_CENTER)
          waypoint_grid.Add(text_joint)

    # waypoint_grid.AddMany([ (wx.Button(self.panel_main, label='Waypoint'), 0, wx.EXPAND),
    #                         (wx.Button(self.panel_main, label='dur'), 0, wx.EXPAND),
    #                         (wx.Button(self.panel_main, label='closed'), 0, wx.EXPAND),
    #                         (wx.Button(self.panel_main, label='1'), 0, wx.EXPAND),
    #                         (wx.Button(self.panel_main, label='2'), 0, wx.EXPAND),
    #                         (wx.Button(self.panel_main, label='3'), 0, wx.EXPAND),
    #                         (wx.Button(self.panel_main, label='4'), 0, wx.EXPAND),
    #                         (wx.Button(self.panel_main, label='5'), 0, wx.EXPAND),
    #                         (wx.Button(self.panel_main, label='6'), 0, wx.EXPAND) ])

    # t1 = wx.TextCtrl(self.panel_main, value = "0.123456789", style =  wx.TE_CENTER)
    # t1.SetMaxLength(8)
    # waypoint_grid.Add(t1,1, wx.EXPAND) 

    self.vbox_waypoints.Add(waypoint_grid, flag = wx.ALIGN_CENTER)
    self.main_box.Add(self.vbox_waypoints, proportion = 3, flag=wx.EXPAND)

    self.panel_main.SetSizer(self.main_box)
    self.panel_main.Layout()  

  def on_exit(self, event):
    # Close the frame, terminating the application
    self.hebi_thread.abort()
    self.Close(True)

  def on_load(self, event):
    # self.contentSaved = False
    # if not self.contentSaved:
    #     if wx.MessageBox("Current waypoint has not been saved! Proceed?", "Lose current ",
    #                      wx.ICON_QUESTION | wx.YES_NO, self) == wx.NO:
    #         return

    if self.hebi_thread.run_mode is "playback":
      wx.MessageBox("You can't load waypoints during playback. Please switch back to training mode.", "Switch back to training mode")
    else:
      with wx.FileDialog(self, "Load waypoints from CSV file", wildcard="CSV files (*.csv)|*.csv",
                        style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as fileDialog:

        if fileDialog.ShowModal() == wx.ID_CANCEL:
          return     # the user changed their mind

        # Proceed loading the file chosen by the user
        pathname = fileDialog.GetPath()
        try:
          with open(pathname, 'r') as file:      
            load = self.hebi_thread.load_from_csv(file)
            if load is False: # this means the load was unsuccesful
              wx.MessageBox("The file you are loading is not compatible with your current arm. There is a mismatch in your number of joints or gripper.",
                          "Incompatible Save File")
            else: # laod was succesful
              wx.MessageBox("Waypoints succesfully loaded.", "Waypoints Loaded")
              # refresh the backdrop
              self.refresh_waypoints_ui()

        except IOError:
          wx.LogError("Cannot open file '%s'." % newfile)

  def on_save(self, event):
    # This button will print an error that is to be EXPECTED on macos.
    # This error does not affect functionality, and is apparantely unavoidable.
    
    if self.hebi_thread.run_mode is "playback":
      wx.MessageBox("You can't save waypoints during playback. Please switch back to training mode.", "Switch back to training mode")
    else:
      # Check that there is at least one waypoint to save
      if len(self.hebi_thread.waypoints) < 1:
        wx.MessageBox("You need to have at least one waypoint before you can save.", "You have not recorded any waypoints")

      # Knowing there is at least one waypoint to save, we proceed
      else:
        with wx.FileDialog(self, "Save waypoints as csv file", wildcard="CSV files (*.csv)|*.csv",
                          style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT) as fileDialog:

          if fileDialog.ShowModal() == wx.ID_CANCEL:
            return     # the user changed their mind

          # save the current contents in the file
          pathname = fileDialog.GetPath()
          try:
            with open(pathname, 'w') as file:
              self.hebi_thread.save_to_csv(file)
          except IOError:
            wx.LogError("Cannot save current data in file '%s'." % pathname)

  def on_add_waypoint(self, event):
    if self.hebi_thread.run_mode is "playback":
      wx.MessageBox("You can't add waypoints during playback. Please switch back to training mode.", "Switch back to training mode")
    else:
      self.hebi_thread.add_waypoint()

      # refresh the backdrop
      self.refresh_waypoints_ui()
      
  def on_add_waypoint_toggle(self, event):
    if self.hebi_thread.run_mode is "playback":
      wx.MessageBox("You can't add waypoints during playback. Please switch back to training mode.", "Switch back to training mode")
    else:
      self.hebi_thread.add_waypoint_toggle()
      # refresh the backdrop
      self.refresh_waypoints_ui()

  def on_clear(self, event):
    if self.hebi_thread.run_mode is "playback":
      wx.MessageBox("You can't clear waypoints during playback. Please switch back to training mode.", "Switch back to training mode")
    else:
      self.hebi_thread.clear_waypoints()
      # refresh the backdrop
      self.refresh_waypoints_ui()

  def on_playback_button(self, event):
    self.hebi_thread.playback_button()

  def on_training_button(self, event):
    self.hebi_thread.training_button()

  def refresh_waypoints_ui(self):
      self.main_box.Hide(self.vbox_waypoints)
      self.main_box.Remove(self.vbox_waypoints)
      self.create_waypoints_ui()
      self.panel_main.Layout()