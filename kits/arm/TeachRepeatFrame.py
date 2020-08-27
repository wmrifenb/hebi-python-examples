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
      print("You can't start playback unless you are in training mode!")

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
      print("You can't return to training unless you are in playback mode!")
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

# STILL TO DO:
# DONE Add a new button and function for "return to training mode" instead of toggle playback
# DONE Finish the "run" logic, so that the arm is sending the commands out to the arm
# DONE this will complete the teach repeat example, gui example. Commit that.

# ToDo: Gotta make sure that packet loss doesn't make the arm jerk around.
# ToDo: Address the gripper issue of retraining later and not capturing proper gripper points

# Now to add the saving and loading of waypoints (after a quick clean up)
# Save the important data (waypoints, durations, and grip states) to a file
# when loading, confirm that joints line up, and that there is a gripper

# Now to add the visual part (after a quick clean up):
# Add the waypoint representation in the gui.
# Have it show new waypoints as they are added.



class TeachRepeatFrame(wx.Frame):

  def __init__(self, arm, *args, **kw):
    # ensure the parent's __init__ is called
    super(TeachRepeatFrame, self).__init__(*args, **kw)

    # Create & start the background thread for the HEBI arm
    # It should pass the arm object that is created
    self.hebi_thread = HebiThread(arm)

    # Save & load variables
    self.contentSaved = False

    # Center the frame on the screen, declare size
    self.SetSize((1000,600))
    self.Centre()
    
    # Create a panel in the frame
    panel_main = wx.Panel(self)
    panel_main.SetBackgroundColour('#8f8f8f')

    # Set status bar
    self.CreateStatusBar()
    self.SetStatusText("HEBI Teach & Repeat GUI")

    # Create a sizer to manage the layout of child widgets
    main_box = wx.BoxSizer(wx.HORIZONTAL)
    vbox_buttons = wx.BoxSizer(wx.VERTICAL)
    vbox_waypoints = wx.BoxSizer(wx.VERTICAL)

    ### Create the left bar of buttons
    vbox_buttons.AddSpacer(10)

    # Button: Load Waypoints
    load_button = wx.Button(panel_main, label = "Load Waypoints")
    load_button.Bind(wx.EVT_BUTTON, self.on_exit)
    vbox_buttons.Add(load_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Save Waypoints
    save_button = wx.Button(panel_main, label = "Save Waypoints")
    save_button.Bind(wx.EVT_BUTTON, self.on_save)
    vbox_buttons.Add(save_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(100)

    # Button: Add Waypoint
    add_button = wx.Button(panel_main, label = "Add Waypoint")
    add_button.Bind(wx.EVT_BUTTON, self.on_add_waypoint)
    vbox_buttons.Add(add_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Add Waypoint w/ Gripper Toggle
    add_toggle_button = wx.Button(panel_main, label = "Add Waypoint w/ Gripper Toggle")
    add_toggle_button.Bind(wx.EVT_BUTTON, self.on_add_waypoint_toggle)
    vbox_buttons.Add(add_toggle_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Clear Waypoints
    add_toggle_button = wx.Button(panel_main, label = "Clear Waypoints")
    add_toggle_button.Bind(wx.EVT_BUTTON, self.on_clear)
    vbox_buttons.Add(add_toggle_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(100)

    # Button: Start Playback
    play_button = wx.Button(panel_main, label = "Start Playback")
    play_button.Bind(wx.EVT_BUTTON, self.on_playback_button)
    vbox_buttons.Add(play_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Return to Training
    training_button = wx.Button(panel_main, label = "Return to Training")
    training_button.Bind(wx.EVT_BUTTON, self.on_training_button)
    vbox_buttons.Add(training_button, flag=wx.EXPAND)

    vbox_buttons.AddSpacer(10)

    # Button: Quit the program
    quit_button = wx.Button(panel_main, label = "Quit")
    quit_button.Bind(wx.EVT_BUTTON, self.on_exit)
    vbox_buttons.Add(quit_button, flag=wx.EXPAND)

    # Add the left bar sizer into the overall sizer
    main_box.Add(vbox_buttons, proportion = 1, flag = wx.EXPAND | wx.ALL)

    # Add spacer to separate left bar from the rest of the conentts 
    main_box.AddSpacer(50)

    ### Add the grid of trajectory things
    vbox_waypoints.AddSpacer(50)
    
    waypoint_grid = wx.GridSizer(2,2,1,1)

    waypoint_grid.AddMany([(wx.Button(panel_main, label='1'), 0, wx.EXPAND),
                (wx.Button(panel_main, label='2'), 0, wx.EXPAND),
                (wx.Button(panel_main, label='3'), 0, wx.EXPAND),
                (wx.Button(panel_main, label='4'), 0, wx.EXPAND) ])

    vbox_waypoints.Add(waypoint_grid, flag = wx.ALIGN_CENTER)
    main_box.Add(vbox_waypoints, proportion = 3, flag=wx.EXPAND | wx.ALL)

    panel_main.SetSizer(main_box)
    panel_main.Layout()
    


  def on_exit(self, event):
    """Close the frame, terminating the application."""
    self.hebi_thread.abort()
    self.Close(True)

  def on_hello(self, event):
    """Say hello to the user."""
    wx.MessageBox("Hello again from wxPython")


  # def OnOpen(self, event):

  #   if not self.contentSaved:
  #       if wx.MessageBox("Current content has not been saved! Proceed?", "Please confirm",
  #                        wx.ICON_QUESTION | wx.YES_NO, self) == wx.NO:
  #           return

  #   # otherwise ask the user what new file to open
  #   with wx.FileDialog(self, "Open XYZ file", wildcard="XYZ files (*.xyz)|*.xyz",
  #                      style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as fileDialog:

  #       if fileDialog.ShowModal() == wx.ID_CANCEL:
  #           return     # the user changed their mind

  #       # Proceed loading the file chosen by the user
  #       pathname = fileDialog.GetPath()
  #       try:
  #           with open(pathname, 'r') as file:
  #               self.doLoadDataOrWhatever(file)
  #       except IOError:
  #           wx.LogError("Cannot open file '%s'." % newfile)


  def save_to_csv(self, file):
    with file as save_file:
      waypoint_writer = csv.writer(save_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

      # first line will contain the degrees of freedom involved
      waypoint_writer.writerow([self.hebi_thread.arm.group.size])
      
      # second line will say if there is a gripper or not
      grip_states_to_save = []
      if self.hebi_thread.arm.gripper is None:
        waypoint_writer.writerow([False])
        grip_states_to_save = [0] * len(self.hebi_thread.durations) # 0 also means the gripper remains open
      else:
        waypoint_writer.writerow([True])
        grip_states_to_save = self.hebi_thread.grip_states
      
      # Create empty arrays for vels and accels
      # set to 0 for stop waypoints
      vels = [0] * self.hebi_thread.arm.group.size
      accels = [0] * self.hebi_thread.arm.group.size

      # Construct the mea of the waypoint saving
      # The structure is:
      # [duration, grip_state, positions[0->n], velocities[0->n], accels[0->n]]
      for i in range (len(self.hebi_thread.waypoints)):
        waypoint_writer.writerow([self.hebi_thread.durations[i]] +
                                 [grip_states_to_save[i]] +
                                 self.hebi_thread.waypoints[i].tolist() + 
                                 vels +
                                 accels)


  def on_save(self, event):
    # This button will print an error that is to be EXPECTED on macos.
    # This error does not affect functionality, and is apparantely unavoidable.
    
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
            self.save_to_csv(file)
        except IOError:
          wx.LogError("Cannot save current data in file '%s'." % pathname)










  def on_add_waypoint(self, event):
    self.hebi_thread.add_waypoint()

  def on_add_waypoint_toggle(self, event):
    self.hebi_thread.add_waypoint_toggle()

  def on_clear(self, event):
    self.hebi_thread.clear_waypoints()

  def on_playback_button(self, event):
    self.hebi_thread.playback_button()

  def on_training_button(self, event):
    self.hebi_thread.training_button()

