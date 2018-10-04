#!/usr/bin/env python3

#This is a very basic example to drive a diff-drive robot using a phone with the Hebi app.

import hebi
from time import sleep
import numpy as np
from numpy.distutils.fcompiler import none

#wait for network interfaces go come up
sleep(10)

# Create lookup
lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate, and print out its contents
sleep(2)

for entry in lookup.entrylist:
    print(entry)

# Create a group
group = lookup.get_group_from_names(['HebiRobot'], ['_LeftWheel', '_RightWheel'])

while group == None:
    print('Wheels not found.')
    group = lookup.get_group_from_names(['HebiRobot'], ['_LeftWheel', '_RightWheel'])

phoneGroup = lookup.get_group_from_names(['HebiRobot'], ['Phone'])
    
while phoneGroup == None:
    print('Phone not found - set phone name to \"Phone\" and family to \"HebiRobot\"')
    phoneGroup = lookup.get_group_from_names(['HebiRobot'], ['Phone'])

print('Found group on network: size {0}'.format(group.size))
print('Found group on network: size {0}'.format(phoneGroup.size))

group_fbk = hebi.GroupFeedback(phoneGroup.size)

group.command_lifetime = 100

group_command = hebi.GroupCommand(group.size)

Left = 0
Right = 0
TurnScale = 0
TurnLimit = 0.5

SpeedScale = 5.0

while True:
    group_fbk = phoneGroup.get_next_feedback(reuse_fbk=group_fbk)
    
    if group_fbk == None:
        print("Phone connetion lost!")
        x = 0
        y = 0
        abort = 0
    else:    
        io_a = group_fbk.io.a
        io_b = group_fbk.io.b
        x = io_a.get_float(7)[0]
        y = io_a.get_float(8)[0]
        abort = io_b.get_int(1)[0]
    
    if y >= 0:
        if x <= 0:
            Left = 1.0
            Right = 1.0 - x
        else:
            Left = 1.0 + x
            Right = 1.0
    else:
        if x <= 0:
            Left = 1.0 - x
            Right = 1.0
        else:
            Left = 1.0
            Right = 1.0 + x
            
    Left = Left * y
    Right = Right * y
    
    if abs(y) > TurnLimit:
        TurnScale = 0
    else:
        TurnScale = 1 - abs(y/TurnLimit)
        
    Left = ((1-TurnScale)*Left + TurnScale*x)*SpeedScale
    Right = ((1-TurnScale)*Right + TurnScale*x*(-1.0))*SpeedScale
    
    if abort == 1:
        group_command.velocity = [0, 0]
        group.send_command(group_command)
        exit(0)
    
    group_command.velocity = [Left, -Right]
    group.send_command(group_command)


