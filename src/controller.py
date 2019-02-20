#!/usr/bin/env python

import sys
import rospy
from group7_proj1.srv import *
from group7_proj1.msg import Command


def move_robot_client(act,tar):
   #possible inputs: act{open,close,moveto,moveover} tar{<block number>}
   rospy.wait_for_service('move_robot')
   try:
      move_robot = rospy.ServiceProxy('move_robot', MoveRobot)
      resp1 = move_robot(act,tar)
      if resp1.Move:
         return "Action successfully performed."
      else:
         return "Action not possible. State of system unchanged."
   except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      
def scatter_blocks():
    block_state = rospy.get_param('/configuration')
    num_stacks = len(block_state)
    for stack in block_state:
        if len(stack) != 0:
           # if False: #rospy.get_param('/hand_mode')== 'both':
            #    else:
            for j in range (1,len(stack)):
                move_robot_client('Open',0)
                move_robot_client('MoveTo',stack[len(stack)-j])
                move_robot_client('Close',0)
                move_robot_client('MoveOver',0)
                move_robot_client('Open',0)
                if rospy.get_param('/hand_mode')== 'both':                    
                    move_robot_client('Clear',0)
                    rospy.set_param('/hand_switch',rospy.get_param('/hand_switch')*-1)

def arrange_blocks(com):
    n = rospy.get_param('/num_blocks')
    if com == "Scatter":
        scatter_blocks()
        
    if com == "Stack_Ascending":
        scatter_blocks()
        for i in range(2,n+1):
            move_robot_client('Open',i)
            move_robot_client('MoveTo',i)
            move_robot_client('Close',i)
            move_robot_client('MoveOver',i-1)
            move_robot_client('Open',n-i)
            if rospy.get_param('/hand_mode')== 'both': 
                move_robot_client('Clear',n-i)
                rospy.set_param('/hand_switch',rospy.get_param('/hand_switch')*-1)
        
    if com == "Stack_Descending":
        scatter_blocks()
        for i in range(1,n):
            move_robot_client('Open',n-i)
            move_robot_client('MoveTo',n-i)
            move_robot_client('Close',n-i)
            move_robot_client('MoveOver',n-i+1)
            move_robot_client('Open',n-i)
            if rospy.get_param('/hand_mode')== 'both': 
                move_robot_client('Clear',n-i)
                rospy.set_param('/hand_switch',rospy.get_param('/hand_switch')*-1)

                              
def clean_up_stacks():
    block_state = rospy.get_param('/configuration')
    num_stacks = len(block_state)
    block_state = filter(None, block_state)
    rospy.set_param('/configuration',block_state)
    
def doCommand(c,b):
    move_commands = {'Open','Close','MoveTo','MoveOver'}
    arrange_commands = {'Scatter','Stack_Ascending','Stack_Descending'}
    if c in move_commands:
        print "%s"%(move_robot_client(c,b))
    if c in arrange_commands:
        print "You want me to %s"%(c)
        arrange_blocks(c)
    clean_up_stacks()
   
def callback(data):
    doCommand(data.desired_configuration,data.target_block)
    
def controller():
    rospy.init_node('controller',anonymous=True)
    rospy.Subscriber("command",Command,callback)
    rospy.spin()

if __name__ == "__main__":

   controller()
