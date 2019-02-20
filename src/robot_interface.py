#!/usr/bin/env python

import rospy
import argparse
import struct
import sys
import math

import baxter_interface

from group7_proj1.srv import *
from group7_proj1.msg import State

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

#set of joint angles for "home" shoulder angle
#element 0 is left, 1 is right
#values set in main()    

def get_Hand():
    hands= {'right','left'}
    if rospy.get_param('/hand_mode') == 'both':
        if rospy.get_param('/hand_switch') == 1:
            return 'right'
        else:
            return 'left'
    elif rospy.get_param('/hand_mode') in hands:
            return rospy.get_param('/hand_mode')
    else:
        return 0

def print_pos(limb):
    limb=baxter_interface.Limb(limb)
    rospy.sleep(0.1)
    end=limb.endpoint_pose()
    print("PRINTING CURRENT END EFFECTOR POSITION:")
    print(end['position'])

def print_angs(limb):
    limb=baxter_interface.Limb(limb)
    rospy.sleep(0.1)
    angs=limb.joint_angles()
    print("PRINTING CURRENT LIMB ANGLES:")
    print(angs)

def ik_test(limb,pos,ori):

    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
#    which_limb=baxter_interface.Limb(limb)

    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=pos[0],
                    y=pos[1],
                    z=pos[2],
                ),
                orientation=Quaternion(
                    x=ori[0],
                    y=ori[1],
                    z=ori[2],
                    w=ori[3],
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=pos[0],
                    y=pos[1],
                    z=pos[2],
                ),
                orientation=Quaternion(
                    x=ori[0],
                    y=ori[1],
                    z=ori[2],
                    w=ori[3],
                ),
            ),
        ),
    }

#    poses = {'left': PoseStamped(header=hdr,pose=Pose(position=Point(x=pos.x,y=pos.y,z=pos.z,),orientation=Quaternion(x=ori.x,y=ori.y,z=ori.z,w=ori.w,),),),
#        'right': PoseStamped(header=hdr,pose=Pose(position=Point(x=pos.x,y=pos.y,z=pos.z,),orientation=Quaternion(x=ori.x,y=ori.y,z=ori.z,w=ori.w,),),), }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {ikreq.SEED_USER: 'User Provided Seed',ikreq.SEED_CURRENT: 'Current Joint Angles',ikreq.SEED_NS_MAP: 'Nullspace Setpoints',}.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#        print "\nIK Joint Solution:\n", limb_joints
#        print "------------------"
#        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.") 
    joint_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))

#    print("*****************************************************\n")
#    print(joint_solution)
#    print("*****************************************************\n")
    print("IK SOLUTION ANGLES:")
    print(joint_solution)
    return joint_solution

def OpenGripper():
    if rospy.get_param('/gripper_open'):
        print("Gripper already open!")
        return False
    else:
        rospy.set_param('/gripper_open',True)
        if rospy.get_param('/bax_or_sym')=='baxter':
            gripper = baxter_interface.Gripper(get_Hand())
            rospy.sleep(1)
            gripper.open()
        return True

def CloseGripper():
    if rospy.get_param('/gripper_open'):
        rospy.set_param('/gripper_open',False)
        if rospy.get_param('/bax_or_sym')=='baxter':
            gripper = baxter_interface.Gripper(get_Hand())
            rospy.sleep(1)
            gripper.close()
        return True
    else:
        print("Gripper already closed!")
        return False

def locate_block(target_block):   # return stack and row that block is in
   block_state = rospy.get_param('/configuration')
   for stack in block_state:
      if target_block in stack:
         stackind = block_state.index(stack)
         rowind = stack.index(target_block)
         return [stackind,rowind]

def check_block_neighbor(target_block):   # check to see if block is on top of stack
   if target_block==0:  # 0 represents the ground, which is always clear
      return True
   [stack,row] = locate_block(target_block)
   if row == len(rospy.get_param('/configuration')[stack])-1:
      on_top = True
   else:
      on_top = False
   return on_top

def MoveTo(target_block):   # move gripper to desired block (if free)

#   print("MOVE TO CHECK")
#   print(rospy.get_param('/gripper_block'))
#   print(target_block)
#   print(rospy.get_param('/gripper_block')==target_block)
#   print("MOVE TO CHECK")

   if rospy.get_param('/gripper_block')==target_block:
      print("Gripper is already at target block. No move necessary.")
      return False
   else:
      if not rospy.get_param('/gripper_open'):
         print("Gripper is closed. Please request to open gripper before moving to a block.")
         return False
      else:
         # check that target is clear (ie. there is no block above the target block)
         target_clear = check_block_neighbor(target_block)
         if target_clear:
            rospy.set_param('/gripper_block', target_block)
            if rospy.get_param('/bax_or_sym')=='baxter':
                
                block_height = rospy.get_param('/block_z')
                hand=get_Hand()
                current_limb=baxter_interface.Limb(hand)
                rospy.sleep(0.1)
                current_baxter_position = current_limb.endpoint_pose()
                pos = current_baxter_position['position']
                ori = current_baxter_position['orientation']
                lift_position = [pos.x,pos.y,rospy.get_param('/max_block_height')]
                block_coords = rospy.get_param('/block_cart_coords')[target_block-1]

                if target_block==0:
                    hover_table_position = [pos.x,pos.y-2.2*block_height*get_param('/hand_switch'),rospy.get_param('/max_block_height')]

                    table_position =[pos.x,pos.y-2.2*block_height*rospy.get_param('/hand_switch'),rospy.get_param('/table_z')]

                    # check that there is not a block in this position
                    block_poss = rospy.get_param('/block_cart_coords')
                    i=0
                    while i < len(block_poss):
                          is_in_range = True
                          while is_in_range == True:
                            dist_to_block = table_position[1] - block_poss[i][1]
                            print(dist_to_block)
                            if math.fabs(dist_to_block) < 2*block_height:
                                table_position[1] = block_poss[i][1]-2.2*block_height*rospy.get_param('/hand_switch')
                                hover_table_position[1] = hover_table_position[1] - 2.2*block_height*rospy.get_param('/hand_switch')
                                i=0
                            else:
                                i=i+1
                                is_in_range = False	


                    j_pos_lift=ik_test(hand,lift_position,ori)

                    current_limb.move_to_joint_positions(j_pos_lift,20)
                    rospy.sleep(2)

                    j_pos_hov=ik_test(hand,hover_table_position,ori)

                    current_limb.move_to_joint_positions(j_pos_hov,20)
                    rospy.sleep(2)

                    j_pos_table=ik_test(hand,table_position,ori)
                    current_limb.move_to_joint_positions(j_pos_table,20)
                    rospy.sleep(1.25)

                else:         
                    block_ori = rospy.get_param('/block_orientation')[target_block-1]
                    hover_position = [block_coords[0],block_coords[1],rospy.get_param('/max_block_height')]
                    current_limb.move_to_joint_positions(ik_test(hand,lift_position,block_ori))
                    rospy.sleep(1.25)
                    current_limb.move_to_joint_positions(ik_test(hand,hover_position,block_ori))
                    rospy.sleep(1.25)
                    current_limb.move_to_joint_positions(ik_test(hand,block_coords,block_ori))
                    rospy.sleep(1.25)
            return True
         else:
            print("There are blocks on top of block "+str(target_block)+". Cannot move to it.")
            return False

def pop_block(block):
   [stack,row] = locate_block(block)
   on_top = check_block_neighbor(block)
   block_state = rospy.get_param('/configuration')
   if on_top:
      block_state[stack].pop(-1)
      rospy.set_param('/configuration',block_state)
   else:
      print("Block not on top, can't pop it")

def append_block(target_block):
   block_state = rospy.get_param('/configuration')
   gripper_block = rospy.get_param('/gripper_block')
   if target_block==0:
      block_state.append([gripper_block])
      rospy.set_param('/configuration',block_state)
   else:
      [stack,row] = locate_block(target_block)
      on_top = check_block_neighbor(target_block)
      if on_top:
         block_state[stack].append(gripper_block)
         rospy.set_param('/configuration',block_state)
      else:
         print('There\'s a block in the way. I can\'t put that there.')

def set_block_coords(coords,tar):
    temp_coord= rospy.get_param('/block_cart_coords')
    temp_coord[tar-1]=coords
    rospy.set_param('/block_cart_coords',temp_coord)
    return 0

def MoveOver(target_block):
    # move gripper over desired block
    # used for placing one block on another, gripper must be closed and have a block
    target_clear = check_block_neighbor(target_block)
    gripper_block = rospy.get_param('/gripper_block')
    if rospy.get_param('/gripper_open'):
        print("You can't place a block with the gripper open.")
        return False
    elif target_clear:
        pop_block(gripper_block)
        append_block(target_block)
        if rospy.get_param('/bax_or_sim')=='baxter':
            #hands = {'left','right', 'both'}
            #if rospy.get_param('/hand_mode') in hands:
                block_height = rospy.get_param('/block_z')
                hand = get_Hand()
                current_limb=baxter_interface.Limb(hand)
                rospy.sleep(0.1)
                current_baxter_position = current_limb.endpoint_pose()
                pos = current_baxter_position['position']
                ori = current_baxter_position['orientation']
                lift_position = [pos.x,pos.y,rospy.get_param('/max_block_height')]
                if target_block==0:
                    hover_table_position = [pos.x,pos.y-2.2*block_height*rospy.get_param('/hand_switch'),rospy.get_param('/max_block_height')]
                    table_position = [pos.x,pos.y-2.2*block_height*rospy.get_param('/hand_switch'),rospy.get_param('/table_z')]
                    # check that there is not a block in this position
                    block_poss = rospy.get_param('/block_cart_coords')
                    i=0
                    while i < len(block_poss):
                          is_in_range = True
                          while is_in_range == True:
                            dist_to_block = table_position[1] - block_poss[i][1]
                            print(dist_to_block)
                            if math.fabs(dist_to_block) < 2*block_height:
                                table_position[1] = block_poss[i][1]-2.2*block_height*rospy.get_param('/hand_switch')
                                hover_table_position[1] = hover_table_position[1] - 2.2*block_height*rospy.get_param('/hand_switch')
                                i=0
                            else:
                                i=i+1
                                is_in_range = False	
                          

                    current_limb.move_to_joint_positions(ik_test(hand,lift_position,ori))
                    rospy.sleep(1.25)
                    current_limb.move_to_joint_positions(ik_test(hand,hover_table_position,ori))
                    rospy.sleep(1.25)
                    current_limb.move_to_joint_positions(ik_test(hand,table_position,ori))
                else:
                    block_coords = rospy.get_param('/block_cart_coords')[target_block-1]
                    block_ori = rospy.get_param('/block_orientation')[target_block-1]
                    hover_position_1 = [pos.x,pos.y,rospy.get_param('/max_block_height')]
                    hover_position_2 = [block_coords[0],block_coords[1],rospy.get_param('/max_block_height')]
                    block_top_position = [block_coords[0],block_coords[1],block_coords[2]+block_height]
                    current_limb.move_to_joint_positions(ik_test(hand,lift_position,ori))
                    rospy.sleep(1.25)
                    current_limb.move_to_joint_positions(ik_test(hand,hover_position_1,block_ori))
                    rospy.sleep(1.25) # maybe not needed?
                    current_limb.move_to_joint_positions(ik_test(hand,hover_position_2,block_ori))
                    rospy.sleep(1.25) # maybe not needed?
                    current_limb.move_to_joint_positions(ik_test(hand,block_top_position,block_ori))
                new_coords=current_limb.endpoint_pose()['position']
                new_coords=[new_coords.x, new_coords.y, new_coords.z]
                set_block_coords(new_coords,rospy.get_param('/gripper_block'))
	
        return True
    else:
        print('Target block is not clear.')
        return False

def handle_move_robot(req):
    if req.Action=="Open":
        isPossible = OpenGripper()
    elif req.Action=="Close":
        isPossible = CloseGripper()
    elif req.Action=="MoveTo":
        isPossible = MoveTo(req.Target)
    elif req.Action=="MoveOver":
        isPossible = MoveOver(req.Target)
    elif req.Action == "Clear":
        isPossible = ClearArm(get_Hand())
    else:
        print("Please enter a valid action: Open, Close, MoveTo, or MoveOver")
    return isPossible

def robot_interface():
    rate = rospy.Rate(.1)
    while not rospy.is_shutdown():
        op  = rospy.get_param('/gripper_open')
        bl   = rospy.get_param('/gripper_block')
        conf = rospy.get_param('/configuration')
        h_switch = rospy.get_param('/hand_switch')
        if h_switch==1:
            arm = "Right"
        else:
            arm = "Left"    
        hello_str = State(op,bl,conf,arm)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def init_blocks():
    init_config = rospy.get_param('/configuration_type')
    num_blocks = rospy.get_param('/num_blocks')
    block_state = [[]]
    if init_config == 'stacked_ascending':
        for i in range (1,num_blocks+1):
            block_state[0].append(i)
    elif init_config == 'stacked_descending':
        for i in range (1,num_blocks+1):
            block_state[0].append(num_blocks+1-i)
    elif init_config == 'scattered':
        for i in range (1,num_blocks+1):
            block_state.append([i])
    else:
        print("Please enter a valid configuration type: 'stacked_ascending', 'stacked_descending', or 'scattered'.")
    rospy.set_param('/configuration',block_state)
    # initialize block poses by querying the robot position after stacking all blocks and moving robot arms manually to top of stack
    if rospy.get_param('/bax_or_sym')== 'baxter':
        hand = get_Hand()   
        current_limb=baxter_interface.Limb(hand)
        rospy.sleep(0.1) 


        # remove when running on real baxter
        r_limb=baxter_interface.Limb('right')
        rospy.sleep(0.1)
        l_limb=baxter_interface.Limb('left')
        rospy.sleep(0.1)
        angles = r_limb.joint_angles()
        angles['right_s0']=.75
        angles['right_s1']=-0.5
        angles['right_e0']=.25
        angles['right_e1']=1.5
        angles['right_w0']=0.5
        angles['right_w1']=0.5
        angles['right_w2']=0.0
        r_limb.move_to_joint_positions(angles)

        angles = l_limb.joint_angles()
        angles['left_s0']=-.75
        angles['left_s1']=-0.5
        angles['left_e0']=0
        angles['left_e1']=1.2
        angles['left_w0']=0.5
        angles['left_w1']=0.5
        angles['left_w2']=0.0
        l_limb.move_to_joint_positions(angles)

        # until here 
        r_grip=baxter_interface.Gripper('right')
        l_grip=baxter_interface.Gripper('left')
        r_grip.calibrate()
        l_grip.calibrate()
        print("calibrated")

        current_baxter_position = current_limb.endpoint_pose()
        pos = current_baxter_position['position']
        ori = current_baxter_position['orientation']
        blocksXYZ = [[0 for x in range(3)] for x in range(num_blocks)]
        blocksORI = [[ori.x , ori.y , ori.z, ori.w] for x in range(num_blocks)]
        blockHeight = rospy.get_param('/block_z')
        for i in range(num_blocks):
            blocksXYZ[i] = [pos.x , pos.y , pos.z-blockHeight*(num_blocks-i-1)]
        rospy.set_param('/table_z',pos.z-blockHeight*(num_blocks-1))
        rospy.set_param('/max_block_height',pos.z+blockHeight*3)
        rospy.set_param('/block_cart_coords',blocksXYZ)
        rospy.set_param('/block_orientation',blocksORI)
        
    
    OpenGripper()


def ClearArm(hand):
    if rospy.get_param('/bax_or_sym')=='baxter':
        limb=baxter_interface.Limb(hand)
        rospy.sleep(0.1)
        pose=limb.endpoint_pose()
        pos=pose['position']
        ori=pose['orientation']
        lift_position = [pos.x,pos.y,rospy.get_param('/max_block_height')]
        limb.move_to_joint_positions(ik_test(hand,lift_position,ori))
        rospy.sleep(2)
        angles = limb.joint_angles()
        if hand == 'right':
            angles['right_s0']=-0.2
        else:
            angles['left_s0']=0.2
        limb.move_to_joint_positions(angles)
        rospy.sleep(3)
    return True     #when should it be false?
    

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )

    args = parser.parse_args(rospy.myargv()[1:])

    return 0

if __name__ == "__main__":
    rospy.init_node('robot_interface')
    pub   = rospy.Publisher('current_state',State,queue_size=10)
    s = rospy.Service('move_robot', MoveRobot, handle_move_robot)
    init_blocks()
    robot_interface()
    sys.exit(main())

