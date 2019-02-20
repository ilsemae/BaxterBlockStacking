#!/usr/bin/env python

import rospy

# baxter_interface - Baxter Python API
import baxter_interface

import argparse
import struct
import sys

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


def ik_test(limb):
    #rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    left_limb=baxter_interface.Limb('left')
    right_limb=baxter_interface.Limb('right')
    left_limb.move_to_neutral()
    right_limb.move_to_neutral()
    left_end=left_limb.endpoint_pose()
    left_pos=left_end['position']
    left_or=left_end['orientation']
#    right_end=right_limb.endpoint_pose()
#    right_pos=right_end['position']
#    right_or=right_end['orientation']
    
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=left_pos.x-.1,
                    y=left_pos.y-.1,
                    z=left_pos.z+.1,
                ),
                orientation=Quaternion(
                    x=left_or.x,
                    y=left_or.y,
                    z=left_or.z,
                    w=left_or.w,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=14.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }

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
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.") 
    joint_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))
    # set arm joint positions to solution
    arm = baxter_interface.Limb(limb)
    while not rospy.is_shutdown():
        print("*****************************************************\n")
    	print(joint_solution)
    	print("*****************************************************\n")   
    
        arm.set_joint_positions(joint_solution)
        rospy.sleep(3)

    return 0


ik_test('left')
