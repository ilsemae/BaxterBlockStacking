#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Example
"""
import argparse
import struct
import sys

import rospy
import baxter_interface

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

def print_pos(limb):
    limb=baxter_interface.Limb(limb)
    rospy.sleep(0.1)
    end=limb.endpoint_pose()
    print("PRINTING CURRENT END EFFECTOR POSITION:")
    print(end['position'])

def ik_test(limb, dx, dy, dz):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    left_limb=baxter_interface.Limb('left')
    rospy.sleep(0.1)
    right_limb=baxter_interface.Limb('right')
    rospy.sleep(0.1)
#    left_limb.move_to_neutral()
#    right_limb.move_to_neutral()
    left_end=left_limb.endpoint_pose()
    left_pos=left_end['position']
    left_or=left_end['orientation']
#    right_end=right_limb.endpoint_pose()
#    right_pos=right_end['position']
#    right_or=right_end['orientation']
    xn=left_pos.x
    yn=left_pos.y
    zn=left_pos.z
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=xn+dx,
                    y=yn+dy,
                    z=zn+dz,
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
                    z=0.0388609422173,
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
 #       print "\nIK Joint Solution:\n", limb_joints
 #       print "------------------"
 #       print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.") 
    joint_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))
    # set arm joint positions to solution
#    arm = baxter_interface.Limb(limb)
#    if not rospy.is_shutdown():
#    print("*****************************************************\n")
#    print(joint_solution)
#    print("*****************************************************\n")   
        
#        arm.set_joint_positions(joint_solution)


    return joint_solution


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    rospy.init_node("rsdk_ik_service_client")
    args = parser.parse_args(rospy.myargv()[1:])
    l_grip=baxter_interface.Gripper('left')
    l_limb=baxter_interface.Limb('left')
    d_block=.04445
#    ik_test(args.limb, 0, 0, 0)
#    print("step0\n")
    
    l_ang=l_limb.joint_angles()
    
    l_ang['left_s0']=-.4

    print_pos('left')
    l_limb.set_joint_positions(ik_test(args.limb, 0, 0, .1))
    rospy.sleep(3)
    print_pos('left')
    
    
#    print("step1\n")
    
#    rospy.sleep(1)
#    l_grip.close()
#    rospy.sleep(1)
#    l_grip.open()
#    print("grip close \n")
    
#    rospy.sleep(3)
#    l_limb.set_joint_positions(ik_test(args.limb, 0, 0, d_block))    
#    print("step2\n")
    
#    rospy.sleep(3)
#    l_limb.set_joint_positions(ik_test(args.limb, 0, 2*d_block, 0))    
#    print("step3\n")
    
#    rospy.sleep(3)
#    l_limb.set_joint_positions(ik_test(args.limb, 0, 0, -2*d_block))
#    print("step4\n")
    
#    rospy.sleep(1)
#    l_grip.open()
    
    print("done\n")
    return 0

if __name__ == '__main__':
    sys.exit(main())
