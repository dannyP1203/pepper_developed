#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
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

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

import sys, select, termios, tty

import qi
import argparse
import time
import almath


msg = """
Control Your Pepper Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

Rotate:
   g         h

Moving head:
        w
   a    s    d

q/z : increase/decrease max speeds by 10%
r/v : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

# X-Y durections
moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,-1),
        'm':(-1,1),
           }

headingBindings = {
        'g':(1),
        'h':(-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'r':(1.1,1),
        'v':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

headBindings = {
        'w':("HeadPitch",1),
        'a':("HeadYaw",1),
        's':("HeadPitch",-1),
        'd':("HeadYaw",-1),
           }

current_angle = {
    'HeadPitch': 0.0,
    'HeadYaw'  : 0.0,
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .5
turn = 0.75

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


def js_callback(data):
    global current_angle
    current_angle['HeadPitch'] = data.position[data.name.index('HeadPitch')]
    current_angle['HeadYaw'] = data.position[data.name.index('HeadYaw')]
    # print current_angle['HeadPitch'],current_angle['HeadYaw']


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # ROS Initialization
    rospy.init_node('teleop_node')

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    head_pub = rospy.Publisher('/cmd_head', JointState, queue_size=5)
    js_sub = rospy.Subscriber('/joint_states', JointState, js_callback)

    x = 0
    y = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed_x = 0
    target_speed_y = 0
    target_turn = 0
    control_speed_x = 0
    control_speed_y = 0
    control_turn = 0

    # Head joint limits converted in radians
    min_pitch = -25.0 * almath.TO_RAD
    max_pitch = 13.0 * almath.TO_RAD
    min_yaw   = -100.0 * almath.TO_RAD
    max_yaw   = 100.0 * almath.TO_RAD
    bounds = {
        'HeadPitch':(min_pitch, max_pitch),
        'HeadYaw':(min_yaw, max_yaw),
    }

    increment = 5.0 * almath.TO_RAD

    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()

            # Linear base movements
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                count = 0

            # Heading base movements
            elif key in headingBindings.keys():
                th = headingBindings[key]
                count = 0

            # Bindins to modify speed
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15

            # Head movements
            elif key in headBindings.keys():
                joint = headBindings[key][0]
                direction = headBindings[key][1]
                current_pose = current_angle[joint]
                target_angle = current_pose + (direction * increment)
                target_angle = round(target_angle, 2)

                if target_angle > bounds[joint][0] and target_angle < bounds[joint][1]:
                    msg = JointState()
                    msg.name = [joint]
                    msg.position = [target_angle]
                    head_pub.publish(msg)
                else:
                    print "Reached joint limit!"

            # Stop keys
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed_x = 0
                control_speed_y = 0
                control_turn = 0

            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed_x = speed * x
            target_speed_y = speed * y
            target_turn = turn * th

            if target_speed_x > control_speed_x:
                control_speed_x = min( target_speed_x, control_speed_x + 0.02 )
            elif target_speed_x < control_speed_x:
                control_speed_x = max( target_speed_x, control_speed_x - 0.02 )
            else:
                control_speed_x = target_speed_x

            if target_speed_y > control_speed_y:
                control_speed_y = min( target_speed_y, control_speed_y + 0.02 )
            elif target_speed_y < control_speed_y:
                control_speed_y = max( target_speed_y, control_speed_y - 0.02 )
            else:
                control_speed_y = target_speed_y

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed_x; twist.linear.y = control_speed_y; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            cmd_pub.publish(twist)

    except:
        print "Error!"

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        cmd_pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
