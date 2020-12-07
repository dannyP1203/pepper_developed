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

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
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


def set_angle(motion_service, name, value):
    names  = [name]
    angles  = [value]
    times = 0.25
    isabsolute = True
    # motion_service.setAngles(names, angles, 0.1)
    motion_service.angleInterpolation(names, angles, times, isabsolute)
    time.sleep(0.1)


def get_angle(motion_service, name):
    useSensors  = True
    current_angle = motion_service.getAngles(name, useSensors)
    return current_angle



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Connect to Naoqi services
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args, unknown = parser.parse_known_args()

    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)


    # ROS Initialization
    rospy.init_node('teleop_node')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    # Head joint limits converted in radians
    min_pitch = -25.0 * almath.TO_RAD
    max_pitch = 13.0 * almath.TO_RAD
    min_yaw   = -100.0 * almath.TO_RAD
    max_yaw   = 100.0 * almath.TO_RAD

    increment = 5.0 * almath.TO_RAD

    # Set head stiffness to 1
    motion_service  = session.service("ALMotion")
    motion_service.setStiffnesses("Head", 1.0)

    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15

            elif key in headBindings.keys():
                joint = headBindings[key][0]
                direction = headBindings[key][1]
                current_pose = get_angle(motion_service,joint)
                target_angle = current_pose[0] + (direction * increment)
                target_angle = round(target_angle, 1)
                if joint == "HeadPitch":
                    if target_angle > min_pitch and target_angle < max_pitch:
                        set_angle(motion_service,joint,target_angle)
                    else:
                        print "Reached pitch limit!"
                elif joint == "HeadYaw":
                    if target_angle > min_yaw and target_angle < max_yaw:
                        set_angle(motion_service,joint,target_angle)
                    else:
                        print "Reached yaw limit!"

            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0

            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        motion_service.setStiffnesses("Head", 0.0)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
