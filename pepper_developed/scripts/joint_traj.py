#!/usr/bin/env python

import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__=="__main__":
    rospy.init_node('joint_traj')
    controller_pub = rospy.Publisher("/pepper/Head_controller/command", JointTrajectory, queue_size=10)

    rospy.sleep(3.0)

    # while True:

    jt = JointTrajectory()
    jt.joint_names.append("HeadYaw" )
    # jt.joint_names.append("HeadPitch" )
    jt.header.stamp = rospy.Time.now()

    jtp = JointTrajectoryPoint()
    jtp.positions.append(50.0)
    # jtp.positions.append(10.0)
    jtp.time_from_start = rospy.Duration(0.25)

    jt.points.append(jtp)
    controller_pub.publish(jt)
