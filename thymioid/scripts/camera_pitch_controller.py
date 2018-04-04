#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import JointState
from thymioid.cfg import CameraConfig


def change_pitch(pub, msg):
    def f(config, level):
        msg.header.stamp = rospy.Time.now()
        msg.position = [config['pitch']]
        pub.publish(msg)
        return config
    return f


def main():
    rospy.init_node("camera_pitch_controller")
    # ns = rospy.get_namespace()
    joint = rospy.get_param('~joint', 'camera_body_support_joint')
    pub = rospy.Publisher("joint_states", JointState, queue_size=1, latch=True)
    msg = JointState()
    msg.name = [joint]
    msg.header.stamp = rospy.Time.now()
    msg.position = [rospy.get_param('~pitch')]
    pub.publish(msg)
    Server(CameraConfig, change_pitch(pub, msg))  # , namespace="{ns}/camera_joint".format(ns=ns))
    rospy.spin()


if __name__ == '__main__':
    main()
