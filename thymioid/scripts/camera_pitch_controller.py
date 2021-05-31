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


def update(pub, msg):
    def f(event):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
    return f


def main():
    rospy.init_node("camera_pitch_controller")
    # ns = rospy.get_namespace()
    tf_prefix = rospy.get_param('~tf_prefix', '')
    joint = rospy.get_param('~joint', 'camera_body_support_joint')
    pub = rospy.Publisher("joint_states", JointState, queue_size=1, latch=True)
    msg = JointState()
    msg.name = [tf_prefix+joint]
    msg.position = [0]

    rospy.Timer(rospy.Duration(10), update(pub, msg))

    Server(CameraConfig, change_pitch(pub, msg))  # , namespace="{ns}/camera_joint".format(ns=ns))
    rospy.spin()


if __name__ == '__main__':
    main()
