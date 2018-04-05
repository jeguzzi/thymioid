#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelConfiguration
from sensor_msgs.msg import JointState


# string model_name
# string urdf_param_name
# string[] joint_names
# float64[] joint_positions
# ---
# bool success
# string status_message


def _change_pitch(set_joint, name='', urdf='robot_description',
                  joints=['camera_body_support_joint']):
    def f(msg):
        for n, p in zip(msg.name, msg.position):
            if n in joints:
                print(msg)
                set_joint(name, urdf, [n], [p])
    return f


def main():
    rospy.init_node("gazebo_camera_pitch_controller")

    name = rospy.get_param('~name')
    set_joint = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    rospy.Subscriber('{name}/joint_states'.format(name=name),
                     JointState, _change_pitch(set_joint, name=name))
    rospy.spin()


if __name__ == '__main__':
    main()
