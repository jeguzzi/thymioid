#!/usr/bin/env python

import rospy
from menu import Menu


class DummyMenu(Menu):

    def target_config(self, value):
        rospy.loginfo("Try to set config to {value} for menu {self.index}".format(**locals()))
        rospy.sleep(1)
        rospy.loginfo("Set config to {value} for menu {self.index}".format(**locals()))
        self.config = value


if __name__ == '__main__':
    rospy.init_node('dummy_menus', anonymous=True)
    DummyMenu(0, 2)
    DummyMenu(1, 4)
    DummyMenu(3, 8)
    rospy.spin()
