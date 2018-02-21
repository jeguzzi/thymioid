#!/usr/bin/env python

import os

import rospy
from menu import Menu
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

_pipe = '/wlan'


def get_configuration(run_path, configurations):
    with open(run_path) as f:
        config = f.read()
        try:
            return configurations.index(config)
        except ValueError:
            rospy.loginfo('Unknown wifi config %s', config)
            return 0


class WifiUI(Menu, FileSystemEventHandler):

    def __init__(self, index, interface='wlan0'):
        self.interface = interface
        self.run_state = '/run/network/ifstate.{interface}'.format(interface=interface)
        self.configurations = ['', interface] + rospy.get_param("~wlan_interfaces", [])
        super(WifiUI, self).__init__(index, len(self.configurations))
        self.config = get_configuration(self.run_state, self.configurations)
        observer = Observer()
        observer.schedule(self, os.path.dirname(self.run_state), recursive=False)
        observer.start()

    def on_modified(self, event):
        self.config = get_configuration(event.src_path, self.configurations)

    def target_config(self, value):
        config = self.configurations[value]
        rospy.loginfo('Try to set wifi to %s' % config)
        with open(_pipe, 'w') as f:
            f.write(config)


if __name__ == '__main__':
    rospy.init_node('wifi_node', anonymous=True)
    WifiUI(0)
    rospy.spin()
