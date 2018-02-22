#!/usr/bin/env python

import os

import rospy
from menu import Menu
from watchdog.events import FileSystemEventHandler
# from watchdog.observers import Observer
# Observer not working in docker
from watchdog.observers.polling import PollingObserver as Observer

_pipe = '/wlan'


def get_configuration(run_path, configurations):
    rospy.loginfo("Get config in %s from file %s", configurations, run_path)
    with open(run_path) as f:
        config = f.read()[:-1]
        # rospy.loginfo("%s %s %s", config.__class__, configurations[1].__class__,
        #               config == configurations[1])
        try:
            return configurations.index(config)
        except ValueError:
            rospy.logwarn('Unknown wifi config %s', config)
            return 0


class WifiUI(Menu, FileSystemEventHandler):

    def __init__(self, index, interface='wlan0'):
        self.interface = interface
        self.run_state = '/run/network/ifstate.{interface}'.format(interface=interface)
        self.configurations = ['', interface] + rospy.get_param("~wlan_interfaces", [])
        rospy.loginfo("Loaded wlan configs %s", self.configurations)
        super(WifiUI, self).__init__(index, len(self.configurations))
        self.config = get_configuration(self.run_state, self.configurations)
        rospy.loginfo("Start observingchanges in %s", os.path.dirname(self.run_state))
        observer = Observer()
        observer.schedule(self, os.path.dirname(self.run_state), recursive=True)
        observer.start()

    def on_modified(self, event):
        if event.src_path == self.run_state:
            rospy.loginfo("Detected wlan change")
            self.config = get_configuration(self.run_state, self.configurations)

    def target_config(self, value):
        config = self.configurations[value]
        rospy.loginfo('Try to set wifi to %s' % config)
        with open(_pipe, 'w') as f:
            f.write('%s\n' % config)


if __name__ == '__main__':
    rospy.init_node('wifi_node', anonymous=True)
    WifiUI(0)
    rospy.spin()
