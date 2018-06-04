#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from dynamic_reconfigure.server import Server
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from thymioid.cfg import SoundConfig


class Sound(object):

    def __init__(self):
        rospy.init_node('sound_node', anonymous=True)
        self.soundhandle = SoundClient(blocking=False)
        self.volume = rospy.get_param('~volume', 0.5)
        self.language = rospy.get_param('~language', 'en-US')
        Server(SoundConfig, self.callback)
        rospy.Subscriber('say', String, self.say, queue_size=1)
        rospy.Subscriber('play', String, self.play, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)
        rospy.spin()

    def callback(self, config, level):
        self.volume = config.volume
        self.language = config.language
        return config

    def say(self, msg):
        self.soundhandle.stopAll()
        # self.soundhandle.say support utf strings
        self.soundhandle.say(msg.data, volume=self.volume, voice=self.language)

    def play(self, msg):
        self.soundhandle.stopAll()
        self.soundhandle.playWaveFromPkg('thymioid', 'sound/{0}.wav'.format(msg.data), volume=self.volume)

    def on_shutdown(self):
        self.soundhandle.stopAll()


if __name__ == '__main__':
    Sound()
