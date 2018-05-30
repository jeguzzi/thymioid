#!/usr/bin/env python

from __future__ import division

import threading

import evdev
import rospy
import wiringpi2 as wp
from evdev.ecodes import ABS_X, ABS_Y, BTN_TOUCH, EV_ABS, EV_KEY
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

KEYS = [1, 4, 5]
IN = 0
OUT = 1


class TFTouch(object):

    def __init__(self):
        rospy.init_node('tft')
        self.continuos = rospy.get_param('~continuos', True)
        rate = rospy.get_param('~rate', 10.0)
        if rate > 0:
            period = 1 / rate
        else:
            period = 0.1
        self.width = rospy.get_param('tft/width', 320)
        self.height = rospy.get_param('tft/height', 240)
        self.dev = evdev.InputDevice('/dev/input/ts_uinput')
        wp.wiringPiSetup()
        for key, pin in enumerate(KEYS):
            wp.pinMode(pin, IN)
        self.key_pub = {pin: rospy.Publisher('tft/key_{key}'.format(key=i + 1), Bool, queue_size=1)
                        for i, pin in enumerate(KEYS)}
        self.state = {pin: 0 for pin in KEYS}
        self.touch = {'x': None, 'y': None, 'down': 0}
        self.joy_pub = rospy.Publisher('tft/touch', Joy, queue_size=1)
        rospy.Timer(rospy.Duration(period), self.update_keys, oneshot=False)
        self.dev_thread = threading.Thread(target=self.update_touch)
        self.dev_thread.daemon = True
        self.dev_thread.start()
        self.buttons = []
        self.axes = []

    def update_touch(self):
        for event in self.dev.read_loop():
            if event.type == EV_ABS:
                if event.code == ABS_X:
                    self.touch['x'] = max(min(event.value, self.width), 0)
                    continue
                if event.code == ABS_Y:
                    self.touch['y'] = max(min((self.height - event.value), self.height), 0)
                    continue
            if event.type == EV_KEY and event.code == BTN_TOUCH:
                self.touch['down'] = event.value
                continue

    def update_keys(self, event):
        # 1 is up, 0 is down
        state = {pin: 1 - wp.digitalRead(pin) for pin in KEYS}

        if self.touch['down'] and self.touch['x'] is not None and self.touch['y'] is not None:
            axes = [2 * self.touch['x'] / self.width - 1, 2 * self.touch['y'] / self.height - 1]
        else:
            axes = [0, 0]
        buttons = [self.touch['down']] + [state[pin] for pin in KEYS]

        if self.continuos or buttons != self.buttons or axes != self.axes:
            msg = Joy(buttons=buttons, axes=axes)
            msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = 'tft'
            self.joy_pub.publish(msg)
            self.buttons = buttons
            self.axes = axes
        for pin, value in state.items():
            if value != self.state.get(pin):
                self.key_pub[pin].publish(value)
        self.state = state


if __name__ == '__main__':
    t = TFTouch()
    rospy.spin()
