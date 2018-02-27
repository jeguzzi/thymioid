#!/usr/bin/env python

# import subprocess

import rospy
from std_msgs.msg import Bool, Empty, Int8
from thymio_msgs.msg import Led

LONG_PRESS = 5  # seconds
MENU_TIMEOUT = 5
SELECTION_TIMEOUT = 3
TARGET_TIMEOUT = 8


# def service_is_running(name):
#     try:
#         status = subprocess.check_output(["/etc/init.d/" + name, "status"])
#         return status.find('running') >= 0
#     except Exception as e:
#         rospy.logerr("Service is running, got exception %s" % e)
#         return False

_shutdown_pipe = '/shutdown'


class UI(object):

    def _init_buttons(self):
        self.long_press = rospy.get_param('~long_press_duration', LONG_PRESS)
        self.menu_timeout = rospy.get_param('~menu_timeout', MENU_TIMEOUT)
        self.selection_timeout = rospy.get_param('~selection_timeout', SELECTION_TIMEOUT)
        self.target_timeout = rospy.get_param('~target_timeout', TARGET_TIMEOUT)
        self.last_time_button_down = {}
        self.buttons = ['left', 'center', 'right', 'forward', 'backward']
        for b in self.buttons:
            rospy.Subscriber('buttons/' + b, Bool, self.on_button(b))

    def _init_menu(self):
        self._menu = self._desired_config = self._target_config = None
        self.config = {i: None for i in range(4)}
        self.menu_ts = self.desired_config_ts = self.target_config_ts = None
        self.config_size = {i: 0 for i in range(4)}

        self.target_config_pub = [
            rospy.Publisher('config/{i}/target'.format(i=i), Int8, queue_size=1)
            for i in range(4)]
        for i in range(4):
            rospy.Subscriber('config/{i}'.format(i=i), Int8, self.on_config(i))
            rospy.Subscriber('config/{i}/size'.format(i=i), Int8, self.on_config_size(i))

    def __init__(self):

        # wait for the thymio

        # rospy.wait_for_service('thymio_is_ready')

        rospy.init_node('ui_node', anonymous=True)

        self.should_exit = False

        self.led_publisher = rospy.Publisher('led', Led, queue_size=10)
        self.shutdown_thymio_pub = rospy.Publisher(
            'shutdown', Empty, queue_size=1)

        self.beat = True

        rospy.on_shutdown(self.on_shutdown)

        self._init_menu()
        self._init_buttons()
        rospy.Timer(rospy.Duration(1.0), self.send_beat)

        r = rospy.Rate(1)
        while not self.should_exit and not rospy.is_shutdown():
            self.check_menu_timeout()
            self.check_desired_config_timeout()
            self.check_target_config_timeout()
            self.check_long_press()
            r.sleep()
        self.on_shutdown()

    @property
    def menu(self):
        return self._menu

    @menu.setter
    def menu(self, value):
        if value != self._menu:
            self._menu = value
            # rospy.loginfo('Select menu %s', value)
            self.menu_ts = rospy.Time.now()
            if value is None:
                self._desired_config = None
                self._target_config = None
            self.update_menu_led()
            self.update_config_led()

    @property
    def target_config(self):
        return self._target_config

    @target_config.setter
    def target_config(self, value):
        if value != self.target_config:
            self._target_config = value
            rospy.loginfo('Set target config to %s', value)
            if value is not None:
                self.target_config_pub[self.menu].publish(value)
                self.target_config_ts = self.menu_ts = rospy.Time.now()

    @property
    def desired_config(self):
        return self._desired_config

    @desired_config.setter
    def desired_config(self, value):
        if value != self.desired_config:
            if value is not None:
                self.menu_ts = rospy.Time.now()
            if value == self.config[self.menu]:
                value = None
            self._desired_config = value
            # rospy.loginfo('Set selection  to %s', value)
            if value is not None:
                self.desired_config_ts = rospy.Time.now()
            self.update_config_led()

    def update_menu_led(self):
        msg = Led(id=Led.BUTTONS, values=([0] * 8))
        if self.menu is not None:
            msg.values[self.menu % 4] = 0.5
        self.led_publisher.publish(msg)

    def update_config_led(self):
        msg = Led(id=Led.CIRCLE, values=(8 * [0]))
        if self.desired_config is not None:
            msg.values[self.desired_config % 8] = 0.07
        else:
            if self.menu is not None and self.config[self.menu] is not None:
                msg.values[self.config[self.menu] % 8] = 1.0
        self.led_publisher.publish(msg)

    def check_menu_timeout(self):
        if self.menu is not None and self.target_config is None:
            dt = rospy.Time.now() - self.menu_ts
            if dt.to_sec() > self.menu_timeout:
                # rospy.loginfo('timeout menu')
                self.menu = None

    def check_desired_config_timeout(self):
        if self.desired_config is not None and self.target_config is None:
            dt = rospy.Time.now() - self.desired_config_ts
            if dt.to_sec() > self.selection_timeout:
                # rospy.loginfo('timeout selection')
                self.desired_config = None

    def check_target_config_timeout(self):
        if self.target_config is not None:
            if (rospy.Time.now() - self.target_config_ts).to_sec() > self.target_timeout:
                # rospy.loginfo('timeout target')
                self.target_config = None

    def on_config(self, menu):
        def cb(msg):
            value = msg.data
            self.menu = menu
            self.set_config(menu, value)
        return cb

    def on_config_size(self, menu):
        def cb(msg):
            value = msg.data
            # rospy.loginfo('Set config size for %s to %s', menu, value)
            self.config_size[menu] = value
            if value == 0 and self.menu == menu:
                self.move_to_next_menu()
            if value and self.config[menu] is None:
                # rospy.loginfo('Set def config for %s to 0', menu)
                self.config[menu] = 0
        return cb

    def set_config(self, menu, value):
        # rospy.loginfo('Set config for %s to %s', menu, value)
        self.desired_config = self.target_config = None
        self.config[menu] = value
        if menu != self.menu:
            self.menu = menu
        else:
            self.menu_ts = rospy.Time.now()
        self.update_config_led()

    def on_shutdown(self):
        self.menu = None

    def send_beat(self, evt):
        if self.menu is not None:
            return
        msg = Led(id=Led.BUTTONS, values=(4 * [0.0]))
        if self.beat:
            msg.values[2] = 1.0
        self.led_publisher.publish(msg)
        self.beat = not self.beat

    def shutdown_odroid(self):
        with open(_shutdown_pipe, 'w') as f:
            f.write('shutdown\n')
        # subprocess.call(['shutdown', 'now'])
        # print "SHUTDOWN ODROID"

    def on_long_press(self, button):
        # rospy.loginfo('long press %s' % button)
        # the button has been pressed for more than LONG_PRESS seconds
        if(button == 'forward'):
            # exit
            self.should_exit = True
            self.on_shutdown()
        if(button == 'backward'):
            # shutdown
            # print "SHUTDOWN THYMIO"
            # subprocess.call(['rostopic','pub','/shutdown','--once','std_msgs/Empty'])
            self.shutdown_thymio_pub.publish(Empty())
            self.shutdown_odroid()
        if(button == 'left'):
            for menu in range(4):
                if self.config_size[menu]:
                    self.menu = menu
                    break

    def on_button(self, button):
        def cb(msg):
            if msg.data:
                # rospy.loginfo('down %s', button)
                self.last_time_button_down[button] = rospy.Time.now()
            else:
                # rospy.loginfo('up %s', button)
                if self.last_time_button_down[button] is not None:
                    self.on_short_press(button)
                self.last_time_button_down[button] = None
        return cb

    def on_short_press(self, button):
        # rospy.loginfo('short press %s' % button)
        if(button == 'right'):
            # rospy.loginfo("L %s %s %s", self.menu, self.target_config, self.desired_config)
            # change desired configuration
            if self.menu is not None and self.target_config is None:
                if self.desired_config is None:
                    value = self.config[self.menu]
                else:
                    value = self.desired_config
                if value is not None:
                    self.desired_config = (value + 1) % self.config_size[self.menu]

        if(button == 'center'):
            # change configuration
            if self.desired_config is not None:
                self.target_config = self.desired_config

        if(button == 'left'):
            # change manu
            self.move_to_next_menu()

    def move_to_next_menu(self):
        # rospy.loginfo('Move to next menu')
        if self.menu is not None:
            for i in range(4):
                menu = (self.menu + i + 1) % 4
                if self.config_size[menu]:
                    self.menu = menu
                    return
            self.menu = None

    def check_long_press(self):
        for b in self.buttons:
            if(self.last_time_button_down.get(b, None) and
               (rospy.Time.now() - self.last_time_button_down.get(b)).to_sec() > self.long_press):
                self.last_time_button_down[b] = None
                self.on_long_press(b)


if __name__ == '__main__':
    ui = UI()
