import rclpy
import rclpy.node
from std_msgs.msg import Bool, Empty, Int8
from thymio_msgs.msg import Led
from typing import Dict, Optional, Callable, Any
from .menu import latching_qos


LONG_PRESS = 5  # seconds
MENU_TIMEOUT = 5
SELECTION_TIMEOUT = 3
TARGET_TIMEOUT = 8

_shutdown_pipe = '/shutdown'
_update_pipe = '/update'


class UI(rclpy.node.Node):  # type: ignore

    def _init_buttons(self) -> None:
        self.long_press = self.declare_parameter('long_press_duration', LONG_PRESS).value
        self.menu_timeout = self.declare_parameter('~menu_timeout', MENU_TIMEOUT).value
        self.selection_timeout = self.declare_parameter('~selection_timeout', SELECTION_TIMEOUT).value
        self.target_timeout = self.declare_parameter('~target_timeout', TARGET_TIMEOUT).value
        self.last_time_button_down: Dict[str, Optional[float]] = {}
        self.buttons = ('left', 'center', 'right', 'forward', 'backward')
        for b in self.buttons:
            self.create_subscription(Bool, f'buttons/{b}', self.on_button(b), 1)

    def _init_menu(self) -> None:
        self._menu: Optional[int] = None
        self._desired_config: Optional[int] = None
        self._target_config: Optional[int] = None
        self.config: Dict[int, Optional[int]] = {i: None for i in range(4)}
        self.menu_ts = self.desired_config_ts = self.target_config_ts = None
        self.config_size = {i: 0 for i in range(4)}
        self.target_config_pub = [
            self.create_publisher(Int8, f'menu_{i}/target', 1) for i in range(4)]
        for i in range(4):
            self.create_subscription(Int8, f'menu_{i}/value', self.on_config(i), latching_qos)
            self.create_subscription(Int8, f'menu_{i}/size', self.on_config_size(i), latching_qos)

    def __init__(self) -> None:

        super(UI, self).__init__('ui_node')
        self.clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.ROS_TIME)
        self.should_exit = False

        self.led_publisher = self.create_publisher(Led, 'led', 10)
        self.shutdown_thymio_pub = self.create_publisher(Empty, 'shutdown', 1)

        self.beat = True

        # TODO(J): not yet there in ROS2
        # rospy.on_shutdown(self.on_shutdown)

        self._init_menu()
        self._init_buttons()
        self.create_timer(1.0, self.update)

        # TODO(J) how to check ...
        # while not self.should_exit and not rospy.is_shutdown():
        #     ...
        # self.on_shutdown()

    def update(self) -> None:
        self.send_beat()
        self.check_menu_timeout()
        self.check_desired_config_timeout()
        self.check_target_config_timeout()
        self.check_long_press()

    @property
    def menu(self) -> Optional[int]:
        return self._menu

    @menu.setter
    def menu(self, value: Optional[int]) -> None:
        if value != self._menu:
            self.get_logger().info(f'Select menu {value}')
            self._menu = value
            # rospy.loginfo('Select menu %s', value)
            self.menu_ts = self.clock.now()
            if value is None:
                self._desired_config = None
                self._target_config = None
            self.update_menu_led()
            self.update_config_led()

    @property
    def target_config(self) -> Optional[int]:
        return self._target_config

    @target_config.setter
    def target_config(self, value: Optional[int]) -> None:
        if value != self.target_config:
            self._target_config = value
            self.get_logger().info(f'Set target config to {value}')
            if value is not None and self.menu is not None:
                self.target_config_pub[self.menu].publish(Int8(data=value))
                self.target_config_ts = self.menu_ts = self.clock.now()

    @property
    def desired_config(self) -> Optional[int]:
        return self._desired_config

    @desired_config.setter
    def desired_config(self, value: Optional[int]) -> None:
        if value != self.desired_config:
            if value is not None:
                self.menu_ts = self.clock.now()
            if self.menu is not None and value == self.config[self.menu]:
                value = None
            self._desired_config = value
            # rospy.loginfo('Set selection  to %s', value)
            if value is not None:
                self.desired_config_ts = self.clock.now()
            self.update_config_led()

    def update_menu_led(self) -> None:
        msg = Led(id=Led.BUTTONS, values=([0] * 8))
        if self.menu is not None:
            msg.values[self.menu % 4] = 0.5
        self.get_logger().debug(f'update_menu_led: send led_publisher {msg}')
        self.led_publisher.publish(msg)

    def update_config_led(self) -> None:
        msg = Led(id=Led.CIRCLE, values=(8 * [0]))
        if self.desired_config is not None:
            msg.values[self.desired_config % 8] = 0.07
        else:
            if self.menu is not None:
                config = self.config[self.menu]
                if config is not None:
                    msg.values[config % 8] = 1.0
        self.led_publisher.publish(msg)

    def check_menu_timeout(self) -> None:
        if self.menu is not None and self.target_config is None:
            dt = self.clock.now() - self.menu_ts
            if dt.nanoseconds * 1e-9 > self.menu_timeout:
                # rospy.loginfo('timeout menu')
                self.menu = None

    def check_desired_config_timeout(self) -> None:
        if self.desired_config is not None and self.target_config is None:
            dt = self.clock.now() - self.desired_config_ts
            if dt.nanoseconds * 1e-9 > self.selection_timeout:
                # rospy.loginfo('timeout selection')
                self.desired_config = None

    def check_target_config_timeout(self) -> None:
        if self.target_config is not None:
            if (self.clock.now() - self.target_config_ts).nanoseconds * 1e-9 > self.target_timeout:
                # rospy.loginfo('timeout target')
                self.target_config = None

    def on_config(self, menu: int) -> Callable[[Int8], None]:
        def cb(msg: Int8) -> None:
            value = msg.data
            self.menu = menu
            self.set_config(menu, value)
        return cb

    def on_config_size(self, menu: int) -> Callable[[Int8], None]:
        def cb(msg: Int8) -> None:
            value = msg.data
            # rospy.loginfo('Set config size for %s to %s', menu, value)
            self.config_size[menu] = value
            if value == 0 and self.menu == menu:
                self.move_to_next_menu()
            if value and self.config[menu] is None:
                # rospy.loginfo('Set def config for %s to 0', menu)
                self.config[menu] = 0
        return cb

    def set_config(self, menu: int, value: int) -> None:
        # rospy.loginfo('Set config for %s to %s', menu, value)
        self.desired_config = self.target_config = None
        self.config[menu] = value
        if menu != self.menu:
            self.menu = menu
        else:
            self.menu_ts = self.clock.now()
        self.update_config_led()

    def on_shutdown(self) -> None:
        self.menu = None

    def send_beat(self) -> None:
        if self.menu is not None:
            return
        msg = Led(id=Led.BUTTONS, values=(4 * [0.0]))
        if self.beat:
            msg.values[2] = 1.0
        self.led_publisher.publish(msg)
        self.beat = not self.beat

    def shutdown_odroid(self) -> None:
        with open(_shutdown_pipe, 'w') as f:
            f.write('shutdown\n')
        # subprocess.call(['shutdown', 'now'])
        # print "SHUTDOWN ODROID"

    def update_odroid(self) -> None:
        with open(_update_pipe, 'w') as f:
            f.write('update\n')

    def on_long_press(self, button: str) -> None:
        self.get_logger().debug(f'Long press {button}')
        # the button has been pressed for more than LONG_PRESS seconds
        if(button == 'forward'):
            # exit
            self.should_exit = True
            self.on_shutdown()
        if(button == 'right'):
            # exit
            self.update_odroid()
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

    def on_button(self, button: str) -> Callable[[Bool], None]:
        def cb(msg: Bool) -> None:
            if msg.data:
                # rospy.loginfo('down %s', button)
                self.last_time_button_down[button] = self.clock.now()
            else:
                # rospy.loginfo('up %s', button)
                if self.last_time_button_down[button] is not None:
                    self.on_short_press(button)
                self.last_time_button_down[button] = None
        return cb

    def on_short_press(self, button: str) -> None:
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
            # change menu
            self.move_to_next_menu()

    def move_to_next_menu(self) -> None:
        self.get_logger().debug(f'Move to next menu')
        if self.menu is not None:
            for i in range(4):
                menu = (self.menu + i + 1) % 4
                if self.config_size[menu]:
                    self.menu = menu
                    return
            self.menu = None

    def check_long_press(self) -> None:
        for b in self.buttons:
            if(self.last_time_button_down.get(b, None) and
               (self.clock.now() - self.last_time_button_down.get(b)).nanoseconds * 1e-9 > self.long_press):
                self.last_time_button_down[b] = None
                self.on_long_press(b)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    ui = UI()
    rclpy.spin(ui)
    ui.destroy_node()
    rclpy.shutdown()
