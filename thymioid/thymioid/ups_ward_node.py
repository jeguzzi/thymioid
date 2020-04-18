import enum
# from time import sleep
from typing import Any, Optional

import rclpy
import rclpy.node
import std_srvs.srv
from std_msgs.msg import Bool, ColorRGBA
from .menu import Menu


class UpsState(enum.Enum):
    AC = 1
    BATTERY = 2
    RESERVE = 3
    UNKNOWN = 4

    @classmethod
    def from_ac_and_battery(cls, ac: Optional[bool], battery: Optional[bool]) -> 'UpsState':
        if ac is None or battery is None:
            return UpsState.UNKNOWN
        if ac == 1:
            return UpsState.AC
        if battery == 1:
            return UpsState.BATTERY
        return UpsState.RESERVE


def _color(red: float = 0.0, green: float = 0.0, blue: float = 0.0) -> ColorRGBA:
    return ColorRGBA(r=red, g=green, b=blue, a=0.0)


ups_color = {
    UpsState.AC: _color(green=1.0),
    UpsState.BATTERY: _color(red=1.0, green=1.0),
    UpsState.RESERVE: _color(red=1.0),
    UpsState.UNKNOWN: _color(blue=1.0)
}


class UpsWard(Menu):

    def __init__(self) -> None:
        super(UpsWard, self).__init__(name='ups_ui')
        # wait for the thymio
        self.create_client(std_srvs.srv.Empty, 'is_ready').wait_for_service()
        self.get_logger().info("Ups UI ready")

        self.led_publishers = [
            self.create_publisher(ColorRGBA, f'led/body/{led}', 1)
            for led in ('top', 'bottom_left', 'bottom_right')]

        self.battery: Optional[bool] = None
        self.ac: Optional[bool] = None
        self._state = UpsState.UNKNOWN
        param = self.declare_parameter('play_alarm', False)
        self.enable_alarm = param.value
        self.playing_alarm = False
        self.alarm_publisher = self.create_publisher(Bool, 'alarm', 1)
        self.batt_sub = self.create_subscription(Bool, 'ups/battery', self.on_battery, 1)
        self.ac_sub = self.create_subscription(Bool, 'ups/ac', self.on_ac, 1)
        self.size = 2
        self.config = 1

        # TODO(J): [still] missing in ROS2
        # rospy.on_shutdown(self.on_shutdown)

    # def on_shutdown(self):
    #     rospy.loginfo("shutdown ups_ward")
    #     self.set_color(ColorRGBA(0.0, 0.0, 0.0, 0.0))

    @property
    def state(self) -> UpsState:
        return self._state

    @state.setter
    def state(self, value: UpsState) -> None:
        if self._state != value:
            self._state = value
            if self.config:
                self.publish_state()

    def publish_state(self) -> None:
        if self.state == UpsState.RESERVE and self.enable_alarm:
            self.start_alarm()
        if self.state != UpsState.RESERVE and self.playing_alarm:
            self.stop_alarm()
        self.set_color(ups_color[self.state])

    def on_battery(self, msg: Bool) -> None:
        self.battery = msg.data

    def on_ac(self, msg: Bool) -> None:
        self.ac = msg.data
        self.state = UpsState.from_ac_and_battery(ac=self.ac, battery=self.battery)

    def stop_alarm(self) -> None:
        if self.playing_alarm:
            self.alarm_publisher.publish(False)
            self.playing_alarm = False

    def start_alarm(self) -> None:
        if not self.playing_alarm:
            self.alarm_publisher.publish(True)
            self.playing_alarm = True

    def set_color(self, color: ColorRGBA) -> None:
        for p in self.led_publishers:
            p.publish(color)

    def set_target_config(self, value: int) -> None:
        if self.config == 0 and value == 1:
            self.publish_state()
        if self.config == 1 and value == 0:
            self.set_color(_color())
        self.config = value


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    ups = UpsWard()
    rclpy.spin(ups)
    ups.destroy_node()
    rclpy.shutdown()
