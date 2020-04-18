from typing import Any

import rclpy
from .menu import Menu


class SimpleMenu(Menu):

    def __init__(self) -> None:
        super(SimpleMenu, self).__init__(name="simple_menu")
        self.size = self.declare_parameter('size', 1).value
        self.config = 0

    def set_target_config(self, value: int) -> None:
        self.get_logger().info(f'Set menu {self.index} to {value}')
        self.config = value


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    try:
        wifi = SimpleMenu()
    except Exception:
        return
    rclpy.spin(wifi)
    wifi.destroy_node()
    rclpy.shutdown()
