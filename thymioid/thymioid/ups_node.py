from typing import Any
import rclpy.node
import rclpy
import wiringpi2 as wp
from std_msgs.msg import Bool

AC_OK = 0
BATT_OK = 2
POWER_LATCH = 3
IN = 0
OUT = 1


def setup_gpio() -> None:
    wp.wiringPiSetup()
    wp.pinMode(AC_OK, IN)
    wp.pinMode(BATT_OK, IN)
    wp.pinMode(POWER_LATCH, OUT)
    wp.digitalWrite(POWER_LATCH, 1)


class Ups(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(Ups, self).__init__('ups')
        setup_gpio()
        self.ac_pub = self.create_publisher(Bool, 'ac', 1)
        self.batt_pub = self.create_publisher(Bool, 'battery', 1)
        self.create_timer(1.0, self.update)

    def update(self) -> None:
        ac_ok = wp.digitalRead(AC_OK)
        batt_ok = wp.digitalRead(BATT_OK)
        self.ac_pub.publish(Bool(data=ac_ok))
        self.batt_pub.publish(Bool(data=batt_ok))


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    ups = Ups()
    rclpy.spin(ups)
    ups.destroy_node()
    rclpy.shutdown()
