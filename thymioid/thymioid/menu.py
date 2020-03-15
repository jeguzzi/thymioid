import rclpy
import rclpy.node
from std_msgs.msg import Int8
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from typing import Optional
from abc import ABC, abstractmethod

latching_qos = QoSProfile(depth=1,
                          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)


class Menu(rclpy.node.Node, ABC):  # type: ignore
    """docstring for Menu."""

    def __init__(self, name: str, index: int, size: Optional[int] = None) -> None:
        super(Menu, self).__init__(name)
        self.get_logger().info(f"Init Menu at index {index}.")
        self.create_subscription
        self._config: Optional[int] = None
        self._size = None
        self.index = index
        self.pub_value = self.create_publisher(Int8, f'config/c{index}/value', latching_qos)
        self.pub_size = self.create_publisher(Int8, f'config/c{index}/size', latching_qos)
        self.create_subscription(Int8, f'config/c{index}/target', self.on_target, 1)
        self.size = size

    def on_target(self, msg: Int8) -> None:
        value = msg.data
        self.get_logger().info(f"Received Menu {self.index} target {value}.")
        if value != self._config and value < self.size:
            self.set_target_config(value)

    @property
    def config(self) -> Optional[int]:
        return self._config

    @config.setter
    def config(self, value: int) -> None:
        if value != self._config and self.size and value < self.size:
            self.get_logger().info(f"Set Menu {self.index} config to {value}.")
            self._config = value
            self.pub_value.publish(Int8(data=value))

    @property
    def size(self) -> Optional[int]:
        return self._size

    @size.setter
    def size(self, value: int) -> None:
        if value != self._size and value > 0:
            self.get_logger().info(f"Set Menu {self.index} size to {value}.")
            self._size = value
            self.pub_size.publish(Int8(data=self._size))

    @abstractmethod
    def set_target_config(self, value: int) -> None:
        ...
