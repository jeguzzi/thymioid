import os
from typing import Any

import rclpy
from watchdog.events import FileSystemEvent, FileSystemEventHandler
# from watchdog.observers import Observer
# Observer not working in docker
from watchdog.observers.polling import PollingObserver as Observer

from .menu import Menu

_pipe = '/wlan'


def get_configuration(run_path: str, configurations: str) -> int:
    with open(run_path) as f:
        config = f.read()[:-1]
        try:
            return configurations.index(config)
        except ValueError:
            raise ValueError(f'Unknown wifi config {config}')


class WifiUI(Menu, FileSystemEventHandler):  # type: ignore

    def __init__(self, index: int, interface: str = 'wlan0') -> None:
        self.interface = interface
        self.run_state = f'/run/network/ifstate.{interface}'
        super(WifiUI, self).__init__(name='wifi_ui', index=index)

        # TODO(J): Maybe should describe it?
        interfaces_param = self.declare_parameter('wlan_interfaces', [])
        self.configurations = ['', interface] + interfaces_param.value

        self.get_logger().info(f"Loaded wlan configs {self.configurations}")
        self.size = len(self.configurations)
        try:
            self.config = get_configuration(self.run_state, self.configurations)
        except FileNotFoundError:
            self.get_logger().error(f"Network file {self.run_state} not found. Will exit.")
            raise FileNotFoundError
        folder = os.path.dirname(self.run_state)
        self.get_logger().info("Start observing changes in {folder}")
        observer = Observer()
        observer.schedule(self, folder, recursive=True)
        observer.start()

    def on_modified(self, event: FileSystemEvent) -> None:
        if event.src_path == self.run_state:
            self.get_logger().info("Detected wlan change")
            self.config = get_configuration(self.run_state, self.configurations)

    def set_target_config(self, value: int) -> None:
        config = self.configurations[value]
        self.get_logger().info(f'Try to set wifi to {config}')
        with open(_pipe, 'w') as f:
            f.write('%s\n' % config)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    try:
        wifi = WifiUI(0)
    except Exception:
        return
    rclpy.spin(wifi)
    wifi.destroy_node()
    rclpy.shutdown()
