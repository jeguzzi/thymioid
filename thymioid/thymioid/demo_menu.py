import docker
import rclpy
from typing import Any
from .menu import Menu


def demo_for_container(container: docker.models.containers.Container) -> str:
    return container.name.split('_')[0]


def demo_is_running(docker_client: docker.client.DockerClient, demo: str) -> bool:
    if not demo:
        return False
    for container in docker_client.containers.list(all=True):
        if container.status != 'running' and demo == demo_for_container(container):
            return False
    return True


def stop_demo(docker_client: docker.client.DockerClient, demo: str) -> None:
    if not demo:
        return
    for container in docker_client.containers.list():
        if container.status == 'running' and demo == demo_for_container(container):
            container.stop()


def start_demo(docker_client: docker.client.DockerClient, demo: str) -> None:
    if not demo:
        return
    for container in docker_client.containers.list(all=True):
        if container.status in ['created', 'exited'] and demo == demo_for_container(container):
            container.start()


class DemoMenu(Menu):  # type: ignore

    def __init__(self, index: int):
        super(DemoMenu, self).__init__(name='demo_ui', index=index)
        param = self.declare_parameter('demos', [])
        self.demos = [None] + param.value
        self.size = len(self.demos)
        self.docker_client = docker.client.DockerClient()
        config = 0
        for i, demo in enumerate(self.demos):
            if demo_is_running(self.docker_client, demo):
                config = i
                break
        self.config = config

    def set_target_config(self, value: int) -> None:
        stop_demo(self.docker_client, self.demos[self.config])
        start_demo(self.docker_client, self.demos[value])
        self.config = value


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    menu = DemoMenu(1)
    rclpy.spin(menu)
    menu.destroy_node()
    rclpy.shutdown()
