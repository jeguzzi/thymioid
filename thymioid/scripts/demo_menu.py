#!/usr/bin/env python

import os

import docker
import rospy
from menu import Menu


def demo_for_container(container):
    return container.name.split('_')[0]


def demo_is_running(docker, demo):
    if not demo:
        return False
    for container in docker.containers.list(all=True):
        if container.status != 'running' and demo == demo_for_container(container):
            return False
    return True


def stop_demo(docker, demo):
    if not demo:
        return
    for container in docker.containers.list():
        if container.status == 'running' and demo == demo_for_container(container):
            container.stop()


def start_demo(docker, demo):
    if not demo:
        return
    for container in docker.containers.list(all=True):
        if container.status in ['created', 'exited'] and demo == demo_for_container(container):
            container.start()


class DemoMenu(Menu):

    def __init__(self, index):
        self.demos = [None] + rospy.get_param("~demos", [])
        super(DemoMenu, self).__init__(index, len(self.demos))
        self.docker_client = docker.client.DockerClient()
        config = 0
        for i, demo in enumerate(self.demos):
            if demo_is_running(self.docker_client, demo):
                config = i
                break
        self.config = config

    def target_config(self, value):
        stop_demo(self.docker_client, self.demos[self.config])
        start_demo(self.docker_client, self.demos[value])
        self.config = value


if __name__ == '__main__':
    rospy.init_node('demo_menu_node', anonymous=True)
    DemoMenu(1)
    rospy.spin()
