#!/usr/bin/env python

from __future__ import division

import array
import os
from collections import deque

import cv2
import numpy as np

import cairo
import cv_bridge
import pygame
import rospkg
import rospy
try:
    import rsvg
except ImportError:
    import gi
    gi.require_version('Rsvg', '2.0')
    from gi.repository import Rsvg as rsvg

from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Empty, Int32, String
from tft.cfg import TftConfig


def color_float2int(value):
    return int(max(min(value, 1.0), 0.0) * 255)


class TFT(object):

    def __init__(self):
        rospy.init_node('tft')
        self.width = rospy.get_param('tft/width', 320)
        self.height = rospy.get_param('tft/height', 240)
        self.background = rospy.get_param('~background', (0, 0, 0))

        font_name = rospy.get_param('~font_name', 'dejavusans')
        font_size = rospy.get_param('~font_size', 24)
        self.font_color = rospy.get_param('~font_color', (255, 255, 255))

        self.text_position = (rospy.get_param('~text_position_x', 0),
                              rospy.get_param('~text_position_y', 120 - font_size / 2))
        self.align = rospy.get_param('~text_alignement', 'center')
        if self.align not in ['left', 'right', 'center']:
            self.align = 'left'
        self.number_of_lines = rospy.get_param('~text_lines', 1)
        self.lines = deque([], maxlen=self.number_of_lines)

        pygame.font.init()
        self.set_font(size=font_size, name=font_name)

        os.environ["SDL_FBDEV"] = rospy.get_param('~fb', '/dev/fb2')
        pygame.display.init()
        self.screen = pygame.display.set_mode((self.width, self.height), 0, 32)

        pygame.mouse.set_visible(False)

        self.clear_screen()
        self.bridge = cv_bridge.CvBridge()

        emoji_folder = os.path.join(rospkg.RosPack().get_path('tft'), 'emoji')
        self.emoji = {}
        for name in os.listdir(emoji_folder):
            codes = name[:-4].split('-')
            if len(codes) == 1:
                self.emoji[codes[0]] = os.path.join(emoji_folder, name)

        self.srv = Server(TftConfig, self.callback)

        rospy.Subscriber('tft/image', Image, self.update_image, queue_size=1)
        rospy.Subscriber('tft/text', String, self.update_text, queue_size=1)
        rospy.Subscriber('tft/emoji', Int32, self.update_emoji, queue_size=1)
        rospy.Subscriber('tft/clear', Empty, self.clear, queue_size=1)
        rospy.Subscriber('tft/color', ColorRGBA, self.update_background_color, queue_size=1)
        rospy.Subscriber('tft/font_color', ColorRGBA, self.update_font_color, queue_size=1)

    def set_font(self, size, name):
        self.font_name = name
        self.font_size = size
        self.font = pygame.font.SysFont(self.font_name, self.font_size, bold=False, italic=False)

    def callback(self, config, level):
        # color = yaml.loads(config.font_color)
        self.set_font(size=config.font_size, name=self.font_name)
        self.text_position = (config.text_position_x, config.text_position_y)
        self.lines = deque(list(self.lines), maxlen=config.text_lines)
        self.align = config.text_alignment
        self.display_text()
        return config

    def update_background_color(self, msg):
        self.background = [color_float2int(x) for x in [msg.r, msg.g, msg.b]]
        self.clear_screen()

    def update_font_color(self, msg):
        self.font_color = [color_float2int(x) for x in [msg.r, msg.g, msg.b]]
        self.display_text()

    def clear_screen(self):
        self.screen.fill(self.background)
        pygame.display.update()

    def clear(self, msg=None):
        self.lines.clear()
        self.clear_screen()

    def update_image(self, msg):
        data = self.bridge.imgmsg_to_cv2(msg)
        data = cv2.resize(data, (self.width, self.height))
        data = np.transpose(data, (1, 0, 2))
        pygame.surfarray.blit_array(self.screen, data)
        pygame.display.update()

    def update_emoji(self, msg):
        # code = hex(msg.data)
        code = "{0:x}".format(msg.data)
        path = self.emoji.get(code)
        print(code, path)
        if path:
            self.clear_screen()
            try:
                svg = rsvg.Handle(file=path)
            except TypeError:
                handle = rsvg.Handle()
                svg = handle.new_from_file(path)
            self.display_svg(svg)

    def display_svg(self, svg):
        svg_data = array.array('c', chr(0) * self.height * self.height * 4)
        svg_surface = cairo.ImageSurface.create_for_data(
            svg_data, cairo.FORMAT_ARGB32, self.height, self.height, self.height * 4)
        ctx = cairo.Context(svg_surface)

        ctx.save()
        ctx.scale(self.height / svg.props.width, self.height / svg.props.height)

        svg.render_cairo(ctx)
        svg_data = np.array(svg_data).reshape((self.height, self.height, 4))
        svg_data = svg_data[..., ::-1]
        svg_data = np.roll(svg_data, -1, axis=2)
        # print(self.height / svg.props.width, self.height / svg.props.height)
        # svg_data[..., 3] = 255
        image = pygame.image.frombuffer(svg_data.tostring(), (self.height, self.height), "RGBA")
        self.screen.blit(image, ((self.width - self.height) / 2, 0))
        pygame.display.update()

    def display_text(self):
        h = self.font.get_linesize()
        x, y = self.text_position
        width = self.width
        self.clear_screen()
        for i, text in enumerate(list(self.lines)[::-1]):
            surface = self.font.render(text, True, self.font_color)
            size = self.font.size(text)
            margin = x
            if self.align == 'center':
                margin = max((width - size[0]) / 2, 0)
            if self.align == 'right':
                margin = width - size[0]
            self.screen.blit(surface, (margin, y + h * i))
        pygame.display.update()

    def update_text(self, msg):
        self.lines.append(msg.data)
        self.display_text()


if __name__ == '__main__':
    t = TFT()
    rospy.spin()
    t.screen.fill((0, 0, 0))
    pygame.display.update()
    pygame.quit()
