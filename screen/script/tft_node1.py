#!/usr/bin/env python

import os

import pygame

import rospy


class TFT(object):

    def __init__(self):
        rospy.init_node('tft_node')
        # os.environ["SDL_FBDEV"] = rospy.get_param('~fb', '/dev/fb2')
        pygame.display.init()


if __name__ == '__main__':
    TFT()
    rospy.spin()
