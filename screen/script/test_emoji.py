#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, String
import codepoints
from itertools import cycle

if __name__ == '__main__':
    rospy.init_node('smile')
    # pub = rospy.Publisher('thymio11/tft/emoji', Int32, queue_size=1)
    # pub_text = rospy.Publisher('thymio11/tft/text', String, queue_size=1)
    pub = rospy.Publisher('tft/emoji', Int32, queue_size=1)
    pub_text = rospy.Publisher('tft/text', String, queue_size=1)
    rospy.sleep(1)
    pub_text.publish('Welcome')
    emoji = u"🍓😀🚀😛😱😵😺🤖👩🐶🐯🐸🙉🐥🐝🦋🍄🍕🌰🏀🚘🚥⏰🛎⛔️🔟🔔🔕"
    for n in cycle(emoji):
        codepoint = codepoints.from_unicode(n)[0]
        rospy.sleep(3)
        pub.publish(codepoint)
