#!/usr/bin/env python

import rospy
from asebaros_msgs.msg import AsebaEvent
from asebaros_msgs.srv import LoadScripts,LoadScriptsResponse
from std_msgs.msg import String
from struct import *


in_topics=["set_speed","set_led_top","set_led_bottom_left","set_led_bottom_right"]
out_topics=["proximity","ground","imu","odometry"]
vrep_ns="/vrep/"
aseba_ns="/aseba/events/"

def pack_int(data):
        string=pack(len(data)*'i',*data)        
        #print data,'->',string
        return string

def unpack_int(string):
        data=unpack((len(string)/4)*'i',string)        
        #print string,'->',data
        return data
        

def on_aseba_event(t):
        p=rospy.Publisher(vrep_ns+t,String,queue_size=1)
        def cb(aseba_msg):
                data=pack_int(aseba_msg.data)
                vrep_msg=String(data)
                p.publish(vrep_msg)
        return cb

def on_vrep_event(t):
        p=rospy.Publisher(aseba_ns+t,AsebaEvent,queue_size=1)
        def cb(vrep_msg):
                data=unpack_int(vrep_msg.data)
                now=rospy.get_rostime()
                #rospy.loginfo("%s Current time %i %i",t,now.secs,now.nsecs)
                aseba_msg=AsebaEvent(now,0,data)
                p.publish(aseba_msg)
        return cb

def load_script(req):
        return LoadScriptsResponse()


def main():
        rospy.init_node("aseba_vrep")
        for t in in_topics:
                rospy.Subscriber(aseba_ns+t,AsebaEvent,on_aseba_event(t))
        for t in out_topics:
                rospy.Subscriber(vrep_ns+t,String,on_vrep_event(t))
        rospy.Service('aseba/load_script',LoadScripts,load_script)
        rospy.spin()


if __name__ == '__main__':
	main()
	
