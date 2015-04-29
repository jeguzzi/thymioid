#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, ColorRGBA

class UpsWard(object):

    def __init__(self):
        rospy.init_node('ups_ward', anonymous=True)

        #wait for the thymio

        rospy.wait_for_service('thymio_is_ready')
        rospy.sleep(2)

        rospy.loginfo("UPS WARD READY TO START")

        self.led_publishers=[rospy.Publisher('led/body/'+led,ColorRGBA,queue_size=1) for led in  ['top','bottom_left','bottom_right']]

        self.alarm_publisher=rospy.Publisher("alarm",Bool,queue_size=1)
        self.batt_sub=rospy.Subscriber('ups/battery',Bool,self.on_battery)
        self.ac_sub=rospy.Subscriber('ups/ac',Bool,self.on_ac)
        self.battery=None
        self.ac=None
        self.ups_state=None

        self.ups_color={}
        self.ups_color['ac']=ColorRGBA(0.0,1.0,0.0,0.0)
        self.ups_color['battery']=ColorRGBA(1.0,1.0,0.0,0.0)
        self.ups_color['reserve']=ColorRGBA(1.0,0.0,0.0,0.0)



    def set_ups_color(self):
        self.set_color(self.ups_color[self.ups_state])

    def on_battery(self,msg):
        self.battery=msg.data

    def on_ac(self,msg):
        self.ac=msg.data
        self.update()

    def stop_alarm(self):
        self.alarm_publisher.publish(False)

    def start_alarm(self):
        self.alarm_publisher.publish(True)

    def set_color(self,color):
        for p in self.led_publishers:
            p.publish(color)

    def update(self):
        if self.ac==1:
            new_state='ac'
        elif self.battery==1:
            new_state='battery'
        else:
            new_state='reserve'
            
        if self.ups_state != new_state:
            if self.ups_state=='reserve':
                self.stop_alarm()
            self.ups_state=new_state
            if(self.ups_state=='reserve'):
                self.start_alarm()
            self.set_ups_color()
    
if __name__ == '__main__':
    ward=UpsWard()
    rospy.spin()
