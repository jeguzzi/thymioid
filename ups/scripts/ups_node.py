#!/usr/bin/env python

import wiringpi2
import rospy
from std_msgs.msg import Bool
import subprocess


AC_OK=0
BATT_OK=2

IN=0
OUT=1




class ups_state_listener:

    def read_pin(self,pin):
        return int(subprocess.check_output(['/usr/local/bin/gpio','read',str(pin)]))        

    def read_pin2(self,pin):
        return wiringpi2.digitalRead()

    def setup_gpio(self):
        # does not need root priviledge
        # the pin have to be already exported
        # for example with "gpio export <number> out"
        wiringpi2.wiringPiSetupSys() 
        # does need root priviledge
        #wiringpi2.wiringPiSetup() 
        wiringpi2.pinMode(AC_OK,IN)
        wiringpi2.pinMode(BATT_OK,IN)

    def __init__(self):
        rospy.init_node('ups', anonymous=True)
        ac_pub = rospy.Publisher('ups/ac', Bool, queue_size=1)
        batt_pub = rospy.Publisher('ups/battery', Bool, queue_size=1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            ac_ok=self.read_pin(AC_OK)
            batt_ok=self.read_pin(BATT_OK)
            ac_pub.publish(ac_ok)
            batt_pub.publish(batt_ok)
            rate.sleep()

if __name__ == '__main__':
    try:
        ups_state_listener()
    except rospy.ROSInterruptException:
        pass
