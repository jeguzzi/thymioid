#!/usr/bin/env python

import wiringpi2
import rospy
from std_msgs.msg import Bool

#gpio pin numbers

AC_OK=0
BATT_OK=2

IN=0
OUT=0


class ups_state_listener:

    def __init__(self):
        rospy.init_node('ups', anonymous=True)
        ac_pub = rospy.Publisher('ups/ac', Bool, queue_size=1)
        batt_pub = rospy.Publisher('ups/battery', Bool, queue_size=1)
        # does not need root priviledge
        # the pin have to be already exported
        # for example with "gpio export <number> out"
        wiringpi2.wiringPiSetupSys() 
        # does need root priviledge
        #wiringpi2.wiringPiSetup() 
        wiringpi2.pinMode(AC_OK,IN)
        wiringpi2.pinMode(BATT_OK,IN)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            ac_ok=wiringpi2.digitalRead(AC_OK)
            batt_ok=wiringpi2.digitalRead(BATT_OK)
            ac_pub.publish(ac_ok)
            batt_pub.publish(batt_ok)
            rate.sleep()

if __name__ == '__main__':
    try:
        ups_state_listener()
    except rospy.ROSInterruptException:
        pass
