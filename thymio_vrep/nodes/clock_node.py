#!/usr/bin/env python

import rospy
from vrep_common.msg import VrepInfo
from rosgraph_msgs.msg import Clock

class SimClock(object):

    def __init__(self):
        rospy.init_node("clock_node")
        self.time=0
        self.clock_publisher = rospy.Publisher("/clock",Clock,queue_size=1)
        rospy.Subscriber("/vrep/info",VrepInfo,self.infoCallback)
        self.should_exit=False

    def infoCallback(self,msg):
        new_time=msg.simulationTime.data
        if(new_time<self.time):
            rospy.signal_shutdown('simulation reset')
        if(new_time>self.time):
            self.state=msg.simulatorState
            self.time=new_time
            self.clock_publisher.publish(Clock(rospy.Time.from_sec(self.time)))
        
def main():
    clock=SimClock()
    rospy.spin()


if __name__ == '__main__':
	main()
	


