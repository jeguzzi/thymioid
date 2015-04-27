#!/usr/bin/env python

import subprocess
import time
from time import sleep
import netifaces
import rospy
from std_msgs.msg import Bool, ColorRGBA
from thymio_driver.msg import Led

LONG_PRESS=8 #seconds
UI_IDLE_TIME=5 #seconds

class WifiUI(object):

    def __init__(self):

        #wait for the thymio

        rospy.wait_for_service('thymio_is_ready')

        rospy.init_node('wifi_node', anonymous=True)
        self.led_publisher=rospy.Publisher('led',Led,queue_size=1)
        rospy.Subscriber('buttons/center',Bool,self.on_button_center)
        rospy.Subscriber('buttons/left',Bool,self.on_button_left)
        rospy.Subscriber('buttons/forward',Bool,self.on_button_forward)
        self.last_time_fwd_button_down=None
        self.init_configuration()
        self.should_exit=False
    
        
    def run(self):
        r = rospy.Rate(1)
        while not self.should_exit and not rospy.is_shutdown():
            self.check_desired_configuration_timeout()
            self.check_long_press()
            r.sleep()

    def on_long_press(self):
        #the fwd button has been pressed for more than LONG_PRESS seconds
        #exit
        self.should_exit=True


    def on_button_forward(self,msg):
        if msg.data:
            self.last_time_fwd_button_down=rospy.Time.now()
        else:
            self.last_time_fwd_button_down=None
                

    
    @property
    def configuration(self):
        return self._configuration

    @configuration.setter
    def configuration(self,value):
        if value<0 or value >=len(self.configurations):
            return
        if(value!=self._configuration):
            print 'set wifi',value
            self.set_wifi(value)
            self._configuration=value


    def on_button_left(self,msg):
        if(msg.data==False):
            
            #change desired configuration            
            if(not self.changing_desired_configuration):
                self.desired_configuration=self.configuration
                self.changing_desired_configuration=True
            self.last_desired_input=rospy.Time.now()
            self.desired_configuration=(self.desired_configuration+1) % len(self.configurations)
            self.update_leds()


    def on_button_center(self,msg):
        if(msg.data==False):
            if self.changing_desired_configuration:
                self.changing_desired_configuration=False
                print 'H G',self.configuration,self.desired_configuration
                self.configuration=self.desired_configuration
                self.update_leds()

    def init_configuration(self):
        self._configuration=None
        self.interface='wlan0'
        self.configurations=[('off',None,None,False),('ac','wlan0','192.168.168.1',True),('casa','wlan_casa','192.168.1.',False)]


        self.set_configuration_from_network()

        self.desired_configuration=self.configuration
        rospy.loginfo('initial conf is %d' % self.configuration)

        self.changing_desired_configuration=False
        self.update_leds()



    def get_network_ip(self):
        try:
            return netifaces.ifaddresses(self.interface).get(netifaces.AF_INET)[0].get('addr')
        except Exception as e:
            #print "Could not retrieve ip address:",e
            return None

            
    def service_is_running(self,name):
        status=subprocess.check_output(["/etc/init.d/"+name,"status"])
        return status.find('running')>=0
                        

    def check_ap(self):
        return self.service_is_running("isc-dhcp-server") and self.service_is_running("hostapd") 


    def set_configuration_from_network(self):
        ip=self.get_network_ip()

        if not ip:
            self._configuration=0
            return

        for i,c in enumerate(self.configurations): 
            name,iface,addr,is_ap=c
            
            if ip and not addr:
                continue

            if ip.find(addr)<0:
                continue


            #hardcoded: 0 is down config!                
            if is_ap and not self.check_ap():
                self._configuration=0
            else:
                self._configuration=i
            return
        
        self._configuration=0

                



    def stop_ap(self):
        #iface should start/stop automatically dhcp server and hostapd (see /etc/network/interfaces)
        print subprocess.check_output(["sudo","/etc/init.d/isc-dhcp-server","stop"])
        print subprocess.check_output(["sudo","/etc/init.d/hostapd","stop"])


    def start_wifi(self,name):
        rospy.loginfo("Connect %s with iface %s" % (self.interface,name))
        print subprocess.check_output(["sudo","ifup",self.interface+"="+name])
        rospy.loginfo('Got ip %s' % self.get_network_ip())
                
    def stop_wifi(self):
        rospy.loginfo("Put down %s" % self.interface)
        print subprocess.check_output(["sudo","ifdown","wlan0"])

    def set_wifi(self,value):
        _,new_iface,_,new_is_ap=self.configurations[value]
        _,iface,_,is_ap=self.configurations[self.configuration]
        #print iface,'->',new_iface
        #print is_ap,'->',new_is_ap
        self.stop_wifi()
        # not sure why this is needed?
        if not new_is_ap:
            self.stop_ap()
                
        time.sleep(2)

        if new_iface:
            self.start_wifi(new_iface)
                        
 
    def check_long_press(self):
        if self.last_time_fwd_button_down and (rospy.Time.now()-self.last_time_fwd_button_down).to_sec()>LONG_PRESS:
            self.on_long_press()

    def check_desired_configuration_timeout(self):
        if self.changing_desired_configuration:
            if (rospy.Time.now()-self.last_desired_input).to_sec()>5:
                self.changing_desired_configuration=False
                self.update_leds()

    def update_leds(self):
        msg=Led()
        msg.id=Led.CIRCLE
        msg.values=8*[0.0]
        if self.changing_desired_configuration:
            msg.values[self.desired_configuration % 8]=0.07
        else:
            msg.values[self.configuration % 8]=1.0

        self.led_publisher.publish(msg)
            


if __name__ == '__main__':
    ui=WifiUI()
    ui.run()
    
