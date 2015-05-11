#!/usr/bin/env python

import subprocess
import time
from time import sleep
import netifaces
import rospy
from std_msgs.msg import Bool, ColorRGBA, Empty
from thymio_msgs.msg import Led

LONG_PRESS=8 #seconds
UI_IDLE_TIME=5 #seconds

interface='wlan0'

class WifiUI(object):

    def __init__(self):

        #wait for the thymio

        rospy.wait_for_service('thymio_is_ready')

        rospy.init_node('wifi_node', anonymous=True)
        self.led_publisher=rospy.Publisher('led',Led,queue_size=1)
        self.shutdown_thymio_pub=rospy.Publisher('shutdown',Empty,queue_size=1)

        self.buttons=['left','center','right','forward','backward']
        for b in self.buttons:
            rospy.Subscriber('buttons/'+b,Bool,self.on_button(b))
        self.last_time_button_down={}

        self.beat=True
        rospy.Timer(rospy.Duration(1.0),self.send_beat)

        self.init_configuration()
        self.should_exit=False

        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        msg=Led()
        msg.id=Led.CIRCLE
        msg.values=8*[0.0]
        self.led_publisher.publish(msg)


    def send_beat(self,evt):
        msg=Led()
        msg.id=Led.BUTTONS
        msg.values=4*[0.0]
        if self.beat:
            msg.values[2]=1.0

        self.led_publisher.publish(msg)
        self.beat=not self.beat


    def run(self):
        r = rospy.Rate(1)
        while not self.should_exit and not rospy.is_shutdown():
            self.check_desired_configuration_timeout()
            self.check_long_press()
            r.sleep()


    def shutdown_odroid(self):
        subprocess.call(['sudo','shutdown','now'])
        #print "SHUTDOWN ODROID"


    def on_long_press(self,button):
        #the button has been pressed for more than LONG_PRESS seconds
        if(button=='forward'):
            #exit
            self.should_exit=True
            self.on_shutdown()

            try:
                subprocess.call(['sudo','service','thymioid','stop'])
            except Exception as e:
                rospy.logerr(e)
        if(button=='backward'):
            #shutdown
            #print "SHUTDOWN THYMIO"
            #subprocess.call(['rostopic','pub','/shutdown','--once','std_msgs/Empty'])
            self.shutdown_thymio_pub.publish(Empty())
            self.shutdown_odroid()


    def on_button(self,button):
        def cb(msg):
            if msg.data:
                self.last_time_button_down[button]=rospy.Time.now()
            else:
                self.last_time_button_down[button]=None
                self.button_up(button)
        return cb



    def button_up(self,button):
        if(button=='left'):
            #change desired configuration
            if(not self.changing_desired_configuration):
                self.desired_configuration=self.configuration
                self.changing_desired_configuration=True
            self.last_desired_input=rospy.Time.now()
            self.desired_configuration=(self.desired_configuration+1) % len(self.configurations)
            self.update_leds()

        if(button=='center'):
            #change configuration
            if self.changing_desired_configuration:
                self.changing_desired_configuration=False
                self.configuration=self.desired_configuration
                self.update_leds()

    @property
    def configuration(self):
        return self._configuration

    @configuration.setter
    def configuration(self,value):
        if value<0 or value >=len(self.configurations):
            return
        if(value!=self._configuration):
            rospy.loginfo('set wifi to %d' % value)
            success=self.set_wifi(value)
            if(success):
                self._configuration=value
            else:
                self.stop_wifi()
                self._configuration=0

    def init_configuration(self):
        self._configuration=None
        self.interface=interface
        self.configurations=[(None,None,False),('wlan0','192.168.168.1',True)]

        additional_interfaces=rospy.get_param("~wlan_interfaces",[])

        for interface in additional_interfaces:
            config=(interface.get('name'),interface.get('ip',''),interface.get('is_ap',False))
            self.configurations.append(config)


        self._configuration=self.read_configuration_from_network()

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
        try:
            status=subprocess.check_output(["/etc/init.d/"+name,"status"])
            return status.find('running')>=0
        except Exception as e:
            rospy.logerr("service is running, got exception %s" % e)
            return False

    def check_ap(self):
        return self.service_is_running("hostapd")
        #return self.service_is_running("dnsmasq") and self.service_is_running("hostapd")


    def read_configuration_from_network(self):
        ip=self.get_network_ip()
        #hardcoded: 0 is down config!

        if not ip:
            return 0

        for i,c in enumerate(self.configurations):
            iface,addr,is_ap=c

            if ip and not addr:
                continue

            if ip.find(addr)<0:
                continue

            if is_ap and not self.check_ap():
                return 0
            else:
                return i
            return

         return 0



    def stop_ap(self):
        #iface should start/stop automatically dhcp server and hostapd (see /etc/network/interfaces)

        try:
            rospy.loginfo(subprocess.check_output(["sudo","/etc/init.d/dnsmasq","stop"]))
            rospy.loginfo(subprocess.check_output(["sudo","/etc/init.d/hostapd","stop"]))
        except Exception as e:
            rospy.logerr("While stopping dnsmaq and hostpad, got exception %s" %e)

    def start_wifi(self,name):
        rospy.loginfo("Connect %s with iface %s" % (self.interface,name))
        #timeout set in /etc/dhcp/dhclient.conf to 60 seconds
        try:
            rospy.loginfo(subprocess.check_output(["sudo","ifup",self.interface+"="+name]))
            rospy.loginfo('Got ip %s' % self.get_network_ip())
            return True
        except Exception as e:
            rospy.logerr(e)
            return False

    def stop_wifi(self):
        rospy.loginfo("Put down %s" % self.interface)
        rospy.loginfo(subprocess.check_output(["sudo","ifdown",self.interface]))

    def set_wifi(self,value):
        new_iface,_,new_is_ap=self.configurations[value]
        iface,_,is_ap=self.configurations[self.configuration]
        #print iface,'->',new_iface
        #print is_ap,'->',new_is_ap
        self.stop_wifi()
        # not sure why this is needed?
        if not new_is_ap:
            self.stop_ap()

        time.sleep(2)

        if new_iface:
            success=self.start_wifi(new_iface)
            if(success and new_value==self.read_configuration_from_network()):
                return True
            else:
                return False

        return True


    def check_long_press(self):
        for b in self.buttons:
            if self.last_time_button_down.get(b,None) and (rospy.Time.now()-self.last_time_button_down.get(b)).to_sec()>LONG_PRESS:
                self.on_long_press(b)

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
