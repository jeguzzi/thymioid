#!/usr/bin/python

#import wiringpi2
import subprocess
import time
import os
import sys
import aseba
from threading import Timer
from time import sleep
import netifaces
import gobject

def on_buttons(evt):
        print evt
        print "buttons"

BATT=2
AC=0 
LATCH=3



class MainLoop(object):
	def __init__(self):
                while True:
                        try:
                                self.aseba=aseba.Aseba()
                                print "Connected to Thymio"
                                break
                        except Exception as e:
                                print "Could not connect to thymio",e
                                time.sleep(1)

                self.aseba.load_scripts("/home/odroid/ros/catkin_ws/src/ros-aseba/thymio_driver/aseba/thymio_ros.aesl")
                self.name="thymio-II"

                for button in ['left','right','center','backward','forward']:
                        self.aseba.on_event("button_"+button,self.on_buttons(button))

                self.hostapd=None
                
                i,state=self.check_wifi()
                self.wifi=i
                self.wifi_state=state
                self.init_configuration()
		self.init_ups()


        def init_ups(self):
                 #wiringpi2.wiringPiSetupSys() 
                 #wiringpi2.pinMode(AC,0)
                 #wiringpi2.pinMode(BATT,0)
                 #wiringpi2.pinMode(LATCH,1)
                 #wiringpi2.digitalWrite(LATCH,1)
                 self.ups_state=None




        def start_ap(self):
                if True: #self.wifi!='ac':
			print "bring wlan0 up"			
			print subprocess.check_output(["sudo","ifup","wlan0"],shell=False)
		time.sleep(0.5)
		print 'got ip?',self.get_ip()
		print 'start dhcp'
                subprocess.check_output(["sudo","service","isc-dhcp-server","start"])
		time.sleep(0.5)
		print 'start hostapd'
                self.hostapd=subprocess.Popen(["/usr/local/bin/hostapd","/etc/hostapd/hostapd.conf"],shell=False)
		print 'done'

        def stop_ap(self):
                if self.hostapd:
                        self.hostapd.kill()
                        print subprocess.check_output(["rm","/var/run/hostapd/wlan0"])
			self.hostpapd=None 
	        else:
			print subprocess.check_output(["sudo","pkill","hostapd"])
		print subprocess.check_output(["sudo","service","isc-dhcp-server","stop"])

        def start_managed(self):
                print subprocess.check_output(["sudo","ifup","wlan0=wlan_casa"])

        def stop_wifi(self):
                print subprocess.check_output(["sudo","ifdown","wlan0"])



        def set_wifi(self,value):
                if self.configuration != -1:#0:
			self.stop_wifi()
                	time.sleep(0.5)
		if self.configuration == 1:
                        self.stop_ap()
                if value is 1:
                        self.start_ap()
                elif value is 2:
                        self.start_managed()
                        

        @property
        def configuration(self):
                return self._configuration
        
        @configuration.setter
        def configuration(self,value):
                value=value % self.number_of_configurations
                if(value!=self._configuration):
                        self.set_wifi(value)
                        self._configuration=value
                        
                                
                
        def get_ip(self):
                try:
                        return netifaces.ifaddresses("wlan0").get(netifaces.AF_INET)[0].get('addr')
                except Exception as e:
                        print "Could not retrieve ip address:",e
                        return None


        def check_ap(self):
                if(not self.hostapd):
                        print "no hostapd"
                        return False
                if(self.hostapd.poll()):
                        print "hostapd is not running"
                        return False
                status=subprocess.check_output(["sudo","service","isc-dhcp-server","status"])
                print "dhcp server status",status
                if(status.find('running')<0):
                        return False
                return True

        def check_wifi(self):
                ip=self.get_ip()
                print 'ip is',ip 
                if(ip is None):
                        return ('off',True)
                if(ip.find('192.168.168')>-1):
                        return ('ap',self.check_ap())
                else:
                        return ('managed',True) 

        def check_thymio(self):
            #should get the state form aseba (like exceptions)
            pass
            

        def set_ups_color(self,rgb):
                self.aseba.send_event("set_led_top",rgb)
                self.aseba.send_event("set_led_bottom_right",rgb)
                self.aseba.send_event("set_led_bottom_left",rgb)

        def start_alarm(self):
                self.alarm_timeout=gobject.timeout_add(3000,self.alarm)
                
        def alarm(self):
                self.aseba.send_event("play_system_sound",[2])

        def update_ups(self,ac,batt):
                #print ac,batt
                if ac==1:
                        new_state='ac'
                elif batt==1:
                        new_state='batt'
                else:
                        new_state='reserve'

                print new_state
                if self.ups_state != new_state:
                        if self.ups_state=='reserve':
                                gobject.source_remove(self.alarm_timeout)
                        self.ups_state=new_state
                        if(self.ups_state=='ac'):
                                self.set_ups_color([0,32,0])
                        elif(self.ups_state=='batt'):
                                self.set_ups_color([32,32,0])
                        else:
                                self.set_ups_color([32,0,0])
                                self.start_alarm()
        
        def check_ups(self):
            ac=self.get_odroid_ac()
            batt=self.get_odroid_battery()
            self.update_ups(ac,batt)

        def get_thymio_battery(self):
            bat=self.aseba.get(self.name,"_vbat")
            return 0.5*(bat[0]+bat[1])

        def get_odroid_battery(self):
                return int(subprocess.check_output(['/usr/local/bin/gpio','read','2']))
                #return wiringpi2.digitalRead(BATT)

        def get_odroid_ac(self):
                return int(subprocess.check_output(['/usr/local/bin/gpio','read','0']))
                #return wiringpi2.digitalRead(AC)

        def on_buttons(self,button):
                def cb(evt):
                        v=int(evt[0])
                        self.on_button(button,v)
                return cb
     
        def on_button(self,button,value):
                if value==0 and button=='left':
                        #change desired configuration
                        if(not self.changing_desired_configuration):
                                self.desired_configuration=self.configuration
                        self.changing_desired_configuration=True
                        self.last_desired_input=time.time()
                        self.desired_configuration=(self.desired_configuration+1) % self.number_of_configurations
                        self.set_led_to_desired_configuration()
                        
                if value==0 and button=="center":
                        if self.changing_desired_configuration:
                                self.configuration=self.desired_configuration
                                self.set_led_to_configuration()
                
        def init_configuration(self):
                self.number_of_configurations=3
                i,s=self.check_wifi()
                if i=='ac' and s:
                        self._configuration=1
                elif i=='managed':
                        self._configuration=2
                else:
                        self._configuration=0
                
                self.desired_configuration=self.configuration
                self.set_led_to_configuration()
                self.changing_desired_configuration=False

        def check_desired_configuration_timeout(self):
                if self.changing_desired_configuration:
                        if time.time()-self.last_desired_input>5:
                                self.changing_desired_configuration=False
                                self.set_led_to_configuration()

        def set_led_to_configuration(self):
                leds=9*[0]
                leds[(self.configuration % 8) +1 ]=32
                self.aseba.send_event("set_led",leds)

                        
        def set_led_to_desired_configuration(self):
                leds=9*[0]
                leds[(self.desired_configuration % 8) +1 ]=2
                self.aseba.send_event("set_led",leds)

        def run2(self):
                gobject.timeout_add(1000,self.update)
                self.aseba.run()

        def update(self):
                print "."
                self.check_desired_configuration_timeout()
                self.check_ups()
                return True

        def run(self):
            while True:
                try:
                        self.check_wifi()
                        time.sleep(10)
                except KeyboardInterrupt:
                        break


def export_dbus_env():
        os.environ["DISPLAY"]=":0"
        p = subprocess.Popen('dbus-launch', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        for var in p.stdout:
                sp = var.split('=', 1)
                print sp
                if(len(sp)>1): 
                        os.environ[sp[0]] = sp[1][:-1]

def main():
        export_dbus_env()
        aseba_medulla = subprocess.Popen(["/home/odroid/ros/catkin_ws/devel/lib/aseba/asebamedulla",  "ser:name=Thymio-II"], shell=False)
        main_loop = MainLoop()
        main_loop.run2()
        aseba_medulla.kill()
        print "exit"
        

if __name__ == '__main__':
        main()



