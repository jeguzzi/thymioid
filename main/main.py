#!/usr/bin/python

import subprocess
import time
import os
import sys
import aseba
from threading import Timer
from time import sleep
import netifaces

class MainLoop(object):
	def __init__(self):
            self.aseba=aseba.Aseba()
            self.name="thymio-II"
            #self.aseba.on_event("buttons",self.on_buttons)
            #filename="<file>"
            #self.aseba.load_scripts(filename)
            #self.check_ups_timer=Timer(1,self.check_ups)
        
        def start_ap(self):
                subprocess.Popen(["sudo","ifup","wlan0"],shell=False)
                self.hostapd=subprocess.Popen(["/usr/local/bin/hostapd","/etc/hostapd/hostapd.conf"],shell=False)
                subprocess.Popen(["sudo","service","isc-dhcp-server","start"],shell=False)

        def stop_ap(self):
                subprocess.Popen(["sudo","ifdown","wlan0"],shell=False)
                if self.hostapd:
                        self.hostapd.kill()
                subprocess.Popen(["sudo","service","isc-dhcp-server","stop"],shell=False)

        def start_managed(self):
                subprocess.Popen(["sudo","ifup","wlan0=wlan_casa"],shell=False)

        def stop_managed(self):
                subprocess.Popen(["sudo","ifdown","wlan0"],shell=False)


        @property
        def wifi(self):
                pass
        
        @wifi.setter
        def wifi(self,value):
                if(value!=self._wifi):
                        pass
                                
                
        def get_ip(self):
                try:
                        return netifaces.ifaddresses("en0").get(netifaces.AF_INET)[0].get('addr')
                except Exception as e:
                        print "Could not retrieve ip address:",e
                        return None


        def check_ap():
                if(not self.hostapd):
                        return False
                if(self.hostapd.poll()):
                        return False
                status=subprocess.check_output(["sudo","service","isc-dhcp-server","service"],shell=False)
                if(status.find('running')<0):
                        return False
                return True

        def check_wifi(self):
                ip=get_ip()
                print 'ip is',ip 
                if(ip is None):
                        return ('off',True)
                if(ip.find('192.168.168')):
                        return ('ap',self.check_ap())
                else:
                        return ('managed',True) 

        def check_thymio(self):
            #should get the state form aseba (like exceptions)
            pass
            

        def set_wifi(self):
            pass
        
        def check_ups(self):
            ac=get_odroid_ac()
            batt=get_odorid_ac()
            self.update_ups(ac,batt)

        def get_thymio_battery(self):
            bat=self.aseba.get(self.name,"_vbat")
            return 0.5*(bat[0]+bat[1])

        def get_odroid_battery(self):
            return 1

        def get_odroid_ac(self):
            return 1

        def on_buttons(self,evt):
            pass

        def run(self):
            while True:
                pass


def main():
    # check command-line arguments
    #aseba_mesulla = subprocess.Popen(["asebamedulla",  "ser:name=Thymio-II"], shell=False)
    main_loop = MainLoop()
    main_loop.run()

if __name__ == '__main__':
    main()



