#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import time
import sys
import rospy
from std_msgs.msg import String


from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from optparse import OptionParser


DEFAULT_PEPPER_IP = '10.10.3.129'
DEFAULT_PEPPER_PORT = 9559

class Led_Setting():

    def __init__(self,pip,port):

        self.led_proxy = ALProxy("ALLeds",pip,port)
        
        self.off_mode()
        self.current_status = 10

    def monitor_led_status(self):
        if(self.current_status==0):
            self.led_proxy.setIntensity("AllLedsBlue", 1)
            self.led_proxy.setIntensity("AllLedsGreen", 1)
            self.led_proxy.setIntensity("AllLedsRed", 0)
        
        elif(self.current_status==1):
            self.searching_for_answer_mode()

        elif(self.current_status==2):
            self.answer_mode()
        
        else:
            self.off_mode()

    def reset_mode(self):
        self.led_proxy.reset('AllLeds')
    
    def answer_mode(self):
        self.led_proxy.setIntensity("AllLedsBlue", 0)
        self.led_proxy.setIntensity("AllLedsGreen", 1)
        self.led_proxy.setIntensity("AllLedsRed", 0)

    def off_mode(self):
        self.led_proxy.off('AllLeds')

    def searching_for_answer_mode(self):
        #self.led_proxy.randomEyes(3.0)
        #self.led_proxy.rasta(1.0)
        #self.led_proxy.rotateEyes(65535,1.0,5.0)
        self.led_proxy.setIntensity("AllLedsBlue", 0)
        self.led_proxy.setIntensity("AllLedsGreen", 1)
        self.led_proxy.setIntensity("AllLedsRed", 1)

    def led_status_callback(self,data):
        self.current_status = int(data.data)
        #print(data.data)


def main():
    ## Main entry point


    parser = OptionParser()


    parser.add_option("-i","--ip",
        help="Pass ip of the robot",
        dest="pip")
    
    parser.add_option("-d","--pepperID",
        help="Pass id of the robot",
        dest="pepperID")

    parser.set_defaults(
        pip = DEFAULT_PEPPER_IP,
        pepperID='1')

    (opts, args_) = parser.parse_args()
    
    pip = opts.pip
    pepperID = opts.pepperID

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    #The broker must stay alive until the program exists
    # No need for a broker in this code
    #     # myBroker = ALBroker("myBroker",
    #     "0.0.0.0",   # listen to anyone
    #     0,           # find a free port and use it
    #     pip,         # parent broker IP
    #     9559)       # parent broker port


    # Warning: HumanGreeter must be a global variable
    # The name given to the constructor must be the name of the
    # variable\

   
    myLedSetting = Led_Setting(pip,DEFAULT_PEPPER_PORT)

    rospy.init_node('led_node', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    rospy.Subscriber("led_status_data_"+pepperID, String, myLedSetting.led_status_callback)

    try:
        while not rospy.is_shutdown():
            
            myLedSetting.monitor_led_status()
            rate.sleep()

    except KeyboardInterrupt:
        
        print ("Interrupted by user, shutting down")
        sys.exit(0)



if __name__ == "__main__":
    main()