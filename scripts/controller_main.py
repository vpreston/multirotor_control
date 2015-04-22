#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('roscopter')
import roscopter.msg
import matplotlib.pyplot as plt
import numpy as np

from std_srvs.srv import *
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Imu, NavSatStatus, NavSatFix
from datetime import datetime
from roscopter.srv import APMCommand
from roscopter.msg import VFR_HUD, State

# This code should launch a quadcopter into the air, approach a desired altitude set by the user, hover there for approximately 1 minute, then descend.  GPS and Altitude drift should be minimal. All control loops are self-designed.  Failsafe is controlled by human and puts it into auto-stabilized RC mode.

class MCN():
    def __init__(self):
        rospy.init_node('main_control_node')
        #set up control joystick
        self.axes = []
        self.buttons = []
        self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500] #Setting to stabilized mode
        self.x = 1500.0 #Roll
        self.y = 1500.0 #Pitch
        self.z = 1000.0 #Throttle
        self.yaw = 1500.0 #Yaw
        #read in data of interest
        self.alt = 0.0
        self.plt_alt = []
        self.armed = False
        self.risen = False
        self.failsafe = False
        #variables that need memory
        self.xerror = 0
        self.xerrorv = 0
        self.xerrora = 0
        self.oldxerror = 0
        self.oldxerrorv = 0
        self.yerror = 0
        self.yerrorv = 0
        self.yerrora = 0
        self.oldyerror = 0
        self.oldyerrorv = 0
        self.zerror = 0
        self.zerrorv = 0
        self.zerrora = 0
        self.oldzerror = 0
        self.oldzerrorv = 0

        self.psierror = 0
        self.psierrv = 0
        self.psierra = 0
        self.oldpsierror = 0
        self.oldpsierrv = 0
        self.phierror = 0
        self.phierrv = 0
        self.phierra = 0
        self.oldphierror = 0
        self.oldphierrv = 0
        self.thetaerror = 0
        self.thetaerrv = 0
        self.thetaerra = 0
        self.oldthetaerror = 0
        self.oldthetaerrv = 0
        #set up communications protocol
        self.pub_rc = rospy.Publisher('/send_rc', roscopter.msg.RC)
        self.sub_state = rospy.Subscriber('/state', State, self.state_check)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.sub_height = rospy.Subscriber('/filtered_pos', FilteredPosition, self.parse_action)
        self.sub_attitude = rospy.Subscriber('/attitude', Attitude, self.parse_attitude)
        self.sub_raw = rospy.Subscriber('/mavlink_raw_imu', Mavlink_RAW_IMU, self.parse_raw)
        self.command_serv = rospy.ServiceProxy('command', APMCommand)
        #desired position vector and control loop constants
        self.desire = {'x':0, 'y':0, 'z':1, 'h':0} #No horizontal movement, only altitude
        self.bh = 1.11
        self.bt = 1.11
        self.mm = 0.075
        self.mq = 1.00
        self.l = 0.25
        self.r = 0.08
        self.jxx = (2*self.mq*self.r**2)/5 + 2*self.mm*self.l**2
        self.jyy = (2*self.mq*self.r**2)/5 + 2*self.mm*self.l**2
        self.jzz = (2*self.mq*self.r**2)/5 + 4*self.mm*self.l**2
        self.g = -9.8
        #controller
        r = rospy.Rate(20) # 0.1 second loop rate
        while not rospy.is_shutdown():
            self.hover_uncompensated()
            r.sleep()


    def joy_callback(self, data):
        #Reads in joystick controller positions and information
        self.axes = data.axes
        self.buttons = data.buttons
        self.x = 1500-self.axes[0]*300 #Scales 1200-1800
        self.y = 1500-self.axes[1]*300 #Scales 1200-1800
        self.z = 2000+(self.axes[3])*1000 #Scales 1000-3000
        self.yaw = 1500-self.axes[2]*300 #Scales 1200-1800

    def state_check(self, data):
        #Determines whether vehicle is armed and therefore able to receive commands
        self.armed = data.armed

    def parse_action(self, data):
        #Reads in data of interest from the vehicle
        self.groundxspeed = data.ground_x_speed/100
        self.groundyspeed = data.ground_y_speed/100
        self.groundzspeed = data.ground_z_speed/100

    def parse_raw(self, data):
        #Reads in data of interest from the vehicle
        self.xacc = data.xacc
        self.yacc = data.yacc
        self.zacc = data.zacc
        self.xgyro = data.xgyro
        self.ygyro = data.ygyro
        self.zgyro = data.zgyro
        self.xmag = data.xmag
        self.ymag = data.ymag
        self.zmag = data.zmag

    def parse_attitude(self, data):
        #Reads in data of interest from the vehicle
        self.roll = data.roll
        self.pitch = data.pitch
        self.yaw = data.yaw
        self.rollspeed = data.rollspeed
        self.pitchspeed = data.pitchspeed
        self.yawspeed data.yawspeed

    def hover_uncompensated(self):
        #Reads in information about vehicle in order to determine action
        if self.buttons: #If button pressed
            if self.buttons[3]: #Disarm and plot data from flight
                self.command_serv(4)
                self.risen = False
                print 'Disarm Quad'
                plt.plot(self.plt_alt)
                plt.show()
            if self.buttons[2]: #Arm
                self.command_serv(3)
                print 'Arm Quad'
            if self.buttons[0]: #Failsafe
                self.command_serv(8)
                self.failsafe = True
                print 'Failsafe'
                rospy.sleep(5)

        if self.armed and not self.failsafe:
            if self.alt < 1.0 and not self.risen:
                print 'Rise'
                #Make it rise, uncompensated
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1550, 1500)
                self.pub_rc.publish(self.twist)
            elif self.alt > 1.0: 
                #TODO Tune for Hovering here
                self.risen = True
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1450, 1500)
                self.pub_rc.publish(self.twist)
            elif self.alt > 0.2 and self.risen:
                print 'Lower'
                #Lower it, uncompensated
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1450, 1500)
                self.pub_rc.publish(self.twist)
            elif self.alt <= 0.2 and self.risen:
                print 'Land'
                #Land itm uncompensated
                (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1000, 1500)
                self.pub_rc.publish(self.twist)
                print 'Test Complete, Copter Disarming'
                self.command_serv(4)
                self.risen = False
        elif self.failsafe:
            self.command_serv(7) #Put in stabilized mode
            (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), int(self.z), int(self.yaw))
        
        self.pub_rc.publish(self.twist)

    def hover_loop(self):
        #TODO Read in desired vector, convert to desired orientation
        xd = self.desire['x']
        yd = self.desire['y']
        zd = self.desire['z']
        hd = self.desire['h']

        #Calculate desired pitch and roll orientations based upon acceleration of x and y

        self.xerror = xd - self.xmag - self.xgyro
        self.yerror = yd - self.ymag - self.ygyro
        self.zerror = zd - self.zmag - self.zgyro
        self.xerrorv = (self.xerror - self.oldxerror)*0.1
        self.xerrora = (self.xerrorv - self.oldxerrorv)*0.1
        self.yerrorv = (self.yerror - self.oldyerror)*0.1
        self.yerrora = (self.yerrorv - self.oldyerrorv)*0.1
        self.zerrorv = (self.zerror - self.oldzerror)*0.1
        self.zerrora = (self.zerrorv - self.oldzerrorv)*0.1

        pd = self.g*self.yerrora*np.cos(self.yaw)
        td = -self.g*self.xerrora*np.cos(self.yaw)

        self.phierror = pd - self.roll - self.rollspeed
        self.psierror = hd - self.yaw - self.yawspeed
        self.thetaerror = td - self.pitch - self.pitchspeed
        self.phierrv = (self.phierror - self.oldphierror)*0.1
        self.phierra = (self.phierrv - self.oldphierrv)*0.1
        self.psierrv = (self.psierror - self.oldpsierror)*0.1
        self.psierra = (self.psierrv - self.oldpsierrv)*0.1
        self.thetaerrv = (self.thetaerror - self.oldthetaerror)*0.1
        self.thetaerra =  (self.thetaerrv - self.oldthetaerrv)*0.1

        phival = self.phierra * self.jxx/(self.l*self.bt)
        psival = self.psierra * self.jzz/(self.bh)
        thetaval = self.thetaerra * self.jyy/(self.l*self.bt)
        zval = self.zerrora / (self.bt*(1/(self.mm+self.mq)) - self.g)

        w4 = np.sqrt((zval - psival - phival)/4)
        w1 = np.sqrt((psival - thetaval + phival + 2*w4**2)/2)
        w2 = np.sqrt(phival + w4**2)
        w3 = np.sqr(thetaval + w1**2)

        sig1 = (w1 + 10)/0.08
        sig2 = (w2 + 10)/0.08
        sig3 = (w3 + 10)/0.08
        sig4 = (w4 + 10)/0.08

        self.oldxerror = self.xerror
        self.oldyerror = self.yerror
        self.oldzerror = self.zerror
        
        self.oldxerrorv = self.xerrorv
        self.oldyerrorv = self.yerrorv
        self.oldzerrorv = self.zerrorv

        self.oldphierror = self.phierror
        self.oldpsierror = self.psierror
        self.oldthetaerror = self.thetaerror

        self.oldphierrv = self.phierrv
        self.oldpsierrv = self.psierrv
        self.oldthetaerrv = self.thetaerrv
        
        #Reads in information about vehicle in order to determine action
        if self.buttons: #If button pressed
            if self.buttons[3]: #Disarm and plot data from flight
                self.command_serv(4)
                self.risen = False
                print 'Disarm Quad'
                plt.plot(self.plt_alt)
                plt.show()
            if self.buttons[2]: #Arm
                self.command_serv(3)
                print 'Arm Quad'
            if self.buttons[0]: #Failsafe
                self.command_serv(8)
                self.failsafe = True
                print 'Failsafe'
                rospy.sleep(5)

        if self.armed and not self.failsafe:
            (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(sig1), int(sig2), int(sig3), int(sig4))

        elif self.failsafe:
            self.command_serv(2) #Sends the land command


if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    #TODO Groundspeed/Airspeed control loop to have stable vertical flight
