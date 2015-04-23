#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('roscopter')
import roscopter.msg
import matplotlib.pyplot as plt
import numpy as np
import tf

from std_srvs.srv import *
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Imu, NavSatStatus, NavSatFix
from datetime import datetime
from roscopter.srv import APMCommand
from roscopter.msg import VFR_HUD, State, Attitude, Mavlink_RAW_IMU, FilteredPosition

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
        self.init_alt = 0.0
        self.plt_alt = []
        self.armed = False
        self.risen = False
        self.failsafe = False
        self.xacc = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.yacc = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.zacc = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        # self.xgyro = 0
        # self.ygyro = 0
        # self.zgyro = 0
        self.xmag = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.ymag = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.zmag = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.count = 0
        self.xpos_init = 0
        self.ypos_init = 0
        self.zpos_init = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.rollspeed = 0
        self.pitchspeed = 0
        self.yawspeed =  0
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

        self.last_x = 0
        #set up communications protocol
        self.pub_rc = rospy.Publisher('/send_rc', roscopter.msg.RC)
        self.sub_state = rospy.Subscriber('/state', State, self.state_check)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.vfr = rospy.Subscriber("/vfr_hud", VFR_HUD, self.parse_altitude)
        self.sub_height = rospy.Subscriber('/filtered_pos', FilteredPosition, self.parse_action)
        self.sub_attitude = rospy.Subscriber('/attitude', Attitude, self.parse_attitude)
        self.sub_raw = rospy.Subscriber('/raw_imu', Mavlink_RAW_IMU, self.parse_raw)
        self.command_serv = rospy.ServiceProxy('command', APMCommand)
        #desired position vector and control loop constants
        self.desire = {'x':0, 'y':0, 'z':1, 'h':0} #No horizontal movement, only altitude
        self.bh = 0.00008
        self.bt = self.bh
        self.mm = 0.75
        self.mq = 1.00
        self.l = 0.25
        self.r = 0.08
        self.jxx = (2*self.mq*self.r**2)/5 + 2*self.mm*self.l**2
        self.jyy = (2*self.mq*self.r**2)/5 + 2*self.mm*self.l**2
        self.jzz = (2*self.mq*self.r**2)/5 + 4*self.mm*self.l**2
        self.g = -9.8
        #controller
        r = rospy.Rate(20) # 0.2 second loop rate
        while not rospy.is_shutdown():
            self.hover_loop()
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
        self.xacc = np.delete(self.xacc, [0])
        self.xacc = np.append(self.xacc,[(float(data.xacc)/1000 * 9.80665)])
        self.yacc = np.delete(self.yacc, [0])
        self.yacc = np.append(self.yacc,[float(data.yacc)/1000 * 9.80665])
        self.zacc = np.delete(self.zacc, [0])
        self.zacc = np.append(self.zacc, [-float(data.zacc)/1000 * 9.80665])
        # self.xgyro = float(data.xgyro)/1000
        # self.ygyro = float(data.ygyro)/1000
        # self.zgyro = -float(data.zgyro)/1000
        self.xmag = np.delete(self.xmag, [0])
        self.xmag = np.append(self.xmag,[(float(data.xmag)/100)])
        self.ymag = np.delete(self.ymag, [0])
        self.ymag = np.append(self.ymag,[float(data.ymag)/100])
        self.zmag = np.delete(self.zmag, [0])
        self.zmag = np.append(self.zmag, [-float(data.zmag)/100])

        if self.count == 10:
            self.xpos_init = (float(data.xmag)/100)
            self.ypos_init = (float(data.ymag)/100)
            self.zpos_init = (float(data.zmag)/100)

        self.count += 1

    def parse_attitude(self, data):
        #Reads in data of interest from the vehicle
        self.roll = float(data.roll)
        self.pitch = float(data.pitch)
        self.yaw = -float(data.yaw)
        self.rollspeed = float(data.rollspeed)
        self.pitchspeed = float(data.pitchspeed)
        self.yawspeed =  float(data.yawspeed)

    def parse_altitude(self, data):
        #Reads in data of interest from the vehicle
        self.alt = float(data.alt)

        if self.count == 9:
            self.init_alt = float(data.alt)

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
        xd = self.desire['x'] + self.xpos_init
        yd = self.desire['y'] + self.ypos_init
        zd = self.desire['z'] + self.init_alt
        hd = self.desire['h']

        xaverage = np.average(self.xmag)
        yaverage = np.average(self.ymag)
        zaverage = np.average(self.zmag)

        #Calculate desired pitch and roll orientations based upon acceleration of x and y
        xerror = xd - xaverage
        yerror = yd - yaverage
        zerror = zd - self.alt
        xerrorv = -(xerror - self.oldxerror)/0.2
        xerrora = -(xerrorv - self.oldxerrorv)/0.2
        yerrorv = -(yerror - self.oldyerror)/0.2
        yerrora = -(yerrorv - self.oldyerrorv)/0.2
        zerrorv = -(zerror - self.oldzerror)/0.2
        zerrora = -(zerrorv - self.oldzerrorv)/0.2

        pd = self.g*yerrora*np.cos(hd)
        td = -self.g*xerrora*np.cos(hd)

        phierror = pd - self.roll - 0.2*self.rollspeed
        psierror = hd - self.yaw - 0.2*self.yawspeed
        thetaerror = td - self.pitch - 0.2*self.pitchspeed
        phierrv = -(phierror - self.oldphierror)/0.2
        phierra = -(phierrv - self.oldphierrv)/0.2
        psierrv = -(psierror - self.oldpsierror)/0.2
        psierra = -(psierrv - self.oldpsierrv)/0.2
        thetaerrv = -(thetaerror - self.oldthetaerror)/0.2
        thetaerra =  -(thetaerrv - self.oldthetaerrv)/0.2

        phival = phierra * self.jxx/(self.l*self.bt)
        psival = psierra * self.jzz/(self.bh)
        thetaval = thetaerra * self.jyy/(self.l*self.bt)
        zval = (zerrora - self.g)/ (self.mm+self.mq)

        T4 = -self.g*(4*self.mm+self.mq)#(zval - psival - 2*phival)/4
        T1 = -self.g*(4*self.mm+self.mq)#(psival - thetaval + phival + 2*T4)/2
        T2 = -self.g*(4*self.mm+self.mq)#phival + T4
        T3 = -self.g*(4*self.mm+self.mq)#thetaval + T1

        print[zerror]

        w1 = np.sqrt(T1/self.bt)
        w2 = np.sqrt(T2/self.bt)
        w3 = np.sqrt(T3/self.bt)
        w4 = np.sqrt(T4/self.bt)

        sig1 = (w1 + 60)/0.5
        sig2 = (w2 + 60)/0.5
        sig3 = (w3 + 60)/0.5
        sig4 = (w4 + 60)/0.5
        #print [int(sig1), int(sig2), int(sig3), int(sig4)]

        self.oldxerror = xerror
        self.oldyerror = yerror
        self.oldzerror = zerror
        
        self.oldxerrorv = xerrorv
        self.oldyerrorv = yerrorv
        self.oldzerrorv = zerrorv

        self.oldphierror = phierror
        self.oldpsierror = psierror
        self.oldthetaerror = thetaerror

        self.oldphierrv = phierrv
        self.oldpsierrv = psierrv
        self.oldthetaerrv = thetaerrv
        
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
            print 'publishing'
            (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(sig1), int(sig2), int(sig3), int(sig4))

        elif self.failsafe:
            self.command_serv(2) #Sends the land command

        #self.pub_rc.publish(self.twist)


if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    #TODO Groundspeed/Airspeed control loop to have stable vertical flight
