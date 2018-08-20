#!/usr/bin/env python
'''
started 23/11/17
use service call to start auto dock
encoders to be checked


'''

import rospy
import tf
from math import sin, cos
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float32
from math import sin, cos, pi, radians, degrees
from phoenix_msgs.srv import *
from phoenix_msgs.msg import Right_joint, Left_joint
from SerialDataGateway import SerialDataGateway


class Arduino(object):
    def _HandleReceivedLine(self, line):
        self._Counter = self._Counter + 1
        # rospy.logdebug(str(self._Counter) + " " + line)
        # if (self._Counter % 50 == 0):
        self._SerialPublisher.publish(String(str(self._Counter) + " " + line))
        if len(line) > 0:
            lineParts = line.split('\t')
            if lineParts[0] == 'o':
                self._BroadcastOdometryInfo(lineParts)
                return
            if lineParts[0] == 'b':
                self._BroadcastBatteryInfo(lineParts)
                return
            if lineParts[0] == 'd':
                self._BroadcastAuto(lineParts)
                return

    def _BroadcastOdometryInfo(self, lineParts):
        partsCount = len(lineParts)
        # rospy.logwarn(partsCount)
        if (partsCount < 6):
            pass

        try:
            x = float(lineParts[1])
            y = float(lineParts[2])
            theta = float(lineParts[3])

            vx = float(lineParts[4])
            omega = float(lineParts[5])

            # quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(theta / 2.0)
            quaternion.w = cos(theta / 2.0)

            rosNow = rospy.Time.now()

            # First, we'll publish the transform from frame odom to frame base_footprint over tf
            # Note that sendTransform requires that 'to' is passed in before 'from' while
            # the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
            self._OdometryTransformBroadcaster.sendTransform(
            (x, y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rosNow,
            "base_footprint",
            "odom")


            # next, we'll publish the odometry message over ROS
            odometry = Odometry()
            odometry.header.frame_id = "odom"
            odometry.header.stamp = rosNow
            odometry.pose.pose.position.x = x
            odometry.pose.pose.position.y = y
            odometry.pose.pose.position.z = 0
            odometry.pose.pose.orientation = quaternion

            odometry.child_frame_id = "base_footprint"
            odometry.twist.twist.linear.x = vx
            odometry.twist.twist.linear.y = 0
            odometry.twist.twist.angular.z = omega
            self._OdometryPublisher.publish(odometry)
            # rospy.loginfo(odometry)

        except:
            rospy.logwarn("Unexpected error odomfrom arduino.py   :" + str(sys.exc_info()[0]))

    def _BroadcastBatteryInfo(self, lineParts):
        partsCount = len(lineParts)
        # rospy.logwarn(partsCount)
        if (partsCount < 1):
            pass
        try:
            self.batteryVoltage = float(lineParts[1])
            self._BatteryStatePublisher.publish(self.batteryVoltage)
            rospy.loginfo(self.batteryVoltage)
        except:
            rospy.logwarn("Unexpected error battery:" + str(sys.exc_info()[0]))

    def _BroadcastAuto(self, lineParts):
        partsCount = len(lineParts)
        # rospy.logwarn(partsCount)
        if partsCount < 1:
            pass
        try:
            self.auto_left = int(lineParts[1])
            self.auto_right = int(lineParts[2])
            self.auto_bumper = int(lineParts[3])
            self.docked = abs(self.auto_bumper - 1)
            self.charged = round((self.docked * self.batteryVoltage), 2)
            self._dock_state_Publisher.publish(self.charged)
        except:
            rospy.logwarn("Unexpected error auto dock:" + str(sys.exc_info()[0]))

    def _WriteSerial(self, message):
        self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
        self._SerialDataGateway.Write(message)

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        '''
        Initializes the receiver class. 
        port: The serial port to listen to.
        baudrate: Baud rate for the serial communication
        '''
        self._Counter = 0

        rospy.init_node('driver_base')

        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baudRate = int(rospy.get_param("~baudRate", 115200))

        rospy.logwarn("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
        rospy.Subscriber("auto_dock", String, self._AutoDock)
        
        
        #group arm 
        rospy.Subscriber('left_joints',Left_joint, self.Left_arm)
        rospy.Subscriber('right_joints',Right_joint, self.Right_arm)
        # A service to manually start Auto dock
        rospy.Service('start_auto_dock', AutoDock, self.Start_Auto_Dock)
        self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
        self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=5)
        self._SerialPublisher = rospy.Publisher('serial', String, queue_size=5)
        self._BatteryStatePublisher = rospy.Publisher("battery", Float32, queue_size=5)
        self._dock_state_Publisher = rospy.Publisher("charge_state", Float32, queue_size=5)
        self._SerialDataGateway = SerialDataGateway(port, baudRate, self._HandleReceivedLine)

    def Start(self):
        rospy.logdebug("Starting")
        self._SerialDataGateway.Start()

    def Stop(self):
        rospy.logdebug("Stopping")
        self._SerialDataGateway.Stop()

    def _HandleVelocityCommand(self, twistCommand):
        #rospy.logwarn("recieved cmd")
        """ Handle movement requests. """
        x = twistCommand.linear.x  # m/s
        if x > 0.3:
            x = 0.3
        th = twistCommand.angular.z  # rad/s
        rospy.logwarn(th)
        if x == 0:
            # Turn in place
            right = th * 0.43 / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * 0.43 / 2.0
            right = x + th * 0.43 / 2.0
        v_des_left = int(left * 16065 / 30)  #
        v_des_right = int(right * 16028 / 30)  # ticks_per_meter 15915
        
        message = 'm %d %d\r' % (v_des_left, v_des_right)
        rospy.logwarn("Sending speed command message: " + message)
        self._SerialDataGateway.Write(message)
        #rospy.logwarn( v_des_left)

    def _AutoDock(self, data):
        # defunct remove after test
        pass

    def Start_Auto_Dock(self, data):
        '''
         start auto dock and charging sequance on the arduino
         it will release when full
        '''
        rospy.logwarn("start auto dock service ")
        return AutoDockResponse()

    def Right_arm(self, data):
        j0 = data.j0
        j1 = data.j1
        j2 = data.j2
        j3 = data.j3
        j4 = data.j4
        j5 = data.j5
        j6 = data.j6
        #j7 = data.j7


        message = 'r %d %d %d %d %d %d %d\r' % (j0, j1, j2, j3, j4, j5, j6)
        rospy.logwarn("Sending speed command message: " + message)
        self._SerialDataGateway.Write(message)

    def Left_arm(self, data):
        j0 = data.j0
        j1 = data.j1
        j2 = data.j2
        j3 = data.j3
        j4 = data.j4
        j5 = data.j5
        

        message = 'l %d %d %d %d %d %d\r' % (j0, j1, j2, j3, j4, j5)
        rospy.logwarn("Sending speed command message: " + message)
        self._SerialDataGateway.Write(message)



if __name__ == '__main__':
    arduino = Arduino()
    try:
        arduino.Start()
        rospy.spin()

    except rospy.ROSInterruptException:
        arduino.Stop()


