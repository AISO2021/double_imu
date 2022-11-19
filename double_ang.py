#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import time
import collections
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
import message_filters

fps = 100.
delay = 1/fps*0.5
autoware_ang = 0
imu_ang = []


class Autoware:
    def __init__(self):
        self.twist = {}
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0
        self.ang_imu = []
        rospy.init_node('Ang')  # , log_level=rospy.DEBUG
        rospy.on_shutdown(self.__on_shutdown)
        self.sub1 = message_filters.Subscriber('/twist_cmd', TwistStamped)
        self.sub2 = message_filters.Subscriber('/imu/data_raw', Imu)
        #self.subscriber = rospy.Subscriber('/twist_cmd', TwistStamped, self.__callback)
        ts = message_filters.ApproximateTimeSynchronizer([self.sub1,self.sub2], 10, delay)
        ts.registerCallback(self.callback)

    def callback(self, raw, data_raw):
        twist = {"speed": raw.twist.linear.x, "ang": raw.twist.angular.z}  # speed: m/s, angular: radian/s
        # angular: 右カーブ -> マイナス
        #          左カーブ -> プラス
        self.ang_imu = [data_raw.angular_velocity.x, data_raw.angular_velocity.y, data_raw.angular_velocity.z]
        rospy.logdebug("Autoware > %s" % self.twist)
        self.ang = twist["ang"]
        self.speed = twist["speed"]
        #power = culc_power(self.speed, self.ang)
        #self.Rpower = power[0][0]
        #self.Lpower = power[1][0]

    def __on_shutdown(self):
        rospy.loginfo("shutdown!")
        #self.subscriber.unregister()

    def get_twist(self):
	#print(self.speed, self.ang, self.Rpower, self.Lpower)
        return self.speed, self.ang, self.Rpower, self.Lpower

    def get_ang(self):
        return self.ang


    def get_ang_imu(self):
        if len(self.ang_imu) == 3:
            return self.ang_imu[2]
        else:
            print(len(self.ang_imu))
            return 0


if __name__ == '__main__':
    try:
        a = Autoware()

        while not rospy.is_shutdown():  # rospy.spin()と同じ
            rospy.sleep(0.1)
            autoware_ang = a.get_ang()
            imu_ang = a.get_ang_imu()

            print('imu', imu_ang, 'autoware', autoware_ang)

    except rospy.ROSInterruptException:
        pass