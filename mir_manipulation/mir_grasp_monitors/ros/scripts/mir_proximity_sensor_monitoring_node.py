#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
import serial
import numpy as np
import time
import re
from serial.serialutil import SerialException
class YoubotGripperGraspMonitor:
    def __init__(self):

        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        self.result_pub = rospy.Publisher("/result", String, queue_size = 10)
        self.sen_reading = Int32()
        self.has_value = False
        self.event =  None
        self.no_of_sensors = 0
        self.if_serial = False
        self.threshold = None

        rospy.Subscriber("~/event_in", String, self.event_in_callback)
        self.result_pub = rospy.Publisher("/result", String, queue_size = 10)

        try:
            self.serial = serial.Serial('/dev/ttyUSB0', 9600)
            self.if_serial = True
            self.no_of_sensors = self.no_of_sensors + 1
        except SerialException:
            rospy.logerr("Check the sensor connection port in the code.")
        print(self.no_of_sensors )

    def event_in_callback(self, msg):
       print msg.data, " event received."
       self.event = msg.data

    def get_sensor_readings(self, serial):
        now = rospy.get_time()
        got_result = False
        prox_value = None
        while rospy.get_time()-now < 2 and prox_value is None:
            serial.flushInput()
            input = ''
            input += serial.readline()
            while input.find("\r\n") != -1:
                chopped_line = input[:input.find("\r\n")]
                input = input[input.find("\r\n")+2:]
                if chopped_line.find('Proximity') != -1:
                    values = chopped_line.split(' ')
                    assert(len(values)>=2)
                    prox_value = int(values[1])
                    break
        print "Proximity sensor value is: ", prox_value
        if prox_value:
            return prox_value
        return None

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        #print "In init "
        if self.event == 'e_start':
            self.threshold = self.get_sensor_readings(self.serial)
            print "self.threshold is", self.threshold 
            self.event = None
            return 'RUNNING'
        if self.event == 'e_trigger':
            return 'RUNNING'
        else:
            return 'INIT'

    def running_state(self):
        print "Running "
        if self.event == 'e_trigger':
            sensor_reading = self.get_sensor_readings(self.serial)
            print "sensor_reading: " , sensor_reading
            self.event = None
            if self.threshold is not None and sensor_reading is not None:
                if (self.threshold +50) < sensor_reading:
                    result = String()
                    result.data = 'grasped'
                    self.result_pub.publish(result)
                else:
                    result = String()
                    result.data = 'not_grasped'
                    self.result_pub.publish(result)
        return 'INIT'

    def get_sensor_readings(self, serial):
        print "get_sensor_readings "        
        now = rospy.get_time()
        got_result = False
        prox_value = None
        while rospy.get_time()-now < 3 and prox_value is None:
            serial.flushInput()
            input = ''
            input += serial.readline()
            while input.find("\r\n") != -1:
                chopped_line = input[:input.find("\r\n")]
                input = input[input.find("\r\n")+2:]
                if chopped_line.find('Proximity') != -1:
                    values = chopped_line.split(' ')
                    assert(len(values)>=2)
                    prox_value = int(values[1])
                    break
        if prox_value:
            #print "prox_value", prox_value
            return prox_value
        return None

    def pub_sensor_readings(self):
        if self.if_serial1:
            val = self.get_sensor_readings(self.serial1)
            if val:
                self.sen_reading1.data = int(val)
                self.sensor1_readings_pub.publish(self.sen_reading1)
                self.has_value1 = True
            else:
                self.has_value1 = False
        else:
            self.has_value1 = False
        if self.if_serial2:
            val = self.get_sensor_readings(self.serial2)
            if val:
                self.sen_reading2.data = int(val)
                self.sensor2_readings_pub.publish(self.sen_reading2)
                self.has_value2 = True

    def get_recent_reading(self):
        rospy.loginfo("get_recent_reading started")
        self.event == None
        #self.serial.flushInput()
        #input = ''
        #input += self.serial.readline()
        now = rospy.get_time()
        got_result = False
        while rospy.get_time()-now < 3 and not got_result:
            self.serial.flushInput()
            input = ''
            input += self.serial.readline()
            while input.find("\r\n") != -1:
                chopped_line = input[:input.find("\r\n")]
                input = input[input.find("\r\n")+2:]
                if chopped_line.find('Proximity') != -1:
                    values = chopped_line.split(' ')
                    assert(len(values)>=2)
                    result = String()
                    if int(values[1])>self.threshold:
                        result.data = 'grasped'
                    else:
                        result.data = 'not_grasped'
                    got_result = True
                    self.result_pub.publish(result)
                    print values[1]
                    break


if __name__ == "__main__":
    rospy.init_node("proximity_sensor_monitor")
    rospy.loginfo("proximity_sensor_monitor started")
    gripper_state = YoubotGripperGraspMonitor()
    gripper_state.start()
    rospy.spin()

