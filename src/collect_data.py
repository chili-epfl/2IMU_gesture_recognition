#!/usr/bin/env python

import csv
import json
import math
import os
import random
import sys
import time
from time import sleep
import numpy as np

import message_filters
import numpy as np
import pandas as pd
import PyKDL as kdl

import rospy
from geometry_msgs.msg import (Point, Quaternion, QuaternionStamped, Vector3,
                               Vector3Stamped)
#remove or add the message type
from std_msgs.msg import Bool, Float32, Int32, String

from pyxhook import *
from ros_cellulo.msg import cellulo_visual_effect


class CollectData():

    def __init__(self, robot_MAC, imu_MAC):

        self.g = 9.8196 # Gravity

        self.address_imu_right = rospy.get_param('~address_imu_r').replace(':', "_").upper()
        self.address_imu_left = rospy.get_param('~address_imu_l').replace(':', "_").upper()
        self.gesture_type = rospy.get_param('~gesture', default='Test')

        self.calibrate_sensor = rospy.get_param('~calibrate', default= False)

        self.session_name = rospy.get_param('~session', default='Default_session')

        # Create the different directories for the gestures and the calibration values
        self.dir_name = "/home/lburget/Documents/EPFL/Master/PDS2/Lucas/Gesture" 
        self.dir_name_data = self.dir_name +  "/Data2/" +self.gesture_type
        self.file_name_calibration  = self.dir_name + '/Calibration/' + self.session_name + '.json'

        # Create target Directory if don't exist
        if not os.path.exists(self.dir_name_data):
            os.mkdir(self.dir_name_data)
            print("Directory " , self.dir_name_data ,  " Created ")
        else:    
            print("Directory " , self.dir_name_data ,  " already exists")


        self.rot_right_sub = message_filters.Subscriber("/metawear_ros_"+self.address_imu_right+"/rotation", QuaternionStamped)
        self.rot_left_sub = message_filters.Subscriber("/metawear_ros_"+self.address_imu_left+"/rotation", QuaternionStamped)

        self.accel_right_sub = message_filters.Subscriber("/metawear_ros_"+self.address_imu_right+"/accel", Vector3Stamped)
        self.accel_left_sub = message_filters.Subscriber("/metawear_ros_"+self.address_imu_left+"/accel", Vector3Stamped)

        self.gyro_right_sub = message_filters.Subscriber("/metawear_ros_"+self.address_imu_right+"/gyro", Vector3Stamped)
        self.gyro_left_sub = message_filters.Subscriber("/metawear_ros_"+self.address_imu_left+"/gyro", Vector3Stamped)

        # Create a message filter used to synchronize all the data coming from the different topics
        collect = message_filters.ApproximateTimeSynchronizer([self.rot_right_sub, self.rot_left_sub, self.accel_right_sub, self.accel_left_sub, self.gyro_right_sub, self.gyro_left_sub],\
            queue_size = 10, slop = 0.1)

        collect.registerCallback(self.gather_data_cb)

        # Topic used to monitor the collecting frequency
        self.hz_out_pub = rospy.Publisher(rospy.get_name() + "/f_collect", Bool,  queue_size=10)

        self.collect_data = False   
        self.ready_to_write = False
        self.data = []

        # Used for the keyboard and mouse detection
        self.hm = HookManager()
        self.hm.HookMouse()
        self.hm.HookKeyboard()
        self.hm.MouseAllButtonsDown = self.mouse_callback
        self.hm.KeyDown = self.keyboard_callback
        self.hm.start()

        # All the boolean used to manage the interraction
        self.clicked = False
        self.collect_data = False
        self.display = True
        self.first_click = False
        self.stop = False

        # Variables used to calibrate
        self.size_windows = 200
        self.yaw_windows_right = []
        self.yaw_windows_left = []
        self.collect_data_calibration = False
        self.calibration_finished = False

        # Corrected rotation matrices
        self.matrix_rotation_left = kdl.Rotation.RPY(0.0, 0.0, 0.0)
        self.matrix_rotation_right = kdl.Rotation.RPY(0.0, 0.0, 0.0)

        # Value used to smooth the quaternion output
        self.prev_d_rot_l = None
        self.prev_d_rot_r = None


    def gather_data_cb(self, d_rot_r, d_rot_l, d_accel_r, d_accel_l, d_gyro_r, d_gyro_l):
        
        # Two different mode: calibration or recording
        if self.collect_data:       
            
            # Quaternion corrected and the stainilized
            quat_r = np.array(( self.matrix_rotation_right * kdl.Rotation.Quaternion(d_rot_r.quaternion.x, d_rot_r.quaternion.y, d_rot_r.quaternion.z, d_rot_r.quaternion.w)\
                  * kdl.Rotation.RPY(0.0, 0.0, np.pi/2)).GetQuaternion())

            quat_l = np.array((self.matrix_rotation_left * kdl.Rotation.Quaternion(d_rot_l.quaternion.x, d_rot_l.quaternion.y, d_rot_l.quaternion.z, d_rot_l.quaternion.w)\
                *  kdl.Rotation.RPY(0.0, 0.0, np.pi/2)).GetQuaternion())

            quat_r = self.stabilize(quat_r, self.prev_d_rot_r)
            quat_l = self.stabilize(quat_l, self.prev_d_rot_l)
            self.prev_d_rot_l = quat_l
            self.prev_d_rot_r = quat_r

            # Current orientation of the IMU
            matrix_rotation_right_current = kdl.Rotation.Quaternion(quat_r[0], quat_r[1], quat_r[2], quat_r[3])
            matrix_rotation_left_current =  kdl.Rotation.Quaternion(quat_l[0], quat_l[1], quat_l[2], quat_l[3])

            # Substract gravity and project acceleration on corrected axis
            lin_accel_r = (matrix_rotation_right_current* kdl.Vector(d_accel_r.vector.x, d_accel_r.vector.y, d_accel_r.vector.z))-kdl.Vector(0.0,0.0,1.0)
            lin_accel_l = (matrix_rotation_left_current* kdl.Vector(d_accel_l.vector.x, d_accel_l.vector.y, d_accel_l.vector.z))-kdl.Vector(0.0,0.0,1.0)

            prov = [d_rot_r.header.stamp]
            prov.extend(quat_r.tolist())
            prov.extend(quat_l.tolist())
            prov.extend(list(lin_accel_r))
            prov.extend(list(lin_accel_l))
            prov.extend(list((matrix_rotation_right_current* kdl.Vector(d_gyro_r.vector.x, d_gyro_r.vector.y, d_gyro_r.vector.z))))
            prov.extend(list((matrix_rotation_left_current* kdl.Vector(d_gyro_l.vector.x, d_gyro_l.vector.y, d_gyro_l.vector.z))))

            self.data.append(prov)


        if self.collect_data_calibration:
            # Get yaw for right and left in order to align the axis
            if len(self.yaw_windows_right)>self.size_windows : 
                self.calibration_finished = True
            else : 

                self.rotation_right = kdl.Rotation.Quaternion(d_rot_r.quaternion.x, d_rot_r.quaternion.y, d_rot_r.quaternion.z, d_rot_r.quaternion.w)* self.matrix_rotation_right
                self.rotation_left = kdl.Rotation.Quaternion(d_rot_l.quaternion.x, d_rot_l.quaternion.y, d_rot_l.quaternion.z, d_rot_l.quaternion.w)* self.matrix_rotation_left
                _, _, yaw_right = self.rotation_right.GetRPY()
                _, _, yaw_left  = self.rotation_left.GetRPY()

                self.yaw_windows_right.append(yaw_right)   
                self.yaw_windows_left.append(yaw_left)    

        # Publish to monitor recording frequency
        b = Bool()
        self.hz_out_pub.publish(b)
     
    def stabilize(self, q, q_prev):
        '''
        Since two opposites quaternion represent the same orientation, the closest to the previous one is choosen
        '''
        if q_prev is None:
            rval = q
        else:
            d1 = np.linalg.norm(q_prev-q)
            d2 = np.linalg.norm(q_prev+q)
            if (d1<d2):
                rval = q
            else:
                rval = -q
        return rval

    def mouse_callback(self, event):
        '''
        Called when an action with the mouse is done. Updates the boolean.
        '''
        if 'mouse left' in event.MessageName:
            
            if not self.first_click:
                self.time_start = time.time()
                self.first_click = True
            else:
                if time.time()-self.time_start < 1:
                    self.clicked = not self.clicked
                    self.first_click = False
                else : 
                    self.time_start = time.time()
                    self.first_click = True

    def keyboard_callback(self, event):
        '''
        Called when an action with the keyboard is done. Updates the boolean.
        '''
        if event.Ascii == 32:
            print("Stop programm")
            self.stop = True

        if event.Ascii == 13:
            self.clicked = not self.clicked


    def calibrate(self):
        '''
        Manage the calibration part
        '''
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            
            if not self.collect_data_calibration:
                if self.display:
                   print("Double click or press enter  to start the calibration, and go to initial position")
                   self.display = not self.display

                if self.clicked :
                    rospy.sleep(1)
                    print("Start recording calibration")
                    self.collect_data_calibration = True
                    self.display = True
                    self.clicked = False
                    rospy.sleep(1)

            if self.calibration_finished:
                self.calibration_values = dict()
                self.calibration_values['yaw_offset_right'] = sum(self.yaw_windows_right)/len(self.yaw_windows_right)
                self.calibration_values['yaw_offset_left'] = sum(self.yaw_windows_left)/len(self.yaw_windows_left)

                with open(self.file_name_calibration, 'w+') as jsonFile:
                    json.dump(self.calibration_values, jsonFile)

                print('calibration finished')
                self.clicked = False
                break

            r.sleep()



    def run_data_acquisition(self):
        '''
        Manage the recording part
        '''

        r = rospy.Rate(100) #100hz

        # Update of the rotation matrix
        self.matrix_rotation_left   = kdl.Rotation.RPY(0.0, 0.0, -self.calibration_values['yaw_offset_left']) 
        self.matrix_rotation_right  = kdl.Rotation.RPY(0.0, 0.0, -self.calibration_values['yaw_offset_right'])

        while not rospy.is_shutdown():

            if not self.collect_data:
                if self.display:
                   print("Double click or press enter  to start recording, and go to initial position")
                   self.display = not self.display

                if self.clicked : 
                    print("You can move")
                    self.collect_data = True
                    self.display = True
                    self.clicked = False
                    rospy.sleep(1)

            else:
                if self.display:
                   print("Double click or press enter to stop recording")
                   self.display = not self.display

                if self.clicked: 
                    print('Here') 
                    self.collect_data = False
                    self.display = True
                    self.clicked = False
                    self.ready_to_write = True


            if self.ready_to_write:

                # Check the maximum id in the directory 
                onlyfiles = [f for f in os.listdir(self.dir_name_data) if os.path.isfile(os.path.join(self.dir_name_data, f))]
                max_id = max([int(file.split('.')[0]) for file in onlyfiles]) if onlyfiles else 0 

                name_file_write = self.dir_name_data + '/'+ str(max_id+1) + '.csv'
                print("Write")

                # Write the data to the csv file.
                print(name_file_write)
                with open(name_file_write, 'w+') as csvFile:
                    writer = csv.writer(csvFile)
                    writer.writerow(['rospy_time','rot_right_x', 'rot_right_y','rot_right_z', 'rot_right_w',
                    'rot_left_x', 'rot_left_y', 'rot_left_z','rot_left_w',
                    'accel_right_x', 'accel_right_y', 'accel_right_z',
                    'accel_left_x', 'accel_left_y', 'accel_left_z',
                    'gyro_right_x', 'gyro_right_y', 'gyro_right_z',
                    'gyro_left_x', 'gyro_left_y','gyro_left_z' ])
                    writer.writerows(self.data)

                csvFile.close()
                del self.data[:]
                self.ready_to_write = False
            r.sleep()


    def read_calibration_data(self):
        print(self.file_name_calibration)
        with open(self.file_name_calibration, 'r') as jsonFile:
            self.calibration_values = json.load(jsonFile)
                    
    def run(self):
        
        print(self.calibrate_sensor)
        if self.calibrate_sensor:
            self.calibrate()
        else : 
            self.read_calibration_data()
        print('The yaw offset found from the file is: left {} right {} '.format(self.calibration_values['yaw_offset_left'], self.calibration_values['yaw_offset_right']))

        self.run_data_acquisition()

        print('Right', self.matrix_rotation_right)
        print('Left', self.matrix_rotation_left)

        self.hm.cancel()
        
if __name__=='__main__':

    rospy.init_node('collect_data')

    imu_MAC = 'E5:4D:16:18:CD:90'
    robot_MAC = 'D8_14_50_A0_D4_87'
    cc = CollectData(robot_MAC, imu_MAC)
    cc.run()
