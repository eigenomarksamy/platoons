#! /usr/bin/python

import sys
import rospy
from nav_msgs.msg import Odometry

VEHICLES_NS_LIST = ['novehicle', 'vehicle', 'leadvehicle', 'stringvehicle1', 'stringvehicle2', 'mergevehicle', 'obsvehicle']

class VehicleCfg:
    def __init__(self, ns):
        self._ns                    = ns
        self._odom_node_name        = self._ns + '_odom'
        self._llc_node_name         = self._ns + '_llc'
        self._ptracker_node_name    = self._ns + '_ptracker'
        self._gazebo_model_name     = self._ns
        self._gazebo_frame_id       = '/' + self._ns + '/base_link'
        self._odom_topic_name       = '/' + self._ns + '/odom'
        self._control_topic_name    = '/' + self._ns + '/control'
        self._brake_topic_name      = '/' + self._ns + '/brake_cmd'
        self._throttle_topic_name   = '/' + self._ns + '/throttle_cmd'
        self._steering_topic_name   = '/' + self._ns + '/steering_cmd'
        self._gear_topic_name       = '/' + self._ns + '/gear_cmd'
        self._turnsignal_topic_name = '/' + self._ns + '/turn_signal_cmd'
        self._cmdvel_topic_name     = '/' + self._ns + '/cmd_vel'

    def get_llc_properties(self):
        return self._llc_node_name, self._brake_topic_name, self._throttle_topic_name, self._steering_topic_name, self._gear_topic_name, self._turnsignal_topic_name, self._cmdvel_topic_name

    def get_odom_properties(self):
        return self._odom_node_name, self._odom_topic_name, self._gazebo_model_name, self._gazebo_frame_id
    
    def get_ptracker_properties(self):
        return self._ptracker_node_name, self._control_topic_name

    def get_control_properties(self):
        return self._control_topic_name

    def get_spawn_pose(self):
        odom = rospy.wait_for_message(self._odom_topic_name, Odometry)
        return (odom.pose.pose.position.x, odom.pose.pose.position.y)

def parse_args():
    default_ns = 'vehicle'
    args_list = list(sys.argv)
    if len(args_list) <= 1:
        vehicle_ns = default_ns
    else:
        args_list.pop(0)
        vehicle_ns = args_list[0]
        if vehicle_ns not in VEHICLES_NS_LIST:
            vehicle_ns = 'novehicle'
    return vehicle_ns