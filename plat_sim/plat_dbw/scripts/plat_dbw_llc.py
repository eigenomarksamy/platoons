#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg       import Float64, Float64MultiArray
from dbw_mkz_msgs.msg   import ThrottleCmd, SteeringCmd, BrakeCmd, GearCmd, TurnSignalCmd, TwistCmd

VEHICLES_NS_LIST = ['novehicle', 'vehicle', 'leadvehicle', 'stringvehicle1', 'stringvehicle2', 'mergevehicle', 'obsvehicle']

class VehicleCfg:
    def __init__(self, ns):
        self._ns                    = ns
        self._llc_node_name         = ns + '_llc'
        self._brake_topic_name      = '/' + ns + '/brake_cmd'
        self._throttle_topic_name   = '/' + ns + '/throttle_cmd'
        self._steering_topic_name   = '/' + ns + '/steering_cmd'
        self._gear_topic_name       = '/' + ns + '/gear_cmd'
        self._turnsignal_topic_name = '/' + ns + '/turn_signal_cmd'
        self._cmdvel_topic_name     = '/' + ns + '/cmd_vel'

    def get_node_properties(self):
        return self._llc_node_name, self._brake_topic_name, self._throttle_topic_name, self._steering_topic_name

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