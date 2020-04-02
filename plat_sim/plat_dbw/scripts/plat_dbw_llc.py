#! /usr/bin/env python

import rospy
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd

VEHICLES_NS_LIST = ['novehicle', 'vehicle', 'leadvehicle', 'stringvehicle1', 'stringvehicle2', 'mergevehicle']

class VehicleCfg:
    def __init__(self, ns):
        self._ns                    = ns
        self._node_name             = ns + '_llc'
        self._out_topic_name        = '/' + ns + '/odom'
        self._brake_topic_name      = '/' + ns + '/brake_cmd'
        self._throttle_topic_name   = '/' + ns + '/throttle_cmd'
        self._steering_topic_name   = '/' + ns + '/steering_cmd'

    def get_node_properties(self):
        return self._node_name, self._brake_topic_name, self._throttle_topic_name, self._steering_topic_name