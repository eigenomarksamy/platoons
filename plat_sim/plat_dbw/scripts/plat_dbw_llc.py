#! /usr/bin/env python

import os
import sys
import rospy
from plat_msg.msg       import PlatMsgVehicleCmd

dirpath = os.path.dirname(os.path.abspath(__file__))
importdir = dirpath + '/cfg'
sys.path.append(importdir)

import cfg
from cfg import VehicleCfg
from cfg import BrakeVehicle
from cfg import ThrottleVehicle
from cfg import SteeringVehicle
from cfg import GearVehicle
from cfg import TurnSignalVehicle

def main():
    vehicle_ns = cfg.parse_args()
    ns_obj = VehicleCfg(vehicle_ns)
    node_name, brake_topic_name, throttle_topic_name, steering_topic_name, gear_topic_name, turnsignal_topic_name, _ = ns_obj.get_llc_properties()
    brake_obj = BrakeVehicle()
    throttle_obj = ThrottleVehicle()
    steering_obj = SteeringVehicle()
    gear_obj = GearVehicle()
    turnsignal_obj = TurnSignalVehicle()
    rospy.init_node(node_name, anonymous=True)
    brake_obj.set_pub(topic_name=brake_topic_name)
    throttle_obj.set_pub(topic_name=throttle_topic_name)
    steering_obj.set_pub(topic_name=steering_topic_name)
    gear_obj.set_pub(topic_name=gear_topic_name)
    turnsignal_obj.set_pub(topic_name=turnsignal_topic_name)

if __name__ == '__main__':
    main()