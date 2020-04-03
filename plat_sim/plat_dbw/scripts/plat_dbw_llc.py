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

def control_callback(control_cmd):
    global g_vel_cmd, g_swa_cmd, g_acc_cmd, g_sar_cmd, g_cfg_type_cmd, g_enable_cmd, g_clear_cmd
    g_vel_cmd       = control_cmd.vel_cmd
    g_swa_cmd       = control_cmd.swa_cmd
    g_acc_cmd       = control_cmd.acc_cmd
    g_sar_cmd       = control_cmd.sar_cmd
    g_cfg_type_cmd  = control_cmd.cfg_type_cmd
    g_enable_cmd    = control_cmd.enable_cmd
    g_clear_cmd     = control_cmd.clear_cmd

def set_control_cfg(cfg_type_cmd_cur):
    global g_vel_cmd, g_swa_cmd, g_acc_cmd, g_sar_cmd
    if cfg_type_cmd_cur == 0:       # CMD_NONE
        cmd_vel_cur = 0.0
        cmd_swa_cur = 0.0
        cmd_acc_cur = 0.0
        cmd_sar_cur = 0.0
    elif cfg_type_cmd_cur == 1:     # CMD_VEL
        cmd_vel_cur = g_vel_cmd
        cmd_swa_cur = 0.0
        cmd_acc_cur = 0.0
        cmd_sar_cur = 0.0
    elif cfg_type_cmd_cur == 2:     # CMD_SWA
        cmd_vel_cur = 0.0
        cmd_swa_cur = g_swa_cmd
        cmd_acc_cur = 0.0
        cmd_sar_cur = 0.0
    elif cfg_type_cmd_cur == 3:     # CMD_VEL_SWA
        cmd_vel_cur = g_vel_cmd
        cmd_swa_cur = g_swa_cmd
        cmd_acc_cur = 0.0
        cmd_sar_cur = 0.0
    elif cfg_type_cmd_cur == 4:     # CMD_ACC
        cmd_vel_cur = 0.0
        cmd_swa_cur = 0.0
        cmd_acc_cur = g_acc_cmd
        cmd_sar_cur = 0.0
    elif cfg_type_cmd_cur == 5:     # CMD_VEL_ACC
        cmd_vel_cur = g_vel_cmd
        cmd_swa_cur = 0.0
        cmd_acc_cur = g_acc_cmd
        cmd_sar_cur = 0.0
    elif cfg_type_cmd_cur == 6:     # CMD_ACC_SWA
        cmd_vel_cur = 0.0
        cmd_swa_cur = g_swa_cmd
        cmd_acc_cur = g_acc_cmd
        cmd_sar_cur = 0.0
    elif cfg_type_cmd_cur == 7:     # CMD_VEL_SWA_ACC
        cmd_vel_cur = g_vel_cmd
        cmd_swa_cur = g_swa_cmd
        cmd_acc_cur = g_acc_cmd
        cmd_sar_cur = 0.0
    elif cfg_type_cmd_cur == 8:     # CMD_SAR
        cmd_vel_cur = 0.0
        cmd_swa_cur = 0.0
        cmd_acc_cur = 0.0
        cmd_sar_cur = g_sar_cmd
    elif cfg_type_cmd_cur == 9:     # CMD_VEL_SAR
        cmd_vel_cur = g_vel_cmd
        cmd_swa_cur = 0.0
        cmd_acc_cur = 0.0
        cmd_sar_cur = g_sar_cmd
    elif cfg_type_cmd_cur == 10:    # CMD_SWA_SAR
        cmd_vel_cur = 0.0
        cmd_swa_cur = g_swa_cmd
        cmd_acc_cur = 0.0
        cmd_sar_cur = g_sar_cmd
    elif cfg_type_cmd_cur == 11:    # CMD_VEL_SWA_SAR
        cmd_vel_cur = g_vel_cmd
        cmd_swa_cur = g_swa_cmd
        cmd_acc_cur = 0.0
        cmd_sar_cur = g_sar_cmd
    elif cfg_type_cmd_cur == 12:    # CMD_ACC_SAR
        cmd_vel_cur = 0.0
        cmd_swa_cur = 0.0
        cmd_acc_cur = g_acc_cmd
        cmd_sar_cur = g_sar_cmd
    elif cfg_type_cmd_cur == 13:    # CMD_VEL_ACC_SAR
        cmd_vel_cur = g_vel_cmd
        cmd_swa_cur = 0.0
        cmd_acc_cur = g_acc_cmd
        cmd_sar_cur = g_sar_cmd
    elif cfg_type_cmd_cur == 14:    # CMD_SWA_ACC_SAR
        cmd_vel_cur = 0.0
        cmd_swa_cur = g_swa_cmd
        cmd_acc_cur = g_acc_cmd
        cmd_sar_cur = g_sar_cmd
    elif cfg_type_cmd_cur == 15:    # CMD_VEL_ACC_SWA_SAR
        cmd_vel_cur = g_vel_cmd
        cmd_swa_cur = g_swa_cmd
        cmd_acc_cur = g_acc_cmd
        cmd_sar_cur = g_sar_cmd
    else:                           # CMD_INVALID
        cmd_vel_cur = 0.0
        cmd_swa_cur = 0.0
        cmd_acc_cur = 0.0
        cmd_sar_cur = 0.0
    return cmd_vel_cur, cmd_swa_cur, cmd_acc_cur, cmd_sar_cur

def main():
    global g_vel_cmd, g_swa_cmd, g_acc_cmd, g_sar_cmd, g_cfg_type_cmd, g_enable_cmd, g_clear_cmd
    g_vel_cmd       = 0.0
    g_swa_cmd       = 0.0
    g_acc_cmd       = 0.0
    g_sar_cmd       = 0.0
    g_cfg_type_cmd  = 0
    g_enable_cmd    = 0
    g_clear_cmd     = 0
    vehicle_ns = cfg.parse_args()
    ns_obj = VehicleCfg(vehicle_ns)
    node_name, brake_topic_name, throttle_topic_name, steering_topic_name, gear_topic_name, turnsignal_topic_name, _ = ns_obj.get_llc_properties()
    control_topic_name = ns_obj.get_control_properties()
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
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber(control_topic_name, PlatMsgVehicleCmd, control_callback)
        cfg_type_cmd_cur    = g_cfg_type_cmd
        enable_cmd_cur      = g_enable_cmd
        clear_cmd_cur       = g_clear_cmd
        vel_cmd_cur, swa_cmd_cur, acc_cmd_cur, sar_cmd_cur = set_control_cfg(cfg_type_cmd_cur)
        vel_cmd_pre         = vel_cmd_cur
        swa_cmd_pre         = swa_cmd_cur
        acc_cmd_pre         = acc_cmd_cur
        sar_cmd_pre         = sar_cmd_cur
        cfg_type_cmd_pre    = cfg_type_cmd_cur
        enable_cmd_pre      = enable_cmd_cur
        clear_cmd_pre       = clear_cmd_cur
        rate.sleep()

if __name__ == '__main__':
    global g_vel_cmd, g_swa_cmd, g_acc_cmd, g_sar_cmd, g_cfg_type_cmd, g_enable_cmd, g_clear_cmd
    main()
