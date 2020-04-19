#! /usr/bin/env python

import os
import sys
import rospy
import numpy as np
from plat_msg.msg       import PlatMsgVehicleCmd

dirpath = os.path.dirname(os.path.abspath(__file__))
importdir = dirpath + '/cfg'
sys.path.append(importdir)

from cfg import parse_args, VehicleCfg, BrakeVehicle, ThrottleVehicle, SteeringVehicle, GearVehicle, TurnSignalVehicle


g_vel_cmd           = 0.0
g_swa_cmd           = 0.0
g_acc_cmd           = 0.0
g_sar_cmd           = 0.0
g_cfg_type_cmd      = 0.0
g_enable_cmd        = 0.0
g_clear_cmd         = 0.0
g_vel_pre           = 0.0
g_vel_err_pre_int   = 0.0
g_vel_err_pre       = 0.0
g_throttle_pre      = 0.0
g_published         = False

def control_callback(control_cmd):
    global g_vel_cmd, g_swa_cmd, g_acc_cmd, g_sar_cmd, g_cfg_type_cmd, g_enable_cmd, g_clear_cmd, g_published
    g_vel_cmd       = control_cmd.vel_cmd
    g_swa_cmd       = control_cmd.swa_cmd
    g_acc_cmd       = control_cmd.acc_cmd
    g_sar_cmd       = control_cmd.sar_cmd
    g_cfg_type_cmd  = control_cmd.cfg_type_cmd
    g_enable_cmd    = control_cmd.enable_cmd
    g_clear_cmd     = control_cmd.clear_cmd
    g_published     = True

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

def lot_control(vel_req):
    global g_vel_pre, g_vel_err_pre_int, g_vel_err_pre, g_throttle_pre
    Kp = 1.0
    Ki = 0.1
    Kd = 0.01
    sample_t = 0.1
    if vel_req < 0.0:
        vel_req = 0.0
    vel_des     = vel_req
    vel_err_cur = vel_des - g_vel_pre
    vel_err_int = g_vel_err_pre_int + vel_err_cur * sample_t
    vel_err_dif = (vel_err_cur - g_vel_err_pre) / sample_t
    des_acc     = Kp * (vel_err_cur) + Ki * (vel_err_int) + Kd * (vel_err_dif)
    throttle = 0.0
    brake = 0.0
    if des_acc > 0:
        brake = 0.0
        throttle = (np.tanh(des_acc) + 1) / 2
        if (throttle - g_throttle_pre) > 0.1:
            throttle = g_throttle_pre + 0.1
    g_vel_pre         = vel_des
    g_vel_err_pre     = vel_err_cur
    g_vel_err_pre_int = vel_err_int
    g_throttle_pre    = throttle
    return throttle, brake

def lat_control(swa_req):
    steering = 0.0
    if swa_req > 1.0:
        swa_req = 1.0
    elif swa_req < -1.0:
        swa_req = -1.0
    swa_req *= 9.6
    steering = swa_req
    return steering

def exec_low_level_control(cfg_type_cmd, vel_cmd, swa_cmd, acc_cmd, sar_cmd):
    brake_req       = 0.0
    throttle_req    = 0.0
    steering_req    = 0.0
    if cfg_type_cmd == 1:
        throttle_req, brake_req = lot_control(vel_cmd)
    elif cfg_type_cmd == 2:
        steering_req = lat_control(swa_cmd)
    elif cfg_type_cmd == 3:
        throttle_req, brake_req = lot_control(vel_cmd)
        steering_req = lat_control(swa_cmd)
    return brake_req, throttle_req, steering_req

def fill_pub_msgs(cmd_type, b_obj, t_obj, s_obj, brake, throt, steer):
    if cmd_type == 1:
        b_obj.set_msg(brake, b_obj._CMD_PERCENT, True)
        t_obj.set_msg(throt, t_obj._CMD_PERCENT, True)
        s_obj.set_msg(0.0, 0.0, 0.0, s_obj._CMD_ANGLE, True)
    elif cmd_type == 2:
        b_obj.set_msg(0.0, b_obj._CMD_PERCENT, True)
        t_obj.set_msg(0.0, t_obj._CMD_PERCENT, True)
        s_obj.set_msg(steer, 0.0, 0.0, s_obj._CMD_ANGLE, True)
    elif cmd_type == 3:
        b_obj.set_msg(brake, b_obj._CMD_PERCENT, True)
        t_obj.set_msg(throt, t_obj._CMD_PERCENT, True)
        s_obj.set_msg(steer, 0.0, 0.0, s_obj._CMD_ANGLE, True)
    else:
        b_obj.set_msg(0.0, b_obj._CMD_NONE, False)
        t_obj.set_msg(0.0, t_obj._CMD_NONE, False)
        s_obj.set_msg(0.0, 0.0, 0.0, s_obj._CMD_ANGLE, False)
    b_obj.exec_pub()
    t_obj.exec_pub()
    s_obj.exec_pub()

def main():
    global g_vel_cmd, g_swa_cmd, g_acc_cmd, g_sar_cmd, g_cfg_type_cmd, g_enable_cmd, g_clear_cmd, g_vel_pre, g_vel_err_pre_int, g_vel_err_pre, g_throttle_pre, g_published
    g_vel_cmd           = 0.0
    g_swa_cmd           = 0.0
    g_acc_cmd           = 0.0
    g_sar_cmd           = 0.0
    g_cfg_type_cmd      = 0
    g_enable_cmd        = 0
    g_clear_cmd         = 0
    g_vel_pre           = 0.0
    g_vel_err_pre       = 0.0
    g_vel_err_pre_int   = 0.0
    g_throttle_pre      = 0.0
    g_published         = False
    vehicle_ns = parse_args()
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
        if g_published:
            cfg_type_cmd_cur    = g_cfg_type_cmd
            vel_cmd_cur, swa_cmd_cur, acc_cmd_cur, sar_cmd_cur = set_control_cfg(cfg_type_cmd_cur)
            brake_req_cur, throttle_req_cur, steering_req_cur = exec_low_level_control(cfg_type_cmd_cur, vel_cmd_cur, swa_cmd_cur, acc_cmd_cur, sar_cmd_cur)
            fill_pub_msgs(cfg_type_cmd_cur, brake_obj, throttle_obj, steering_obj, brake_req_cur, throttle_req_cur, steering_req_cur)
        g_published = False
        rate.sleep()

if __name__ == '__main__':
    main()
