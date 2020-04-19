#! /usr/bin/python

import os
import sys
import math
import rospy

from std_msgs.msg import Int64
from plat_msg.msg import PlatMsgVehicleCmd
from dbw_mkz_msgs.msg import BrakeCmd

dir_path = os.path.dirname(os.path.abspath(__file__))
dir_cont = dir_path + '/controller'

sys.path.append(dir_path)
sys.path.append(dir_cont)

from pid import PID

DISTANCE_SET_POINT = 20
LEADER_SPEED = 100

g_tpr = 0.0
pid_controller = None
velo_follower = LEADER_SPEED
pedal_follower = 0.0
prev_error = 0
error_trend_decreasing = True

def parse_args():
    args_list = list(sys.argv)

    node_name = args_list[2]
    follower = args_list[1]
    leader = None
    lead_vehicle_flag = False
    
    if follower == 'stringvehicle1':
        leader = 'leadvehicle'
    elif follower == 'stringvehicle2':
        leader = 'stringvehicle1'
    
    if leader is None:
        sub_topic_name = 'distance_ahead/stringvehicle1_to_' + follower
        lead_vehicle_flag = True
    else:
        sub_topic_name = 'distance_ahead/' + follower + '_to_' + leader

    pub_topic_name_follower = follower + '/control'
    pub_topic_brake_follower = follower + '/brake_cmd'

    return node_name, sub_topic_name, pub_topic_name_follower, pub_topic_brake_follower, lead_vehicle_flag

def update_velocities(error, ts):
    global pid_controller

    if ts == 0.0:
        ts = 0.01

    return pid_controller.update(error, ts)

def main():
    global g_tpr
    global pid_controller
    global velo_follower
    global pedal_follower
    global prev_error
    global error_trend_decreasing

    node_name, sub_topic_name, pub_topic_name_follower, pub_topic_brake_follower, lead_vehicle_flag = parse_args()
    
    rospy.init_node(node_name, anonymous=True)

    stanely_ctrl_pub_follower = rospy.Publisher(pub_topic_name_follower, PlatMsgVehicleCmd, queue_size=1)
    brake_pub_follower = rospy.Publisher(pub_topic_brake_follower, BrakeCmd, queue_size=1)

    control_msg_follower = PlatMsgVehicleCmd()
    control_msg_follower.cfg_type_cmd = 1
    control_msg_follower.enable_cmd = True

    cmd_follower = BrakeCmd()        
    cmd_follower.pedal_cmd = 100
    cmd_follower.pedal_cmd_type = 2    # percent
    cmd_follower.enable = True

    # tunables 
    brake_gain = 3.3
    delay_margin = 5
    delay_margin_leader = 5

    pid_controller = PID(Kp=8.0, Ki=5.0, Kd=25.0, windupVal=80)

    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        g_ttt = rospy.get_time()
        ts = float(g_ttt - g_tpr)

        dist_ahead = rospy.wait_for_message(sub_topic_name, Int64)
        error = dist_ahead.data - DISTANCE_SET_POINT
        output = update_velocities(abs(error), ts)

        # calc trend
        if error > prev_error:
            error_trend_decreasing = False
        else:
            error_trend_decreasing = True

        # if vehicle is leader vehicle
        if lead_vehicle_flag:
            if (error <= delay_margin_leader) and (error_trend_decreasing == True):  
                velo_follower = output
                pedal_follower = 0
            elif (error >= -1*delay_margin_leader) and (error_trend_decreasing == False):
                velo_follower = 0
                pedal_follower = output * brake_gain
            elif (error <= 0):
                velo_follower = output
                pedal_follower = 0
            else:
                velo_follower = 0
                pedal_follower = output * brake_gain
        else:
            if (error < delay_margin) and (error_trend_decreasing == False):
                velo_follower = 0
                pedal_follower = output * brake_gain
            elif (error >= -1*delay_margin) and (error_trend_decreasing == True):
                velo_follower = output
                pedal_follower = 0
            elif (error < 0):  
                velo_follower = 0
                pedal_follower = output * brake_gain
            else:
                velo_follower = output
                pedal_follower = 0

        print('*************************')
        print('topic    : ' + node_name)
        print('error    : ' + str(error))
        print('velocity : ' + str(velo_follower))
        print('brake    : ' + str(pedal_follower))

        control_msg_follower.vel_cmd = velo_follower
        cmd_follower.pedal_cmd = pedal_follower

        brake_pub_follower.publish(cmd_follower)
        stanely_ctrl_pub_follower.publish(control_msg_follower)

        g_tpr = g_ttt
        prev_error = error
            
        rate.sleep()

if __name__ == '__main__':
    main()