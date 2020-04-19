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

DISTANCE_SET_POINT = 10

g_tpr = 0.0
pid_controller = None
velo_1 = velo_2 = 0

def parse_args():
    args_list = list(sys.argv)

    node_name = args_list[3]
    vehicle1 = args_list[1]
    vehicle2 = args_list[2]
    
    sub_topic_name = 'distance_ahead/' + vehicle2 + '_to_' + vehicle1
    pub_topic_name_1 = vehicle1 + '/control'
    pub_topic_name_2 = vehicle2 + '/control'
    pub_topic_brake_1 = vehicle1 + '/brake_cmd'
    pub_topic_brake_2 = vehicle2 + '/brake_cmd'

    return node_name, sub_topic_name, pub_topic_name_1, pub_topic_name_2, pub_topic_brake_1, pub_topic_brake_2

def update_velocities(error, ts):
    global pid_controller

    if ts == 0.0:
        ts = 0.1

    return pid_controller.update(error, ts)

def main():
    global g_tpr
    global pid_controller
    global velo_1
    global velo_2

    # parse topic names and node name
    node_name, sub_topic_name, pub_topic_name_1, pub_topic_name_2, pub_topic_brake_1, pub_topic_brake_2 = parse_args()
    
    # init pid controller
    pid_controller = PID(Kp=8.0, Ki=0.1, Kd=0.01, windupVal=80)

    # init node and publishers
    rospy.init_node(node_name, anonymous=True)
    stanely_ctrl_pub_1 = rospy.Publisher(pub_topic_name_1, PlatMsgVehicleCmd, queue_size=100)
    stanely_ctrl_pub_2 = rospy.Publisher(pub_topic_name_2, PlatMsgVehicleCmd, queue_size=100)
    brake_pub_1 = rospy.Publisher(pub_topic_brake_1, BrakeCmd, queue_size=2)
    brake_pub_2 = rospy.Publisher(pub_topic_brake_2, BrakeCmd, queue_size=1)

    # init control msg
    control_msg_1 = PlatMsgVehicleCmd()
    control_msg_1.cfg_type_cmd = 1
    control_msg_1.enable_cmd = True

    control_msg_2 = PlatMsgVehicleCmd()
    control_msg_2.cfg_type_cmd = 1
    control_msg_2.enable_cmd = True

    # init brake commands
    cmd_1 = BrakeCmd()
    cmd_2 = BrakeCmd()

    cmd_1.pedal_cmd = 100
    cmd_1.pedal_cmd_type = 2    # percent
    cmd_1.enable = True

    cmd_2.pedal_cmd = 100
    cmd_2.pedal_cmd_type = 2    # percent
    cmd_2.enable = True

    # compute error and publish distances
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        g_ttt = rospy.get_time()
        ts = float(g_ttt - g_tpr)

        dist_ahead = rospy.wait_for_message(sub_topic_name, Int64)
        error = DISTANCE_SET_POINT - dist_ahead.data

        # distance is smaller than SP, increase vehicle 1 speed, brake vehicle 2
        if error > 0:
            velo_1 = update_velocities(error, ts)

            cmd_1.enable = False
            cmd_2.enable = True
            cmd_2.pedal_cmd = (velo_1*5)

        # distance is greater than SP, brake vehicle 1, increase vehicle 2 speed
        elif error < 0:
            velo_2 = update_velocities(-1*error, ts)
            
            cmd_2.enable = False
            cmd_1.enable = True
            cmd_1.pedal_cmd = (velo_2*5)

        # distance is equal to SP, maintain speeds
        else:
            cmd_1.enable = False
            cmd_2.enable = False
            velo_1 = velo_2 = pid_controller._max_output

        print('*************************')
        print('topic  :' + sub_topic_name)
        print('error  : ' + str(error))
        print('veloc 1: ' + str(velo_1))
        print('veloc 2: ' + str(velo_2))

        control_msg_1.vel_cmd = velo_1
        control_msg_2.vel_cmd = velo_2

        brake_pub_1.publish(cmd_1)
        brake_pub_2.publish(cmd_2)

        stanely_ctrl_pub_1.publish(control_msg_1)
        stanely_ctrl_pub_2.publish(control_msg_2)

        g_tpr = g_ttt
        rate.sleep()

if __name__ == '__main__':
    main()