#! /usr/bin/python

import os
import sys
import math
import rospy

from std_msgs.msg import Int64
from plat_msg.msg import PlatMsgVehicleCmd

dir_path = os.path.dirname(os.path.abspath(__file__))
dir_cont = dir_path + '/controller'

sys.path.append(dir_path)
sys.path.append(dir_cont)

from pid import PID

DISTANCE_SET_POINT = 10

g_tpr = 0.0
pid_controller = None

def parse_args():
    args_list = list(sys.argv)

    node_name = args_list[3]
    vehicle1 = args_list[1]
    vehicle2 = args_list[2]
    
    sub_topic_name = 'distance_ahead/' + vehicle2 + '_to_' + vehicle1
    pub_topic_name_1 = vehicle1 + '/control'
    pub_topic_name_2 = vehicle2 + '/control'

    return node_name, sub_topic_name, pub_topic_name_1, pub_topic_name_2

def update_velocities(error, ts):
    global pid_controller

    if ts == 0.0:
        ts = 0.1

    return pid_controller.update(error, ts)

def main():
    global g_tpr
    global pid_controller

    h_gain = 0.25

    # parse topic names and node name
    node_name, sub_topic_name, pub_topic_name_1, pub_topic_name_2 = parse_args()
    
    # init pid controller
    pid_controller = PID(Kp=5.0, Ki=0.1, Kd=0.01, windupVal=80)

    # init node and publishers
    rospy.init_node(node_name, anonymous=True)
    stanely_ctrl_pub_1 = rospy.Publisher(pub_topic_name_1, PlatMsgVehicleCmd, queue_size=100)
    stanely_ctrl_pub_2 = rospy.Publisher(pub_topic_name_2, PlatMsgVehicleCmd, queue_size=100)


    control_msg_1 = PlatMsgVehicleCmd()
    control_msg_1.cfg_type_cmd = 1
    control_msg_1.enable_cmd = True

    control_msg_2 = PlatMsgVehicleCmd()
    control_msg_2.cfg_type_cmd = 1
    control_msg_2.enable_cmd = True

    # compute error and publish distances
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        g_ttt = rospy.get_time()
        ts = float(g_ttt - g_tpr)

        dist_ahead = rospy.wait_for_message(sub_topic_name, Int64)
        error = DISTANCE_SET_POINT - dist_ahead.data

        velo_1 = velo_2 = 0
        # distance is smaller than SP, increase vehicle 1 speed, decrease vehicle 2 speed
        if error > 0:
            # velo_1 = update_velocities(error, ts)
            # velo_2 = velo_1 - (h_gain * velo_1)
            velo_1 = 10
            velo_2 = 0
        
        # distance is greater than SP, decrease vehicle 1 speed, increase vehicle 2 speed
        elif error < 0:
            # velo_2 = update_velocities(-1*error, ts)
            # velo_1 = velo_2 - (h_gain * velo_2)
            velo_1 = 0
            velo_2 = 10

        # distance is equal to SP, maintain speeds
        else:
            # velo_1 = velo_2 = pid_controller._max_output
            velo_1 = velo_2 = 10

        print('*************************')
        print('topic  :' + sub_topic_name)
        print('error  : ' + str(error))
        print('veloc 1: ' + str(velo_1))
        print('veloc 2: ' + str(velo_2))

        control_msg_1.vel_cmd = velo_1
        control_msg_2.vel_cmd = velo_2

        stanely_ctrl_pub_1.publish(control_msg_1)
        stanely_ctrl_pub_2.publish(control_msg_2)

        g_tpr = g_ttt
        rate.sleep()

if __name__ == '__main__':
    main()