#! /usr/bin/python

import os
import sys
import math
import rospy

from nav_msgs.msg import Odometry
from plat_msg.msg import PlatMsgVehicleCmd

dir_path = os.path.dirname(os.path.abspath(__file__))
dir_cont = dir_path + '/controller'
dir_plan = dir_path + '/planner'
dir_conf = dir_path + '/cfg'

sys.path.append(dir_path)
sys.path.append(dir_cont)
sys.path.append(dir_plan)
sys.path.append(dir_conf)

import ptracker_cfg
from wphandler      import Path
from stanley        import Stanley
from pid            import PID

global g_xxx, g_yyy, g_yaw, g_vel, g_ttt, g_tpr

g_tpr = 0.0

def quat_to_euler(qx, qy, qz, qw):
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def odom_callback(odom_data):
    global g_xxx, g_yyy, g_yaw, g_vel
    g_xxx = odom_data.pose.pose.position.x
    g_yyy = odom_data.pose.pose.position.y
    quat_x = odom_data.pose.pose.orientation.x
    quat_y = odom_data.pose.pose.orientation.y
    quat_z = odom_data.pose.pose.orientation.z
    quat_w = odom_data.pose.pose.orientation.w
    _, _, g_yaw = quat_to_euler(quat_x, quat_y, quat_z, quat_w)
    vx = odom_data.twist.twist.linear.x
    vy = odom_data.twist.twist.linear.y
    g_vel = ((vx) ** 2 + (vy) ** 2) ** 0.5

def main():
    global g_xxx, g_yyy, g_yaw, g_vel, g_ttt, g_tpr
    g_xxx = 0.0
    g_yyy = 0.0
    g_yaw = 0.0
    g_vel = 0.0
    vehicle_ns = ptracker_cfg.parse_args()
    ns_obj = ptracker_cfg.VehicleCfg(vehicle_ns)
    node_name, pub_topic_nam = ns_obj.get_ptracker_properties()
    _, sub_topic_name, _, _ = ns_obj.get_odom_properties()
    wp_obj = Path()
    wp_obj.generate_path()
    wp_obj.generate_vp()
    x_path, y_path, v_path = wp_obj.get_current_path()
    x_path = list(x_path)
    y_path = list(y_path)
    v_path = list(v_path)
    _ = wp_obj.get_wp_list()
    pid_controller = PID(Kp=10.0, Ki=0.1, Kd=0.01, windupVal=80)
    pid_controller._max_output = 100.0
    stanley_controller = Stanley(0.35, 1.0)
    rospy.init_node(node_name, anonymous=True)
    stanely_ctrl_pub = rospy.Publisher(pub_topic_nam, PlatMsgVehicleCmd, queue_size=100)
    rate = rospy.Rate(10)
    control_msg = PlatMsgVehicleCmd()
    while not rospy.is_shutdown():
        rospy.Subscriber(sub_topic_name, Odometry, odom_callback)
        g_xxx += 1.5 * math.cos(g_yaw)
        g_yyy += 1.5 * math.sin(g_yaw)
        g_ttt = rospy.get_time()
        ts = float(g_ttt - g_tpr)
        if ts == 0.0:
            ts = 0.1
        xf, yf, _ = wp_obj.get_last_wp()
        wx, wy, wv = wp_obj.get_closest_wp(g_xxx, g_yyy)
        vel_req = pid_controller.update(wv, g_vel, ts)
        stanley_controller.setVehicleData(g_xxx, g_yyy, g_vel, g_yaw)
        stanley_controller.setWayPoints(xf, yf, wx, wy)
        steer_req = stanley_controller.controller_update()
        control_msg.vel_cmd = vel_req
        control_msg.swa_cmd = steer_req
        control_msg.cfg_type_cmd = 3
        control_msg.enable_cmd = True
        stanely_ctrl_pub.publish(control_msg)
        g_tpr = g_ttt
        rate.sleep()

if __name__ == '__main__':
    main()