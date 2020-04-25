#! /usr/bin/env python

import os
import sys
import rospy

from nav_msgs.msg import Odometry

dir_path = os.path.dirname(os.path.abspath(__file__))

dir_conf = dir_path + '/../cfg'

sys.path.append(dir_path)
sys.path.append(dir_conf)

import ptracker_cfg

def record_callback(odom_data):
    global gen_odom_file
    odom_header_seq = odom_data.header.seq
    odom_pose_pose_position_x = odom_data.pose.pose.position.x
    odom_pose_pose_position_y = odom_data.pose.pose.position.y
    odom_pose_pose_orientation_x = odom_data.pose.pose.orientation.x
    odom_pose_pose_orientation_y = odom_data.pose.pose.orientation.y
    odom_pose_pose_orientation_z = odom_data.pose.pose.orientation.z
    odom_pose_pose_orientation_w = odom_data.pose.pose.orientation.w
    odom_data_parse = [str(odom_header_seq), str(odom_pose_pose_position_x), str(odom_pose_pose_position_y), str(odom_pose_pose_orientation_x), str(odom_pose_pose_orientation_y), str(odom_pose_pose_orientation_z), str(odom_pose_pose_orientation_w)]
    gen_file = open(gen_odom_file, "a")
    for i in range(len(odom_data_parse)):
        gen_file.write(odom_data_parse[i])
        gen_file.write("\n")
    gen_file.write("-------------------------\n")
    gen_file.close()

def main():
    global gen_odom_file
    vehicle_ns = ptracker_cfg.parse_args()
    ns_obj = ptracker_cfg.VehicleCfg(vehicle_ns)
    node_name, odom_topic_name = ns_obj.get_rec_odom_properties()
    ns_obj.config_gen_odom_dir()
    gen_odom_file = ns_obj.get_gen_odom_properties()
    tmp_file = open(gen_odom_file, "w")
    tmp_file.close()
    try:
        os.remove(gen_odom_file)
    except:
        pass
    rospy.init_node(node_name, anonymous=True)
    rospy.Subscriber(odom_topic_name, Odometry, record_callback)
    rospy.spin()

if __name__ == '__main__':
    gen_odom_file = []
    main()