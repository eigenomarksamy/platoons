#! /usr/bin/env python

import os
import sys
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

dirpath = os.path.dirname(os.path.abspath(__file__))
importdir = dirpath + '/cfg'
sys.path.append(importdir)

import cfg
from cfg import VehicleCfg

def main():
    vehicle_ns  = cfg.parse_args()
    ns_obj      = VehicleCfg(vehicle_ns)
    node_name, topic_name, model_name, frame_id = ns_obj.get_odom_properties()
    rospy.init_node(node_name, anonymous=True)
    odom_pub = rospy.Publisher(topic_name, Odometry, queue_size=100)
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    header = Header()
    odom = Odometry()
    header.frame_id = frame_id
    model = GetModelStateRequest()
    model.model_name = model_name
    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        result              = get_model_srv(model)
        odom.pose.pose      = result.pose
        odom.twist.twist    = result.twist
        header.stamp        = rospy.Time.now()
        odom.header         = header
        odom_pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    main()