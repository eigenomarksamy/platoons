#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

VEHICLES_NS_LIST = ['novehicle', 'vehicle', 'leadvehicle', 'stringvehicle1', 'stringvehicle2', 'mergevehicle', 'obsvehicle']

class VehicleCfg:
    def __init__(self, ns):
        self._ns                = ns
        self._odom_node_name    = ns + '_odom'
        self._odom_topic_name   = '/' + ns + '/odom'
        self._gazebo_model_name = ns
        self._gazebo_frame_id   = '/' + ns + '/base_link'

    def get_node_properties(self):
        return self._odom_node_name, self._odom_topic_name, self._gazebo_model_name, self._gazebo_frame_id

def parse_args():
    default_ns = 'vehicle'
    args_list = list(sys.argv)
    if len(args_list) <= 1:
        vehicle_ns = default_ns
    else:
        args_list.pop(0)
        vehicle_ns = args_list[0]
        if vehicle_ns not in VEHICLES_NS_LIST:
            vehicle_ns = 'novehicle'
    return vehicle_ns

def main():
    vehicle_ns  = parse_args()
    ns_obj      = VehicleCfg(vehicle_ns)
    node_name, topic_name, model_name, frame_id = ns_obj.get_node_properties()
    rospy.init_node(node_name, anonymous=True)
    odom_pub = rospy.Publisher(topic_name, Odometry, queue_size=100)
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    header = Header()
    odom = Odometry()
    header.frame_id = frame_id
    model = GetModelStateRequest()
    model.model_name = model_name
    rate = rospy.Rate(100)
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