#! /usr/bin/env python

import sys
import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64

def parse_args():
    args_list = list(sys.argv)
    vehicle1_ns = args_list[1]
    vehicle2_ns = args_list[2]
    node_name   = args_list[3]
    topic_name = 'distance_ahead/' + vehicle1_ns + '_to_' + vehicle2_ns 
    return vehicle1_ns, vehicle2_ns, node_name, topic_name

def calc_euclidean_distance(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def calc_y_distance(y1, y2):
    diff = y1-y2
    if diff<0:
        return diff*-1
    else:
        return diff

def compute_dist_ahead(vehicle1_topic, vehicle2_topic):
    odom_1 = rospy.wait_for_message(vehicle1_topic, Odometry)
    odom_2 = rospy.wait_for_message(vehicle2_topic, Odometry)
    # distance = calc_euclidean_distance(odom_1.pose.pose.position, odom_2.pose.pose.position)
    distance = calc_y_distance(odom_1.pose.pose.position.y, odom_2.pose.pose.position.y)
    return distance
    
def main():
    vehicle1_ns, vehicle2_ns, node_name, topic_name = parse_args()

    rospy.init_node(node_name, anonymous=True)
    dist_pub = rospy.Publisher(topic_name, Int64, queue_size=100)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        dist_ahead = compute_dist_ahead(vehicle1_ns + '/odom', vehicle2_ns + '/odom')
        dist_pub.publish(dist_ahead)
        rate.sleep()

if __name__ == '__main__':
    main()