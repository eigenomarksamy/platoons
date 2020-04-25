#! /usr/bin/python

import os
import sys
import matplotlib.pyplot as plt

dir_path = os.path.dirname(os.path.abspath(__file__))
dir_conf = dir_path + '/../../cfg'

sys.path.append(dir_path)
import csp_generator

sys.path.append(dir_conf)
import ptracker_cfg

def get_path():
    waypoints_x = []
    waypoints_y = []
    target_velocity = 10.0 / 3.6
    vehicle_ns = ptracker_cfg.parse_args()
    waypoints_x, waypoints_y, course_x, course_y, course_yaw, course_k, course_s, final_goal, speed_profile = csp_generator.generate_path(vehicle_ns, is_default = True)
    plan = [waypoints_x, waypoints_y, final_goal, course_x, course_y, course_yaw, course_k, course_s, target_velocity, speed_profile]
    # csp_generator.generate_txt_course(plan)
    return plan


def plot_path(cx, cy):
    plt.plot(cx, cy, "-r", label="course")
    plt.show()


def main():
    plan = get_path()
    plot_path(plan[0], plan[1])


if __name__ == '__main__':
    main()