#! /usr/bin/env python

import os
import sys
import matplotlib.pyplot as plt

dir_path = os.path.dirname(os.path.abspath(__file__))

dir_conf = dir_path + '/../cfg'

sys.path.append(dir_path)
sys.path.append(dir_conf)

import ptracker_cfg

class FileOdomPoint:
    def __init__(self, seq, x, y, qx, qy, qz, qw):
        self._seq = seq
        self._x = x
        self._y = y
        self._qx = qx
        self._qy = qy
        self._qz = qz
        self._qw = qw

def decompose_obj(object):
    return object._seq, object._x, object._y, object._qx, object._qy, object._qz, object._qw

def get_course_path(obj_list):
    x_path = []
    y_path = []
    yaw_path = []
    for i in range(len(obj_list)):
        _, x, y, qx, qy, qz, qw = decompose_obj(obj_list[i])
        yaw, _, _ = ptracker_cfg.quat_to_euler(qx, qy, qz, qw)
        x_path.append(x)
        y_path.append(y)
        yaw_path.append(yaw)
    return x_path, y_path, yaw_path

def generate_txt(x_path, y_path, yaw_path, ns):
    x_file_path, y_file_path, yaw_file_path = ptracker_cfg.config_dir('gen_path_points', ns)
    x_file = open(x_file_path, 'w')
    y_file = open(y_file_path, 'w')
    yaw_file = open(yaw_file_path, 'w')
    for i in range(len(x_path)):
        x_file.write(str(x_path[i]))
        x_file.write(" ")
    x_file.close()
    for i in range(len(y_path)):
        y_file.write(str(y_path[i]))
        y_file.write(" ")
    y_file.close()
    for i in range(len(yaw_path)):
        yaw_file.write(str(yaw_path[i]))
        yaw_file.write(" ")
    yaw_file.close()

def main():
    vehicle_ns = ptracker_cfg.parse_args()
    ns_obj = ptracker_cfg.VehicleCfg(vehicle_ns)
    odom_file = ns_obj.get_gen_odom_properties()
    try:
        file_obj = open(odom_file)
        lines = file_obj.readlines()
        odom_list = []
        for i in range(0, len(lines) - 7, 8):
            tmp_seq = float(lines[i])
            tmp_x = float(lines[i + 1])
            tmp_y = float(lines[i + 2])
            tmp_qx = float(lines[i + 3])
            tmp_qy = float(lines[i + 4])
            tmp_qz = float(lines[i + 5])
            tmp_qw = float(lines[i + 6])
            odom_obj = FileOdomPoint(tmp_seq, tmp_x, tmp_y, tmp_qx, tmp_qy, tmp_qz, tmp_qw)
            odom_list.append(odom_obj)
        file_obj.close()
        x_path, y_path, yaw_path = get_course_path(odom_list)
        generate_txt(x_path, y_path, yaw_path, vehicle_ns)
        plt.figure()
        plt.plot(x_path, y_path, 'bo')
        plt.show()
    except:
        print("No File!")

if __name__ == '__main__':
    main()