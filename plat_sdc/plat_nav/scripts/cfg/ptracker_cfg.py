#! /usr/bin/python

import os
import sys
import math
import matplotlib.pyplot as plt

VEHICLES_NS_LIST = ['novehicle', 'vehicle', 'leadvehicle', 'stringvehicle1', 'stringvehicle2', 'mergevehicle', 'obsvehicle']

class VehicleCfg:
    def __init__(self, ns):
        self._ns                    = ns
        self._odom_node_name        = self._ns + '_odom'
        self._llc_node_name         = self._ns + '_llc'
        self._ptracker_node_name    = self._ns + '_ptracker'
        self._recodom_node_name     = self._ns + '_rec_dodm'
        self._gazebo_model_name     = self._ns
        self._gazebo_frame_id       = '/' + self._ns + '/base_link'
        self._odom_topic_name       = '/' + self._ns + '/odom'
        self._control_topic_name    = '/' + self._ns + '/control'
        self._brake_topic_name      = '/' + self._ns + '/brake_cmd'
        self._throttle_topic_name   = '/' + self._ns + '/throttle_cmd'
        self._steering_topic_name   = '/' + self._ns + '/steering_cmd'
        self._gear_topic_name       = '/' + self._ns + '/gear_cmd'
        self._turnsignal_topic_name = '/' + self._ns + '/turn_signal_cmd'
        self._cmdvel_topic_name     = '/' + self._ns + '/cmd_vel'
        self._gen_odom_rel_file_path = 'genOdomData/' + self._ns + '_gen_odom_data.txt'

    def get_llc_properties(self):
        return self._llc_node_name, self._brake_topic_name, self._throttle_topic_name, self._steering_topic_name, self._gear_topic_name, self._turnsignal_topic_name, self._cmdvel_topic_name

    def get_odom_properties(self):
        return self._odom_node_name, self._odom_topic_name, self._gazebo_model_name, self._gazebo_frame_id
    
    def get_ptracker_properties(self):
        return self._ptracker_node_name, self._control_topic_name

    def get_control_properties(self):
        return self._control_topic_name
    
    def get_rec_odom_properties(self):
        return self._recodom_node_name, self._odom_topic_name
    
    def get_gen_odom_properties(self):
        return self._gen_odom_rel_file_path
    
    def config_gen_odom_dir(self):
        config_dir('odom')
        self._gen_odom_rel_file_path = self._ns + '_gen_odom_data.txt'


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

def plot_vp(vp):
    seq = []
    for i in range(len(vp)):
        seq.append(i + 1)
    _, _ = plt.subplots(1)
    plt.plot(seq, vp, 'g-')
    plt.grid(True)
    plt.xlabel("WP Sequence")
    plt.ylabel("Velocity [m/s]")


def plot_xy(x, y):
    _, _ = plt.subplots(1)
    plt.plot(x, y, 'b-')
    plt.grid(True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

def plot(x, y, v):
    plot_xy(x, y)
    plot_vp(v)
    plt.show()

def config_dir(gen_type, ns=[]):
    if gen_type == 'path':
        try:
            os.chdir('../planner/genPathData')
        except OSError:
            print("Generation Directory Could Not Be Found!")
            os.chdir(os.getcwd())
            os.chdir('../planner/')
            print("Creating \"genPathData\" at " + str(os.getcwd()))
            os.mkdir('genPathData')
            print("Generation Directory \"genPathData\" Created Successfully")
            os.chdir('genPathData/')
    elif gen_type == 'odom':
        try:
            os.chdir('../planner/genOdomData')
        except OSError:
            print("Generation Directory Could Not Be Found!")
            os.chdir(os.getcwd())
            os.chdir('../planner/')
            print("Creating \"genOdomData\" at " + str(os.getcwd()))
            os.mkdir('genOdomData')
            print("Generation Directory \"genOdomData\" Created Successfully")
            os.chdir('genOdomData/')
    elif gen_type == 'gen_path_points':
        try:
            os.chdir('../planner/genPathData')
        except OSError:
            print("Generation Directory Could Not Be Found!")
            os.chdir(os.getcwd())
            os.chdir('../planner/')
            print("Creating \"genPathData\" at " + str(os.getcwd()))
            os.mkdir('genPathData')
            print("Generation Directory \"genPathData\" Created Successfully")
            os.chdir('genPathData/')
        try:
            os.remove(ns + '_parse_x_points.txt')
        except:
            pass
        try:
            os.remove(ns + '_parse_y_points.txt')
        except:
            pass
        try:
            os.remove(ns + '_parse_yaw_points.txt')
        except:
            pass
        x_file = ns + '_parse_x_points.txt'
        y_file = ns + '_parse_y_points.txt'
        yaw_file = ns + '_parse_yaw_points.txt'
        return x_file, y_file, yaw_file

def generate_wp_file(data, ns):
    file_name = ns + '_waypoints.txt'
    try:
        os.remove(file_name)
    except:
        pass
    data_list = [list(data.x), list(data.y), list(data.v)]
    names_list = ['wp_x', 'wp_y', 'wp_v']
    wp_file = open(file_name, 'w')
    for i in range(len(data_list)):
        tmp_list = data_list[i]
        tmp_name = names_list[i]
        wp_file.write(tmp_name)
        wp_file.write(":\n")
        for j in range(len(tmp_list)):
            wp_file.write(str(tmp_list[j]))
            wp_file.write(" ")
        wp_file.write("\n")
        wp_file.write("-------------------------------------------")
        wp_file.write("\n")
    wp_file.close()