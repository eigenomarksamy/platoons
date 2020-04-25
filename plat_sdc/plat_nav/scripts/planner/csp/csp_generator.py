#! /usr/bin/python


import os
import math
import imp
import sys

dir_path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(dir_path)
import cubic_spline_planner as csp


class Path:

    def __init__(self, dir='csp', x_path = [], y_path = []):
        self.x_path = x_path
        self.y_path = y_path
        self._dir = dir

    def default_raw_path(self):
        file_path = ''
        if self._dir == 'csp':
            file_path = '../genPathData/' + self._ns + '_parse_x_points.txt'
        elif self._dir == 'ros':
            abs_path = os.getcwd()
            file_path = abs_path + '/planner/genPathData/' + self._ns + '_parse_x_points.txt'
        elif self._dir == 'wphandler':
            file_path = 'genPathData/' + self._ns + '_parse_x_points.txt'
        self._x_path_file = open(file_path, "r")
        self._xpath_raw = []
        for self._val in self._x_path_file.read().split():
            self._xpath_raw.append(float(self._val))
        self._x_path_file.close()
        if self._dir == 'csp':
            file_path = '../genPathData/' + self._ns + '_parse_y_points.txt'
        elif self._dir == 'ros':
            abs_path = os.getcwd()
            file_path = abs_path + '/planner/genPathData/' + self._ns + '_parse_y_points.txt'
        elif self._dir == 'wphandler':
            file_path = 'genPathData/' + self._ns + '_parse_y_points.txt'
        self._y_path_file = open(file_path, "r")
        self._ypath_raw = []
        for self._val in self._y_path_file.read().split():
            self._ypath_raw.append(float(self._val))
        self._y_path_file.close()
        self.x_raw = self._xpath_raw
        self.y_raw = self._ypath_raw
        return self.x_raw, self.y_raw

    def default_refactor(self):
        self._x_path = self.x_raw
        self._y_path = self.y_raw
        self._x_path = [self._counter_x_path / 2.0 for self._counter_x_path in self._x_path]
        self._y_path = [self._counter_y_path / 2.0 for self._counter_y_path in self._y_path]
        self._raw_props = self.path_props(self._x_path, self._y_path)
        self._def_x_initial = self._raw_props[0][0]
        self._def_y_initial = self._raw_props[0][1]
        self._def_x_size_path = self._raw_props[2][0]
        self._def_y_size_path = self._raw_props[2][1]
        self._def_path_size = self._def_x_size_path
        self._counter_x_path_2 = 0
        self._counter_y_path_2 = 0
        if self._def_x_initial >= 0.0:
            for self._counter_x_path_2 in range(self._def_x_size_path):
                self._x_path[self._counter_x_path_2] = self._x_path[self._counter_x_path_2] - self._def_x_initial
        else:
            for self._counter_x_path_2 in range(self._def_x_size_path):
                self._x_path[self._counter_x_path_2] = self._x_path[self._counter_x_path_2] + self._def_x_initial
        if self._def_y_initial >= 0.0:
            for self._counter_y_path_2 in range(self._def_y_size_path):
                self._y_path[self._counter_y_path_2] = self._y_path[self._counter_y_path_2] - self._def_y_initial
        else:
            for self._counter_y_path_2 in range(self._def_y_size_path):
                self._y_path[self._counter_y_path_2] = self._y_path[self._counter_y_path_2] + self._def_y_initial
        self.refacto_x = self._x_path
        self.refacto_y = self._y_path
        return self.refacto_x, self.refacto_y

    def remove_initpoint(self, x_path, y_path):
    	x_path.pop(0)
    	y_path.pop(0)
    	self.x_noinit = x_path
    	self.y_noinit = y_path
    	return self.x_noinit, self.y_noinit

    def remove_singular(self, x_path, y_path, path_props):
        self._singular_list = []
        if path_props[2][0] == path_props[2][1]:
            for i in range(path_props[2][0] - 1):
                if x_path[i] == x_path[i + 1]:
                    if y_path[i] == y_path[i + 1]:
                        self._singular_list.append(i)
            for j in range(len(self._singular_list)):
                x_path.pop(self._singular_list[j] - j)
                y_path.pop(self._singular_list[j] - j)
        self.x_sing = x_path
        self.y_sing = y_path
        return self.x_sing, self.y_sing

    def path_props(self, x_path, y_path):
        self._x_size_path = x_path.__len__()
        self._y_size_path = y_path.__len__()
        self._x_min_path = min(x_path)
        self._y_min_path = min(y_path)
        self._x_max_path = max(x_path)
        self._y_max_path = max(y_path)
        self._x_final_path = x_path[-1]
        self._y_final_path = y_path[-1]
        self._x_initial = x_path[0]
        self._y_initial = y_path[0]
        self._start_point = [self._x_initial, self._y_initial]
        self._goal_point = [self._x_final_path, self._y_final_path]
        self._path_len = [self._x_size_path, self._y_size_path]
        self._path_min = [self._x_min_path, self._y_min_path]
        self._path_max = [self._x_max_path, self._y_max_path]
        self.path_properties = [self._start_point, self._goal_point, self._path_len, self._path_min, self._path_max]
        return self.path_properties

    def get_ref_speed(self):
        # self._refspeed_file = open("/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/target_max_speed.txt", "r")
        # self._refspeed_text = []
        # for self._text in self._refspeed_file.read().split():
        #     self._refspeed_text.append(self._text)
        # self._refspeed_file.close()
        return (10.0/3.6)
    
    def set_ns(self, ns):
        self._ns = ns


def generate_path(ns, outer = False, is_default = True):
    x_path = []
    y_path = []
    if is_default == True:
        path_obj = Path(outer)
        path_obj.set_ns(ns)
        x_raw, y_raw = path_obj.default_raw_path()
        path_properties = path_obj.path_props(x_raw, y_raw)
        x_path, y_path = path_obj.remove_singular(x_raw, y_raw, path_properties)
        x_refacto, y_refacto = path_obj.remove_initpoint(x_path, y_path)
        path_properties = path_obj.path_props(x_refacto, y_refacto)
        waypoints_x = x_path
        waypoints_y = y_path
        final_goal = path_properties[1]
        course_x, course_y, course_yaw, course_k, course_s = csp.calc_spline_course(waypoints_x, waypoints_y, ds=0.1)
        ref_speed = path_obj.get_ref_speed()
        speed_profile = calc_speed_profile(course_x, course_y, course_yaw, ref_speed)
    else:
        print "No Other than default"
    return x_path, y_path, course_x, course_y, course_yaw, course_k, course_s, final_goal, speed_profile


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0
        if switch:
            direction *= -1
        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed
        if switch:
            speed_profile[i] = 0.0
    for i in range(40):
        speed_profile[-i] = target_speed / (50 - i)
        if speed_profile[-i] <= 1.0 / 3.6:
            speed_profile[-i] = 1.0 / 3.6
    return speed_profile


# def generate_txt_course(plan):
#     names_l = ["wpx", "wpy", "fg", "cx", "cy", "cyaw", "ck", "cs", "tv", "sp"]
#     course_file = "/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/course_txt.txt"
#     try:
#         os.remove(course_file)
#     except:
#         pass
#     course_file_obj = open(course_file, "w")
#     for i in range(len(plan)):
#         tmp_l = plan[i]
#         tmp_name = names_l[i]
#         if i == 8:
#             tmp_l = [tmp_l]
#         course_file_obj.write(tmp_name)
#         course_file_obj.write("\n")
#         for j in range(len(tmp_l)):
#             course_file_obj.write(str(tmp_l[j]))
#             course_file_obj.write(" ")
#         course_file_obj.write("\n")
#         course_file_obj.write("-------------------------------------------")
#         course_file_obj.write("\n")
#     course_file_obj.close()
