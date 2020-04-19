#! /usr/bin/env python

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

class Path:
    def __init__(self, layout='straight_line'):
        self._layout = layout
        if self._layout == 'straight_line':
            self._x_init    = 125.0
            self._y_init    = -299.0
            self._yaw_init  = 1.57
            self._x_goal    = self._x_init
            self._y_goal    = 500.0
            self._yaw_goal  = self._yaw_init
            self._max_vel   = 10.0
            self._step      = 1.0
            self._path_len  = abs(int((abs(self._y_goal) - abs(self._y_init)) / self._step))
            self._max_acc   = 1.0
            self._idx       = 0
        elif self._layout == 'merge_path':
            self._x_init    = 147.0
            self._y_init    = -274.0
            self._yaw_init  = 1.68
            self._x_goal    = 134.0
            self._y_goal    = -169.0
            self._yaw_goal  = self._yaw_init
            self._max_vel   = 10.0
            self._step      = 1.0
            self._path_len  = abs(int((abs(self._y_goal) - abs(self._y_init)) / self._step))
            self._max_acc   = 1.0
            self._idx       = 0

    def generate_path(self):
        if self._layout == 'straight_line':
            self._y_path_arr = np.arange(self._y_init, (self._y_goal + self._step), self._step)
            x_list = [self._x_init] * (self._path_len + 1)
            yaw_list = [self._yaw_init] * (self._path_len + 1)
            self._x_path_arr = np.array(x_list)
            self._yaw_path_arr = np.array(yaw_list)
        elif self._layout == 'merge_path':
            x_path = [0.0] * (self._path_len + 1)
            y_path = [0.0] * (self._path_len + 1)
            yaw_path = [self._yaw_init] * (self._path_len + 1)
            x1 = self._x_init
            x2 = self._x_goal
            y1 = self._y_init
            y2 = self._y_goal
            m = (y2 - y1) / (x2 - x1)
            c = y1 - m * x1
            self._path_slope = m
            self._path_inter = c
            x_path[0] = 147.0
            y_path[0] = -274.0
            y_arr = np.arange(y1, (y2 + self._step), self._step)
            y_path = list(y_arr)
            self._y_path_arr = y_arr
            for i in range(self._path_len + 1):
                y_tmp = y_path[i]
                x_path[i] = (y_tmp - c) / m
            self._x_path_arr = np.array(x_path)
            self._yaw_path_arr = np.array(yaw_path)

    def generate_vp(self):
        self._spd_prof = []
        self._euc_dist = []
        for i in range(self._path_len + 1):
            self._euc_dist.append(((self._x_init - self._x_path_arr[i]) ** 2 + (self._y_init - self._y_path_arr[i]) ** 2) ** 0.5)
        v_sat       = self._max_vel
        v_i = v_f   = 0.0
        a_0         = self._max_acc
        s_a         = (v_sat ** 2 - v_i ** 2) / (2 * a_0)
        s_f         = self._euc_dist[-1]
        s_b         = s_f - ((v_f - v_sat ** 2)) / (2 * -a_0)
        for i in range(self._path_len + 1):
            if self._euc_dist[i] <= s_a:
                self._spd_prof.append(((2 * a_0 * self._euc_dist[i]) + (v_i ** 2)) ** 0.5)
            elif self._euc_dist[i] > s_a and self._euc_dist[i] < s_b:
                self._spd_prof.append(v_sat)
            elif self._euc_dist[i] >= s_b and self._euc_dist[i] < s_f:
                self._spd_prof.append((2 * - a_0 * (self._euc_dist[i] - s_b) + (v_sat ** 2)) ** 0.5)
            else:
                self._spd_prof.append(0.0)

    def get_current_path(self):
        return self._x_path_arr, self._y_path_arr, self._spd_prof
    
    def get_wp_list(self, wp_conc_list=[]):
        if wp_conc_list == []:
            wp_conc_list = [list(self._x_path_arr), list(self._y_path_arr), list(self._spd_prof)]
        self._wp_x = wp_conc_list[0]
        self._wp_y = wp_conc_list[1]
        self._wp_v = wp_conc_list[2]
        self._wp    = list(zip(self._wp_x, self._wp_y, self._wp_v))
        self._wp_np = np.array(self._wp)
        self._wp_d  = []
        for i in range(1, self._wp_np.shape[0]):
            self._wp_d.append(np.sqrt((self._wp_np[i, 0] - self._wp_np[i - 1, 0]) ** 2 + (self._wp_np[i, 1] - self._wp_np[i - 1, 1])))
        self._wp_d.append(0)
        self._wp_len = self._path_len
        self._wp_interp = []
        self._wp_interp_hash = []
        interp_counter = 0
        for i in range(self._wp_np.shape[0] - 1):
            self._wp_interp.append(list(self._wp_np[i]))
            self._wp_interp_hash.append(interp_counter)
            interp_counter += 1
            num_pts_to_interp = int(np.floor(self._wp_d[i] / float(0.01)) - 1)
            wp_vector = self._wp_np[i + 1] - self._wp_np[i]
            wp_uector = wp_vector / np.linalg.norm(wp_vector)
            for j in range(num_pts_to_interp):
                next_wp_vector = 0.01 * float(j + 1) * wp_uector
                self._wp_interp.append(list(self._wp_np[i] + next_wp_vector))
                interp_counter += 1
        self._wp_interp.append(list(self._wp_np[-1]))
        self._wp_interp_hash.append(interp_counter)
        interp_counter += 1
        return [self._wp_x, self._wp_y, self._wp_v, self._wp, self._wp_np, self._wp_d]
    
    def get_closest_wp(self, x_cur, y_cur):
        dx = [x_cur - itx for itx in self._wp_x]
        dy = [y_cur - ity for ity in self._wp_y]
        d = [abs(np.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        tmp_idx = d.index(min(d))
        self._idx += tmp_idx
        wx = self._wp_x[tmp_idx]
        wy = self._wp_y[tmp_idx]
        wv = self._wp_v[tmp_idx]
        return wx, wy, wv

    def get_closest_wp_adv(self, x_cur, y_cur):
        close_idx = self._idx
        close_dist = np.linalg.norm(np.array([self._wp_np[close_idx, 0] - x_cur, self._wp_np[close_idx, 1] - y_cur]))
        new_dist = close_dist
        new_idx = close_idx
        while new_dist <= close_idx:
            close_dist = new_dist
            close_idx = new_idx
            new_idx += 1
            if new_idx >= self._wp_np.shape[0]:
                break
            new_dist = np.linalg.norm(np.array([self._wp_np[new_idx, 0] - x_cur, self._wp_np[new_idx, 1] - 1]))
        new_dist = close_dist
        new_idx = close_idx
        while new_dist <= close_dist:
            close_dist = new_dist
            close_idx = new_idx
            new_idx -= 1
            if new_idx < 0:
                break
            new_dist = np.linalg.norm(np.array([self._wp_np[new_idx, 0] - x_cur, self._wp_np[new_idx, 1] - 1]))
        wp_subs_first_idx = close_idx - 1
        if wp_subs_first_idx < 0:
            wp_subs_first_idx = 0
        wp_subs_last_idx = close_idx
        tot_dist_ahead = 0
        while tot_dist_ahead < 20:
            tot_dist_ahead += self._wp_d[wp_subs_last_idx]
            wp_subs_last_idx += 1
            if wp_subs_last_idx >= self._wp_np.shape[0]:
                wp_subs_last_idx = self._wp_np.shape[0] - 1
                break
        new_wp = self._wp_interp[self._wp_interp_hash[wp_subs_first_idx] : self._wp_interp_hash[wp_subs_last_idx] + 1]
        wx = new_wp[0][0]
        wy = new_wp[0][1]
        min_idx = 0
        min_dist = float("inf")
        des_speed = 0
        for i in range(len(self._wp)):
            dist = np.linalg.norm(np.array([self._wp[i][0] - x_cur, self._wp[i][1] - y_cur]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._wp) - 1:
            des_speed = self._wp[min_idx][2]
        else:
            des_speed = self._wp[-1][2]
        wv = des_speed
        return wx, wy, wv

    def get_last_wp(self):
        wx = self._wp_x[-1]
        wy = self._wp_y[-1]
        wv = self._wp_v[-1]
        return wx, wy, wv

    def get_wp_arr(self):
        wx = self._wp_x
        wy = self._wp_y
        wv = self._wp_v
        return wx, wy, wv

    def get_idx(self):
        return self._idx

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

def main():
    path_obj = Path('merge_path')
    path_obj.generate_path()
    path_obj.generate_vp()
    x_path, y_path, v_path = path_obj.get_current_path()
    x_path = list(x_path)
    y_path = list(y_path)
    v_path = list(v_path)
    path_props = path_obj.get_wp_list()
    plot(path_obj._wp_x, path_obj._wp_y, path_obj._wp_v)

if __name__ == '__main__':
    main()
