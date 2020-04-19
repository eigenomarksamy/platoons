#! /usr/bin/env python

import sys
import math

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
    args_list = list(sys.argv)
    args_list.pop(0)
    x_in = float(args_list[0])
    y_in = float(args_list[1])
    z_in = float(args_list[2])
    w_in = float(args_list[3])
    return x_in, y_in, z_in, w_in

def main():
    quat_x, quat_y, quat_z, quat_w = parse_args()
    r, p, y = quat_to_euler(quat_x, quat_y, quat_z, quat_w)
    print("Roll:    " + str(r))
    print("Pitch:   " + str(p))
    print("Yaw:     " + str(y))

if __name__ == '__main__':
    main()