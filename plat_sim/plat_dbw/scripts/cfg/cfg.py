#! /usr/bin/python

import sys
import rospy
from plat_msg.msg       import PlatMsgVehicleCmd
from dbw_mkz_msgs.msg   import ThrottleCmd, SteeringCmd, BrakeCmd, GearCmd, TurnSignalCmd, TwistCmd

VEHICLES_NS_LIST = ['novehicle', 'vehicle', 'leadvehicle', 'stringvehicle1', 'stringvehicle2', 'mergevehicle', 'obsvehicle']

class VehicleCfg:
    def __init__(self, ns):
        self._ns                    = ns
        self._odom_node_name        = self._ns + '_odom'
        self._llc_node_name         = self._ns + '_llc'
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

    def get_llc_properties(self):
        return self._llc_node_name, self._brake_topic_name, self._throttle_topic_name, self._steering_topic_name, self._gear_topic_name, self._turnsignal_topic_name, self._cmdvel_topic_name

    def get_odom_properties(self):
        return self._odom_node_name, self._odom_topic_name, self._gazebo_model_name, self._gazebo_frame_id

    def get_control_properties(self):
        return self._control_topic_name

class BrakeVehicle:
    def __init__(self):
        self._CMD_NONE          = 0
        self._CMD_PEDAL         = 1                 # Unitless, range 0.15 to 0.50
        self._CMD_PERCENT       = 2                 # Percent of maximum torque, range 0 to 1
        self._CMD_TORQUE        = 3                 # Nm, range 0 to 3412, open-loop
        self._CMD_TORQUE_RQ     = 4                 # Nm, range 0 to 3412, closed-loop
        self._CMD_DECEL         = 6                 # m/s^2, range 0 to 10
        self._TORQUE_BOO        = 520.0             # Nm, brake lights threshold
        self._TORQUE_MAX        = 3412.0            # Nm, maximum torque
        self._msg               = BrakeCmd()

    def set_msg(self, pedal_cmd=0, pedal_cmd_type=0, enable=0, clear=0, ignore=0, count=0):
        self._pedal_cmd                 = pedal_cmd
        self._pedal_cmd_type            = pedal_cmd_type
        self._enable                    = enable            # Enable
        self._clear                     = clear             # Clear driver overrides
        self._ignore                    = ignore            # Ignore driver overrides
        self._count                     = count             # Watchdog counter (optional)
        self._msg.pedal_cmd             = self._pedal_cmd
        self._msg.pedal_cmd_type        = self._pedal_cmd_type
        self._msg.enable                = self._enable
        self._msg.clear                 = self._clear
        self._msg.ignore                = self._ignore
        self._msg.count                 = self._count

    def set_pub(self, topic_name):
        self._pub = rospy.Publisher(topic_name, BrakeCmd, queue_size=100)

    def exec_pub(self):
        self._pub.publish(self._msg)

class ThrottleVehicle:
    def __init__(self):
        self._CMD_NONE          = 0
        self._CMD_PEDAL         = 1                 # Unitless, range 0.15 to 0.80
        self._CMD_PERCENT       = 2                 # Percent of maximum torque, range 0 to 1
        self._msg               = ThrottleCmd()

    def set_msg(self, pedal_cmd=0, pedal_cmd_type=0, enable=0, clear=0, ignore=0, count=0):
        self._pedal_cmd            = pedal_cmd
        self._pedal_cmd_type       = pedal_cmd_type
        self._enable               = enable            # Enable
        self._clear                = clear             # Clear driver overrides
        self._ignore               = ignore            # Ignore driver overrides
        self._count                = count             # Watchdog counter (optional)
        self._msg.pedal_cmd        = self._pedal_cmd
        self._msg.pedal_cmd_type   = self._pedal_cmd_type
        self._msg.enable           = self._enable
        self._msg.clear            = self._clear
        self._msg.ignore           = self._ignore
        self._msg.count            = self._count
    
    def set_pub(self, topic_name):
        self._pub = rospy.Publisher(topic_name, ThrottleCmd, queue_size=100)
    
    def exec_pub(self):
        self._pub.publish(self._msg)

class SteeringVehicle:
    def __init__(self):
        self._CMD_ANGLE     = 0                             # Unitless, range 0.15 to 0.80
        self._CMD_TORQUE    = 1                             # Percent of maximum torque, range 0 to 1
        self._ANGLE_MAX     = 9.6                           # rad, maximum angle
        self._VELOCITY_MAX  = 17.5                          # rad/s, maximum velocity
        self._TORQUE_MAX    = 8.0                           # Nm, maximum torque
        self._msg           = SteeringCmd()

    def set_msg(self, steering_wheel_angle_cmd=0.0, steering_wheel_angle_velocity=0.0, steering_wheel_torque_cmd=0.0, cmd_type=0, enable=0, clear=0, ignore=0, quiet=0, count=0):
        self._steering_wheel_angle_cmd             = steering_wheel_angle_cmd      # rad, range -9.6 to 9.6
        self._steering_wheel_angle_velocity        = steering_wheel_angle_velocity # rad/s, range 0 to 17.5, 0 = maximum
        self._steering_wheel_torque_cmd            = steering_wheel_torque_cmd     # Nm, range -8.0 to 8.0
        self._cmd_type                             = cmd_type
        self._enable                               = enable                        # Enable
        self._clear                                = clear                         # Clear driver overrides
        self._ignore                               = ignore                        # Ignore driver overrides
        self._quiet                                = quiet                         # Disable the driver override audible warning
        self._count                                = count                         # Watchdog counter (optional)
        self._msg.steering_wheel_angle_cmd         = self._steering_wheel_angle_cmd
        self._msg.steering_wheel_angle_velocity    = self._steering_wheel_angle_velocity
        self._msg.steering_wheel_torque_cmd        = self._steering_wheel_torque_cmd
        self._msg.cmd_type                         = self._cmd_type
        self._msg.enable                           = self._enable
        self._msg.clear                            = self._clear
        self._msg.ignore                           = self._ignore
        self._msg.quiet                            = self._quiet
        self._msg.count                            = self._count
    
    def set_pub(self, topic_name):
        self._pub = rospy.Publisher(topic_name, SteeringCmd, queue_size=100)
    
    def exec_pub(self):
        self._pub.publish(self._msg)

class GearVehicle:
    def __init__(self):
        self._NONE      = 0
        self._PARK      = 1
        self._REVERSE   = 2
        self._NEUTRAL   = 3
        self._DRIVE     = 4
        self._LOW       = 5
        self._msg       = GearCmd()

    def set_msg(self, cmd=0, clear=0):
        self._cmd           = cmd           # Gear command enumeration
        self._clear         = clear         # Clear driver overrides
        self._msg.cmd.gear  = self._cmd
        self._msg.clear     = self._clear

    def set_pub(self, topic_name):
        self._pub = rospy.Publisher(topic_name, GearCmd, queue_size=100)
    
    def exec_pub(self):
        self._pub.publish(self._msg)

class TurnSignalVehicle:
    def __init__(self):
        self._NONE  = 0
        self._LEFT  = 1
        self._RIGHT = 2
        self._msg   = TurnSignalCmd()

    def set_msg(self, cmd=0):
        self._cmd           = cmd       # Turn signal command enumeration
        self._msg.cmd.value = self._cmd

    def set_pub(self, topic_name):
        self._pub = rospy.Publisher(topic_name, TurnSignalCmd, queue_size=100)
    
    def exec_pub(self):
        self._pub.publish(self._msg)

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