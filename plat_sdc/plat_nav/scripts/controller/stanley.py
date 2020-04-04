#! /usr/bin/env python

import math
import numpy as np

class Stanley():
    
    def __init__(self, Ke, Kv):
        self._Ke                    = Ke
        self._Kv                    = Kv
        self._current_way_x         = 0.0
        self._current_way_y         = 0.0
        self._prev_way_x            = 0.0
        self._prev_way_y            = 0.0
        self._current_pos_x_g       = 0.0
        self._current_pos_y_g       = 0.0
        self._vel_g                 = 0.0       
        self._current_yaw_g         = 0.0
        self._cte                   = 0.0
        self._he                    = 0.0
        
        
    def reset(self):
        self._current_way_x    = 0.0
        self._current_way_y    = 0.0
        self._prev_way_x       = 0.0
        self._prev_way_y       = 0.0
        self._current_pos_x_g  = 0.0
        self._current_pos_y_g  = 0.0
        self._vel_g            = 0.0       
        self._current_yaw_g    = 0.0
        
    def normalizeAngle(self, angle):
        newAngle = angle
        while (newAngle <= -180):
            newAngle += 360
        while (newAngle > 180):
            newAngle -= 360
        return newAngle 
            
    def controller_update(self):

        '''
        Fill Vehicle data
        '''
        current_yaw   = self._current_yaw_g   
        current_pos_x = self._current_pos_x_g 
        current_pos_y = self._current_pos_y_g
        vel           = self._vel_g           
           
        '''
        Fill the waypoints to Stanely controller
        '''
        current_way_x   = self._current_way_x 
        current_way_y   = self._current_way_y 
        prev_way_x      = self._prev_way_x    
        prev_way_y      = self._prev_way_y    
        
        '''
        Satnely Calculation
        '''        
        # 0. get a,b,c factors of trajectory line
        temp_den = current_way_x - prev_way_x
        if(abs(temp_den) > 0.0001):
            a =  (current_way_y - prev_way_y)/(temp_den)
        else:
            a = 0.0
        
        b = -1.0
        c = prev_way_y - (prev_way_x*a)
        den = np.sqrt(a**2 + b**2)
        
        # 1. calculate heading error
        yaw_des = np.arctan((-a)/(b))
        yaw_err = yaw_des - current_yaw
        yaw_err = self.normalizeAngle(yaw_err) 
        
        # 2. calculate crosstrack error
        crosstrack_err = (a*current_pos_x +b*current_pos_y+c)/den
       
        # 3. Stanely final Equation
        if(vel > 0.001):
            yaw_err_crosstrack = np.arctan(self._Ke * crosstrack_err / (self._Kv*vel))
        else:
            yaw_err_crosstrack=0.0
            
        steer_output = yaw_err + yaw_err_crosstrack
        steer_output = self.normalizeAngle(steer_output)

            
        # 4. Limit the output
        steer_output = min(1.0, steer_output)
        steer_output = max(-1.0, steer_output)
        
        #save internal data
        self._cte = crosstrack_err
        self._he  = math.degrees(yaw_err)       
    
        return steer_output

    def setWayPoints(self, current_way_x, current_way_y, prev_way_x, prev_way_y):
        self._current_way_x  =  current_way_x 
        self._current_way_y  =  current_way_y 
        self._prev_way_x     =  prev_way_x    
        self._prev_way_y     =  prev_way_y
        
    def setVehicleData(self, current_pos_x_g, current_pos_y_g, vel_g, current_yaw_g):
        self._current_pos_x_g  =  current_pos_x_g 
        self._current_pos_y_g  =  current_pos_y_g 
        self._vel_g            =  vel_g    
        self._current_yaw_g    =  current_yaw_g
        
    def getCte(self):
        return self._cte
        
    def getHe(self):
        return self._he
