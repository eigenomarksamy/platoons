#! /usr/bin/env python

class PID():
    '''
    classdocs
    '''
    def __init__(self ,Kp, Ki, Kd, windupVal=0):
        '''
        Constructor
        '''
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        
        self._max_output = 30.0
        self._min_output = 10.0
        self._cyclic_time = 0.0
        self._windupVal = windupVal
        
        self._error = 0.0
        self._intergal = 0.0
        self._derivative = 0.0
        self._output = 0.0
    
    def _clamp(self, value):
        if value is None:
            return None
        elif value > self._max_output:
            return self._max_output
        elif value < self._min_output:
            return self._min_output
        else:
            return value
    
    def _antiWindUp(self):
        if self._windupVal != 0:
            if self._intergal > self._windupVal:
                self._intergal = self._windupVal
            elif self._intergal < -self._windupVal:
                self._intergal = -self._windupVal
            
    def update(self, error, cyclic_time):
        self._cyclic_time = cyclic_time
        if(self._cyclic_time > 0.0):
            last_error = self._error
            last_intgeral     = self._intergal

            #Error Part
            self._error = error
            
            #Intgeral Part
            self._intergal = last_intgeral + (self._error * self._cyclic_time) 
            self._antiWindUp()
            
            #derivative part
            self._derivative = (self._error - last_error) / self._cyclic_time
            
            #Final Ouput
            self._output = (self._Kp * self._error) + (self._Ki * self._intergal) + (self._Kd * self._derivative)
            self._output = self._clamp(self._output)

        else:
            self._output = 0.0
            self._intergal =0.0
            self._derivative-0.0

        return (self._output)
        
    def get_sumOutput(self):
        self._output
