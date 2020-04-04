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
        
        self._max_output = 0.0
        self._min_output = 0.0
        self._cyclic_time = 0.0
        self._windupVal = windupVal
        
        self._error = 0.0
        self._intergal = 0.0
        self._derivative = 0.0
        self._output = 0.0
                
    def output_limits(self):
        return self._min_output, self._max_output
    
    def _clamp(self, value, limits):
        lower, upper = limits
        if value is None:
            return None
        elif upper is not None and value > upper:
            return upper
        elif lower is not None and value < lower:
            return lower
        return value
    
    def _antiWindUp(self):
        if self._windupVal != 0:
            if self._intergal > self._windupVal:
                self._intergal = self._windupVal
            elif self._intergal < -self._windupVal:
                self._intergal = -self._windupVal
        
    def update(self, set_point, current_val, cyclic_time):
        self._cyclic_time = cyclic_time
        if(self._cyclic_time > 0.0):
            last_error = self._error
            last_intgeral     = self._intergal

            #Error Part
            self._error = set_point - current_val
            
            #Intgeral Part
            self._intergal = last_intgeral + (self._error * self._cyclic_time) 
            self._antiWindUp()
            
            #derivative part
            self._derivative = (self._error - last_error) / self._cyclic_time
            
            #Final Ouput
            self._output = (self._Kp * self._error) + (self._Ki * self._intergal) + (self._Kd * self._derivative)
            # self._output = self._clamp(self._output, self.output_limits)

        else:
            self._output = 0.0
            self._intergal =0.0
            self._derivative-0.0

        return (self._output)
    
        
    def get_sumOutput(self):
        self._output
