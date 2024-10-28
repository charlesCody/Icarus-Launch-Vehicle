import numpy as np 

class Controller:
    def __init__(self, throttle_Min, throttle_Max, num_Engines):
        self.throttle_Min = throttle_Min 
        self.throttle_Max = throttle_Max
        self.num_Engines = num_Engines
        
        
    def main_Controller(self, Controlled_State, error, error_history, error_integral, proportional_Gain, integral_Gain, derivative_Gain, throttle, throttle_Min, throttle_Max, num_Engines):
        if Controlled_State:
            integrator = True
            proportional_Response = proportional_Gain * (error_history[-1]) if len(error_history) > 0 else 0

            if integrator:
                
                error_integral = np.append(error_integral, error_history[-1])
            integral_Response = integral_Gain * np.sum(error_integral)
            
            if len(error_history) > 1:
                derivative_Response = derivative_Gain * error_history[-1] - error_history[-2]
            else:
                derivative_Response = 0
                
            throttle = proportional_Response + integral_Response + derivative_Response
            throttle = max(throttle_Min, min(throttle_Max, throttle))
            
            if throttle > throttle_Max:
                throttle = throttle_Max
            elif throttle < throttle_Min: 
                throttle = throttle_Min
                
            if (throttle == throttle_Max or throttle == throttle_Min) and np.sign(error) == np.sign(throttle):
                integrator = False
            
            if throttle < (throttle_Min + 0.03) and num_Engines > 0:
                num_Engines, throttle = self.engine_Shutdown(num_Engines, throttle)
        else:
            pass
        #print(proportional_Response, integral_Response, derivative_Response, throttle)
        return num_Engines, throttle
    
    def engine_Shutdown(self, num_Engines, throttle):
        if num_Engines == 1:
            pass
        else:
            throttle *= (num_Engines)/(num_Engines - 1)
            num_Engines -= 1
        return num_Engines, throttle