#Main file for the Icarus launch vehicle. 
#Import Declarations
import numpy as np 
import matplotlib.pyplot as plt 
from Icarus_Interface import Icarus_Interface 
import time

#Physical Constants
r_Earth = 6378000
G_parameter_Earth = 3.986 * 10 ** 14
g0 = 9.8066

#Functions
def earth_Gravity(state):
    r = state[2,0] + r_Earth
    g = G_parameter_Earth / (r * r) 
    return g

def standard_Atmosphere(state):
    if state[2,0] < 11000:
        Temperature = 15.05 - 0.00649 * state[2,0] 
        atmospheric_Pressure = 101.29 * ((Temperature + 273.1)/ 288.08) ** 5.256
        
    if state[2,0] >= 11000 and state[2,0] < 25000:
        Temperature = -56.46
        atmospheric_Pressure = 22.65 * np.exp(1.73 - 0.000157 * state[2,0])
    else:
        Temperature = -131.21 + 0.00299 * state[2,0]
        atmospheric_Pressure = 2.488 * ((Temperature + 273.1)/216.6) ** -11.388 
    return Temperature, atmospheric_Pressure

def atmosphere_Pressure_to_Density(state):
    Temperature, atmospheric_Pressure = standard_Atmosphere(state)
    air_Density = atmospheric_Pressure / (.2869 * (Temperature + 273.1))
    return air_Density

def drag_Force(drag_Coefficient, cross_Sectional_Area, state):
    air_Density = atmosphere_Pressure_to_Density(state)
    drag = 0.5 * drag_Coefficient * air_Density * cross_Sectional_Area * np.linalg.norm(state[:,1]) ** 2
    return drag

def dynamic_Pressure(air_Density, state):
    Q = 0.5 * air_Density * state[2,1] ** 2
    return Q

def controller(Controlled_State, state, velocity_Target, error, error_history, error_integral, proportional_Gain, integral_Gain, derivative_Gain):
    if Controlled_State:
        global throttle
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
            
    else:
        pass
    #print(proportional_Response, integral_Response, derivative_Response, throttle)
    return throttle
    
#Simulation Initialization
t_Initial = 0
state_Initial = np.zeros((3,3))
g = np.zeros(3)
velocity_history = np.array([])
#Vehicle Configuration
mass_Propellant = 550000
mass_Vehicle = 30000
mass_Payload = 20000
mass = mass_Propellant + mass_Vehicle + mass_Payload
drag_Coefficient = 0.75
cross_Sectional_Area = 12.56 

#Engine Coxnfiguration
engine_Mass_Flow = 313.76
engine_isp = 325
engine_exhaust_Velocity = engine_isp * g0
thrust_Engine = engine_Mass_Flow * engine_exhaust_Velocity
num_Engines = 9
ignited_Engines = num_Engines
thrust_Total = thrust_Engine * num_Engines
total_Mass_flow = engine_Mass_Flow * num_Engines
throttle = 1
throttle_Min = 0.2
throttle_Max = 1
Controlled_State = True
proportional_Gain = 0.05
integral_Gain = 0.1
derivative_Gain = 1
error_history = np.array([])
error_integral = np.array([])
#Simulation Setup
t = t_Initial
t_Max = 1000
dt = 0.1
state = state_Initial

g[2] = g0
readout_Array = np.array([["Time Elapsed (s)", "Position", "Velocity", "Acceleration"]])
velocity_Target = np.array([0,0,1000])

#Main Program Loop
while t <= t_Max and state[2,0] >= 0:
    error = velocity_Target[2] - state[2,1]
    error_history = np.append(error_history, error)
    velocity_half = state[:,1] + state[:,2] * 0.5 * dt

    g[2] = earth_Gravity(state)
    drag = drag_Force(drag_Coefficient, cross_Sectional_Area, state)
    state[:,0] += (velocity_half * dt)
    current_Readout = np.array([[str(t), str(state[2,0]), str(state[2,1]), str(state[2,2])]])
    readout_Array = np.append(readout_Array, current_Readout, axis = 0)
    
    t += dt
    throttle = controller(Controlled_State, state, velocity_Target, error, error_history, error_integral, proportional_Gain, integral_Gain, derivative_Gain)
    state[2,2] = (throttle * thrust_Total - drag)/ mass
    state[2] -= g
    
    state[:,1] = velocity_half + (state[:,2] * 0.5 * dt)
    Mass_flow = total_Mass_flow * throttle
    mass -= (Mass_flow * dt)
    mass_Propellant -= (Mass_flow * dt)
    
    if mass_Propellant <= 0:
        total_Mass_Flow = 0
    else:
        pass
    print(t, throttle, state[2,1], state[2,2], velocity_half[2], state[2,0], mass_Propellant)
    #time.sleep(0.05)
    
#for row in readout_Array:
    #print(" ".join(f"{col:20}" for col in row))


icarus_Interface = Icarus_Interface(readout_Array)
icarus_Interface.kinematic_plot(readout_Array)

