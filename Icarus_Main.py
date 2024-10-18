#Main file for the Icarus launch vehicle. 
#Import Declarations
import numpy as np 

#Physical Constants
r_Earth = 6378000
G_parameter_Earth = 3.986 * 10 ** 14
g0 = 9.8066

#Functions
def earth_Gravity(position):
    r = position + r_Earth
    g0 = G_parameter_Earth / (r * r) 
    return g0

#Simulation Settings
t_Initial = 0
position_Initial = 0
velocity_Initial = 0 
acceleration_Initial = 0

#Vehicle Configuration
mass_Propellant = 550000
mass_Vehicle = 30000
mass_Payload = 20000
mass = mass_Propellant + mass_Vehicle + mass_Payload

#Engine Configuration
engine_Mass_Flow = 313.76
engine_isp = 325
engine_exhaust_Velocity = engine_isp * g0
thrust_Engine = engine_Mass_Flow * engine_exhaust_Velocity
num_Engines = 9
thrust_Total = thrust_Engine * num_Engines
total_Mass_flow = engine_Mass_Flow * num_Engines

#Simulation Setup
t = t_Initial
t_Max = 10
dt = 0.1
position = position_Initial
velocity = velocity_Initial
acceleration = acceleration_Initial
g = g0
readout_Array = np.array([["Time Elapsed (s)", "Position", "Velocity", "Acceleration"]])

#Main Program Loop
while t <= t_Max:
    acceleration = thrust_Total / mass
    acceleration -= g
    
    current_Readout = np.array([[str(t) + " s", str(position), str(velocity), str(acceleration)]])
    readout_Array = np.append(readout_Array, current_Readout, axis = 0)
    
    t += dt
    position += (velocity * dt)
    velocity += (acceleration * dt)
    mass -= (total_Mass_flow * dt)
    
    if mass_Propellant <= 0:
        engine_Mass_Flow = 0
    else:
        pass
    g = earth_Gravity(position)
    
    if position <= 0 and t > 2:
        break
    else:
        pass
    
for row in readout_Array:
    print(" ".join(f"{col:20}" for col in row))


