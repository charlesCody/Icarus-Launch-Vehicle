#Main file for the Icarus launch vehicle. 
#Import Declarations
import numpy as np 
import matplotlib.pyplot as plt 

#Physical Constants
r_Earth = 6378000
G_parameter_Earth = 3.986 * 10 ** 14
g0 = 9.8066

#Functions
def earth_Gravity(position):
    r = position[2] + r_Earth
    g = G_parameter_Earth / (r * r) 
    return g

def standard_Atmosphere(position):
    if position[2] < 11000:
        Temperature = 15.05 - 0.00649 * position[2] 
        atmospheric_Pressure = 101.29 * ((Temperature + 273.1)/ 288.08) ** 5.256
        
    if position[2] >= 11000 and position[2] < 25000:
        Temperature = -56.46
        atmospheric_Pressure = 22.65 * np.exp(1.73 - 0.000157 * position[2])
    else:
        Temperature = -131.21 + 0.00299 * position[2]
        atmospheric_Pressure = 2.488 * ((Temperature + 273.1)/216.6) ** -11.388 
    return Temperature, atmospheric_Pressure

def atmosphere_Pressure_to_Density(position):
    Temperature, atmospheric_Pressure = standard_Atmosphere(position)
    air_Density = atmospheric_Pressure / (.2869 * (Temperature + 273.1))
    return air_Density

def drag_Force(drag_Coefficient, cross_Sectional_Area, velocity, position):
    air_Density = atmosphere_Pressure_to_Density(position)
    drag = 0.5 * drag_Coefficient * air_Density * cross_Sectional_Area * velocity[2] ** 2
    return drag

def dynamic_Pressure(air_Density, velocity):
    Q = 0.5 * air_Density * velocity ** 2
    return Q

#Simulation Initialization
t_Initial = 0
position_Initial = np.zeros(3)
velocity_Initial = np.zeros(3) 
acceleration_Initial = np.zeros(3)
g = np.zeros(3)

#Vehicle Configuration
mass_Propellant = 550000
mass_Vehicle = 30000
mass_Payload = 20000
mass = mass_Propellant + mass_Vehicle + mass_Payload
drag_Coefficient = 0.75
cross_Sectional_Area = 12.56 

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
t_Max = 100
dt = 0.1
position = position_Initial
velocity = velocity_Initial
acceleration = acceleration_Initial
g[2] = g0
readout_Array = np.array([["Time Elapsed (s)", "Position", "Velocity", "Acceleration"]])

#Main Program Loop
while t <= t_Max and position[2] >= 0:
    g[2] = earth_Gravity(position)
    drag = drag_Force(drag_Coefficient, cross_Sectional_Area, velocity, position)
    
    acceleration[2] = (thrust_Total - drag)/ mass
    acceleration[2] -= g[2]
    
    current_Readout = np.array([[str(t), str(position[2]), str(velocity[2]), str(acceleration[2])]])
    readout_Array = np.append(readout_Array, current_Readout, axis = 0)
    
    t += dt
    position[2] += (velocity[2] * dt)
    velocity[2] += (acceleration[2] * dt)
    mass -= (total_Mass_flow * dt)
    
    if mass_Propellant <= 0:
        total_Mass_Flow = 0
    else:
        pass
     
#for row in readout_Array:
    #print(" ".join(f"{col:20}" for col in row))

t_Plot = readout_Array[1:, 0].astype(float)
z_Plot = readout_Array[1:, 1].astype(float)
vz_Plot = readout_Array[1:, 2].astype(float)
az_Plot = readout_Array[1:, 3].astype(float)

plt.plot(t_Plot, z_Plot, label = "z-position")
plt.plot(t_Plot, vz_Plot, label = "z-velocity")
plt.plot(t_Plot, az_Plot, label = "z-acceleration")
plt.legend()
plt.grid()
plt.show()