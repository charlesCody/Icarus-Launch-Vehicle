import numpy as np 
import matplotlib.pyplot as plt 

class Icarus_Interface:
    def __init__(self, readout_Array):
        self.readout_Array = readout_Array
        
    def kinematic_plot(self,readout_Array):
        #Creating Telemetry Arrays for Plot
        t_Plot = readout_Array[1:, 0].astype(float)
        z_Plot = readout_Array[1:, 1].astype(float)
        vz_Plot = readout_Array[1:, 2].astype(float)
        az_Plot = readout_Array[1:, 3].astype(float)

        #Plotting Telementry
        #plt.plot(t_Plot, z_Plot, label = "z-position")
        plt.plot(t_Plot, vz_Plot, label = "z-velocity")
        plt.plot(t_Plot, az_Plot, label = "z-acceleration")
        plt.legend()
        plt.grid()
        plt.show()