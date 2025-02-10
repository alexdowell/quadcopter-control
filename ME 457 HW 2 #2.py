# -*- coding: utf-8 -*-
"""
Created on Sun Sep  5 20:12:27 2021

@author: alex
"""
import numpy as np

# Converts Radians to degrees
Rad2Deg = 180.0/3.14

# Two sensor components
SensorX = np.array([2.24, 2.01, 1.63, 1.21, 0.99, 0.56, 0.19, 0.0])
SensorZ = np.array([7.63, 7.89, 8.11, 8.54, 8.97, 9.11, 9.66, 9.81])

# Length of array
#L = len(SensorX)

# Create variable to store angle data
#Angle = np.zeros(L) 

# Estimate angle from sensors
#for i in range(0,L):
Angle = np.arctan2(SensorX, SensorZ) * Rad2Deg
    
# Display results
print (Angle)
    

