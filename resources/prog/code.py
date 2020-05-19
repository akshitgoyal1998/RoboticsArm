## -*- coding: utf-8 -*-
#"""
#Created on Sat May  2 15:23:40 2020
#
#@author: Dell
#"""
#
#import pyfabrik
#from vectormath import Vector3
#
#initial_joint_positions = [Vector3(12, 0, 17), Vector3(12, 0, 75), Vector3(36, 0, 203), Vector3(160, 0, 203),Vector3(213, 0,203)]
#tolerance = 1
#
## Initialize the Fabrik class (Fabrik, Fabrik2D or Fabrik3D)
#fab = pyfabrik.Fabrik3D(initial_joint_positions, tolerance)
#print(fab.move_to(Vector3(100, 23, 250)) )# Return 249 as number of iterations executed
#print(fab.angles_deg) # Holds [43.187653094161064, 3.622882738369357, 0.0]
import pyfabrik
from vectormath import Vector3

initial_joint_positions = [Vector3(12, 0, 17), Vector3(12, 0, 75), Vector3(36, 0, 203), Vector3(160, 0, 203),Vector3(213, 0,203)]
tolerance = 0.01

# Initialize the Fabrik class (Fabrik, Fabrik2D or Fabrik3D)
fab = pyfabrik.Fabrik3D(initial_joint_positions, tolerance)

fab.move_to(Vector3(30, 10, 23))
fab.angles_deg # Holds [0.0, 0.0, 0.0]

print(fab.move_to(Vector3(60, 0, 60)) )# Return 249 as number of iterations executed
print(fab.angles_deg )# Holds [43.187653094161064, 3.622882738369357, 0.0]