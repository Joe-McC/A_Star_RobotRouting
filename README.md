# A_Star_RobotRouting
Implementation of a simulated robot which builds up a simple map of its environment before utilising the A* search algorithm to find its way to a pre-defined destination.

This robot contains the remApi.m and remApi.dll which are needed to interface send commands and receive sensor data from the V-Rep simulation package. An up to date version of V-Rep EDU (download from https://www.coppeliarobotics.com/), which needs to run the cwMap_VREP scene. The scene then acts as the simulation and server neccessary for the MATLAB code to send and receive data to and from.
