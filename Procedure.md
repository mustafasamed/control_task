Procedure

Purpose: To make vehicle follow the given trajectory.

Stages:

1 - Taking trajectory points from file and visualizing 50 of them in Rviz.

2 Making a longitudinal controller.

4- Visualizing the errors on PlotJuggler.

5- Tuning lateral and longitudinal control parameters with respect to the trajectory.

General Method: 
	
	* This assignment was done with ChatGPT assistance. 
	* PID is used in longitudinal controller
	* PD is used in lateral controller

Stage1-Taking trajectory points from file and visualizing 50 of them in Rviz.:

* #include <fstream> was added.
	=>this is to include waypoints.txt file into controller.cpp. This makes a txt file is read or written by the controller.cpp.

* waypoint.txt file was read fully at first.

