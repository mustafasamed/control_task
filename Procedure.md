## Procedure

**Purpose**: To make vehicle follow the given trajectory.

**Stages**:

1 - Taking trajectory points from file and visualizing 50 of them in Rviz.

2 - Making a longitudinal controller.

4 - Visualizing the errors on PlotJuggler.

5 - Tuning lateral and longitudinal control parameters with respect to the trajectory.

**General Method**: 
	
	* This assignment was done with ChatGPT assistance. 
	* PID is used in longitudinal controller
	* PD is used in lateral controller

**Stage1** - Taking trajectory points from file and visualizing 50 of them in Rviz.:
	
 	* waypoint.txt file was read fully at first.
![image](https://github.com/mustafasamed/control_task/assets/68030580/ed5fccd6-4b11-484e-8fff-a3fecd8e4771)

	* In order to choose 50 points, the closest trajectory point should be found.
file:///home/mustafasamed/Pictures/Screenshots/Screenshot%20from%202023-07-16%2023-55-02.png
