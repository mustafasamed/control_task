## Procedure

**Purpose**: To make vehicle follow the given trajectory.

**Stages**:

1 - Taking trajectory points from file and visualizing 50 of them in Rviz.

2 - Making a lateral controller.

3 - Making a longitudinal controller.

4 - Visualizing the errors on PlotJuggler.

5 - Tuning lateral and longitudinal control parameters with respect to the trajectory.

**General Method**: 
	
	* This assignment was done with ChatGPT assistance. 
	* PID is used in longitudinal controller
	* PD is used in lateral controller

**Stage1** - Taking trajectory points from file and visualizing 50 of them in Rviz.:
	
 	* waypoint.txt file was read fully at first.
![image](https://github.com/mustafasamed/control_task/assets/68030580/ed5fccd6-4b11-484e-8fff-a3fecd8e4771)

	* In order to choose 50 points, the closest trajectory point should be found. For this a function was created.
![Screenshot from 2023-07-16 23-55-02](https://github.com/mustafasamed/control_task/assets/68030580/addb7d93-ad77-4a43-ad58-37f73ceb540c)

	* Now, we can choose 50 points beginning from the position of the vehicle.(This part is onTimer() function and repeated for every 30ms) 
 ![Screenshot from 2023-07-17 00-00-19](https://github.com/mustafasamed/control_task/assets/68030580/0c66723b-192b-426e-8f30-2429eefadd50)

 	* 

