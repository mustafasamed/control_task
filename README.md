## How to run simulator

* After you completed the steps in setup section, source the workspace and run:
`$ ros2 launch tier4_simulator_launch simulator.launch.xml vehicle_model:=sample_vehicle map_path:=src/task_map
`

**FYI**: You need to modify the path of the task_map file.

* To initialize the vehicle run this command in another terminal:
```python
  $ ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  stamp:
    sec: 1685222164
    nanosec: 844445475
  frame_id: map
pose:
  pose:
    position:
      x: 59201.94921875
      y: 43143.6171875
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7534506808962064
      w: 0.6575044269486275
  covariance:
  - 0.25
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.25
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.06853891909122467"
```

* After that you should see the car and the map on Rviz