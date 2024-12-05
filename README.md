For starting the simulation, one must start the navigation nodes for each of the executor robots:
[Changing the namespace variable to match the robot]
```
ros2 launch turtlebot4_navigation load_nav.launch.py namespace:=/Turtlebot_02490 map:=/home/gssi-lab/navigation/lab_empty.yaml
```

```
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/Turtlebot_02490
```


