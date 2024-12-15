

The repo contains the software for the Adaptive Multi-Robot Coordination platform Multi-3. It is a ROS2 workspace that contains the following packages:
* `multi3_coordinator`: The mission controller that orchestrates the execution of the mission and sends the mission fragments to the executor robots.
* `multi3_executor`: This package contains the node that should be run in each robot involved in the mission, it should receive the mission fragments sent by the mission controller and execute them.
* `multi3_interfaces`: This package implements the interfaces utilized by the mission controller to send the mission fragments and communicate with the robots.
* `multi3_tests`: Run automated tests by spawning different configuration and mission specifications and evaluate the framework.

## To perform the tests
* Edit the test configuration in `src/multi3_tests/config/test_config.json` to adjust the mission size, robot count and other parameters about the tests.
* Execute `ros2 run multi3_tests generate` to generate all the test specifications.
* Run the tests automatically with `python3 run_tests.py` (This script will look for the test files generated with the previous command, that are stored in `src/multi3_tests/multi3_tests/tests`)

### To run one particular test
* Execute `ros2 launch multi3_tests test.launch.py test_id:=<test_name>`


## Real Robot Navigation with Multi-3
For starting the system, one must start the navigation nodes for each of the executor robots:
[Changing the namespace variable to match the robot]
```
ros2 launch turtlebot4_navigation load_nav.launch.py namespace:=/Turtlebot_02490 map:=/home/gssi-lab/navigation/lab_empty.yaml
```

```
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/Turtlebot_02490
```

