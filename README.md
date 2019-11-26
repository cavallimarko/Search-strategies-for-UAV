# Search-strategies-for-UAV
Installation Instructions
-------------------------
Prerequisites:
ROS Melodic

https://github.com/larics/mmuav_gazebo/ (branch mbzirc-test)

https://github.com/larics/larics_gazebo_worlds  (branch mbzirc-test)

https://github.com/larics/topp_ros (branch master)

Basic Usage
-----------

1.Launch the simulator with a UAV.

```
$ roslaunch uav_search test.launch ->uav gazebo simulation
$ cd ~/catkin_ws/src/uav_search/src
$ python main_spiral.py ->trajectory generator

```
2.Turn on object detection and spawn boxes.

```
$ cd ~/catkin_ws/src/uav_search/src
$ python detect_cv_bridge.py ->object detection
```
3.Generate trajectory(spiral or stochastic).

```
$ cd ~/catkin_ws/src/uav_search/src
$ python main_spiral.py ->trajectory generator in spiral shape
$ python main_random.py ->stochastic trajectory generator
```
4.Send trajectory in trajectory generator window and start the search.

5. Monitor results in object detection window.
