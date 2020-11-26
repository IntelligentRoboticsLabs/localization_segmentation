# Localization Segmentation

## Dependencies:

It is necessary to clone the *foxy-devel* branch of the following packages:

* [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)

* [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

## Launchers:

* **tb3_sim_burger.launch.py:** Launch a TB3 Burger with RPLidar and Intel Realsense RGBD camera. Previously, specify the path to the models folder of this package like that:

```
export GAZEBO_MODEL_PATH=~/foxy_ws/src/localization_segmentation/tb3_sim/models
```

* **tb3_sim_waffle.launch.py:** launch a TB3 waffle with RPLidar and Intel Realsense RGBD camera. Previously, specify the path to the models folder of *turtlebot3_gazebo* package like that:

```
export GAZEBO_MODEL_PATH=~/foxy_ws/src/turtlebo3_simulations/turtlebot3_gazebo/models
```
