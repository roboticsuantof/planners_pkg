# planners_pkg
This package provide a severals of algorithm implemented to use it in ROS.

The algorithm are inspired in the Driving in Point Cloud philosophy (Driving on Point Clouds: Motion Planning, Trajectory Optimization, and Terrain Assessment in Generic Nonplanar Environments, 2016, Krüsi, Philipp and Furgale, Paul and Bosse, Michael and Siegwart, Roland). So, not costmap are use, only distance to obstacles in Point Cloud.

The planner can used segmentated Point Cloud to compute solution. The Node in charge for segmentatios is able in : https://github.com/roboticsuantof/maps_manager_pkg. 

## Steps for use

To use the planner, consider the launch ```planner.launch``` as follow:

```
roslaunch planner_pkg planner.launch
```

IMPORTANT: if is the first time that you execute de map, a grid will star to compute.

Everytime that the launch is use, a Trilination Interpolation is executed. Wait till the computation is done.

Then use the rosservice ```/planner_node/get_algorithm```. For example, if the start position is (-25.0, -40.0, -3.0) and the goal point (-25.0,-40.0,-3.0), the service is:

```
rosservice call /planner_node/get_algorithm "start:
  x: -25.0
  y: -40.0
  z: -3.0
goal:
  x: -7.0
  y: 18.0
  z: 0.0" 
```
The path information is publisher in ```/planner_node/computed_path``` and the path markers is publishe rin ```/planner_node/path_marker_planner```.

