Turtlebot & BeSpoon Tracker 
===== 

## Launch `BeSpoon` package  

```base 
# Move to ROS workspace directory 
catkin_make  
source devel/setup.bash 
roslaunch bespoon bespoon.launch  
```

`bespoon.launch` will do following

* Launch **BeSpoon Tracker (Java)**, (topic: `/bespoon`)      
* Launch **ROS marker visualizer for BeSpoon tags & anchor** (topic: `visualization_marker_array`)    
* Launch **Custom navigation goal publisher**, (topic: `/custom_coord_nav_goal`)     
* Launch **Current position publisher**, (ROS & Custom coordinate topic: `/current_positions`)    

## Check publish data:    
Example:  

```bash  
rostopic echo bespoon 
rostopic echo current_positions 
``` 

## Send a navigation goal:    
Example:  use xy coordinates based on our customized XY Scale  

```bash
rostopic pub -1 custom_coord_nav_goal geometry_msgs/Pose '{position : { x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}'
``` 

## Coordinate conversion:  
Run the following script  

`python my_cord_converter.py`  

## Custom XY coordindate:  
You can set `ROS` scale factor in module `cord_transform.py` by changing constants:  `custom_xy_scale_factor`, `bespoon_plane_angle_with_ros_plane`, `bespoon_center_in_ros_plane`, `ros_axis_scale_factor`    
Default it is centered in `ROS map center (0,0)` and `x-axis` aligned.  
 

Ex:  
`cord_transform.custom_xy_scale_factor=0.20`

