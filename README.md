BeSpoon Localization in ROS 
===== 

## Launch `BeSpoon` package  

Copy the package to `ros` workspace directory and start the launcher 

```base 
# Move to ROS workspace directory 
catkin_make  
source devel/setup.bash 
# launch bespoon localization setup 
roslaunch bespoon bespoon.launch  
```

`bespoon.launch` will do following

* Launch **BeSpoon Tracker (Java)**, (topic: `bespoon`)      
* Launch **ROS marker visualizer for BeSpoon tags & anchor** (topic: `visualization_marker_array`)    
* Launch **Simple coordinate navigation goal publisher**, (topic: `simple_coord_nav_goal`)     
* Launch **Simple coordindate publisher**, (topic: `simpleRosLocation`, `simpleBeSpoonLocation`)    

## Check publish data:    
Example:  

```bash  
rostopic echo bespoon 
rostopic echo simpleRosLocation
rostopic echo simpleBeSpoonLocation 
# check type of topic 
rostopic info simpleRosLocation 
``` 

## Send a simple navigation goal:    
Example:  use xy coordinates based on our customized XY Scale  

```bash
rostopic pub -1 simple_coord_nav_goal geometry_msgs/Pose '{position : { x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}'
``` 

## Coordinate conversion:  
Run the following script  

```bash  
python my_cord_converter.py
```

## Simple XY coordindate:  
You can set `ROS` scale factor in module `cord_transform.py` by changing constants:  `simple_xy_scale_factor`, `bespoon_plane_angle_with_ros_plane`, `bespoon_center_in_ros_plane`, `ros_axis_scale_factor`    
Default it is centered in `ROS map center (0,0)` and `x-axis` aligned.  
 

Ex:  
`cord_transform.simple_xy_scale_factor=0.20`   


## BeSpoonTracker Config

Add the location of `BeSpoonTracker.jar` and `bespoon.propteries` in launcher ([`bespoon.launch`](./launch/bespoon.launch))

```bash 
<env name="BESPOON_PROP_FILE" value="$(find bespoon)/bespoon.properties" />
<env name="BESPOON_JAR_FILE" value="$(find bespoon)/bespoon.jar" />
```

## Evaluate ROS and BeSpoon Simple location 

```bash
rosrun bespoon benchmark.py 
```