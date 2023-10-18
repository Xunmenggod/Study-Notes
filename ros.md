# Note about ROS
## catkin 
- `catkin_create_pkg <package name> [dependencies, eg.roscpp rospy std_msgs]`
- catkin_make / catkin build [packages]


## ROS init & NodeHandle
- Specific namespace for the node handle by `ros::NodeHandle nh("my_namespace")`, then the node handle will be located at <node_namespace>/my_namespace
- Private namespace: `ros::NodeHandle priNh('~')`
- Way to analyse the node handle
???

## Launch file
- node opening, eg.
```xml
<launch>
    <node name="ros init node name" pkg="my_package_name" type="executable file name" output="screen/" ns="my node namespace"/>
```
- include command: it could help import another launch file into current one, eg. `<inlcude file="$(find <my_package>/launch/previous_file.launch)"/>`
- param
`<param name="my_param" type="int value="666"/>`, it will be stored at the parameter server
Another one is rosparam which could help import massive parameters from a YAML file, eg. `<rosparam command="load" file="$(my_package)/config/parameters.yaml"/>`
- arg
it could change the parameters in a flexible way from the terminal, eg. `<arg name="arg_name" default="666"/>`. To change the value of arg_name via terminal command by `roslaunch pkg_name launch_file.launch arg_name:=777`
example of change the node parameter `<param name="my_param" value="$(arg arg_name)"/>`
- group
It could help with different launch effect based on different situation, eg. 
```xml
<arg name="boolean_expression" default="true"/>
<group if="$(arg boolean_expression)">
    <node name="true_node" type="true_file" pkg="my_package"/>
    ...
<group/>
<group unless="$(arg boolean_expression)">
    <node name="false_node" type="false_file" pkg="my_package"/>
    ...
    <group/>
```
## Gazebo
- roslaunch with gazebo
```xml
<!--include the launch file for empty.world-->
<include file="$(find gazebo_ros)/launch/empty_world.launch"/>
<!--spawn model-->
<node name="<prefered_name>" pkg="gazebo_ros" type="spawn_model">
    <args="-m model (-urdf | -sdf) (-param -file: how to load model xml description) -x -y -z -R -P -Y">
</node>
```

## ROS Control


## Export Ros Plugin Lib



