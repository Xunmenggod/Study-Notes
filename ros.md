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
## xacro and urdf
- link center are all located at the centorid for primitive shape such as sphere, box, and cylinder
- To convert xacro file to urdf, we could run `rosrun xacro xacro xxx.xacro > xxx.urdf
- We could add a line to launch file for automatic conversion
```xml
    <param name="robot_description" command="$(find xacro)/xacro $(find <pkg-name>)/<xacro file relatibe path to the pkg directory>"/>
```
- **Notice**: The new link center is located at the position of the joint configuration, if want offset of the center, could use the link origin element for offset
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


## ROS2 migration
- basic commands: ros2 topic ; ros2 launch ; rqt_grapah; ros2 run ; ros2 interface show <msg type> (check the data structure of the ros msg); ros2 action {list/info/send_goal} ;
- Construction of ros2 package
    1. Create a ros2 workspace with the src folder
    2. Create a ros package: ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name> --dependencies <ros2 dependencies>
    3. Build the package: colcon build; To build the selected package: colcon build --packages-select <my_package>
- Ros2 package development
    - main header file: "rclcpp/rclcpp.hpp"
    - ros2 init and create the node:  
    ```C++
        // callback
        void message_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received message: '%s'", msg->data.c_str());
        }

        rclcpp::init(argc, argv);
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_publisher");

        // publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub = node->create_publisher<std_msgs::msg::String>("topic_name", 10);
        // subscriber
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub = node->create_subscription<std_msgs::msg::String>(
        "topic_name", 10, message_callback);

        
        // spin and shutdown
        rclcpp::spin(node);
        rclcpp::shutdown();
    ```
    - Modification on the cmake list and the package.xml: In package.xml, add the dependencies including self ros2 package: 
    ```xml
        <depend>rclcpp</depend> 
        <depend>std_msgs</depend> 
    ```
    In CmakeLists:
    ```xml
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)

        add_executable(<executable file name> src/<cpp source file>)
        ament_target_dependencies(<executable file name> <dependencies>)

        install(TARGETS
            <executable file name>
            DESTINATION lib/${PROJECT_NAME})
    ```
    - custom msgs and services
    ```C++
        // service function for server
        void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                            request->a, request->b);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);

            // service server object
             rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
                node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

            // service client object
            rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
                node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

            // calling service from client
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = atoll(argv[1]);
            request->b = atoll(argv[2]);
            // 异步发送请求
            auto result = client->async_send_request(request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
            }
        }
    ```
    
- Multi-PC communication
    - Ros2 use distributed network communication dds instead of master/slave, 只要两台电脑在同一个局域网就可以通过ros2互通
    - 但是有时候局域网多个电脑运行不同的ros2的包你可以通过ROS_DOMAIN_ID决定哪些电脑需要互联, eg. export ROS_DOMAIN_ID=5, domain id最好设置为1-232