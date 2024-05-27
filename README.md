# diffdrive_roboteq
A ROS2_Control Hardware Interface for a Roboteq Motor Controller working with differential drive.

This interface is designed to provide a ros2_control hardware interface for a Roboteq Motor Controller. It is designed to be used with a diff_drive_controller from ros2_control. It is expected to communicate via serial and to have two motors, each with velocity control and position/velocity feedback.

# Getting Started
Head into {your_ros2_ws}/src and clone this repo.
Move one directory back into your ros2_ws and use ```colcon build --packages-select diffdrive_roboteq``` in order to build the package.
In order for the package to succesfully work the serial package will also have to be build. I have used the following: [serial](https://github.com/wjwwood/serial/tree/ros2?tab=MIT-1-ov-file).

# Configuration
The package needs to be correctly configured before it can be used.
In order to customize the feedback to be received from the motor controller, you can comment in/out in /config/query.yaml.

As well a ros2_control tag will have to be set up according to the following template:
```
<ros2_control name="RoboteqInterface" type="system">
            <hardware>
                <plugin>diffdrive_roboteq/DiffDriveRoboteqHardware</plugin>
                <param name="serial_port"></param> <!--string-->
                <param name="baud_rate"></param> <!--int-->
                <param name="closed_loop"></param> <!--bool-->
                <param name="wheel_radius"></param> <!--double-->
                <param name="wheel_circumference"></param> <!--double-->
                <param name="max_rpm"></param> <!--double-->
                <param name="frequency"></param> <!--int-->
                <param name="count_per_revolution"></param> <!--int-->
                <param name="gear_reduction"></param> <!--double-->  
                <param name="query_config"></param> <!--string-->
            </hardware>
            <joint name="{left_wheel_name}">
                <command_interface name="velocity"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
            </joint>
            <joint name="{right_wheel_name}">
                <command_interface name="velocity" />
                <state_interface name="position"/>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
```
