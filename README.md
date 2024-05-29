# diffdrive_roboteq
A ROS2_Control Hardware Interface for a Roboteq Motor Controller designed for differential drive systems.

## Overview
This package provides a ROS2 Control hardware interface for interacting with a Roboteq Motor Controller. It is specifically tailored for use with the `diff_drive_controller` from `ros2_control`. The interface is configured to communicate via serial connection and assumes the presence of two motors, each capable of velocity control with position/velocity feedback. This hardware interface has been tested with [SBLMG2360T Roboteq Motor Controller](https://www.roboteq.com/products/products-brushless-dc-motor-controllers/sblm2360t-452-detail).

## Getting Started
To begin using `diffdrive_roboteq`, follow these steps:

1. Clone this repository into your ROS 2 workspace:
   ```
   cd {your_ros2_ws}/src
   git clone https://github.com/KNinteman/diffdrive_roboteq.git
   ```
2. Navigate to your ROS 2 workspace root directory and build the package:
   ```
   cd ..
   colcon build --packages-select diffdrive_roboteq
   ```
3. Ensure that the serial package is also built. You can use the following link for reference: [serial](https://github.com/wjwwood/serial/tree/ros2?tab=MIT-1-ov-file).

## Configuration
Before using the package, it needs to be properly configured. Here are some steps to consider:

- Customize the feedback received from the motor controller by editing the /config/query.yaml file.
- Set up a ros2_control tag in your robot's URDF according to the template provided below:
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
Ensure that you fill in the necessary parameters such as serial_port, baud_rate, etc., based on your specific hardware setup.

## Troubleshooting
If you encounter any issues during installation or configuration, consider the following troubleshooting tips:

Double-check your serial communication settings to ensure they match those specified in the configuration files.
Verify that all dependencies, including the serial package, are properly installed and built.
Refer to the ROS 2 documentation or community forums for additional support if needed.

## Acknowledgments
I would like to acknowledge the contributions of the ROS 2 community and the developers of the ros2_control and serial packages for their support.
