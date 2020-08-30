#简介[参考Wiki](http://wiki.ros.org/yocs_velocity_smoother)

ROS导航模块move_base输出的/cmd_vel topic指定了为机器人规划的线速度和角速度, 但是这个输出值还是不够友好导致机器人运动不够流畅，
这就需要对这个输出速度值进行一个平滑的过程。ROS中的yocs_smoother_velocity是一个非常好的速度插值的包, 可以对速度、加速度进行限
制，用来防止机器人的速度、转速变化过快或过慢, 使其运行平滑流畅。
========

# 输入的TOPIC
```
~raw_cmd_vel (geometry_msgs/Twist)
```
Input velocity commands. 输入的速度值，一般就是move_base的输出topic /cmd_vel
```
~odometry (nav_msgs/Odometry)
```
We compare the output velocity commands to "real" velocity to ensure we don't create very big jumps in the
velocity profile.里程计数据，这个可以是直接的里程计，也可以是经过位姿估计调整后的里程计。
```
~robot_cmd_vel (geometry_msgs/Twist)
```
Alternatively, we can also compare the output velocity commands to end robot velocity commands to ensure we
don't create very big jumps in the velocity profile. See robot_feedback parameter description below for more
details.这个数据一般是base_controller实际发送给机器人电机的速度值，把这个值发布出来，yocs_smoother_velocity可以参考这个值
避免大的数据波动。
========

# 输出的TOPIC
```
~smooth_cmd_vel (geometry_msgs/Twist)
```
Smoothed output velocity commands respecting velocity and acceleration limits.
输出的cmd_vel.  Bas_ controller 的输入topic。不使用yocs_smoother_velocity的话，base controller的输入是move_base的
输出/cmd_vel, 现在就是用/smooth_cmd_vel了。

3 配置参数
```
* accel_lim_v (double) Linear acceleration limit. Mandatory. 线加速度的最大值
* accel_lim_w (double) Angular acceleration limit. Mandatory. 角加速度的最大值
* speed_lim_v (double) Linear velocity limit. Mandatory. 线速度的最大值
* speed_lim_w (double) Angular velocity limit. Mandatory. 角速度的最大值
* decel_factor (double, default: 1.0)  Deceleration/acceleration ratio. Useful to make deceleration more
aggressive, for example to safely brake on robots with high inertia.减或加速度系数，需要紧急减速时有用，例如高惯性的
机器人急刹车时，此系数越大，越有效。
* frequency (double, default: 20.0) Output messages rate. The velocity smoother keeps it regardless incoming
messages rate, interpolating whenever necessary.输出数据的频率，不管输入的数据频率，velocity smoother会保持这个频率
发布数据
* robot_feedback (int, default: 0)  Specifies which topic to use as robot velocity feedback (0 - none, 1 - 
odometry, 2 - end robot commands). 机器人对此发布数据的反馈
```
# yocs_velocity_smoother的安装与配置
```
sudo apt-get install ros-kinetic-yocs_velocity_smoother
```
Launch 文件的配置例子如下:
```
<include file="$(find yocs_velocity_smoother)/launch/velocity_smoother.launch">
    <arg name="node_name"             value="$(arg node_name)"/>
    <arg name="nodelet_manager_name"  value="$(arg nodelet_manager_name)"/>
    <arg name="config_file"           value="$(arg config_file)"/>  //配置参数的yaml文件
    <arg name="raw_cmd_vel_topic"     value="$(arg raw_cmd_vel_topic)"/>
    <arg name="smooth_cmd_vel_topic"  value="$(arg smooth_cmd_vel_topic)"/>
    <arg name="robot_cmd_vel_topic"   value="$(arg robot_cmd_vel_topic)"/>
    <arg name="odom_topic"            value="$(arg odom_topic)"/>
</include>
```
配置参数是yaml文件格式:
```
    # Example configuration:
    # - velocity limits are around a 10% above the physical limits
    # - acceleration limits are just low enough to avoid jerking
     
    # Mandatory parameters
    speed_lim_v: 0.6
    speed_lim_w: 0.5
     
    accel_lim_v: 0.1
    accel_lim_w: 0.25
     
    # Optional parameters
    frequency: 20.0
    decel_factor: 20.0
     
    # Robot velocity feedback type:
    #  0 - none
    #  1 - odometry
    #  2 - end robot commands
    robot_feedback: 0
    ```
    
