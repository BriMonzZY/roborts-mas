# roborts_localization



[roborts_localization 参数](https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/roborts_localization?id=%e7%9b%b8%e5%85%b3%e5%8f%82%e6%95%b0)

UWB矫正参数？



roborts_localization模块**订阅：**

/map

/tf(odom->base_link)

/initialpose

/uwb

broborts_localization模块**发布：**

/amcl_pose

/particlecloud

/tf(odom->map)



## AMCL



AMCL订阅 **base_link->odom** 的tf消息，计算odom和map间的差距作为odometry的累计误差

/tf信息以及/odom信息由仿真器或者base模块提供



### 参数

| 参数名                    | 作用                                                         | 备注 |
| ------------------------- | :----------------------------------------------------------- | ---- |
| use_map_topic             | 订阅map主题而不是服务                                        |      |
| first_map_only            | 仅使用第一次订阅到的map                                      |      |
| gui_publish_rate          | 可视化时信息发布的最高频率                                   |      |
| laser_min_range           | 激光雷达的最小有效测距距离                                   |      |
| laser_max_range           | 激光雷达的最大有效测距距离                                   |      |
| laser_max_beams           | 激光雷达的波束数(更新过滤器时，在每次扫描中使用多少均匀间隔的光束) |      |
| min_particles             | 粒子滤波器的最小粒子数                                       |      |
| max_particles             | 粒子滤波器的最大粒子数                                       |      |
| kld_err                   | 真实分布和估计分布之间的最大误差                             |      |
| kld_z                     | (1-p)的标准正态分布的上分位数。p是估计误差小于kld_err的概率  |      |
| laser_model               | 激光雷达测距仪模型。(roborts目前只支持似然域改进模型likelihood_field_prob) |      |
| z_hit                     | 似然域模型中的$z_{hit}$参数                                  |      |
| z_rand                    | 似然域模型中的$z_{rand}$参数                                 |      |
| sigma_hit                 | 似然域模型中的$\sigma_{hit}$参数                             |      |
| lambda_short              | 模型z_short部分的指数衰减参数                                |      |
| laser_likelihood_max_dist | 似然域模型中的测距点与障碍物间最大距离                       |      |
| **do_beamskip**           | Position Tracking阶段忽略部分激光波束，以避免不可预料的误差，比如动态物体等等（设置要求跳过的光束百分比） |      |
| beam_skip_distance        | 忽略波束的障碍物距离阈值（用于设置考虑跳过的最大距离即障碍物激光点栅格坐标与地图障碍物最近距离阈值（单位为m）。会忽略掉似然场模型中大多数粒子与地图不一致的光束） |      |
| beam_skip_threshold       | 波束忽略的阈值（设置要求跳过的光束百分比）                   |      |
| beam_skip_error_threshold | 波束忽略的错误阈值(由于收敛不良而未匹配地图以强制完全更新后的光束百分比) |      |
| odom_model                | 机器人运动模型。roborts目前只支持全向轮里程计模型（ODOM_MODEL_OMNI） |      |
| odom_alpha1               | 里程计模型的误差参数（基于机器人运动的旋转分量指定里程计旋转估计中的预期噪声） |      |
| odom_alpha2               | 里程计模型的误差参数（基于机器人运动的平移分量指定里程计旋转估计中的预期噪声） |      |
| odom_alpha3               | 里程计模型的误差参数（基于机器人运动的平移分量指定里程测量平移估计中的预期噪声） |      |
| odom_alpha4               | 里程计模型的误差参数（基于机器人运动的旋转分量指定里程计平移估计中的预期噪声） |      |
| odom_alpha5               | 平移相关噪声参数（仅在模型omni中使用）                       |      |
| update_min_d              | 滤波器更新的位移阈值                                         |      |
| update_min_a              | 滤波器更新的旋转阈值                                         |      |
| resample_interval         | 重采样周期                                                   |      |
| transform_tolerance       | tf发布间隔                                                   |      |
| recovery_alpha_slow       | Augmented_MCL中的$\alpha_{slow}$参数（用于通过添加随机姿势来决定何时恢复操作的慢速平均权重滤波器的指数衰减率） |      |
| recovery_alpha_fast       | Augmented_MCL中的$\alpha_{fast}$参数（用于决定何时通过添加随机姿势恢复操作的快速平均权重过滤器的指数衰减率） |      |
| use_global_localization   | 是否初始随机初始定位                                         |      |
| random_heading            | 是否初始随机角度的初始定位                                   |      |
| laser_filter_weight       | 权重阈值，用于筛选出权重较低的激光雷达测量值                 |      |
| max_uwb_particles         | 重采样阶段，以UWB为均值的最大重采样数                        |      |
| uwb_cov_x                 | 重采样阶段，以UWB为均值的高斯分布的方差x                     |      |
| uwb_cov_y                 | 重采样阶段，以UWB为均值的高斯分布的方差y                     |      |
| **resample_uwb_factor**   | 重采样因子，用于判断对称定位                                 |      |



## imu_tools

[imu_tools - ROS Wiki](http://wiki.ros.org/imu_tools?distro=noetic)

### imu_filter_madgwick

[imu_filter_madgwick wiki](http://wiki.ros.org/imu_filter_madgwick)

[imu过滤和odom融合 blog](https://www.codeleading.com/article/67504287797/)

将来自常规IMU设备的角速度，加速度和磁力计读数（可选）融合成方向数据。只需要输入6轴或者9轴的原始数据即可

安装部署：

```shell
sudo apt-get install ros-noetic-imu-tools
```



订阅：

imu/data_raw（[sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)）

imu/mag（[sensor_msgs/MagneticField](http://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html)）（可选）

发布：

imu/data（[sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)）



#### 参数

orientation_stddev：方向估计的标准差

fixed_frame：tf的父节点

publish_tf：是否发布tf来表示imu的方向，使用 fixed_frame 中指定的框架作为父框架，并将输入 imu 消息中给出的帧作为子帧





启动imu_filter_madgwick节点打开rviz界面中的*[rviz_imu_plugin](http://wiki.ros.org/rviz_imu_plugin?distro=noetic)*插件可以可视化imu状态



## robot_localization

### ekf_localization_node

[ekf-localization-node wiki](https://docs.ros.org/en/lunar/api/robot_localization/html/state_estimation_nodes.html#ekf-localization-node)

[blog](https://www.guyuehome.com/7701)

[github](https://github.com/cra-ros-pkg/robot_localization)

安装部署：

```shell
sudo apt-get install ros-noetic-geographic-msgs -y
sudo apt-get install ros-noetic-geographic-* -y
sudo apt-get install geographiclib-* libgeographic-* -y
sudo apt install ros-noetic-robot-localization -y
# git clone -b noetic-devel https://github.com/cra-ros-pkg/robot_localization.git
```

扩展卡尔曼滤波器的实现。它使用全向运动模型及时向前投射状态，并使用感知的传感器数据校正该投射估计



发布：

odometry/filtered ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))



#### 参数

[参数介绍翻译](https://blog.csdn.net/learning_tortosie/article/details/103346412)

**fram**：如果仅融合连续位置数据（例如车轮编码器里程计，视觉里程计或IMU数据），则将**world_frame设置为odom_frame的值**。这是robot_localization中状态估计节点的默认行为，也是**最常见**的用法。map_frame，odom_frame和base_link_frame的默认值分别是map，odom和base_link。base_link_output_frame参数默认为base_link_frame的值。world_frame参数默认为odom_frame的值。（base_footprint是base_link在地面的投影）

**fram**：如果仅融合连续位置数据（例如车轮编码器里程计，视觉里程计或IMU数据），则将`world_frame`设置为`odom_frame`的值。这是`robot_localization`中状态估计节点的默认行为，也是最常见的用法。`map_frame`，`odom_frame`和`base_link_frame`的默认值分别是`map`，`odom`和`base_link`。`base_link_output_frame`参数默认为`base_link_frame`的值。`world_frame`参数默认为`odom_frame`的值。（base_footprint是base_link在地面的投影）



详细参数在`robot_localization.yaml`文件中配置



launch文件：

```launch
<node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization" output="screen">
    <remap from="odometry/filtered" to="odom"/>
    <param name="imu0" value="/imu/imu_data"/>
    <param name="odom0" value="odom_raw"/>
    <param name="odom_frame" value="/odom"/>
    <param name="world_frame" value="/odom"/>
    <param name="base_link_frame" value="/base_link"/>
    <rosparam command="load" file="$(find bringup)/param/robot_localization.yaml"/>
</node>
```







## rplidar_ros

[wiki](http://wiki.ros.org/rplidar)



