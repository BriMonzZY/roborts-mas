Mas哨兵开发手册

autor：brimonzzy







# costmap

Static Map Layer：静态地图层

Obstacle Map Layer：障碍地图层

Inflation Layer：膨胀层



[costmap_2d - ROS Wiki](http://wiki.ros.org/costmap_2d)

**Costmap_2D**提供了一种2D代价地图的实现方案，该方案利用输入传感器数据，构建数据2D或者3D代价地图

因为雷达感知的动态障碍物轮廓比实际小，容易与周围战车发生碰撞，所以加入动态障碍物层，修正动态障碍物的轮廓



`costmap_interface.cpp`中修改costmap层的叠加关系

`costmap_parameter_config_for_global_plan.prototxt` 和 `costmap_parameter_config_for_local_plan.prototxt`中修改层的参数，设置是否加入层到costmap



[机器人路径远离障碍物的方法](https://blog.csdn.net/qq_14977553/article/details/104362666)

[膨胀半径选取经验](https://blog.csdn.net/qq_41821678/article/details/125594451)



## dynamic obstacle layer

dynamic_obstacle_layer_config.prototxt：

obstacle_size 参数可以改变动态障碍物被扩大的程度



一些函数的作用：

**LaserScanCallback()**

激光雷达信息回调函数，将ROS点云信息转换为PCL格式并对点云进行聚类分割，找到中心坐标。将新的动态障碍物的轮廓更新到costmap中

再调用ObservationBuffer类的bufferCloud函数，将点云数据存到buffer中。

bufferCloud函数：当接收到sensor_msgs::PointCloud2格式的点云后，它将点云转换为pcl::PointCloud < pcl::PointXYZ >格式后，调用bufferCloud的重载函数。将origin_frame下的传感器原点（0，0，0）转换到global系下，得到传感器的坐标，并放进`bufferobservation_list_.front().origin_` 中。然后将传入的点云cloud转换到global系下，得到global_frame_cloud，接下来，开始遍历点云中的点，剔除z坐标过小或过大的点，将合乎要求的点放进buffer的observation_list_.front().cloud_。这样，便得到了经过筛选后的点云及传感器原点。

**updateBounds()**

函数主要完成：clearing、marking以及确定bound。先判断是否是rolling地图，若是则更新地图原点。

第一步：clearing_observations中的点云点执行clearing操作，即将其与传感器的连线上的点标记为FREE_SPACE

第二步：marking操作，即将点云中的点标记为障碍物。在标记时通过二重循环，外层迭代各观测轮次，内层迭代一次观测得到的点云点，剔除本身太高（z坐标过大）、与传感器距离太远的点，将符合要求的障碍点坐标从global系转换到map系，并在本层地图上标记致命障碍。并调用touch函数，确保标记的障碍点包含在 bound内。

最后调用updateFootprint函数，它的作用是基于机器人当前位置确定该位置下的足迹，并在内部调用touch函数保证足迹包含在bound范围内。

**updateCosts()**

更新障碍地图代价

函数将机器人足迹范围内设置为FREE_SPACE，并且在bound范围内将本层障碍地图的内容合并到主地图上。

**RaytraceFreespace()**

清理传感器到障碍物间的cell。先将传感器坐标转换到地图坐标系。处理越界问题，保证传感器原点在bound范围内。



## detection layer

订阅roborts_msgs::ArmorsPos类型的消息

将ArmorsPos消息中对应的位置膨胀

（暂时应该没有什么用）



## local static layer

针对机器人后方雷达扫描不到的部分加入静态地图

`UpdateCosts()`函数获取机器人坐标并添加局部静态层



## inflation layer

`inflation_radius`：调整**膨胀半径**



`cost_scaling_factor`：膨胀过程中应用到代价值的比例因子

exp(-1.0 * **cost_scaling_factor** * (distance_from_obstacle - inscribed_radius)) * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)

增大比例因子会降低代价值。





# laser_detection

## laser_detection_node

处理激光雷达点云信息，聚类得到动态障碍，并对动态障碍追踪（最近的）



发布的话题：

laser_armor：**roborts_msgs::ArmorPos**

enemies：**roborts_msgs::ArmorsPos**

dynamic_obstacle：**sensor_msgs::PointCloud2**

in_map_pc：**sensor_msgs::PointCloud2**



```shell
rosrun laser_detection laser_detection_node
```



## laser_tracking_node

处理激光雷达和相机装甲板检测的信息，得出活跃度目标和阵亡的目标



发布的话题：

laser_tracking_enemies：**roborts_msgs::ArmorsPos**

enemy_0_path：**nav_msgs::Path**

enemy_1_path：**nav_msgs::Path**



```shell
rosrun laser_detection laser_tracking_node
```



**FusionLaserCamera()**

融合激光雷达摄像头数据，维护active_cars和UpdateDiedCar

被ReceiveEnemies()调用，属于enemies话题的订阅者的回调函数



**UpdateCameraBuffer()**

属于armors_camera话题的订阅者的回调函数

维护 camera_history_buffer_ 列表



## cost_map_node

订阅costmap的消息并使用cv库画出来

```shell
 rosrun laser_detection cost_map_node
```

