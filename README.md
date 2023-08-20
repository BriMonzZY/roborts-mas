# RobotRTS_Mas

**Mas哨兵开发框架文档**



相关链接：

> RoboRTS文档：https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/architecture
>
> RoboRTS仓库：https://github.com/RoboMaster/RoboRTS
>
> 嵌入式端仓库：https://github.com/RoboMaster/RoboRTS-Firmware
>
> roborts_base模块最新仓库：https://github.com/RoboMaster/RoboRTS-Base
>
> 中科院自动化所rmua：https://github.com/DRL-CASIA/RMAI2020-Planning
>
> 中科院自动化所rmua：https://github.com/DRL-CASIA/RMAI2020-Perception
>
> 中科院自动化所rmua：https://github.com/DRL-CASIA/RMAI2020-Decision
>
> 哈工深ruma：https://github.com/Critical-HIT-hitsz/RMUA2022
>
> gazebo仿真：https://github.com/nwpu-v5-team/ICRA-Firefly-Emulator





## 整体架构

RobotRTS_Mas基于 [RoboRTS](https://github.com/RoboMaster/RoboRTS) ，运行于ros-noetic

整个工程使用**抽象工场模式**来构建

下面是框架包含的软件包和对应的功能以及依赖的软件包

| Package              | 功能               | 内部依赖                                                     |
| -------------------- | ------------------ | ------------------------------------------------------------ |
| roborts              | Meta-package       | -                                                            |
| roborts_base         | 嵌入式通信接口     | roborts_msgs                                                 |
| roborts_camera       | 相机驱动包         | roborts_common                                               |
| roborts_common       | 通用依赖包         | -                                                            |
| roborts_decision     | 机器人决策包       | roborts_common roborts_msgs roborts_costmap                  |
| roborts_detection    | 视觉识别算法包     | roborts_msgs roborts_common|
|  roborts_camera                   |||
| roborts_localization | 机器人定位算法包   | -                                                            |
| roborts_costmap      | 代价地图相关支持包 | roborts_common                                               |
| roborts_msgs         | 自定义消息类型包   | -                                                            |
| roborts_planning     | 运动规划算法包     | roborts_common roborts_msgs roborts_costmap                  |
| roborts_bringup      | 启动包             | roborts_base roborts_common roborts_localization roborts_costmap roborts_msgs roborts_planning |
| roborts_tracking     | 视觉追踪算法包     | roborts_msgs                                                 |



## RoboRTS_Mas依赖环境和编译工具

- Ubuntu 20.04LTS
- ROS-noetic

- OpenCV 4.2.0**(一定要4.2.0)**：https://github.com/opencv/opencv/archive/4.2.0.zip

直接执行下面的命令以安装依赖：

```shell
sudo apt-get install -y ros-noetic-cv-bridge           \
                        ros-noetic-image-transport     \
                        ros-noetic-stage-ros           \
                        ros-noetic-map-server          \
                        ros-noetic-laser-geometry      \
                        ros-noetic-interactive-markers \
                        ros-noetic-tf                  \
                        ros-noetic-pcl-*               \
                        ros-noetic-libg2o              \
                        ros-noetic-rplidar-ros         \
                        ros-noetic-rviz                \
                        ros-noetic-amcl			   	   \
                        ros-noetic-navigation		   \
                        ros-noetic-usb-cam*			   \
                        ros-noetic-image-view		   \
                        ros-noetic-teleop-twist-keyboard \
                        python3-catkin-tools		   \
                        protobuf-compiler              \
                        libprotobuf-dev                \
                        libsuitesparse-dev             \
                        libgoogle-glog-dev             \
                        libeigen3-dev				   \
                        openni2-utils				   \
                        libpcap-dev					   \
                        liburdfdom-tools
```

**使用catkin_tools工具编译：(注意不是使用catkin_make工具)**

```shell
sudo apt-get install python3-catkin-tools -y

# 常用命令
catkin build # 编译
catkin build package_name # 编译指定包
catkin clean # 清除build文件
catkin clean package_name # 清除指定build文件
```

在启动节点前务必执行：（或放到~/.bashrc中）

```shell
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
```



## 什么是ActionLib

[Action Lib wiki](http://wiki.ros.org/actionlib)

ActionLib是类似于service的问答通讯机制，适用于控制机器人运动到地图中某一目标位置一类的任务

ActionClient和ActionServer之间使用action protocol通信

ROS Messages：goal、cancel、status、feedback、result



## roborts_base

**使用最新开源的base模块** [RoboRTS-Base](https://github.com/RoboMaster/RoboRTS-Base)



**Module**是父类，定义了一个虚函数Module，和一个Handle类类型的智能指针handle_

sdk中有自己的协议命令订阅和发布结构



使用base.launch文件启动base模块，调用base.yaml，可以在base.yaml中修改参数



---

### 协议

协议的代码描述在`protocol.cpp`和`protocol.c`中

协议内容和数据包内容在`protocol_content.h`中定义

协议包含：**帧头数据 + 命令码ID + 数据 + 帧尾校验数据**

> 协议数据按照通信方式分为
>
> - 底层发送给上层的数据：
>
>   1 反馈信息：包含各个机构传感器反馈信息、底层计算出来的一些反馈信息；
>
>   2 底层状态信息：包含底层设备运行状态、底层对上层数据的一些响应等；
>
>   3 转发数据：包含裁判系统的全部信息、服务器端的自定义信息；
>
> - 底层接收的上层数据：
>
>   1 控制信息：上层对底层 3 个执行机构的控制信息；

#### 帧头数据

```c++
typedef struct Header {
  uint32_t sof : 8;
  uint32_t length : 10;
  uint32_t version : 6;
  uint32_t session_id : 5;
  uint32_t is_ack : 1;
  uint32_t reserved0 : 2; // Always 0
  uint32_t sender: 8;
  uint32_t receiver: 8;
  uint32_t reserved1 : 16;
  uint32_t seq_num : 16;
  uint32_t crc : 16;
} Header;
```

| 帧头数据     | 占用字节 | 描述                     | 备注                          |
| ------------ | -------- | ------------------------ | ----------------------------- |
| SOF          | 1        | 数据的域ID               | 0xAA                          |
| ver_data_len | 2        | 每帧数据长度和协议版本号 | length、version               |
| session      | 1        | 包序号                   | session_id、is_ack、reserved0 |
| sender       | 1        | 发送者地址               |                               |
| receiver     | 1        | 接收者地址               |                               |
| res          | 2        | 保留位                   |                               |
| seq          | 2        | 包序号                   |                               |
| crc16        | 2        | 帧头的CRC16校验结果      |                               |

帧头在一个数据包中占用16个字节



#### 命令码

命令码包含了一帧的具体信息数据

```c++
uint8_t cmd_set_prefix[] = {cmd_id, cmd_set};
```

| 命令码 | 占用字节 |
| ------ | -------- |
| cmdid  | 2        |

**（下面的表格应该是错误的，来自官方）**

| 命令码 | 传输方向  | 功能介绍               | 频率               |
| ------ | --------- | ---------------------- | ------------------ |
| 0x0001 | 主控-->PC | 比赛时机器人状态       | 裁判系统10Hz       |
| 0x0002 | 主控-->PC | 实时伤害数据           | 受到攻击时发送     |
| 0x0003 | 主控-->PC | 实时射击数据           | 裁判系统           |
| 0x0004 | 主控-->PC | 实时功率、热量数据     | ICRA不使用，不发送 |
| 0x0005 | 主控-->PC | 场地 RFID 数据         | 检测到 IC 卡发送   |
| 0x0006 | 主控-->PC | 比赛结果数据           | 比赛结束时发送     |
| 0x0007 | 主控-->PC | 获得 buff 数据         | 裁判系统           |
| 0x0008 | 主控-->PC | 场地 UWB 数据          | 裁判系统           |
| 0x0204 | 主控-->PC | 机器人底盘相关信息     | 100Hz定频          |
| 0x0304 | 主控-->PC | 机器人云台相关信息     | 100Hz定频          |
| 0x0402 | 主控-->PC | UWB相关信息            | UWB更新频率        |
| 0x0206 | PC-->主控 | 设置底盘速度           |                    |
| 0x0208 | PC-->主控 | 设置底盘速度(有加速度) |                    |
| 0x0309 | PC-->主控 | 控制摩擦轮转速         | 开启摩擦轮使用     |
| 0x030A | PC-->主控 | 控制射击               |                    |
| 0x0403 | PC-->主控 | 云台相关校准信息       | 需要校准云台时发送 |



正确的表格（来自代码）：

| 命令码 | cmd_set | cmd_id | 传输方向  | 功能介绍                 | 频率  |
| ------ | ------- | ------ | --------- | ------------------------ | ----- |
| 0x0001 | 0x00    | 0x01   | PC-->主控 | ROS运行心跳              | 300ms |
| 0x0002 | 0x00    | 0x02   | PC-->主控 | SDK初始化发布version信息 | 300ms |





#### 数据

| 数据 | 占用字节    |
| ---- | ----------- |
| data | data_length |

##### 裁判系统学生串口信息

查看裁判系统手册

##### 控制信息与推送信息

`protocol_content.h`

从机接收：`RoboRTS-Firmware/application/infantry_cmd.h`

主机发送：`roborts_base/roborts_sdk/protocol/protocol_define.h`

`/roborts_base/roborts_sdk/include/protocol_content.h`



#### 校验数据

对一帧的数据进行CRC32校验

| 校验数据 | 占用字节 |
| -------- | -------- |
| crc_data | 4        |



---

### roborts_sdk

硬件层

协议层





## roborts_localization

得到机器人在地图中坐标系中的坐标

框架默认使用amcl算法来做定位

如果使用amcl算法，可以直接使用ros提供的amcl包来实现

```shell
sudo apt install ros-noetic-amcl -y
```



### AMCL自适应蒙特卡洛定位





## roborts_camera

[文档](https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/roborts_camera)

本软件包为相机驱动，发布图像数据以及相机信息

启动节点：

```shell
rosrun roborts_camera roborts_camera_node
```

在`config/camera_param.prototxt`中修改相机参数



## roborts_detection

检测装甲板

编译`catkin build roborts_detection`

### 图像输入说明



可以直接使用ros的usb_cam包调用usb摄像头，而**不使用roborts_camera发布摄像头图像信息**：

（roborts_camera也能用）

安装usb_cam：

```shell
sudo apt install ros-noetic-usb-cam* -y
sudo apt install ros-noetic-image-view -y
```

启动usb_cam：

```shell
roslaunch usb_cam usb_cam-test.launch
```

launch位置：`/opt/ros/noetic/share/usb_cam/launch`

rqt查看图像：`rqt_image_view`

随后将`/roborts_detection/armor_detection/config/armor_detection.prototxt`中的摄像头名称改为usb_cam

启动节点：`rosrun roborts_detection armor_detection_node`



#### 摄像头标定

[ros标定](https://blog.csdn.net/weixin_44543463/article/details/120704327)

然后将`~/.ros/camera_info`中的文件复制到`roborts_bringup/params`中

如果不能读取文件可以把标定文件换成绝对路径

roborts_bringup的usb_cam.launch文件里修改标定文件的路径



#### 使用视频作为输入(videototopic)

工作环境下的videototopic包提供了视频发布topic的服务

使用：

```
 roslaunch roborts_bringup image_publisher.launch
```

即可将视频发布到指定的topic

可调参数位置：rm_ws/src/videototopic/config/**videototopic.yaml**

本节点同时输出camera_info信息

（开发者记：image_raw和camera_info必须同步发布（使用advertiseCamera），否则detection模块无法启动）



### 装甲板检测

**TODO：修改移植自瞄算法，在框架实现的基础上细化**

在armor_detection_node.cpp的**ExecuteLoop()**函数中执行检测算法并发布姿态信息

装甲板检测流程均在constraint_set.cpp的**DetectArmor()**中执行

**需要启动调试节点**`rosrun roborts_detection armor_detection_client`并输入“1”以启动检测线程



在没有摄像头的情况下可以使用videototopic节点和detection节点进行快速验证



## roborts_costmap

代价地图模块

global_planner和local_planner创建对应的costmap用于导航





## roborts_planning

可以使用ros的navigation包替代

```shell
sudo apt install ros-noetic-navigation
```



### global_planner

global_planner_test节点



### local_planner

local_planner节点负责局部路径规划的逻辑调度



#### Vel_Converter

vel_converter节点在仿真时将局部路径规划速度转为cmd_vel发布



```shell
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' # 1米/秒往前走

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 6.28}}' # 原地打转

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: -1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' # 1米/秒往后走
```



## move base

http://wiki.ros.org/move_base



机器人前进一米：

```shell
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
'{ header: { frame_id: "base_link" }, pose: { position: { x: 1.0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
```



> global_planner使用A*或者Dijkstra
>
> local_planner中TEB的效果最好



使用Dijkstra和TEB的launch文件：

```xml
<launch>
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
    <rosparam file="$(find package)/config1/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find package)/config1/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find package)/config1/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find package)/config1/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find package)/config1/teb_local_planner_params.yaml" command="load" />
 
     <param name="base_global_planner" value="global_planner/GlobalPlanner"/> 
     <param name="planner_frequency" value="1.0" />
     <param name="planner_patience" value="5.0" />
 
     <param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS" />
     <param name="controller_frequency" value="15.0" />
     <param name="controller_patience" value="15.0" />
  </node>
</launch>
```



## roborts_decision

**决策模块**

（划掉）（可以使用` sudo apt-get install ros-kinetic-behaviortree-cpp-v3`）



地图在roborts_stage.launch中<arg *name*="map" *value*="rmul2023"/>处修改



实际地图尺寸和map中像素的换算关系：1m=20个像素



运行节点测试：见启动文件说明



## roborts_tracking

KCF追踪算法(目前未打算使用)

```shell
rosrun roborts_tracking roborts_tracking_test
```







## 启动文件说明

base模块：

```shell
roslaunch roborts_bringup base.launch
```

costmap测试：

```shell
roslaunch roborts_bringup test_costmap.launch
```

启动amcl+move_base+stage仿真：

```shell
roslaunch roborts_bringup move_base_brimon.launch
```

启动stage仿真：（roborts_planning+roborts_localization）

启动的节点：localization_node(定位模块)、vel_converter_node(速度转换模块)、local_planner_node(局部定位模块)、global_planner_node(全局定位模块)、laser_detection_node(激光雷达目标监测模块)、global_planner_test(可以使用rviz的2D navigation工具确定目标点并导航的节点)。（根据需要添加或者去除）

```shell
roslaunch roborts_bringup roborts_stage.launch
```

启动usb摄像头：

```shell
roslaunch roborts_bringup usb_cam.launch
```

视频发布话题：（注意修改rm_ws/src/videototopic/config/videototopic.yaml中的视频路径）

```shell
roslaunch roborts_bringup image_publisher.launch
```

启动装甲板检测(roborts_detection)

```shell
rosrun roborts_detection a rmor_detection_node
rosrun roborts_detection armor_detection_client
```

一键启动装甲板检测：

```shell
roslaunch roborts_bringup armor_detection.launch
```

决策模块：

```shell
roslaunch roborts_bringup roborts_stage.launch  # 先启动仿真
rosrun roborts_decision behavior_test_node  # 再启动决策模块
```





Linux虚拟串口：

```shell
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```







