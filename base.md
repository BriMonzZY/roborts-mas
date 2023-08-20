# roborts_base

[wiki](https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/roborts_base)



base模块提供底盘里程计信息（/odom）以及tf（base_link -> odom、base_link -> gimbal）



**订阅：**

/cmd_vel

/cmd_vel_acc

/cmd_gimbal_angle

/set_gimbal_mode

/cmd_fric_wheel

/cmd_shoot

**发布：**

/tf

/odom

/uwb



下位机通过麦轮编码器计算好x，y后通过串口发送给上位机，上位机稍作处理发布为/odom话题



