# imu_serial

串口调试工具：

```shell
sudo apt-get install cutecom -y
sudo apt install minicom -y
sudo cutecom
dmesg | grep ttyS*
sudo minicom -s
```



虚拟串口：

```shell
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

WSL2接入windows串口：[WSL2下的usb串口设备使用](https://blog.csdn.net/qq_28695769/article/details/125202748)



消息格式：

[sensor_msgs/Imu Documentation (ros.org)](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)

[sensor_msgs/MagneticField Documentation (ros.org)](http://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html)

[ros-mpu9250-imu](https://github.com/jusgomen/ros-mpu9250-imu/blob/master/src/talker.cpp)



安装依赖（serial）：

```shell
sudo apt install ros-noetic-serial -y
```

启动imu信息发布节点：

```shell
rosrun imu_serial imu_serial_node
```



上位机与stm32协议：

imu_data共13字节

| 数据     | 说明 | 占用字节 |
| -------- | ---- | -------- |
| SOF      | 0xAA | 1        |
| acc_x_h  |      | 1        |
| acc_x_l  |      | 1        |
| acc_y_h  |      | 1        |
| acc_y_l  |      | 1        |
| acc_z_h  |      | 1        |
| acc_z_l  |      | 1        |
| gryo_x_h |      | 1        |
| gryo_x_l |      | 1        |
| gryo_y_h |      | 1        |
| gryo_y_l |      | 1        |
| gryo_z_h |      | 1        |
| gryo_z_l |      | 1        |

