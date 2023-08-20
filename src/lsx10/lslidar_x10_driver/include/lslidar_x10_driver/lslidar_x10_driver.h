/*
 * This file is part of lslidar_x10 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LSLIDAR_X10_DRIVER_H
#define LSLIDAR_X10_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include "input.h"

#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "lsiosr.h"
#include <sensor_msgs/LaserScan.h>

#include <lslidar_x10_msgs/LslidarX10Packet.h>
#include <std_msgs/Byte.h>
namespace lslidar_x10_driver {

typedef struct {
    double degree;
    double range;
    double intensity;
} ScanPoint;

uint16_t PACKET_SIZE ;

class LslidarX10Driver {
public:

    LslidarX10Driver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarX10Driver();

    bool initialize();
    bool polling();
    int getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration);
    void data_processing(unsigned char *packet_bytes,int len);
    typedef boost::shared_ptr<LslidarX10Driver> LslidarX10DriverPtr;
    typedef boost::shared_ptr<const LslidarX10Driver> LslidarX10DriverConstPtr;
    void recvThread_crc(int &count_2,int &link_time);
private:
    boost::mutex mutex_; 
    boost::mutex pubscan_mutex_;
    boost::condition_variable pubscan_cond_;

    boost::thread *recv_thread_;
    boost::thread *pubscan_thread_ ;
    void Interface_selection();
    bool createRosIO();
    void close_serial();
    void open_serial();
    void pubScanThread();
    void lidar_order(const std_msgs::Int8 msg);
    int receive_data(unsigned char *packet_bytes);
    void initParam();
    uint8_t N10_CalCRC8(unsigned char * p, int len);
    // Ethernet relate variables
    int UDP_PORT_NUMBER;
	bool is_start;
    // ROS related variables
    LSIOSR * serial_;
    std::string serial_port_;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    std::string  interface_selection;
    boost::shared_ptr<Input> msop_input_;
    ros::Publisher packet_pub;
	ros::Subscriber difop_switch;
    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_bak_;
    std::string frame_id_;
    std::string lidar_name;
    std::string scan_topic_;
    std::string dump_file;

    double angle_able_min;
    double angle_able_max;
    bool use_gps_ts;
    int count_num;
    int package_points;
    int data_bits_start;
    int degree_bits_start;
    int end_degree_bits_start;
    int rpm_bits_start;
	int baud_rate_;
    int points_size_;
    ros::Time pre_time_;
    ros::Time time_;
    uint64_t get_gps_stamp(tm t);
    tm pTime;    
    uint64_t sub_second;
    ros::Publisher pub_;
    uint64_t sweep_end_time_gps;
    uint64_t sweep_end_time_hardware;

    int idx = 0;
    int link_time = 0;
    double last_degree = 0.0;	

    int truncated_mode_;
    double min_distance,max_distance;
    std::vector<int> disable_angle_min_range, disable_angle_max_range, disable_angle_range_default;

};

typedef LslidarX10Driver::LslidarX10DriverPtr LslidarX10DriverPtr;
typedef LslidarX10Driver::LslidarX10DriverConstPtr LslidarX10DriverConstPtr;

} // namespace lslidar_driver

#endif // _LSLIDAR_X10_DRIVER_H_
