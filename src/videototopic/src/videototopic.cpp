#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;


struct Config {
  void GetParam() {
    ros::NodeHandle nh;
    nh.param<std::string>("path", path, "/home/brimon/rm_ws/src/videototopic/001.avi");
	nh.param<std::string>("cam_name", cam_name, "usb_cam");
  }
  std::string path;
  std::string cam_name;
};


sensor_msgs::CameraInfoPtr getCameraInfo(void){        // extract cameraInfo.
    sensor_msgs::CameraInfoPtr cam(new sensor_msgs::CameraInfo());

    vector<double> D{0.000094, -0.011701, 0.000383, -0.000507, 0.000000};
    boost::array<double, 9> K = {
        404.005825, 0.000000, 335.580380,
        0.000000, 404.368809, 250.727020,
        0.000000, 0.000000, 1.000000  
    };
    
     boost::array<double, 12> P = {
        402.124725, 0.000000, 335.482488, 0.000000,
        0.000000, 403.765045, 250.954855, 0.000000,
        0.000000, 0.000000, 1.000000, 0.000000
    };
    boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cam->height = 480;
    cam->width = 640;
    cam->distortion_model = "plumb_bob";
    cam->D = D;
    cam->K = K;
    cam->P = P;
    cam->R = r;
    cam->binning_x = 0;
    cam->binning_y = 0;
    cam->header.frame_id = "usb_cam";
    cam->header.stamp = ros::Time::now();
    cam->header.stamp.nsec = 0;

    return cam;
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"image_publisher");

	Config config;
	config.GetParam();
	ROS_INFO("open video : %s\n", config.path.c_str());
	ROS_INFO("camera name : %s\n", config.cam_name.c_str());

	ros::NodeHandle nh;
	nh = ros::NodeHandle(config.cam_name);

	// image publisher
	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher pub = it.advertiseCamera("image_raw", 1, true);
	sensor_msgs::CameraInfoPtr camera_info_dyn(new sensor_msgs::CameraInfo());
	camera_info_dyn = getCameraInfo();

	ros::Rate loop_rate(30);  // update rate

	VideoCapture cap(config.path);  //open video from the path
	if(!cap.isOpened()){
		ROS_ERROR("open video failed!");
		return -1;
	}
	else
	ROS_INFO("open video success!");

	cv::Mat frame;
	bool isSuccess = true;
	while(nh.ok()){
		isSuccess = cap.read(frame);
		// if the video ends, then break
		if(!isSuccess) {
			ROS_INFO("video ends");
			break;
		}
		// transform cv::Mat to built-in msg
		sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		img_msg->header.stamp = ros::Time::now();
		img_msg->header.frame_id = config.cam_name;

		camera_info_dyn->header.stamp = img_msg->header.stamp;
		pub.publish(img_msg, camera_info_dyn);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
