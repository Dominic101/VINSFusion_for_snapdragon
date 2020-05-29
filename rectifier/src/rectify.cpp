#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv.h>
#include <highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <snapstack_msgs/IMU.h>
#include <snapstack_msgs/ImuArray.h>

using namespace cv;

Mat intrinsics = Mat::zeros(3, 3, CV_64F);
Mat dist_coeffs = cv::Mat::zeros(1, 4, CV_64F);
image_transport::Publisher pub;
ros::Publisher imu_pub;

void readFromYamlFile(const std::string& filename)
{
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened())
    {
        printf("rectify node failed to open params file");
    }
    ROS_INFO("parameters set");
    //int m_imageWidth = static_cast<int>(fs["image_width"]);
    //int m_imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["distortion_parameters"];
    dist_coeffs.at<double>(0,0) = .003908; //static_cast<double>(n["k1"]);
    dist_coeffs.at<double>(0,1) = -.009574; //static_cast<double>(n["k2"]);
    dist_coeffs.at<double>(0,2) = .010173; //static_cast<double>(n["p1"]);
    dist_coeffs.at<double>(0,3) = -.003329; //static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    intrinsics.at<double>(2,2) = 1;
    intrinsics.at<double>(0,0) = 275; //static_cast<double>(n["fx"]);
    intrinsics.at<double>(1,1) = 275; //static_cast<double>(n["fy"]);
    intrinsics.at<double>(0,2) = 320; //static_cast<double>(n["cx"]);
    intrinsics.at<double>(1,2) = 240; //static_cast<double>(n["cy"]);
    //ROS_INFO("%d",intrinsics.at<double>(1,2));
}

void rect_callback(const sensor_msgs::ImageConstPtr& msg)
{
	Mat image_undistorted;
	cv_bridge::CvImagePtr distorted_msg_ptr;
	distorted_msg_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	fisheye::undistortImage(distorted_msg_ptr->image, image_undistorted, intrinsics, dist_coeffs, intrinsics);

//reproject to pixel space
	sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_undistorted).toImageMsg();

img->header.stamp = msg->header.stamp;
//msg->header.stamp.nsec = ros::Time::now().nsec;
	
	pub.publish(img);
}



void imu_callback(const snapstack_msgs::ImuArray &msg)
{
double a_x = 0.0;
double a_y = 0.0;
double a_z = 0.0;
double l_x = 0.0;
double l_y = 0.0;
double l_z = 0.0;

for (int i =0; i<1; i++){
a_x += msg.imu_samples[i].angular_velocity.x;
a_y += msg.imu_samples[i].angular_velocity.y;
a_z += msg.imu_samples[i].angular_velocity.z;

l_x += msg.imu_samples[i].linear_acceleration.x;
l_y += msg.imu_samples[i].linear_acceleration.y;
l_z += msg.imu_samples[i].linear_acceleration.z;
}

a_x = a_x/1;
a_y = a_y/1;
a_z = a_z/1;
l_x = l_x/1;
l_y = l_y/1;
l_z = l_z/1;

snapstack_msgs::IMU imu_msg;
imu_msg.header.stamp = msg.header.stamp;
imu_msg.accel.x = l_x;
imu_msg.accel.y = l_y;
imu_msg.accel.z = l_z;
imu_msg.gyro.x = a_x;
imu_msg.gyro.y = a_y;
imu_msg.gyro.z = a_z;

imu_pub.publish(imu_msg);
}

int main (int argc, char **argv)
{
	readFromYamlFile("/home/dominic/driverless_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml");
	ros::init(argc, argv, "rectify");
	ros::NodeHandle rectify;
	image_transport::ImageTransport it(rectify);
	image_transport::Subscriber sub = it.subscribe("/HX55/image_raw", 1, rect_callback);
	pub = it.advertise("/rectified",1);

	ros::Subscriber imu = rectify.subscribe("/HX55/imu_raw_array", 1, imu_callback);
	imu_pub = rectify.advertise<snapstack_msgs::IMU>("/imu_avg", 1);
	ros::spin();
return 0;
}
