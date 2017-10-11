#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Image.h"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <visp3/core/vpImageConvert.h>
#include "computepose/libPoseQr.h"

void image_callback(const sensor_msgs::ImageConstPtr& img_msg) {
  
  cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  cv::Mat image = img_ptr->image;

  vpImage<unsigned char> I;  
  vpImageConvert::convert(image, I);  //将cv::Mat转换为visp可处理的格式I；

  poseQr(I);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/cam0/image_raw", 1, image_callback);

  ros::Rate loop_rate(1);  //处理图片频率为1Hz；
  while (ros::ok())
  {
    /*...TODO...*/ 
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;  

}
  