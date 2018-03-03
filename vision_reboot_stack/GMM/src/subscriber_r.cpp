#include "ros/ros.h"
#include <std_msgs/Int64MultiArray.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int flag_c=0;
int flag_s=0;


void callback1(const std_msgs::Int64MultiArray::ConstPtr& num )
{
 ROS_INFO("Centre: [%ld, %ld] :: Radius: %ld \n", num->data[0], num->data[1], num->data[2]);
 cout<<flag_c++<<" flag_c"<<endl;
 return;
}


int main(int argc, char **argv)
{
 ros::init(argc, argv, "print");
 ros::NodeHandle n;
 ros::Rate loop_rate(10);

 ros::Subscriber sub_n = n.subscribe("state", 1, callback1);
 cout<<flag_s++<<" flag_s"<<endl;
 ros::spin();
 loop_rate.sleep();
 return 0;
}
