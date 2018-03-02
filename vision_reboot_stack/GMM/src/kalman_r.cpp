/* The application of a Kalman filter on GMM is to measure the confidence of a detector at each time instance.
When the variance of KF goes below a threshold the Template offered by Gaussian Mixture Model is made Public.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int64MultiArray.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

using namespace cv;
using namespace std;

ros::Publisher pub_n;
 Mat_<float> measurement(2,1);
static Mat_<float> intial(2,1);
static KalmanFilter KF(4, 2, 0);
int flag = 0;

void kfilter(const std_msgs::Int64MultiArray::ConstPtr& num) {

  float dt = 1;

  KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,dt,0,   0,1,0,dt,  0,0,1,0,  0,0,0,1);

  setIdentity(KF.measurementMatrix);
  setIdentity(KF.processNoiseCov, Scalar::all(0.5));
  setIdentity(KF.measurementNoiseCov, Scalar::all(0.1));
  setIdentity(KF.errorCovPost, Scalar::all(1));

  std_msgs::Int64MultiArray s;


  // PREDICTION
  	// static intial(1)=num->data[0];
  	// static intial(0)=num->data[1];
     Mat prediction = KF.predict();
 	   Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

	// MEASUREMENT
     measurement(1)= num->data[0];
   	 measurement(0) = num->data[1];

    	Point measPt(measurement(0), measurement(1));
   	 	cout << "MEASUREMENT :  " << measPt << endl;
  // CORRECTION
     Mat estimated = KF.correct(measurement);
 	   Point statePt(estimated.at<float>(0),estimated.at<float>(1));
	   cout<<"ESTIMATED :  "<<statePt<<endl;

 	 	if(abs((statePt.x-measPt.x)*(statePt.y-measPt.y)) < 100){
 			flag++;
 			cout << flag << endl ;
 		}
 		else{
 			flag = 0;
 		}
 		if (flag==50)
 		{
			s.data.push_back(statePt.x);
			s.data.push_back(statePt.y);
			s.data.push_back(num->data[2]);
			pub_n.publish(s);
			ros::shutdown();

		}

  ROS_INFO("Values sent to topic-print\n");


 return ;
}


int main(int argc, char** argv) {

 ros::init(argc, argv, "measurement");
 ros::NodeHandle nh;

 pub_n = nh.advertise<std_msgs::Int64MultiArray>("measurement", 1);

 ros::Rate loop_rate(10);

 ros::Subscriber sub_n = nh.subscribe("state", 1, kfilter);

 ros::spin();
 return 0;
}
