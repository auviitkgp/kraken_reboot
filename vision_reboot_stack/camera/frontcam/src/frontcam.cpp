#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>



using namespace std;
using namespace cv;

int camMsg = 0;  // 0 : front cam   1:  bottom cam
int cameraNo = 0;

bool camOpen = false;
int flag = 0;

VideoCapture cam;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontcam");

    ros::NodeHandle _nh;
    image_transport::ImageTransport _it(_nh);
    image_transport::Publisher _image_pub = _it.advertise("kraken/frontcam", 1);
    //ros::Subscriber _sub = _nh.subscribe(topics::CAMERA_CAM_SWITCH, 1, msgCallback);

    sensor_msgs::ImagePtr _publishImage;
    cv_bridge::CvImage _image;
    _image.encoding = "bgr8";

    if (argc >= 2)
    {
        cameraNo = atoi(argv[1]);
        cam.open(cameraNo);
        camOpen = true;
    }
    else
    {
        ROS_ERROR("You must pass in an integer (0 or 1). 0 -> Front camera, 1 -> Bottom camera");
        ros::shutdown();
    }

    ros::Rate _looprate(10);

    while(ros::ok())
    {
        if(camOpen)
        {
            cam >> _image.image;

            _publishImage = _image.toImageMsg();

            _image_pub.publish(_publishImage);
        }

        ros::spinOnce();
        _looprate.sleep();
    }

    return 0;
}
