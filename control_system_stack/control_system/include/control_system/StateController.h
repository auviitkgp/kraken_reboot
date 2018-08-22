#ifndef STATECONTROLLER_H
#define STATECONTROLLER_H

#include "kraken_msgs/thrusterData6Thruster.h"
#include "nav_msgs/Odometry.h"
#include "control_system/ControlParameters.h"
#include "iostream"
#include "fstream"
#include "tf/transform_listener.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "resources/topicHeader.h"
#include "control_system/paramsConfig.h"
//#include "kraken_msgs/PIDError.h"

namespace kraken_controller{
    class StateController{
    public:
        StateController();
        ~StateController();
        void poseParam();
        void twistParam();
        void stop();
        void updateState(const nav_msgs::Odometry &);
        void updatePID(geometry_msgs::Pose, geometry_msgs::Twist);
        void loadParams(const std::vector<std::string> &);
        void changeParams(control_system::paramsConfig &, int);
        bool checkError();
        void setThrusters(kraken_msgs::thrusterData6Thruster *);
        tf::TransformListener listener;
        ros::Publisher _PidErrorPub;
        std::string _gain_file;
        nav_msgs::Odometry _feedback;
        std::vector<ControlParameters*> _controlParams;
        std::map<std::string, int> _controlParams_index;
        double _pose_error[18]; //_vel_error[18];
        int GoalType;
    };
}
#endif
