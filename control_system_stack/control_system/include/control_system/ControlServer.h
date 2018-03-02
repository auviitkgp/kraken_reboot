#ifndef CONTROLSERVER_H
#define CONTROLSERVER_H

#include "ros/ros.h"
#include "ros/rate.h"
#include "kraken_msgs/thrusterData6Thruster.h"
#include "kraken_msgs/advancedControllerAction.h"
#include "control_system/StateController.h"
#include "control_system/ControlParameters.h"
#include "actionlib/server/simple_action_server.h"
#include "control_system/paramsConfig.h"
#include "tf/transform_listener.h"

//#include "resources/topicHeader.h"

namespace kraken_controller{
    class ControlServer{
    public:
        ControlServer(float freq = 10);
        ~ControlServer();
        void timeCallBack(const ros::TimerEvent&);
        void setServer(actionlib::SimpleActionServer<kraken_msgs::advancedControllerAction> *);
        void executeGoalCB(const kraken_msgs::advancedControllerGoalConstPtr &);
        void transformGoal(geometry_msgs::Pose *, geometry_msgs::Twist *);
        void loadParams(const std::vector<std::string> &filenames);
        void callback0(control_system::paramsConfig &msg, uint32_t level);
        void callback1(control_system::paramsConfig &msg, uint32_t level);
        void callback2(control_system::paramsConfig &msg, uint32_t level);
        void callback3(control_system::paramsConfig &msg, uint32_t level);
        void callback4(control_system::paramsConfig &msg, uint32_t level);
        void callback5(control_system::paramsConfig &msg, uint32_t level);

    private:
        ros::Publisher _pub6;
        ros::Subscriber _subState;
        ros::Timer _time;
        StateController _state;
        actionlib::SimpleActionServer<kraken_msgs::advancedControllerAction> *_ControlServer;
        geometry_msgs::Pose _pose_Goal;
        geometry_msgs::Twist _twist_Goal;
        bool start;
        bool GoalFlag;   ///////
    };
}

#endif
