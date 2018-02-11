#include "control_system/ControlServer.h"
namespace kraken_controller{
    ControlServer::ControlServer(float freq){
        ros::NodeHandle n;
        _time = n.createTimer(ros::Duration(1.0/freq), &ControlServer::timeCallBack, this);
        _pub6 = n.advertise<kraken_msgs::thrusterData6Thruster>(topics::CONTROL_PID_THRUSTER6, 5);
        //_subState = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 5, &StateController::updateState, &_state);
        start = true;
        _pose_Goal.position.x = 0;
        _pose_Goal.position.y = 0;
        _pose_Goal.position.z = 0;
        _pose_Goal.orientation.x = 0;
        _pose_Goal.orientation.y = 0;
        _pose_Goal.orientation.z = 0;
        _pose_Goal.orientation.w = 0;
        GoalFlag = false;
    }

    ControlServer::~ControlServer(){}

    void ControlServer::setServer(actionlib::SimpleActionServer<kraken_msgs::advancedControllerAction> *server){
        _ControlServer = server;
        _ControlServer->start();
    }

    void ControlServer::executeGoalCB(const kraken_msgs::advancedControllerGoalConstPtr &msg){
        _state.GoalType = msg->GoalType;
        if(msg->GoalType == 0){
            _state.poseParam();
            _pose_Goal.position.x = msg->pose.position.x;
            _pose_Goal.position.y = msg->pose.position.y;
            _pose_Goal.position.z = msg->pose.position.z;
            _pose_Goal.orientation.x = msg->pose.orientation.x;
            _pose_Goal.orientation.y = msg->pose.orientation.y;
            _pose_Goal.orientation.z = msg->pose.orientation.z;
        }
        else if(msg->GoalType == 1){
            _state.twistParam();
            _twist_Goal.linear.x = msg->twist.linear.x;
            _twist_Goal.linear.y = msg->twist.linear.y;
            _twist_Goal.linear.z = msg->twist.linear.z;
            _twist_Goal.angular.x = msg->twist.angular.x;
            _twist_Goal.angular.y = msg->twist.angular.y;
            _twist_Goal.angular.z = msg->twist.angular.z;
        }
        else{
            ROS_INFO("WRONG GOALTYPE RECEIVED");
        }
        start = false;
    }

    void ControlServer::transformGoal(geometry_msgs::Pose *pose, geometry_msgs::Twist *twist){
        geometry_msgs::Pose transPose;
        geometry_msgs::Twist transTwist;

        geometry_msgs::PointStamped tempPose;
        geometry_msgs::PointStamped tempTwist;

        tempPose.header.frame_id = "odom"; //////
        tempPose.header.stamp = ros::Time();
        tempPose.point.x = pose->position.x;
        tempPose.point.y = pose->position.y;
        tempPose.point.z = pose->position.z;

        tempTwist.header.frame_id = "base_link";  //////
        tempTwist.header.stamp = ros::Time();
        tempTwist.point.x = twist->linear.x;
        tempTwist.point.y = twist->linear.y;
        tempTwist.point.z = twist->linear.z;

        try{
            geometry_msgs::PointStamped temp;
            _state.listener.transformPoint("base_link", tempPose, temp);
            transPose.position.x = temp.point.x;
            transPose.position.y = temp.point.y;
            transPose.position.z = temp.point.z;
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("RECEIVED AN EXCEPTION IN TRANSFORMING THE POSE_GOAL %s", ex.what());
        }
        try{
            geometry_msgs::PointStamped temp;
            _state.listener.transformPoint("base_link", tempTwist, temp);
            transTwist.linear.x = temp.point.x;
            transTwist.linear.y = temp.point.y;
            transTwist.linear.z = temp.point.z;
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("RECEIVED AN EXCEPTION IN TRANSFORMING THE TWIST_GOAL %s", ex.what());
        }

        _state.updatePID(transPose, transTwist);
    }

    void ControlServer::timeCallBack(const ros::TimerEvent &){
        if(start){
            transformGoal(&_pose_Goal, &_twist_Goal);
            kraken_msgs::thrusterData6Thruster thrust;
            _state.setThrusters(&thrust);
            _pub6.publish(thrust);
        }
        else if(GoalFlag){
            if (_ControlServer->isPreemptRequested() || !ros::ok()){
                 _ControlServer->setPreempted();
                 _state.stop();
                 ROS_INFO("CONTROLSERVER IS PREEMPTED");
            }
            else{
                transformGoal(&_pose_Goal, &_twist_Goal);
                kraken_msgs::thrusterData6Thruster thrust;
                _state.setThrusters(&thrust);
                _pub6.publish(thrust);
                if(_state.checkError() == true){
                    GoalFlag = false;
                    start = true;
                    _ControlServer->setSucceeded();
                }
            }
        }
    }

    void ControlServer::loadParams(const std::vector<std::string> &filenames){
        _state.loadParams(filenames);
    }

    void ControlServer::callback0(control_system::paramsConfig &msg, uint32_t level){
        if(GoalFlag) _state.changeParams(msg, 0);
    }

    void ControlServer::callback1(control_system::paramsConfig &msg, uint32_t level){
        if(GoalFlag)  _state.changeParams(msg, 1);
    }

    void ControlServer::callback2(control_system::paramsConfig &msg, uint32_t level){
        if(GoalFlag)  _state.changeParams(msg, 2);
    }

    void ControlServer::callback3(control_system::paramsConfig &msg, uint32_t level){
        if(GoalFlag)  _state.changeParams(msg, 3);
    }

    void ControlServer::callback4(control_system::paramsConfig &msg, uint32_t level){
        if(GoalFlag)  _state.changeParams(msg, 4);
    }

    void ControlServer::callback5(control_system::paramsConfig &msg, uint32_t level){
        if(GoalFlag)  _state.changeParams(msg, 5);
        else GoalFlag = true;
    }
}
