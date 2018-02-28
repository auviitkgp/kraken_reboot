#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "resources/topicHeader.h"
#include "kraken_msgs/advancedControllerAction.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "PIDTuning");

    actionlib::SimpleActionClient<kraken_msgs::advancedControllerAction> _actionClient(topics::CONTROL_ADVANCEDCONTROLLER_ACTION, true);

    ROS_INFO("WAITING FOR CONTROL_SERVER TO START.");
    _actionClient.waitForServer();
    ROS_INFO("CONTROL_SERVER STARTED, SENDING GOAL.");
    kraken_msgs::advancedControllerGoal _goal;
    if(argv[1] == "surge"){
        _goal.pose.position.x = 2;
        _goal.pose.position.y = 0;
        _goal.pose.position.z = 0;
        _goal.pose.orientation.x = 0;
        _goal.pose.orientation.y = 0;
        _goal.pose.orientation.z = 0;
        _goal.pose.orientation.w = 0;
    }
    if(argv[1] == "depth"){
        _goal.pose.position.x = 0;
        _goal.pose.position.y = 0;
        _goal.pose.position.z = 1;
        _goal.pose.orientation.x = 0;
        _goal.pose.orientation.y = 0;
        _goal.pose.orientation.z = 0;
        _goal.pose.orientation.w = 0;
    }
    if(argv[1] == "yaw"){
        _goal.pose.position.x = 0;
        _goal.pose.position.y = 0;
        _goal.pose.position.z = 0;
        _goal.pose.orientation.x = 0;
        _goal.pose.orientation.y = 0;
        _goal.pose.orientation.z = 0;
        _goal.pose.orientation.w = 1;
    }
    _actionClient.sendGoal(_goal);
    bool finished_before_timeout = _actionClient.waitForResult(ros::Duration(30.0));
    if(finished_before_timeout){
        actionlib::SimpleClientGoalState state = _actionClient.getState();
        ROS_INFO("GOAL FINISHED: %s", state.toString().c_str());
    }
    else
        ROS_INFO("GOAL NOT FINISHED.");
    return 0;
    }
