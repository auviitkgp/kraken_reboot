#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "resources/topicHeader.h"
#include "kraken_msgs/advancedControllerAction.h"
#include "tf/transform_listener.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "PIDTuning");

    //argv[1] : type of goal ("roll", "pitch", ..)
    //argv[2] : position /orientation (1) **Quaternion
    //argv[3] : orientation(2) **Quaternion

    actionlib::SimpleActionClient<kraken_msgs::advancedControllerAction> _actionClient(topics::CONTROL_ADVANCEDCONTROLLER_ACTION, true);

    ROS_INFO("WAITING FOR CONTROL_SERVER TO START.");
    _actionClient.waitForServer();
    ROS_INFO("CONTROL_SERVER STARTED, SENDING GOAL.");
    kraken_msgs::advancedControllerGoal _goal;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("base_link", "odom",ros::Time(0), transform);
    }
    catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    }

    _goal.GoalType = 0;
    if(argv[1] == std::string("surge")){
        geometry_msgs::PoseStamped temp;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/base_link";
        pose.header.stamp = ros::Time();
        pose.pose.position.x = atof(argv[2]);
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        //_state.listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(30), ros::Duration(0.01), "FAILED");
        listener.transformPose("/odom", pose, temp);

        _goal.pose.position.x = temp.pose.position.x ;
        _goal.pose.position.y = temp.pose.position.y ;
        _goal.pose.position.z = temp.pose.position.z ;
        _goal.pose.orientation.x = 0;      //desired orientation in odom frame
        _goal.pose.orientation.y = 0;      //desired orientation in odom frame
        _goal.pose.orientation.z = 0;      //desired orientation in odom frame
        _goal.pose.orientation.w = 1;      //desired orientation in odom frame
        std::cout<<argv[1]<<" \n";
    }
    else if(argv[1] == std::string("depth")){
        geometry_msgs::PoseStamped temp;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/base_link";
        pose.header.stamp = ros::Time();
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = atof(argv[2]);
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        //_state.listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(30), ros::Duration(0.01), "FAILED");
        listener.transformPose("/odom", pose, temp);

        _goal.pose.position.x = temp.pose.position.x ;
        _goal.pose.position.y = temp.pose.position.y ;
        _goal.pose.position.z = temp.pose.position.z ;
        _goal.pose.orientation.x = 0;
        _goal.pose.orientation.y = 0;
        _goal.pose.orientation.z = 0;
        _goal.pose.orientation.w = 1;
        std::cout<<argv[1]<<" \n";
    }
    else if(argv[1] == std::string("yaw")){
        tf::Quaternion q = tf::createQuaternionFromRPY(0,0,atof(argv[2]));
        q.normalize();
        geometry_msgs::PoseStamped temp;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/base_link";
        pose.header.stamp = ros::Time();
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        //_state.listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(30), ros::Duration(0.01), "FAILED");
        listener.transformPose("/odom", pose, temp);

        _goal.pose.position.x = temp.pose.position.x ;
        _goal.pose.position.y = temp.pose.position.y ;
        _goal.pose.position.z = temp.pose.position.z ;
        _goal.pose.orientation.x = q.x();
        _goal.pose.orientation.y = q.y();
        _goal.pose.orientation.z = q.z();
        _goal.pose.orientation.w = q.w();
        std::cout<<argv[1]<<" \n";
    }
    else if(argv[1] == std::string("pitch")){
        tf::Quaternion q = tf::createQuaternionFromRPY(0,atof(argv[2]),0);
        q.normalize();
        geometry_msgs::PoseStamped temp;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/base_link";
        pose.header.stamp = ros::Time();
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        //_state.listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(30), ros::Duration(0.01), "FAILED");
        listener.transformPose("/odom", pose, temp);

        _goal.pose.position.x = temp.pose.position.x ;
        _goal.pose.position.y = temp.pose.position.y ;
        _goal.pose.position.z = temp.pose.position.z ;
        _goal.pose.orientation.x = q.x();
        _goal.pose.orientation.y = q.y();
        _goal.pose.orientation.z = q.z();
        _goal.pose.orientation.w = q.w();
        std::cout<<argv[1]<<" \n";
    }
    else
        return 0;
    _actionClient.sendGoal(_goal);
    bool finished_before_timeout = _actionClient.waitForResult(ros::Duration(30.0));
    if(finished_before_timeout){
        actionlib::SimpleClientGoalState state = _actionClient.getState();
        ROS_INFO("GOAL FINISHED: %s", state.toString().c_str());
        _actionClient.cancelGoal();
    }
    else
        ROS_INFO("GOAL NOT FINISHED.");
    return 0;
    }
