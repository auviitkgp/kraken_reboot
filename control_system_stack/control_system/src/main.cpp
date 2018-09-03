#include "ros/ros.h"
#include "control_system/ControlServer.h"
#include "dynamic_reconfigure/server.h"
#include "control_system/paramsConfig.h"
#include "resources/topicHeader.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "CONTROL_SERVER");
    ros::NodeHandle n;
    ros::NodeHandle n_a("~/surge");
    ros::NodeHandle n_b("~/depth");
    ros::NodeHandle n_c("~/yaw");
    ros::NodeHandle n_d("~/pitch");

    if(argc > 2){
        kraken_controller::ControlServer _server;
        std::vector<std::string> _files;
        for(int i = 1; i<argc; i++){
            _files.push_back(argv[i]);
        }

        _server.loadParams(_files);
        actionlib::SimpleActionServer<kraken_msgs::advancedControllerAction> _ControlServer(n, topics::CONTROL_ADVANCEDCONTROLLER_ACTION, boost::bind(&kraken_controller::ControlServer::executeGoalCB, &_server, _1), false);
        _server.setServer(&_ControlServer);

        dynamic_reconfigure::Server<control_system::paramsConfig> surge(n_a);
        dynamic_reconfigure::Server<control_system::paramsConfig> depth(n_b);
        dynamic_reconfigure::Server<control_system::paramsConfig> yaw(n_c);
        dynamic_reconfigure::Server<control_system::paramsConfig> pitch(n_d);
        
        surge.setCallback(boost::bind(&kraken_controller::ControlServer::callback0, &_server, _1, _2));
        depth.setCallback(boost::bind(&kraken_controller::ControlServer::callback1, &_server, _1, _2));
        yaw.setCallback(boost::bind(&kraken_controller::ControlServer::callback2, &_server, _1, _2));
        pitch.setCallback(boost::bind(&kraken_controller::ControlServer::callback3, &_server, _1, _2));
        
        ros::spin();
    }
    else std::cerr<<"Server 'file1' 'file2' .... "<<std::endl;
    return 0;
}
