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
    // ros::NodeHandle n_c("~/left_surge");
    // ros::NodeHandle n_d("~/right_surge");
    // ros::NodeHandle n_e("~/front_sway");
    // ros::NodeHandle n_f("~/back_sway");

    if(argc > 2){
        kraken_controller::ControlServer _server;
        std::vector<std::string> _files;
        for(int i = 1; i<argc; i++){
            _files.push_back(argv[i]);
        }

        _server.loadParams(_files);
        actionlib::SimpleActionServer<kraken_msgs::advancedControllerAction> _ControlServer(n, topics::CONTROL_ADVANCEDCONTROLLER_ACTION, boost::bind(&kraken_controller::ControlServer::executeGoalCB, &_server, _1), false);
        _server.setServer(&_ControlServer);

        dynamic_reconfigure::Server<control_system::paramsConfig> thruster0(n_a);
        dynamic_reconfigure::Server<control_system::paramsConfig> thruster1(n_b);
        // dynamic_reconfigure::Server<control_system::paramsConfig> thruster2(n_c);
        // dynamic_reconfigure::Server<control_system::paramsConfig> thruster3(n_d);
        // dynamic_reconfigure::Server<control_system::paramsConfig> thruster4(n_e);
        // dynamic_reconfigure::Server<control_system::paramsConfig> thruster5(n_f);
        thruster0.setCallback(boost::bind(&kraken_controller::ControlServer::callback0, &_server, _1, _2));
        thruster1.setCallback(boost::bind(&kraken_controller::ControlServer::callback1, &_server, _1, _2));
        // thruster2.setCallback(boost::bind(&kraken_controller::ControlServer::callback2, &_server, _1, _2));
        // thruster3.setCallback(boost::bind(&kraken_controller::ControlServer::callback3, &_server, _1, _2));
        // thruster4.setCallback(boost::bind(&kraken_controller::ControlServer::callback4, &_server, _1, _2));
        // thruster5.setCallback(boost::bind(&kraken_controller::ControlServer::callback5, &_server, _1, _2));
        ros::spin();
    }
    else std::cerr<<"Server 'file1' 'file2' .... "<<std::endl;
    return 0;
}
