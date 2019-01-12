#include "control_system/StateController.h"

namespace kraken_controller{
    StateController::StateController(){
        for(int i = 0; i<18; i++){
            _pose_error[i] = 0;
            //_vel_error[i] = 0;
        }
    }

    StateController::~StateController(){}

    void StateController::poseParam(){
        ROS_INFO("POSITION CONTROL");
        _gain_file = "poseParam";
    }

    void StateController::twistParam(){
        ROS_INFO("TWIST CONTROL");
        _gain_file = "twistParam";
    }

    void StateController::stop(){
        ROS_INFO("STOP");
        _gain_file = "stop";
    }

    void StateController::updateState(const nav_msgs::Odometry &msg){
        // _feedback.pose.position.x = msg->pose.position.x;
        // _feedback.pose.position.y = msg->pose.position.y;
        // _feedback.pose.position.z = msg->pose.position.z;
        // _feedback.pose.orientation.x = msg->pose.orientation.x;
        // _feedback.pose.orientation.y = msg->pose.orientation.y;
        // _feedback.pose.orientation.z = msg->pose.orientation.z;
        // _feedback.pose.orientation.w = msg->pose.orientation.w;
        //
        // _feedback.twist.linear.x = msg->twist.linear.x;
        // _feedback.twist.linear.y = msg->twist.linear.y;
        // _feedback.twist.linear.z = msg->twist.linear.z;
        // _feedback.twist.angular.x = msg->twist.angular.x;
        // _feedback.twist.angular.y = msg->twist.angular.y;
        // _feedback.twist.angular.z = msg->twist.angular.z;
    }

    void StateController::updatePID(geometry_msgs::Pose transPose, geometry_msgs::Pose goalPose){
        _pose_error[3] = transPose.position.x - _pose_error[0];
        _pose_error[4] = transPose.position.y - _pose_error[1];
        _pose_error[5] = transPose.position.z - _pose_error[2];
        _pose_error[6] = _pose_error[6] + transPose.position.x;
        _pose_error[7] = _pose_error[7] + transPose.position.y;
        _pose_error[8] = _pose_error[8] + transPose.position.z;
        _pose_error[0] = transPose.position.x;
        _pose_error[1] = transPose.position.y;
        _pose_error[2] = transPose.position.z;

        double roll_g, pitch_g, yaw_g, roll_c, pitch_c, yaw_c;
        //tf::Quaternion q_g(goalPose.orientation.x, goalPose.orientation.y, goalPose.orientation.z, goalPose.orientation.w);
        tf::Quaternion q_c(transPose.orientation.x, transPose.orientation.y, transPose.orientation.z, transPose.orientation.w);
        //std::cout<<transPose.position.x<<"x\n"<<transPose.position.y<<"y\n"<<transPose.position.z<<"z\n";
        //printf("X%f Y%f Z%f W%f \n", transPose.orientation.x, transPose.orientation.y, transPose.orientation.z, transPose.orientation.w);
        //printf("X%f Y%f Z%f W%f \n", goalPose.orientation.x, goalPose.orientation.y, goalPose.orientation.z, goalPose.orientation.w);
        //tf::Matrix3x3 t_g(q_g);
        //t_g.getRPY(roll_g, pitch_g, yaw_g);

        tf::Matrix3x3 t_c(q_c);
        t_c.getRPY(roll_c, pitch_c, yaw_c);
        //ROS_INFO("%f\n",yaw_c);
        
        _pose_error[12] = roll_c -  _pose_error[9];//_pose_error[12] = (roll_g - roll_c) -  _pose_error[9];
        _pose_error[13] = pitch_c - _pose_error[10];//_pose_error[13] = (pitch_g - pitch_c) - _pose_error[10];
        _pose_error[14] = yaw_c - _pose_error[11];//_pose_error[14] = (yaw_g - yaw_c) - _pose_error[11];
        _pose_error[15] = _pose_error[15] + roll_c;//_pose_error[15] = _pose_error[15] + (roll_g - roll_c);
        _pose_error[16] = _pose_error[16] + pitch_c;//_pose_error[16] = _pose_error[16] + (pitch_g - pitch_c);
        _pose_error[17] = _pose_error[17] + yaw_c;//_pose_error[17] = _pose_error[17] + (yaw_g - yaw_c);
        _pose_error[9] = roll_c;//_pose_error[9] = roll_g - roll_c;
        _pose_error[10] = pitch_c;//_pose_error[10] = pitch_g - pitch_c;
        _pose_error[11] = yaw_c;//_pose_error[11] = yaw_g - yaw_c;

        // for(int i = 0; i<3; i++){
        //     std::cout<<i<<"---"<<_pose_error[i]<<"\n";
        //  }
        // _vel_error[3] = _vel_error[0] - transTwist.linear.x;
        // _vel_error[4] = _vel_error[1] - transTwist.linear.y;
        // _vel_error[5] = _vel_error[2] - transTwist.linear.z;
        // _vel_error[6] = _vel_error[6] + transTwist.linear.x;
        // _vel_error[7] = _vel_error[7] + transTwist.linear.y;
        // _vel_error[8] = _vel_error[8] + transTwist.linear.z;
        // _vel_error[0] = transTwist.linear.x;
        // _vel_error[1] = transTwist.linear.y;
        // _vel_error[2] = transTwist.linear.z;

        // _vel_error[12] = _vel_error[9] - transTwist.angular.x;
        // _vel_error[13] = _vel_error[10] - transTwist.angular.y;
        // _vel_error[14] = _vel_error[11] - transTwist.angular.z;
        // _vel_error[15] = _vel_error[15] + transTwist.angular.x;
        // _vel_error[16] = _vel_error[16] + transTwist.angular.y;
        // _vel_error[17] = _vel_error[17] + transTwist.angular.z;
        // _vel_error[9] = transTwist.angular.x;
        // _vel_error[10] = transTwist.angular.y;
        // _vel_error[11] = transTwist.angular.z;
        //  for(int i=0; i<18; i++){
        //      printf("%d %f \n",i, _pose_error[i]);
        // }
        geometry_msgs::Pose error;
        error.position.x = _pose_error[0];
        error.position.y = _pose_error[1];
        error.position.z = _pose_error[2];
        error.orientation.x = _pose_error[9];
        error.orientation.y = _pose_error[10];
        error.orientation.z = _pose_error[11];
        error.orientation.w = 0;
        _PidErrorPub.publish(error);
    }

    void StateController::loadParams(const std::vector<std::string> &filenames){
        for(int i = 0; i<filenames.size(); i++){
            ControlParameters *param = new ControlParameters();
            param->load(filenames[i]);
            _controlParams.push_back(param);
            _controlParams_index[param->getName()] = i;
            std::cout<<_controlParams_index[param->getName()]<<"----"<<filenames[i]<<"\n";
            param->write(std::cerr);
        }
    }

    void StateController::changeParams(control_system::paramsConfig &msg, int ThrusterSelection){
        int n_map = _controlParams_index[_gain_file];
        double *offset = _controlParams[n_map]->getOffset();
        double **gain = _controlParams[n_map]->getGain();
        offset[ThrusterSelection] = msg.offset;
        gain[ThrusterSelection][0] = msg.Gain_Kp;
        gain[ThrusterSelection][1] = msg.Gain_Kd;
        gain[ThrusterSelection][2] = msg.Gain_Ki;

        std::string str = "/home/yash/teamauv_ws/src/kraken_reboot/control_system_stack/control_system/parameters/";
        //std::string str = "/home/teamauv/teamauv_ws/src/kraken_reboot/control_system_stack/control_system/parameters/";
        std::fstream fp;
        str = str.append(_gain_file).c_str();
        fp.open(str.append(".cp").c_str(), std::ios::trunc | std::ios::out);
        if(fp.is_open()) _controlParams[n_map]->write(&fp);
        else ROS_INFO("UNABLE TO OPEN FILE %s", _gain_file.c_str());
        std::vector<std::string> filenames;
    }

    bool StateController::checkError(){
        for(int i = 0; i<3; i++){
            //if(GoalType == 0){
                if(fabs(_pose_error[i]) >= 0.01)
                    return false;
            //}
            //else if(GoalType == 1){
                if(fabs(_pose_error[i+9]) >= 0.01)
                    return false;
            //}
        }
        return true;
    }

    void StateController::setThrusters(kraken_msgs::thrusterData6Thruster *thrust){
        int n_map = _controlParams_index[_gain_file];
        double *offset = _controlParams[n_map]->getOffset();
        double **gain = _controlParams[n_map]->getGain();

        thrust->data[0] = -(offset[1] + gain[1][0]*_pose_error[2] + gain[1][1]*_pose_error[5] + gain[1][2]*_pose_error[8]) + (offset[3] + gain[3][0]*_pose_error[10] + gain[3][1]*_pose_error[13] + gain[3][2]*_pose_error[16]);
        thrust->data[1] = -(offset[1] + gain[1][0]*_pose_error[2] + gain[1][1]*_pose_error[5] + gain[1][2]*_pose_error[8]) - (offset[3] + gain[3][0]*_pose_error[10] + gain[3][1]*_pose_error[13] + gain[3][2]*_pose_error[16]);
        thrust->data[2] =   offset[0] + gain[0][0]*_pose_error[0] + gain[0][1]*_pose_error[3] + gain[0][2]*_pose_error[6] -  (offset[2] + gain[2][0]*_pose_error[11] + gain[2][1]*_pose_error[14] + gain[2][2]*_pose_error[17]);
        thrust->data[3] =   offset[0] + gain[0][0]*_pose_error[0] + gain[0][1]*_pose_error[3] + gain[0][2]*_pose_error[6] +  (offset[2] + gain[2][0]*_pose_error[11] + gain[2][1]*_pose_error[14] + gain[2][2]*_pose_error[17]);
        thrust->data[4] = 0;
        thrust->data[5] = 0;
        // for(int i=0; i<4; i++) {
        //     if(thrust->data[i] < -100)
        //         thrust->data[i] = -100;
        //     else if(thrust->data[i] > 100)
        //         thrust->data[i] = 100;
        //}
    }
}
