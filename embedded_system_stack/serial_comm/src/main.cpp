#include "ros/ros.h"
#include "string.h"
#include "serial/serial.h"
#include "kraken_msgs/thrusterCmd.h"
#include "resources/topicHeader.h"

void commCB(const kraken_msgs::thrusterCmdConstPtr &msg);
uint8_t data[18];
serial::Serial *temp;

int main(int argc, char **argv){
  ros::init(argc, argv, "serial_comm_node");
  if(argc < 2){
    ROS_INFO("BAUDRATE OR PORT NOT SPECIFIED");
    return 0;
  }
  std::string port(argv[2]);
  uint32_t baud;
  sscanf(argv[1], "%u", &baud);
  ros::NodeHandle n;
  ros::Subscriber CmdSub = n.subscribe<kraken_msgs::thrusterCmd>(topics::CONTROL_SEABOTIX, 1000, commCB);
  serial::Serial comm(port, baud, serial::Timeout::simpleTimeout(1000));
  temp = &comm;
  ros::Rate loop_rate(8);
  while(ros::ok()){
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void commCB(const kraken_msgs::thrusterCmd::ConstPtr &msg){
  for(int i = 0; i<6 ; i++){
    data[3*i+0] = msg->data_array[i].thruster_id;
    data[3*i+1] = msg->data_array[i].speed;
    data[3*i+2] = msg->data_array[i].info;
  }
  std::cout <<"SUCCESS\n";
  if(temp->isOpen()) ROS_INFO("SERIAL PORT IS OPEN");
    else{
      temp->open();
      ROS_INFO("SERIAL PORT IS OPENED");
    }
    size_t bytes_written = temp->write(data, 18);
    std::cout << "BYTES SEND: "<<bytes_written<< "\n";
    //temp->close();
}
