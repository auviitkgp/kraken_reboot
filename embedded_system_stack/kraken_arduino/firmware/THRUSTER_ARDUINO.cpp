#include <Wire.h>
#include <ros.h>
#include <ros/time.h>
#include <kraken_msgs/thrusterCmd.h>
#include <Arduino.h>

int i=0;

ros::NodeHandle nh;
void thrust_cb (const kraken_msgs::thrusterCmd& cmd)
{
  for(i=0; i<6; i++)
  {
    Wire.beginTransmission((cmd.data_array[i].thruster_id)>>1);
    Wire.write(cmd.data_array[i].speed);
    Wire.write(cmd.data_array[i].info);
    Wire.write(cmd.data_array[i].thruster_id+cmd.data_array[i].speed+cmd.data_array[i].info);
    Wire.endTransmission(false);
  }
}

ros::Subscriber<kraken_msgs::thrusterCmd> sub("/kraken/control/seabotixData", &thrust_cb );


void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  Wire.begin();
  nh.subscribe(sub);
  pinMode(13,OUTPUT);
}

void loop()
{
  nh.spinOnce();
}



