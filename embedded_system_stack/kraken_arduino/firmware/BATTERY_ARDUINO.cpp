#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/BatteryState.h>
#include <Arduino.h>

int i=0;
const int curr_calib=514;
//float v_gain=(211.3)/33.0;
float v_gain=25.5;
int BAT_ipin=A0, BAT_vpin=A4;

ros::NodeHandle nh;
void prepareBatteryStates();
void getCurrent();
void getVoltage();

sensor_msgs::BatteryState BAT_State;
ros::Publisher BAT_StatePub("/kraken/Battery",&BAT_State);


void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(BAT_StatePub);
  pinMode(13,OUTPUT);
  delay(50);
  prepareBatteryStates();
}

void loop()
{
  getCurrent();
  getVoltage();
  BAT_State.header.stamp=nh.now();
  BAT_StatePub.publish(&BAT_State);
  nh.spinOnce();
  delay(100);
}

void prepareBatteryStates()
{
  //Preparing Battery State
  BAT_State.header.frame_id=0;
  BAT_State.voltage=NAN;
  BAT_State.current=NAN;
  BAT_State.charge=NAN;
  BAT_State.capacity=NAN;
  BAT_State.design_capacity=10.0;
  BAT_State.percentage=NAN;
  BAT_State.power_supply_status=BAT_State.POWER_SUPPLY_STATUS_UNKNOWN;
  BAT_State.power_supply_health=BAT_State.POWER_SUPPLY_HEALTH_UNKNOWN;
  BAT_State.power_supply_technology=BAT_State.POWER_SUPPLY_TECHNOLOGY_LIPO;
  BAT_State.present=true;
  // BAT_State.cell_voltage={NAN,NAN,NAN,NAN,NAN,NAN};
  BAT_State.location="Universal";
}

void getCurrent()
{
  //Reading and Storing Battery Current and Supply State
  int curr_val=0;
  if(BAT_State.present)
  {
    curr_val=analogRead(BAT_ipin);
    BAT_State.current=(curr_val-curr_calib)*5000.0/1023.0/56.0;
    if(BAT_State.current>0.1)
    {
      BAT_State.power_supply_status=BAT_State.POWER_SUPPLY_STATUS_DISCHARGING;
      nh.loginfo("\nBattery Discharging\n");
    }
    else if (BAT_State.current>-0.1&&BAT_State.current<0.1)
    {
      BAT_State.power_supply_status=BAT_State.POWER_SUPPLY_STATUS_NOT_CHARGING;
      nh.loginfo("\nBattery Idle\n");
    }
    else
    {
      BAT_State.power_supply_status=BAT_State.POWER_SUPPLY_STATUS_CHARGING;
      nh.loginfo("\nBattery Charging\n");
    }
  }
}

void getVoltage()
{
  //Reading and Storing Battery Voltage and Supply Health
  if(BAT_State.present)
  {
    BAT_State.voltage=analogRead(BAT_vpin)*v_gain/1023.0;
    if(BAT_State.voltage>24.5)
    {
      BAT_State.power_supply_health=BAT_State.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
      nh.logfatal("\nBattery is above its maximum voltage!!!\n");
    }
    else if (BAT_State.voltage<20.0)
    {
      BAT_State.power_supply_status=BAT_State.POWER_SUPPLY_HEALTH_DEAD;
      nh.logfatal("\nBattery is Dead!!!\n");
    }
    else
    {
      BAT_State.power_supply_status=BAT_State.POWER_SUPPLY_STATUS_CHARGING;
      nh.loginfo("\nBattery OK\n");
    }
  }
}
