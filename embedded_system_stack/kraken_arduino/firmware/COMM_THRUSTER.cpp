#include <Wire.h>
#include <Arduino.h>

void setup(){
  Serial.begin(57600);
  Wire.begin();
  pinMode(13,OUTPUT);
}

void loop(){
  uint8_t data[18];
  int i;
  for(i = 0;i<18; i++){
    while(!Serial.available()){
      data[0] = 90;
      data[1] = 127;
      data[2] = 100;
      data[3] = 96;
      data[4] = 127;
      data[5] = 100;
      data[6] = 80;
      data[7] = 127;
      data[8] = 100;
      data[9] = 92;
      data[10] = 127;
      data[11] = 100;
      data[12] = 82;
      data[13] = 127;
      data[14] = 100;
      data[15] = 94;
      data[16] = 127;
      data[17] = 100;
      for(int j=0; j<6; j++){
        Wire.beginTransmission((data[j*3])>>1);
        Wire.write(data[j*3+1]);
        Wire.write(data[j*3+2]);
        Wire.write(data[j*3] + data[j*3+1] + data[j*3+2]);
        Wire.endTransmission(false);
      }
      //for(int k = 0; k<18; k++){
        //Serial.println(data[k]);
      //}
    }
    data[i] = Serial.read();
    Serial.println(data[i]);
  }
  for(i=0; i<6; i++){
    Wire.beginTransmission((data[i*3])>>1);
    Wire.write(data[i*3+1]);
    Wire.write(data[i*3+2]);
    Wire.write(data[i*3] + data[i*3+1] + data[i*3+2]);
    Wire.endTransmission(false);
  }
  delay(125);
}
