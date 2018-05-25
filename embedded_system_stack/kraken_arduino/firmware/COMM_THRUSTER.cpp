#include <Wire.h>
#include <Arduino.h>

void setup(){
  Serial.begin(57600);
  Wire.begin();
  pinMode(13,OUTPUT);
}

void loop(){
  uint8_t data[18];
  for(int i = 0;i<18; i++){
    while(!Serial.available());
    data[i] = Serial.read();
  }
  delay(10);
  for(int i=0; i<6; i++){
    Wire.beginTransmission((data[i*3])>>1);
    Wire.write(data[i*3+1]);
    Wire.write(data[i*3+2]);
    Wire.write(data[i*3] + data[i*3+1] + data[i*3+2]);
    Wire.endTransmission(false);
  }
  delay(115);
}
