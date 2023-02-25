/*
Code by Kyle Rucker
Modified from: https://medium.com/@werneckpaiva/how-to-read-rc-receiver-signal-with-arduino-54e0447f6c3f
*/


#include <Arduino.h>
#define GEAR 2 //Gear Switch
#define THRO 3 //Throttle
#define AILE 4 //Aileron R
#define ELEV 5 //Elevator
#define RUDD 6 //Rudder
#define AUX1 7 //Aux1(Aileron L)

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH);
  //Serial.println(ch);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}


void setup(){
  Serial.begin(115200);
  pinMode(GEAR, INPUT_PULLUP);
  pinMode(THRO, INPUT_PULLUP);
  pinMode(AILE, INPUT_PULLUP);
  pinMode(ELEV, INPUT_PULLUP);
  pinMode(RUDD, INPUT_PULLUP);
  pinMode(AUX1, INPUT_PULLUP);
}

int THROValue, AILEValue, ELEVValue, RUDDValue, AUX1Value, GEARValue;

void loop() {
  int minLimitThro = 0;
  int maxLimitThro=100;
  int minLimit = -100;
  int maxLimit=100;

  GEARValue = readChannel(GEAR, minLimitThro, maxLimitThro, 0);
  THROValue = readChannel(THRO, minLimitThro, maxLimitThro, 0);
  AILEValue = readChannel(AILE, minLimit, maxLimit, 0);
  ELEVValue = readChannel(ELEV, minLimit, maxLimit, 0);
  RUDDValue = readChannel(RUDD, minLimit, maxLimit, 0);
  AUX1Value = readChannel(AUX1, minLimit, maxLimit, 0);
  
  Serial.print(" Gear: ");
  Serial.println(GEARValue);
  Serial.print(" Throttle: ");
  Serial.println(THROValue);
  Serial.print(" Aileron R: ");
  Serial.println(AILEValue);
  Serial.print(" Elevator: ");
  Serial.println(ELEVValue);
  Serial.print(" Rudder: ");
  Serial.println(RUDDValue);
  Serial.print(" Aileron L: ");
  Serial.println(AUX1Value);
  Serial.println();
  delay(500);
}