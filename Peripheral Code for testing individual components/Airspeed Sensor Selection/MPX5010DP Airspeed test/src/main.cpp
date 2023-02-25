#include <Arduino.h>
#include <SPI.h>

const int mpx5010dp_Pin = A0;

/* Function to read airspeed from MPX5010DP */
float airSpeedMPX5010DP() {
  return 2.23694*sqrt(18.024409*analogRead(mpx5010dp_Pin)-738.279808); //Transfer function from datasheet without +- error term, modified for velocity output in mph
} //end airSpeedMPX5010DP()

double voltageRead() {
  return analogRead(mpx5010dp_Pin)*5/1023;
}

void setup() {
  Serial.begin(115200);
} //end setup()

void loop() {
  Serial.println(airSpeedMPX5010DP(), 8);
  //Serial.println(voltageRead(), 8);
} //end loop()