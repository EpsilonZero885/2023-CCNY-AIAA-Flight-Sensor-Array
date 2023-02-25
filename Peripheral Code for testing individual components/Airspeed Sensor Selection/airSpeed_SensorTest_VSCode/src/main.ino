/*
* note that timing is critical - see delay (5)
* and bps set to 115200
* works ok, but clean up 5-19-2019 jaf
*/
#include <Arduino.h> //Include this whenever writing code for arduino in VSCode
#include <Wire.h>

#define AIRSPEED_ADDRESS  0x75
#define WRITE_BIT         0x00
#define READ_BIT          0x01
float velocityKMH;
float V_0 = 5.0; // supply voltage to the pressure sensor
float rho = 1.204; // density of air
int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;

void setup() {
    Wire.begin();
    Serial.begin(115200);
    Serial.println("Program started");

  /*
  for (int ii=0;ii<offset_size;ii++){
    Wire.beginTransmission(AIRSPEED_ADDRESS);  
    Wire.requestFrom(AIRSPEED_ADDRESS, 2);
    offset += Wire.read()-(1023/2);
    Wire.endTransmission();
  } //End for
*/
  offset /= offset_size;
}

void loop() {
    float adc_avg = 0;
    float veloc = 0.0;
    for (int ii=0;ii<veloc_mean_size;ii++){
        Wire.beginTransmission(AIRSPEED_ADDRESS);  
        Wire.requestFrom(AIRSPEED_ADDRESS, 2);
        adc_avg+= Wire.read()-offset;
        Wire.endTransmission();
    } //endfor 
    adc_avg/=veloc_mean_size;
    Serial.println(adc_avg);

    if (adc_avg>512-zero_span and adc_avg<512+zero_span) {
    } //endif 
    else{
        if (adc_avg<512) {
        veloc = -sqrt((-10000.0*((adc_avg/1023.0)-0.5))/rho);
        } 
        else {
            veloc = sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
        } //endif
    } //endelse
  Serial.print(veloc);
  Serial.print("\t");
  Wire.beginTransmission(AIRSPEED_ADDRESS);  
  Wire.requestFrom(AIRSPEED_ADDRESS, 2);
  float printVel = Wire.read();
  Wire.endTransmission();
  Serial.println(printVel);
  delay(5);
} //end void loop()
