/*
ADD LICENSING, DATES, MY NAME, etc
btw don't forget to edit the readme.txt!
*/

/* Libraries */
#include <Arduino.h> //Include this to code from Platformio, not needed in Arduino IDE
#include "mpu9250.h"
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/* Mpu9250 object */
bfs::Mpu9250 imu;
const int chipSelect = 10; //IMU CS Pin to Arduino Pin10

/* Global Variables for Iterative File Naming */
String file;

/* Global Variables for Transmitter Switch Trigger (GEAR) */
#define GEARPIN 2
volatile long StartTime = 0;
volatile long CurrentTime = 0;
volatile long gear = 0;
int gearState = 0;
bool tf = 1;

/* Global Variables for reading servo channel pwm */
#define THRO 3 //Throttle
#define AILE 4 //Aileron R
#define ELEV 7 //Elevator
#define RUDD 6 //Rudder
#define AUX1 5 //Aux1(Aileron L)
int THROValue, AILEValue, ELEVValue, RUDDValue, AUX1Value;

/* Global Variables for BME280 */
#define SEALEVELPRESSURE_HPA (1020.6575) //UNITS hPa --> Default = 1013.25hPa -->Change this to known data!!!   Change this to known data!!!    Change this to known data!!!    Change this to known data!!!    Change this to known data!!!
Adafruit_BME280 bme; // Naming the library class "bme"? Idrk

/* Function used for reading gear pwm signal with a hardware interrupt */
void PulseTimer(){
  //Measure the time elapsed between interrupts:
  CurrentTime = micros();
  if (CurrentTime > StartTime){
    gear = CurrentTime - StartTime;
    StartTime = CurrentTime;
  } //endif
} //end PulseTimer()

/* Function used to read servo pwm without using interrupts */
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH);
//  Serial.println(ch); //For testing
  if (ch < 100) return defaultValue; //Channel is offline, return 0 as a default value
  return map(ch, 1000, 2000, minLimit, maxLimit); //1500us is the "center" or "zero" line for pwm servos, min is 1000, max is 2000. Map the throttle to 0->100 and the servos to -100->100 in the main loop below
} //end readChannel()

/* Setup Loop */
void setup() {
  Serial.begin(115200);
  while(!Serial) {} //Pause until Serial coms begin

  /* Initialize I2C */
  Wire.begin(); //Start I2C Bus
  Wire.setClock(400000); //I2C Clock

  /* Initialize IMU */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM); //Configure IMU I2C Stuff
  if (!imu.Begin()) { //Initialize IMU
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }//endif

  //Set the sample rate divider:
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }//endif
  
  /* Setup for PWM */
  //Set Pin 2 to read PWM from gear channel of the receiver:
  pinMode(GEARPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GEARPIN),PulseTimer,CHANGE);
  //Set pins for reading servo pwm:
  pinMode(THRO, INPUT_PULLUP);
  pinMode(AILE, INPUT_PULLUP);
  pinMode(ELEV, INPUT_PULLUP);
  pinMode(RUDD, INPUT_PULLUP);
  pinMode(AUX1, INPUT_PULLUP);

  /* BME280 Setup */
  bme.begin();

  /* Initialize SD Card: */
  Serial.print("Initializing SD card...");
  //See if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);// Pause the script here
  }//endif
  Serial.println("card initialized.");

  /* Give a prompt to the user */
  Serial.println();
  Serial.println("Waiting for PWM signal from GEAR Channel");

} //end setup()

/* Main Loop */
void loop() {
  if (tf==1) {
    /* Read the SD for existing filenames; increment files for different test cases */
    String fileTitle = "data";
    String fileType = ".txt";
    for (int fileCounter = 0; fileCounter<10000; fileCounter++) {
        String fileName = fileTitle + String(fileCounter)+fileType;
        File dataFile = SD.open(fileName);
        if (!dataFile.available()) { //Returns TRUE if the file has data, FALSE if the file is empty
          file=fileName;
          break;
        } //end if !dataFile.available
        dataFile.close(); //Always close after reading/writing
    } //end for fileCounter=0  
  } //end if tf==1

   if (gear < 2000) {
     gearState = gear;
//     Serial.println(gearState); //For testing
   } //end if gear<2000
  
  while (gearState > 1800) {
    tf=0;
    if (gear < 2000) {
      gearState = gear;
    } //end if gear < 2000
//    Serial.println(gearState); //for testing

    if (imu.Read()) {
      /* Reading servo pwm inputs */
      int minLimitThro = 0; //Minmax for throttle is 0-100
      int maxLimitThro=100;
      int minLimit = -100; //Minmax for servos is -100-100
      int maxLimit=100;
      THROValue = readChannel(THRO, minLimitThro, maxLimitThro, 0);
      AILEValue = readChannel(AILE, minLimit, maxLimit, 0);
      ELEVValue = readChannel(ELEV, minLimit, maxLimit, 0);
      RUDDValue = readChannel(RUDD, minLimit, maxLimit, 0);
      AUX1Value = readChannel(AUX1, minLimit, maxLimit, 0);

      /* Store data to a datastring: */
      String dataString=""; //initialize string variable
      dataString+=String(millis()); //Column 1, Time in ms
      dataString+=("\t");
      dataString+=String(imu.accel_x_mps2()); //Column 2 Accel x, m/s^2
      dataString+=("\t");
      dataString+=String(imu.accel_y_mps2()); //Column 3 Accel y
      dataString+=("\t");
      dataString+=String(imu.accel_z_mps2()); //Column 4 Accel z
      dataString+=("\t");
      dataString+=String(imu.gyro_x_radps()); //Column 5 Gyro x, rad/s
      dataString+=("\t");
      dataString+=String(imu.gyro_y_radps()); //Column 6 Gyro y
      dataString+=("\t");
      dataString+=String(imu.gyro_z_radps()); //Column 7 Gyro z
      dataString+=("\t");
      dataString+=String(imu.mag_x_ut()); //Column 8, Magnetometer x, uT
      dataString+=("\t");
      dataString+=String(imu.mag_y_ut()); //Column 9, Magnetometer y, uT
      dataString+=("\t");
      dataString+=String(imu.mag_z_ut()); //Column 10, Magnetometer z, uT
      dataString+=("\t");
      dataString+=String(imu.die_temp_c()); //Column 11 Temperature, C
      dataString+=("\t");
      dataString+=String(bme.readAltitude(SEALEVELPRESSURE_HPA)); //Column 12, Altitude (m)
      dataString+=("\t");
      dataString+=THROValue; //Column 13, Throttle pwm, 0-100
      dataString+=("\t");
      dataString+=AILEValue; //Column 14, R Aileron pwm, -100-100
      dataString+=("\t");
      dataString+=AILEValue; //Column 15, L Aileron pwm, -100-100
      dataString+=("\t");
      dataString+=RUDDValue; //Column 16, Rudder pwm, -100-100
      dataString+=("\t");
      dataString+=ELEVValue; //Column 17, Elevator pwm, -100-100
      Serial.println(dataString); // For Testing
      
      /* Write datastring to SD card: */
      File dataFile = SD.open(file, FILE_WRITE); //Open the file
      //If it's available, write to it:
//      Serial.print("Trying to write to "); //For testing
//      Serial.println(file); //For testing
      if (dataFile) {
         /* Serial Prints for Testing: */
//        Serial.print("Currently Writing to ");
//        Serial.println(file);
//        Serial.println(dataString);      
        dataFile.println(dataString);
        dataFile.close();
      }//end if dataFile

      /* Print Error if the SD isnt available: */
      else {
          Serial.println(dataFile);
          Serial.print("error opening ");
          Serial.println(file);
      }//end else
    } //endif imu.Read
  } //end while Gearstate
  tf=1; //Write a new file once the switch is turned back off
}//end void loop
