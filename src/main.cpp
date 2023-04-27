/*
ADD LICENSING, DATES, MY NAME, etc
btw don't forget to edit the readme.txt!
*/

/* Include Libraries */
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

/* Global Variables for MPX5010DP Analog Airspeed Sensor */
const int mpx5010dp_Pin = A3;

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
//Variables for mapping the pwm to the correct range:
const int8_t minLimitThro = 0; //Minmax for throttle is 0-100
const int8_t maxLimitThro=100;
const int8_t minLimit = -100; //Minmax for servos is -100-100
const int8_t maxLimit=100;

/* Global Variables for BME280 */
#define SEALEVELPRESSURE_HPA (1020.6575) //UNITS hPa --> Default = 1013.25hPa -->Change this to known data!!!   Change this to known data!!!    Change this to known data!!!    Change this to known data!!!    Change this to known data!!!
Adafruit_BME280 bme; // Naming the library class "bme"? Idrk

/* Function to read airspeed (mph) from MPX5010DP */
float airSpeedMPX5010DP() {
  const float vs = 5; //Supply voltage in V
  const float vOffset = 0.2; //Offset voltage in V
  float pressure; //Pressure in kPa
  float vMS; //Velocity in m/s
  float vMPH; //Velocity in mph
  float rho = 1.225; //Density of air in kg/m^3
  float voltage; //Voltage in V
  float val; //Analog value from MPX5010DP

  /* Read MPX5010DP Analog Output */
  val=analogRead(mpx5010dp_Pin);
  /* Convert Analog Values to Voltages in V */
  voltage = val * 5 / 1023-vOffset;
  /* Convert Voltage to Pressure in kPa */
  pressure = (1 / 0.09) * (((voltage) / vs) - 0.04); //Transfer function from datasheet))
  /* Convert Pressure to Airspeed in m/s */
  if (pressure < 0) {
    pressure = 0;
  } //end if (pressure < 0)
  vMS = sqrt(2*pressure*1000/rho);
  /* Convert Airspeed in m/s to Airspeed in mph */
  vMPH = vMS*2.23694;
 
  return vMPH; //Transfer function from datasheet without +- error term, modified for velocity output in mph} //end airSpeedMPX5010DP()
} //end airSpeedMPX5010DP()

/* Function to convert the analog reading from the MPX5010DP to Volts*/
/* Use this for initial testing, but the function above incorporates this unit conversion */
/*Delete when testing is successful and complete*/
double voltageRead() {
  return analogRead(mpx5010dp_Pin)*5/1023;
} //end voltageRead()

/* Function used for reading gear pwm signal with a hardware interrupt */
void PulseTimer(){
  //Measure the time elapsed between interrupts:
  CurrentTime = micros();
  if (CurrentTime > StartTime){
    gear = CurrentTime - StartTime;
    StartTime = CurrentTime;
  } //end if (CurrentTime > StartTime)
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
  } //end if (!imu.Begin())

  //Set the sample rate divider:
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  } //end if (!imu.ConfigSrd(19))
  
  /* Setup for PWM */
  //Set Pin 2 to read PWM from gear channel of the receiver:
  pinMode(GEARPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GEARPIN),PulseTimer,CHANGE); //Attach interrupt to GEARPIN
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
  } //end if (!SD.begin(chipSelect))
  Serial.println("card initialized."); //Card is initialized

  /* Give a prompt to the user */
  Serial.println("Waiting for PWM signal from GEAR Channel");

} //end setup()

/* Main Loop */
void loop() {
  /* Read the SD for existing filenames; increment files for different test cases: */
  if (tf==1) { //True if the switch state has changed to ON, False if the switch state has not changed since the last loop (Check this in later versions)
    const String fileTitle = "data"; //filename, arbitrary
    const String fileType = ".txt"; //Change this to .csv for excel

    /* Check whether files already have data. If yes, ignore and create a new file with an iterative name*/
    for (int fileCounter = 0; fileCounter<10000; fileCounter++) {
      String fileName = fileTitle + String(fileCounter)+fileType;
      File dataFile = SD.open(fileName);
      if (!dataFile.available()) { //Returns TRUE if the file has data, FALSE if the file is empty
        file=fileName;
        break;
      } //end if (!dataFile.available)
      dataFile.close(); //Always close after reading/writing
    } //end for (int fileCounter = 0; fileCounter<10000; fileCounter++)  
  } //end if (tf==1)

  /* Update gearState */
  if (gear < 2000) { //Always TRUE
    gearState = gear; //Update gearState
//     Serial.println(gearState); //For testing
  } //end if (gear<2000)
  
  while (gearState > 1800) {
    tf=0; //Prevent a new file from being created until the switch is turned off and back on

    /* Update gearState */
    if (gear < 2000) { //Always TRUE
      gearState = gear; //Update gearState
    } //end if (gear < 2000)
//    Serial.println(gearState); //for testing

    /* Check for new imu data: */
    if (imu.Read()) { //True if the imu records new data, false if the imu is offline
      /* Read the servo pwm inputs. Each read causes a ~2-3ms delay in the code, by the nature of the pulseIn() function used in the readChannel() function */
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
      dataString+=String(airSpeedMPX5010DP()); //Column 13, Airspeed, mph
      dataString+=("\t");
      dataString+=THROValue; //Column 14, Throttle pwm, 0-100
      dataString+=("\t");
      dataString+=AILEValue; //Column 15, R Aileron pwm, -100-100
      dataString+=("\t");
      dataString+=AILEValue; //Column 16, L Aileron pwm, -100-100
      dataString+=("\t");
      dataString+=RUDDValue; //Column 17, Rudder pwm, -100-100
      dataString+=("\t");
      dataString+=ELEVValue; //Column 18, Elevator pwm, -100-100
      Serial.println(dataString); // For Testing
      
      /* Write datastring to SD card: */
      File dataFile = SD.open(file, FILE_WRITE); //Open the file
      //If it's available, write to it:
//      Serial.print("Trying to write to "); //For testing
//      Serial.println(file); //For testing
      if (dataFile) { //True if the file is available, false if the file is not available
         /* Serial Prints for Testing: */
//        Serial.print("Currently Writing to ");
//        Serial.println(file);
//        Serial.println(dataString);      
        dataFile.println(dataString);
        dataFile.close();
      }//end if dataFile

      /* Print Error if the SD isnt available: */
      else { //If the file is not available
          Serial.println(dataFile);
          Serial.print("error opening ");
          Serial.println(file);
      }//end else
    } //endif (imu.Read())
  } //end while (gearState > 1800)
  tf=1; //Write a new file once the switch is turned back off
} //end void loop()
