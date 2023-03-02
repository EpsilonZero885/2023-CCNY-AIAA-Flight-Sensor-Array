Contact:	Kyle Rucker 
		krucker000@citymail.cuny.edu

Description:
	Development of an Arduino-based flight data recorder for use in the 2023 AIAA DBF competition by the CCNY AIAA chapter 

Goal: 
	Record sensor data and pwm motor inputs during flight testing for post-processing in MATLAB

Github:
	https://github.com/EpsilonZero885/2023-CCNY-AIAA-Flight-Sensor-Array.git

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parts ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Kyle's stuff:
	Arduino Uno
	Adafruit 5V Ready Micro-SD Breakout board+
	Adafruit Powerboost 1000C
	3.7V 1200mAh PKCELL LP503562 LiPo Battery
	16GB Micro-SD Card (w Adapter for plugging into cpu)
	Misc M/F Dupont cables
	MicroUSB Cable to charge battery
	Adafruit BME280 Humidity + Barometric Pressure + Temperature Sensor Breakout
	Protoboard, misc wire
	Very tiny switch

AIAA Club stuff:
	MPU9250 IMU
	Spektrum DX6i Transmitter
	Spektrum AR8000 X Receiver
	MPX5010DP Analog Differential Pressure Sensor
	4x Parallax digital Standard Servo #900-00005

I need:
	USB-A to USB-B (connect powerboost to uno, only needed for flight-ready version)

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Thoughts ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

I have a gps module. Only accurate to within ~5m though, maybe not helpful?

Should we use a real-time clock in lieu of the millis() function?

Servo stuttering issues at Vin=5V. Tested with a Vin=5.5V supply and these issues went away!

NOTE: I get bad results when the Arduino and the receiver are powered over seperate lines. This may be because of a voltage difference between the arduino and the receiver being transmitted over the pwm pins to the arduino?
	If I cannot fix this, this means we need to supply 5.5V BEC voltage to the arduino/sensors. This is not the end of the world.
	Ideally, I would like to power the arduino+sensors off one LiPo, and the receiver/servos over the BEC line from the ESC, powered from a seperate battery. 
		This would allow us to measure battery life w.out considering the arduino+sensors. How to do this?
			Try: Common the ground! 
				This worked! Updated the Protoboard to add another GND breakout which will run to the Arduino
 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Pinouts ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
IMU (I2C com)
	Arduino		IMU
	5V		VCC
	GND		GND
	A4		SDA
	A5		SCL

SD Breakout
	Arduino		SD Breakout
	5V		5V
	GND		GND
	13		CLK
	12		DO
	11		DI
	10		CS
	N/A		CD
Spektrum AR8000 Receiver
	Arduino		Receiver
	5V		THRO+ (Center pin, used in testing only)
	GND		THRO- (Closest to label, used in testing only)
	Pin2		GEAR PWM (All pwm pins are furthest from the label)
	Pin3		THRO PWM
	Pin4		AILE PWM (Right Aileron)
	Pin5		ELEV PWM
	Pin6		RUDD PWM
	Pin7		AUX1 PWM (Left Aileron)

Adafruit BME280 Humidity + Barometric Pressure + Temperature Sensor Breakout
	Arduino		BME280
	5V		VIN
	GND		GND
	A4 (SDA)	SDI
	A5 (SCL)	SCK

MPX5010DP Analog Differential Pressure Sensor (Pin with a Notch is Pin1)
	Arduino		MPX5010DP
	A0		1
	GND		2
	5V		3
	N/C		4
	N/C		5
	N/C		6

Adafruit Powerboost 1000C
	Powerboost	Arduino (and basically all the other sensors too tbh)
	5V		5V
	GND		GND
	ENA		Connected to tiny switch, switches between N/C and common GND


As tested on my home psu, at 5V the SD+IMU+BME280+Uno draw ~80mA. At 1200mAh this testing apparatus can run for 15hours
	This value jumps to about 130mA when the psu also supplies 5V to the receiver, about 9.23 hours at 1200mAh
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Programming ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			~~~VERSION NOTES~~~
v1: Initial script. 
	Reads IMU values. 
	Converts Magnetometer readings from uT to Degrees.
	Writes sensor values to SD card, iterative file naming

v2: Almost the same as v1
	Iterative filenaming happens in setup loop, v1 was creating lots of new files in the main loop

v3: Using GEAR pwm as a trigger to start data collection
	Moved iterative filenaming to main and added a counter
	Data collection is triggered by a >1800 signal from the GEAR pwm pin
	All works well, ready for deployment
	
v4: Added BME280 for altitude data
	Also started experimenting with the airspeed sensor, so far results have been NFG

v5: Using VSCode with platformio instead of arduino ide! 
	Working toward using icint pins to receive all servo channel signals:
		THRO (Throttle)
		AILE (Right Aileron)
		AUX1 (Left Aileron)
		ELEV (Elevator)
		RUDD (Rudder)

v6: pcint pins are tricky!
	I can use pulseIn to read pwm from any pin w.out using interrupts, sacrificing speed because nothing else will execute while pulseIn is reading. 
	Servo pwm ranges from 1000us to 2000us (1-2ms). Assuming worst case (3ms delay per reading) for 5 channels, this causes a 15ms delay between each reading. 
		This may be acceptable!
	Creating a protoboard to parallel each channel to a servo and to the arduino, will add pinout/schematic when done.
	Also removed Eagle Tree Airspeed Microsensor due to lack of documentation, esp wrt a transfer function. 
		Ordering MPX5010DP analog pressure sensor, will incorporate after delivery

v7: created protoboard to parallel out channels for THRO, AILE, ELEV, RUDD, and AUX1
	Switched pins 5 and 7, updated the code to reflect this
	Per Michael's request, removed heading in degrees from printout. Instead printing uT values for each axis in individual columns 

v8: Incorporating MPX5010DP analog pressure sensor using transfer function from data sheet.
	Did not incorporate the +- error term of the transfer function. Check calibration upon delivery

			~~~GENERAL NOTES~~~
IMU script using "Bolder Flight Systems MPU9250 by Brian Taylor" Library
	URL: https://github.com/bolderflight/invensense-imu
		Acceleration is given m*s^-2
		Angular Velocity is given rad/s
		Magnetometer is given microTeslas*
			Change from uT to cardinal directions:
			https://www.circuitbasics.com/how-to-setup-a-magnetometer-on-the-arduino/
		Temperature is given degC

SD Breakout Script given by Adafruit, is actually built-in to IDE already
	Format SD FAT16 or FAT32 ONLY!!!
	File names are limited to 8 Characters only! so dataXXXX.txt where XXXX=0000 Through 9999, 10k total files

Map one of the auxillary channels on the spektrum ar8000 to the spektrum dx6i transmitter: https://github.com/TheBoredRobot/Arduino-Examples/blob/master/RC_Read.ino
	NOTE: We are using DUALAILERON Wingtail Mix. The left aileron is controlled by AUX1 while the right is controlled by AILE
	Mapped our arduino pin2 to GEAR
		In Travel Adjust Menu, set GEAR to 0, what happens? PWM Range is too small, set it to 100% for better Arduino readings
Record pwm inputs from rc receiver channels to servo motors

The BME280 can measure humidity, temperature, and pressure.  
	I am ONLY using this sensor to read altitude, which is calculated using pressure readings by the bme280 adafruit library. To increase accuracy, input specific sea-level atmospheric pressure specific to your day/time.


I2C Sensor Addresses:
	IMU	0x68
	BME280	0x77
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Links ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Arduino Uno:
https://docs.arduino.cc/hardware/uno-rev3

Powerboost:
https://www.adafruit.com/product/2465

LiPo:
https://www.adafruit.com/product/258

SD Breakout:
https://www.adafruit.com/product/254

IMU:
https://learn.sparkfun.com/tutorials/mpu-9250-hookup-guide/all

BME280:
https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout

Reading PWM signals:
https://www.theboredrobot.com/post/reading-values-from-an-rc-receiver-using-arduino - Used for GEAR Channel
https://medium.com/@werneckpaiva/how-to-read-rc-receiver-signal-with-arduino-54e0447f6c3f - Used to read the other 5 channels

MPX5010DP:
https://www.nxp.com/docs/en/data-sheet/MPX5010.pdf
https://docs.arduino.cc/built-in-examples/basics/ReadAnalogVoltage