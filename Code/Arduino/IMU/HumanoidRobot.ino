/*
    Name:       HumanoidRobot.ino
    Author:     Marvin Gravert
*/
//This code works with the teensy 3.6 with 3 MPU 9250 on 3 different i2c Pins with 4.7k pull up resistors
//it prints the roll pitch yaw from the Earth->Sensor Transformation to the serial connection to be read
//by either the serial monitor/Simulink running on PC/Simulink running on the Pi.
//gyroscope and magnometer can be calibrated 

//General Improvements: 
//Better calibration. The magnometer calibration is rather poor, hence the yaw angle 
//is not correctly calcd by the sensor fusion . Only the offset in the gyro is calibrated and no calibration
//is done for the accelerometer. Though it didnt impact the results nearly as much as the magnometer.
//Switch to SPI, if i2c is  bottelnecking your speed
//Try a different sensor fusion algorithm. Magdwick is fine if you only have limited processor speeds. The teensy offers more than enough
//therefore it might be sensible to switch to a more computational intensive one who provides higher accuracy

//Possible Code Improvements: 
//MultiThreading, general code quality(eg enums,std::array/std::vector), optimazation, EEPROM to save calibration values 


#include <i2c_t3.h>
#include "MadgwickAHRS.h"
#include "MPU9250.h"

float pitch, yaw, roll, pitch2, roll2, yaw2,roll3,pitch3,yaw3;
float deltaT = 0.0f; //used for integration in sensor fusion
uint32_t lastUpdate = 0; // time of last sensor fusion update

MPU9250 mySensor(Wire);//PIN  SCL 19 SDA 18
MPU9250 mySensor2(Wire1);//PIN SCL 37 SDA 38
MPU9250 mySensor3(Wire2);//PIN SCL 3 SDA 4

MadgwickAHRS fusion1;
MadgwickAHRS fusion2;
MadgwickAHRS fusion3;

float accVal[3],accVal2[3],accVal3[3];
float gyroVal[3],gyroVal2[3],gyroVal3[3];
float magVal[3],magVal2[3],magVal3[3];

uint32_t currentTime,lastTimeMag,outputTimer,lastTimeSampledSens1 ,lastTimeSampledSens2 ,lastTimeSampledSens3 ;
uint32_t gyro1SampleDelay = 0,gyro2SampleDelay = 0,gyro3SampleDelay = 0;

uint32_t startTime = 0, accEndTime = 0, gyroEndTime = 0, magEndTime = 0;

typedef union
{
	float number;
	uint8_t bytes[4];
} FLOATUNION_t;


//currently all serial prints are commeted out due to sending the information to simulink on the pi. If you output to a serial monitor 
//i suggest you comment them in. So debugging is easier(such as scanning the i2c connection for i2c devices to check connection to the
//sensors. 
//Its highly recommended to use a serial monitor to run the calibration. In this implementation the values are printed to the serial monitor
//and then manually entered into the programm code. Calibration can be started by setting 'doCalibration' to true (located in setup)
void setup()
{
	Serial.begin(230400);
	while (!Serial);
	Wire.begin();//PINS 19/18  SCL:19 SDA 18
	Wire1.begin();//PINS 37/38  SLC 37 SDA 38
	Wire2.begin();//PINS 3/4  SCL 3 SDA 4
	Wire.setClock(400000L);
	Wire1.setClock(400000L);
	Wire2.setClock(400000L);

	//SCAN I2C PINS TO VERIFY CONNECTION

	//Serial.println("Sensor 1");
	//i2cScanner(&Wire);
	//Serial.println("Sensor 2");
	//i2cScanner(&Wire1);
	//Serial.println("Sensor 3");
	//i2cScanner(&Wire2);


	//INIT the sensors
	mySensor.initMPU();
	//Serial.println("Frist done");
	mySensor2.initMPU();
	//Serial.println("Second Done");
	mySensor3.initMPU();
	//Serial.println("Third Done");

	delay(1000);
	
	bool doCalibration = false;

	//either calibrate gyroscope/ magnometer or use values from previous calibration 
	if (doCalibration) {
		Serial.print("First the gyroscopes of all Sensors will be calibrated dont move the sensors");
		delay(2000);
		mySensor.calibrateGyro();
		mySensor2.calibrateGyro();
		mySensor3.calibrateGyro();
		Serial.println("Gyroscope calibration done. Printing values");
		Serial.println("Offset of gyroscope of Sensor on Pin 18/19");
		mySensor.printGyroOffset();
		Serial.println("Offset of gyroscope of Sensor on Pin 37/38");
		mySensor2.printGyroOffset();
		Serial.println("Offset of gyroscope of Sensor on Pin 3/4");
		mySensor3.printGyroOffset();
		delay(20000);
		Serial.println("Now the magnometer will be calibrated, starting with the sensor on Port 18/19");
		Serial.println("Get ready to move the sensor in elliptical curves. Start in roughly 5 seconds");
		mySensor.calibrateMag();
		Serial.println("First is done");
		delay(5000);
		Serial.println("Now the magnometer of the sensor on Port 37/38 will be calibrated");
		Serial.println("Get ready. Start in roughly 5seconds");
		mySensor2.calibrateMag();
		Serial.println("Second is done");
		delay(5000);
		Serial.println("Now the magnometer of the sensor on Port 3/4 will be calibrated");
		Serial.println("Get ready. Start in roughly 5seconds");
		mySensor3.calibrateMag();
		Serial.println("Third is done");
		Serial.println("Now the values will be printed");
		Serial.println("Sensor on Pin 18/19:");
		mySensor.printMagCalibration();
		Serial.println("Sensor on Pin 37/38");
		mySensor2.printMagCalibration();
		Serial.println("Sensor on Pin 3/4");
		mySensor3.printMagCalibration();
		Serial.println("Thats all calibration. Procedding as normal in 20seconds");
		delay(40000);
	}
	else {

		//in the following the values for the offsets and scale of the sensors derived from previous calibration are entered  
		//Sensor 1 
		float mag1ErrorScale[] = { 1.15f,1.25f,0.97f };
		float mag1Offset[] = { 16.0f,10.7f,-47.94f};
		float gyro1Offset[] = { 1.045f,0.795f,-0.94f};

		//Sensor 2
		float mag2ErrorScale[] = { 1.01f,1.055f,0.995f };
		float mag2Offset[] = { 19.14f,0.18f,-12.615f };
		float gyro2Offset[] = { 3.465f,1.29f, -0.77f };

		//Sensor 3
		float mag3ErrorScale[] = { 0.96f,1.02f,0.995f };
		float mag3Offset[] = { 9.045f,28.69f,-39.045f };
		float gyro3Offset[] = { -0.415f,0.03f, -0.0f };

		//Set the offset and errorscales
		mySensor.setMagCalibration(&mag1ErrorScale[0], &mag1Offset[0]);
		mySensor2.setMagCalibration(&mag2ErrorScale[0], &mag2Offset[0]);
		mySensor3.setMagCalibration(&mag3ErrorScale[0], &mag3Offset[0]);

		mySensor.setGyroOffset(&gyro1Offset[0]);
		mySensor2.setGyroOffset(&gyro2Offset[0]);
		mySensor3.setGyroOffset(&gyro3Offset[0]);

		// PrInt to monitor for debugging
		/*Serial.println("30seconds to compare, for debuggig purposes");
		mySensor.printMagCalibration();
		mySensor2.printMagCalibration();
		mySensor3.printMagCalibration();

		mySensor.printGyroOffset();
		mySensor2.printGyroOffset();
		mySensor3.printGyroOffset();
		delay(3000);*/
	}
	delay(500);

	currentTime = micros();

	mySensor.readAcc(&accVal[0]);
	mySensor.readGyro(&gyroVal[0]);
	mySensor.readMag(&magVal[0]);

	mySensor2.readAcc(&accVal2[0]);
	mySensor2.readGyro(&gyroVal[0]);
	mySensor2.readMag(&magVal2[0]);

	mySensor3.readAcc(&accVal3[0]);
	mySensor3.readGyro(&gyroVal3[0]);
	mySensor3.readMag(&magVal3[0]);


	//different times due to possibly different lowpass filter
	gyro1SampleDelay = mySensor.getGyroSampleDelayInMicro();
	gyro2SampleDelay = mySensor2.getGyroSampleDelayInMicro();
	gyro3SampleDelay = mySensor3.getGyroSampleDelayInMicro();

	lastTimeSampledSens1 = currentTime;
	lastTimeSampledSens2 = currentTime;
	lastTimeSampledSens3 = currentTime;
	lastTimeMag = currentTime;
	outputTimer = currentTime;
}
void loop()
{	
	currentTime = micros();
	//gyro updates its values 
	//Sensor 1 Pin 19/18
	if ((currentTime - lastTimeSampledSens1) > gyro1SampleDelay) {
		/*mySensor.readGyro(&gyroVal[0]);
		mySensor.readAcc(&accVal[0]);*/
		mySensor.readAccGyro(&accVal[0], &gyroVal[0]);
		lastTimeSampledSens1 = currentTime;
	}
	//Sensor 2 Pin 37/38
	if ((currentTime - lastTimeSampledSens2) > gyro2SampleDelay) {
		/*mySensor2.readGyro(&gyroVal2[0]);
		mySensor2.readAcc(&accVal2[0]);*/
		mySensor2.readAccGyro(&accVal2[0], &gyroVal2[0]);
		lastTimeSampledSens2 = currentTime;
	}
	////Sensor 3 Pin 3/4
	if ((currentTime - lastTimeSampledSens3) > gyro3SampleDelay) {
	/*	mySensor3.readGyro(&gyroVal3[0]);
		mySensor3.readAcc(&accVal3[0]);*/
		mySensor3.readAccGyro(&accVal3[0], &gyroVal3[0]);
		lastTimeSampledSens3 = currentTime;
	}
	//the mag is decoupeld from the other sensor and doesnot have a lowpass filter
	if ((currentTime-lastTimeMag)> 10000 ) {//mag updates with 100Hz=>0.01s aka 10000 us
		mySensor.readMag(&magVal[0]);
		mySensor2.readMag(&magVal2[0]);
		mySensor3.readMag(&magVal3[0]);
		lastTimeMag = currentTime;
	}

	currentTime = micros();
	
	//time difference for the integration in the fusion
	deltaT = ((currentTime - lastUpdate) / 1000000.0f);
	lastUpdate = currentTime;
	
	//note that magX and magY are swapped and the minus infront of magZ. This was done to allign the KOS of both sensors on the MPU
	fusion1.update(gyroVal[0], gyroVal[1], gyroVal[2], accVal[0], accVal[1], accVal[2], magVal[1], magVal[0], -magVal[2], deltaT);
	fusion2.update(gyroVal2[0], gyroVal2[1], gyroVal2[2], accVal2[0], accVal2[1], accVal2[2], magVal2[1], magVal2[0], -magVal2[2], deltaT);
	fusion3.update(gyroVal3[0], gyroVal3[1], gyroVal3[2], accVal3[0], accVal3[1], accVal3[2], magVal3[1], magVal3[0], -magVal3[2], deltaT);
	
	if (lastUpdate-outputTimer > 100000L){
		roll = fusion1.getRoll();
		pitch = fusion1.getPitch();
		yaw = fusion1.getYaw();
		roll2 = fusion2.getRoll();
		pitch2 = fusion2.getPitch();
		yaw2 = fusion2.getYaw();
		roll3 = fusion3.getRoll();
		pitch3 = fusion3.getPitch();
		yaw3 = fusion3.getYaw();

		//FOR SIMULINK ON PC 
		/*send2Simulink(roll);
		send2Simulink(pitch);
		send2Simulink(yaw);
		send2Simulink(roll2);
		send2Simulink(pitch2);
		send2Simulink(yaw2);
		send2Simulink(roll3);
		send2Simulink(pitch3);
		send2Simulink(yaw3);
		Serial.print('\n');*/

		//FOR SIMULINK RUNNING ON THE PI. Note the missing terminator
		//I suggest you comment out every other Serial.print that may execute 
		//so the serial read block wont be confused(mind you no terminator!!)
		send2Simulink(roll);
		send2Simulink(pitch);
		send2Simulink(yaw);
		send2Simulink(roll2);
		send2Simulink(pitch2);
		send2Simulink(yaw2);
		send2Simulink(roll3);
		send2Simulink(pitch3);
		send2Simulink(yaw3);

		//FOR SERIAL MONITOR

		/*Serial.print(roll);
		Serial.print(" ");
		Serial.print(pitch);
		Serial.print(" ");
		Serial.print(yaw);
		Serial.print("     ");
		Serial.print(roll2);
		Serial.print(" ");
		Serial.print(pitch2);
		Serial.print(" ");
		Serial.print(yaw2);
		Serial.print("     ");
		Serial.print(roll3);
		Serial.print(" ");
		Serial.print(pitch3);
		Serial.print(" ");
		Serial.print(yaw3);
		Serial.print('\n');*/


		outputTimer = lastUpdate;
	}
}	

void i2cScanner(i2c_t3 *Wire) {
	byte error, address;
	int nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire->beginTransmission(address);
		error = Wire->endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknown error at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");

	delay(5000);           // wait 5 seconds for next scan
}

void send2Simulink(float tt) {
	//the input argument is sent byte for byte to the serial ports (little endian)
	FLOATUNION_t fa;
	fa.number = tt;
	for (int i = 0; i<4; i++)
	{
		Serial.write(fa.bytes[i]);
	}
}