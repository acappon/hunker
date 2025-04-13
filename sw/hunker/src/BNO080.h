/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.
*/

#pragma once

//The default I2C address for the BNO080 on the SparkX breakout is 0x4B. 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4A

//Platform specific configurations

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#else

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Registers
const char CHANNEL_COMMAND = 0;
const char CHANNEL_EXECUTABLE = 1;
const char CHANNEL_CONTROL = 2;
const char CHANNEL_REPORTS = 3;
const char CHANNEL_WAKE_REPORTS = 4;
const char CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_UNCALIBRATED_GYRO 0x07
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define TARE_NOW 0
#define TARE_PERSIST 1
#define TARE_SET_REORIENTATION 2

#define TARE_AXIS_ALL 0x07
#define TARE_AXIS_Z   0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

class BNO080
{
public:
    BNO080();
    ~BNO080();

	bool begin(); //Returns true if the BNO080 is detected on the I2C bus   

	void softReset();	  //Try to reset the IMU via software
	bool hasReset(); //Returns true if the sensor has reported a reset. Reading this will unflag the reset.
	unsigned char resetReason(); //Query the IMU for the reason it last reset
	void modeOn();	  //Use the executable channel to turn the BNO on
	void modeSleep();	  //Use the executable channel to put the BNO to sleep

	float qToFloat(short fixedPointValue, unsigned char qPoint); //Given a Q value, converts fixed point floating to regular floating point number

	bool waitForI2C(); //Delay based polling for I2C traffic
    bool waitForI2C_Blocking(int timeoutMs);

	bool receivePacket(void);
	bool getData(unsigned short bytesRemaining); //Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
	bool sendPacket(unsigned char channelNumber, unsigned char dataLength);
    std::string getHexText_8(unsigned char* data, int length=1);
    std::string getHexText_16(unsigned short* data, int length=1);
    std::string getHexText_32(unsigned int* data, int length=1);
    std::string getPacketText(void);
	std::string getHeaderText(void); //Prints the current shtp header (only)
 
	void enableRotationVector(unsigned short timeBetweenReports);
	void enableGameRotationVector(unsigned short timeBetweenReports);
	void enableARVRStabilizedRotationVector(unsigned short timeBetweenReports);
	void enableARVRStabilizedGameRotationVector(unsigned short timeBetweenReports);
	void enableAccelerometer(unsigned short timeBetweenReports);
	void enableLinearAccelerometer(unsigned short timeBetweenReports);
	void enableGravity(unsigned short timeBetweenReports);
	void enableGyro(unsigned short timeBetweenReports);
	void enableUncalibratedGyro(unsigned short timeBetweenReports);
	void enableMagnetometer(unsigned short timeBetweenReports);
	void enableTapDetector(unsigned short timeBetweenReports);
	void enableStepCounter(unsigned short timeBetweenReports);
	void enableStabilityClassifier(unsigned short timeBetweenReports);
	void enableActivityClassifier(unsigned short timeBetweenReports, unsigned int activitiesToEnable, unsigned char (&activityConfidences)[9]);
	void enableRawAccelerometer(unsigned short timeBetweenReports);
	void enableRawGyro(unsigned short timeBetweenReports);
	void enableRawMagnetometer(unsigned short timeBetweenReports);
	void enableGyroIntegratedRotationVector(unsigned short timeBetweenReports);

	bool dataAvailable(void);
	unsigned short getReadings(void);
	unsigned short parseInputReport(void);   //Parse sensor readings out of report
	unsigned short parseCommandReport(void); //Parse command responses out of report

	void getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, unsigned char &accuracy);
	float getQuatI();
	float getQuatJ();
	float getQuatK();
	float getQuatReal();
	float getQuatRadianAccuracy();
	unsigned char getQuatAccuracy();

	void getAccel(float &x, float &y, float &z, unsigned char &accuracy);
	float getAccelX();
	float getAccelY();
	float getAccelZ();
	unsigned char getAccelAccuracy();

	void getLinAccel(float &x, float &y, float &z, unsigned char &accuracy);
	float getLinAccelX();
	float getLinAccelY();
	float getLinAccelZ();
	unsigned char getLinAccelAccuracy();

	void getGyro(float &x, float &y, float &z, unsigned char &accuracy);
	float getGyroX();
	float getGyroY();
	float getGyroZ();
	unsigned char getGyroAccuracy();

	void getUncalibratedGyro(float &x, float &y, float &z, float &bx, float &by, float &bz, unsigned char &accuracy);
	float getUncalibratedGyroX();
	float getUncalibratedGyroY();
	float getUncalibratedGyroZ();
	float getUncalibratedGyroBiasX();
	float getUncalibratedGyroBiasY();
	float getUncalibratedGyroBiasZ();
	unsigned char getUncalibratedGyroAccuracy();


	void getFastGyro(float &x, float &y, float &z);
	float getFastGyroX();
	float getFastGyroY();
	float getFastGyroZ();

	void getMag(float &x, float &y, float &z, unsigned char &accuracy);
	float getMagX();
	float getMagY();
	float getMagZ();
	unsigned char getMagAccuracy();

	void getGravity(float &x, float &y, float &z, unsigned char &accuracy);
	float getGravityX();
	float getGravityY();
	float getGravityZ();
	unsigned char getGravityAccuracy();

	void calibrateAccelerometer();
	void calibrateGyro();
	void calibrateMagnetometer();
	void calibratePlanarAccelerometer();
	void calibrateAll();
	void endCalibration();
	void saveCalibration();
	void requestCalibrationStatus(); //Sends command to get status
	bool calibrationComplete();   //Checks ME Cal response for byte 5, R0 - Status

	void tareNow(bool zAxis=false, unsigned char rotationVectorBasis=TARE_ROTATION_VECTOR);
	void saveTare();
	void clearTare();
	
	unsigned char getTapDetector();
	unsigned int getTimeStamp();
	unsigned short getStepCount();
	unsigned char getStabilityClassifier();
	unsigned char getActivityClassifier();

	short getRawAccelX();
	short getRawAccelY();
	short getRawAccelZ();

	short getRawGyroX();
	short getRawGyroY();
	short getRawGyroZ();

	short getRawMagX();
	short getRawMagY();
	short getRawMagZ();

	float getRoll();
	float getPitch();
	float getYaw();

	void setFeatureCommand(unsigned char reportID, unsigned short timeBetweenReports);
	void setFeatureCommand(unsigned char reportID, unsigned short timeBetweenReports, unsigned int specificConfig);
	void sendCommand(unsigned char command);
	void sendCalibrateCommand(unsigned char thingToCalibrate);
	void sendTareCommand(unsigned char command, unsigned char axis=TARE_AXIS_ALL, unsigned char rotationVectorBasis=TARE_ROTATION_VECTOR);

	//Metadata functions
	short getQ1(unsigned short recordID);
	short getQ2(unsigned short recordID);
	short getQ3(unsigned short recordID);
	float getResolution(unsigned short recordID);
	float getRange(unsigned short recordID);
	unsigned int readFRSword(unsigned short recordID, unsigned char wordNumber);
	void frsReadRequest(unsigned short recordID, unsigned short readOffset, unsigned short blockSize);
	bool readFRSdata(unsigned short recordID, unsigned char startLocation, unsigned char wordsToRead);

    bool openI2C(const char* device, int address);
    void sleep_ms(int ms);
    bool isAirborne();

	//Global Variables
	unsigned char shtpHeader[4]; //Each packet has a header of 4 bytes
	unsigned char shtpData[MAX_PACKET_SIZE];
	unsigned char sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	unsigned char commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	unsigned int metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

private:
	//Variables
	unsigned char _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this.

	bool _printDebug = false; //Flag to print debugging variables

	bool _hasReset = false;		// Keeps track of any Reset Complete packets we receive. 

	//These are the raw sensor values (without Q applied) pulled from the user requested Input Report
	unsigned short rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	unsigned short rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	unsigned short rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	unsigned short rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ, rawBiasX, rawBiasY, rawBiasZ, UncalibGyroAccuracy;
	unsigned short rawMagX, rawMagY, rawMagZ, magAccuracy;
	unsigned short rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	unsigned short rawFastGyroX, rawFastGyroY, rawFastGyroZ;
	unsigned short gravityX, gravityY, gravityZ, gravityAccuracy;
	unsigned char tapDetector;
	unsigned short stepCount;
	unsigned int timeStamp;
	unsigned char stabilityClassifier;
	unsigned char activityClassifier;
	unsigned char *_activityConfidences;						  //Array that store the confidences of the 9 possible activities
	unsigned char calibrationStatus;							  //Byte R0 of ME Calibration Response
	unsigned short memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
	unsigned short memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
	unsigned short memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor

	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	short rotationVector_Q1 = 14;
	short rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
	short accelerometer_Q1 = 8;
	short linear_accelerometer_Q1 = 8;
	short gyro_Q1 = 9;
	short magnetometer_Q1 = 4;
	short angular_velocity_Q1 = 10;
	short gravity_Q1 = 8;

    int m_i2c_fd = -1;
};