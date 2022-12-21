/*********************************************************************
 * Project Power meter Modbus_Project
 * Description:  Creating a project that communicates with a
 *               peripherial device, power meter, using Modbus RTU
 *               over RS485.
 * Author:       Allwyn Joseph
 * Date:         12/15/22
 *********************************************************************/
#include "Particle.h"

/********************************************************************/
//! /brief library includes
// This #include statement was automatically added by the Particle IDE.
#include <ModbusMaster.h>

// This #include statement was automatically added by the Particle IDE.
#include <JsonParserGeneratorRK.h>

/********************************************************************/
//! /brief  declarations
#define SUCCESS 1 // success code for modbus
#define FAIL -1	  // fail code for modbus
#define BAUD 9600 // baud rate for modbus and UART
#define MODADDR 1 // sets server (slave) address for modbus
#define SERIAL1 1 // used to select Serial1, RX/TX pins for Modbus

/********************************************************************/
//! /brief global constants
float voltage1;
float voltage2;
float voltage3;
float current1;
float current2;
float current3;

const int RT = D5; // constant for R/T pin set to D5, PWM1 on MikroBUS1

int result; // modbus error code

/********************************************************************/
//! /brief global variables
int senFreq = 30; // sensor publish frequency (in seconds), preset to 30 seconds
int warFreq = 30; // warning publish frequency (in seconds), preset to 30 seconds

/********************************************************************/
//! /brief Other settings
// Name of published event with temperature/humidity information
const char *sensorEventName = "modbus-sensor";

// Name of the published event for error/warning messages. This is typically the
// same as the sensorEventName, but could be different if you prefer.
const char *errorEventName = sensorEventName;

// How often to check the sensor. Default: Every 1 second
const std::chrono::milliseconds checkInterval = 1s; // check sensor every 1 second

/********************************************************************/
//! /brief particle platform initialization
SYSTEM_MODE(AUTOMATIC); // connect app to cloud automatically
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

// To enable more debugging, use this version instead of the one above:
//SerialLogHandler logHandler(LOG_LEVEL_TRACE);

/********************************************************************/
//! /brief modbus initialization
ModbusMaster node(SERIAL1, MODADDR); // selects serial1 port and address id 1

/********************************************************************/
//! /brief forward declarations
int setPublishFrequency(String value);
int setWarningFrequency(String value);

int getSensorValues();
void publishSensorValues();
void publishWarningMessage(int errorCode = 0);

/********************************************************************/
//! /brief setup loop, only runs once at inital startup
void setup()
{
	//! setup cloud functions, best practice to do first in setup loop
	Particle.function("SensorPublish", setPublishFrequency);  // set senor publish frequency (seconds)
	Particle.function("WarningPublish", setWarningFrequency); // set warning publish frequency (seconds)

	//! initialize Modbus communication baud rate and TX pin
	node.begin(BAUD);	  // set baud rate for modbus
	node.enableTXpin(RT); // TX enable pin of RS485 driver
}

/********************************************************************/
//! /brief main code loop, runs continuously
void loop()
{
	static unsigned long lastCheck = 0;
	static unsigned long lastSend = 0;
	static unsigned long lastWarning = 0;

	if (millis() - lastCheck >= checkInterval.count())
	{
		lastCheck = millis();

		//! get current values and check for modbus comm error
		if (getSensorValues() == SUCCESS) // get current values
		{
			//! publish data at set interval
			if (millis() - lastSend >= (senFreq * 1000))
			{
			    lastSend = millis();

				//! publish sensor data
				publishSensorValues(); // send temp and humidity
			}
			
		}

		//! modbus error occurred
		else
		{
			//! publish data at set interval (same as warning interval)
			if (millis() - lastWarning >= (warFreq * 1000))
			{
				lastWarning = millis();
				publishWarningMessage(result); // send warning message
			}
		}
	}
}

/********************************************************************/
//! /brief function for reading temp and humidity
int getSensorValues()
{
	//! local variables
	uint16_t data[17]; // create a 6 element array of 16 bit ints
	
	long int vol1;
    long int vol2;
    long int vol3;
    long int cur1;
    long int cur2;
    long int cur3;
    
	// readHoldingRegisters and readInputRegisters take two parameters:
	// - the register to start reading from
	// - the number of registers to read (1, 2, ...)

	// Some sensors have the temperature and humidity in holding register 0 and 1. If so, use this version:
	//result = node.readHoldingRegisters(0x0016,6);

	// Some sensors have the temperature and humidity in input registers 1 and 2. If so, use this version:
	result = node.readInputRegisters(16, 17);

	// If you get Modbus Read Error 0x02 (ku8MBIllegalDataAddress), you probably have the wrong register
	// or input/holding selection for your sensor.

	//! read was successful
	if (result == node.ku8MBSuccess)
	{
		//! parse response buffer
		for (uint8_t i = 0; i < 17; i++)
		{
			data[i] = node.getResponseBuffer(i);
		}

		vol1 = (((unsigned long)data[0] << 16) | data[1]);      // Two 16 bit registers are combined to make a 32 bit long int
		memcpy(&voltage1, &vol1, 4);                            // memory copy 32 bit int to float voltage 1
		vol2 = (((unsigned long)data[2] << 16) | data[3]);      // Two 16 bit registers are combined to make a 32 bit long int
		memcpy(&voltage2, &vol2, 4);                            // memory copy 32 bit int to float voltage 2
		vol3 = (((unsigned long)data[4] << 16) | data[5]);      // Two 16 bit registers are combined to make a 32 bit long int
		memcpy(&voltage3, &vol3, 4);                            // memory copy 32 bit int to float voltage 3
		
		cur1 = (((unsigned long)data[12] << 16) | data[13]);    // Two 16 bit registers are combined to make a 32 bit long int
		memcpy(&current1, &cur1, 4);                            // memory copy 32 bit int to float current 1
		cur2 = (((unsigned long)data[14] << 16) | data[15]);    // Two 16 bit registers are combined to make a 32 bit long int
		memcpy(&current2, &cur2, 4);                            // memory copy 32 bit int to float current 2
		cur3 = (((unsigned long)data[16] << 16) | data[17]);    // Two 16 bit registers are combined to make a 32 bit long int
		memcpy(&current3, &cur3, 4);                            // memory copy 32 bit int to float current 3
	
		// debug serial messages
		Log.trace("%.2f", voltage1);

		return SUCCESS; // return success code
	}

	//! communication failure occured
	else
	{
		// debug serial messages
		Log.info("Modbus Read Error 0x%02x", result);

		return FAIL; // return fail code
	}
}

/********************************************************************/
//! /brief function for changing the sensor publish frequency in seconds
//! @param value is a string used for changing the frequency, value should be a whole integer or null. Example value = 60
int setPublishFrequency(String value)
{
	//! null case, return current value
	if (value == NULL)
	{
		return senFreq; // return current value
	}

	//! update to new value passed and return new value
	else
	{
		senFreq = value.toInt(); // set new volue to global variable
		Log.info("senFreq=%d", senFreq);
		return senFreq; // return new value
	}
}

/********************************************************************/
//! /brief function for changing the warning publish frequency
//! @param value is a string used for changing the frequency, value should be a whole integer or null. Example value = 20
int setWarningFrequency(String value)
{
	//! null case, return current value
	if (value == NULL)
	{
		return warFreq; // return current value
	}

	//! update to new value passed and return new value
	else
	{
		warFreq = value.toInt(); // set new volue to global variable
		Log.info("warFreq=%d", warFreq);
		return warFreq; // return new value
	}
}

/********************************************************************/
//! /brief function to publish the sensor state to cloud in JSON
void publishSensorValues()
{
	//! create JSON buffer and write values to it
	JsonWriterStatic<256> jw; // creates a 256 byte buffer to write JSON to
	{
		JsonWriterAutoObject obj(&jw);									  // creates an object to pass JSON
		jw.insertKeyValue("Voltage_1", voltage1);						  // set field for voltgae1
		jw.insertKeyValue("Voltage_2", voltage2);						  // set field for voltage2
		jw.insertKeyValue("Voltage_3", voltage3);						  // set field for voltage3
		jw.insertKeyValue("Current_1", current1);						  // set field for current1
		jw.insertKeyValue("Current_2", current2);						  // set field for current1
		jw.insertKeyValue("Current_3", current3);						  // set field for current1
		jw.insertKeyValue("Time", Time.format(TIME_FORMAT_ISO8601_FULL)); // set field for time stamp
	}

	Log.info("%s %s", sensorEventName, jw.getBuffer());

	//! send publish only if cloud is connected
	if (Particle.connected() == TRUE)
	{
		Particle.publish(sensorEventName, jw.getBuffer());
	}
}

/********************************************************************/
//! /brief function to publish the sensor state to cloud in JSON
void publishWarningMessage(int errorCode)
{
	//! create JSON buffer and write values to it
	JsonWriterStatic<256> jw; // creates a 256 byte buffer to write JSON to
	{
		JsonWriterAutoObject obj(&jw);

		jw.insertKeyValue("Warning", "Modbus Read Error"); // set field for warning message
		jw.insertKeyValue("ErrorCode", errorCode);
		jw.insertKeyValue("Time", Time.format(TIME_FORMAT_ISO8601_FULL)); // set field for time stamp
	}

	Log.info("%s %s", errorEventName, jw.getBuffer());

	//! send publish only if cloud is connected
	if (Particle.connected() == TRUE)
	{
		Particle.publish(errorEventName, jw.getBuffer());
	}
}
