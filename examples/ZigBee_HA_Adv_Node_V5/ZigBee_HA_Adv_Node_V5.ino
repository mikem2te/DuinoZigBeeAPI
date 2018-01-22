//***********************************************
// ZigBee-Home Automation Communications v2          
// Author: Professor Mayhem,
// Yoyodyne Monkey Works, Inc.
//
//
// 3/30/2017
//
// Revision 1.0
//
// ZigBee-Home Automation Communications is an
// Arduino port from a version of the same name
// written in the Propeller Spin language
// by John.Rucker(at)Solar-Current.com
//
// This new version adds a local switch/button to
// also turn the LED off and on. Using the local
// button will update the SmartThings status of
// the LED. This will only work with the S2C
// versions of the Xbee. The S2C version provides
// the ZigBee binding needed for the Arduino to
// originate a message to SmartThings hub.
// 
//***********************************************

// Define high level hardware connected
#define BatteryPowered
//#define BMP280
#define DHTTYPE DHT11
#define xBeeTemp
#define OnOffButton

    
#include <ZigBeeAPI.h>
#include "ZigBee.h"
#include <SoftwareSerial.h>
#include <avr/sleep.h>


EndpointCluster endpointClusters[]= { 
  {1, cluster_Basic},
  {1, cluster_PowerConfiguration},
  {1, cluster_OnOff},
  {1, cluster_Temperature},
  {1, cluster_RelativeHumidity},
  {2, cluster_Temperature}
}; 


// On Off setup
#ifdef OnOffButton
  #define OnOffButtonMomentary
  boolean buttonReleased = true;
  const byte buttonPin = 2;          // This is the pin that the button is attached to that will locally turn the LED on and off. Connect the button between GND and this pin
  const byte LEDPin=LED_BUILTIN;     // This is the pin that is connected to an LED's anode (positive side)
                                     // Make sure to use a current limiting resistor in series with the LED
                                     // Connect the other end of the LED to ground  
#endif  


// BMP setup
#ifdef BMP280
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp; // I2C
  #define TempSensorPowerPin 7
#endif

#ifdef DHTTYPE
  #include "DHT.h"
  #define DHTPIN 6 
  #define TempSensorPowerPin 7
  DHT dht(DHTPIN, DHTTYPE);
#endif


#define _DEBUG 0

/*
Hardware requirements:
Arduino, xBee shield, and Digi Xbee. Tested on Arduino Uno, Duemilanove, Leonardo, and Mega 2560 using a Seeed Studio Xbee Shield.
Tested both with Xbee Pro ZB SMT (Surface Mount) Digi part number XBP24CZ7PIS-004 and with the Xbee S2 XB24-Z7WIT-004 TH (Through Hole).
Configure the above xBee with the following settings: ZS = 2, NJ = 0x5a, EE = Enable, EO = 1,
KY = 0X5A6967426565416C6C69616E63653039, BD = 9600, D6 = None, AP = true, AO = 3

This demo allows you to control a LED attached to the LEDPin from a public ZigBee Home Automation network or a button on the Arduino itself. 
The demo is designed to work with a SmartThings hub available at SmartThings.com.
A custom SmartThings device type is available at:
https://github.com/JohnRucker/Nuts-Volts/blob/master/devicetypes/johnrucker/parallax-propeller-demo.src/parallax-propeller-demo.groovy
Install the above device driver in the SmartThings developer section for your hub.
Once installed SmartThings will recognize this device and will add a tile to their smart phone app when this device joins their Hub's network.
This new tile will allow you to send on / off commands to this device turning the LED on and off.

Conversation overview of the initial connection to the SmartThings home automation network.
  1) At power up the xBee will look for a valid ZigBee Home Automation coordinator or router to join, if the network is allowing new devices to
     join it will join. An Association number will be displayed on the Serial Monitor until the network is joined. To join a ZigBee Home Automation
     network the xBee must be configured with the proper security key and security settings outlined above in the hardware configuration.
  2) Once joined to a valid ZigBee Home Automation network a device announce packet (ZDO Cluster 0x00013) will be sent to the coordinator.
  3) The coordinator should respond back with a request for active end points. This request will be received and a response report will be
     sent back (ZDO Cluster 0x8005) to the coordinator telling it what end points are supported by this application.
  4) Next the coordinator will request a description of each end point's configuration. This application will respond to this request with
     a report of supported device types and clusters for each end point (ZDO Cluster 0x8004).
  5) At this point the home automation network knows what this device is and how to talk to it.
     loop() will continue to listen to ZigBee packets and will respond to packets that send the on or off command (Cluster 0x0006).
*/
//**************************************
// Start ZigBee API Communications 
//***************************************

  // SoftwareSerial is not needed on Leonardo, or Mega. Attach the xBee to pins 0 & 1 of the Leonardo, or pins 19, & 18 of the Mega.
  // No other changes are required.
  //
  // SoftwareSerial Limitations:
  // 
  // On Mega and Mega 2560, only the following pins can be used for RX:
  // 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).
  //
  // On Leonardo and Micro, only the following pins can be used for RX:
  // 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
  // On Arduino or Genuino 101 the current maximum RX speed is 57600bps
  // On Arduino or Genuino 101 RX doesn't work on Pin 13
  //

  int xBeeTx=4;         // This is the Arduino pin connected to the xBee's transmit pin. Not needed for Leonardo or Mega
  int xBeeRx=5;         // This is the Arduino pin connected to the xBee's receive pin. Not needed for Leonardo or Mega
  int xBeeRTS=8;        // This is the Arduino pin connected to the xBee's RTS pin.
  int xBeeReset=12;     // This is the Arduino pin connected to the xBee's reset pin, not needed if using soft reset (AT FR)
  int xBeeOnSleep = 3;  // This is the Arduino pin connected to the xBee's on/sleep pin. Not needed if using a powered node
  int xBeeSleepRQ = 9;  // This is the Arduino pin connected to the xBee's sleep rq pin. Not needed if using a powered node
  int xBeeBaud=9600;    // The baud rate of the xBee must match this number (set with the xBee's BD command)

  #if !defined (__AVR_ATmega32U4__) && !defined (__MK20DX128__)
    SoftwareSerial Serial1(xBeeTx, xBeeRx);
  #endif  

  extern ZigBeeAPI zb;                

  #ifdef BatteryPowered
    char  Model[] = "Arduino xBee (Battery)";     // ZigBee Basic Device ModelIdentifier 0 to 32 byte char string P79 of ZCL
  #else
    char  Model[] = "Arduino xBee (Powered)";     // ZigBee Basic Device ModelIdentifier 0 to 32 byte char string P79 of ZCL
  #endif
  char Manufacturer[] = "Arduino";   // ZigBee Basic Device ManufacturerName 0 to 32 bytes
  char SWVersion[] = "1.0.0"; 
  byte AppVersion = 1;               // Application version
  byte HardwareVersion = 1;          // Hardware version 

  unsigned long SensorCheck_RetryMillis = 10000;
  unsigned long SensorCheck_FreqWake = 2; // Wake cycles for sleepy device, ms for non sleepy
  unsigned long SensorCheck_FreqMillis = 20000; // Wake cycles for sleepy device, ms for non sleepy



  

// Interrupt Service Routines and Sleep management
// -----------------------------------------------
void buttonPressed()
{ 
}

// Setup
// -----
void setup()
{
  //***************************************
  // Start Serial Monitor Communications
  //***************************************
  Serial.begin(9600);

  while (!Serial)
  {
    delay(1); // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println();
  Serial.println(F("On Line."));
  
  #if _DEBUG
    Serial.print(F("Compiled on: "));
    Serial.print(__DATE__);
    Serial.print(F(" "));
    Serial.println(__TIME__);
  #endif

         
  // On Off cluster setup
  #ifdef OnOffButton
    pinMode(LEDPin, OUTPUT);              // Set LEDPin as Output
    pinMode(buttonPin, INPUT);            // Set Button pin Set as Input
    digitalWrite(buttonPin, HIGH);
    pinMode(10, OUTPUT);                  // Set this pin as outpt and low to (not enough GND pins)
    digitalWrite(10, LOW);
  #endif
   

  // Temperature Sensor Power Pin setup
  #ifdef TempSensorPowerPin
    pinMode(TempSensorPowerPin, OUTPUT);         // Power pin for the BME
    digitalWrite(TempSensorPowerPin, HIGH);
  #endif

  
  #ifdef BMP280
    bool status; 
    do {
      status = bmp.begin();  
      if (!status) {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
      } else {
        Serial.println(F("A valid BME280 sensor has been found!"));
      }
    } while (!status);
  #endif

  #ifdef DHTTYPE
    dht.begin();
  #endif
    

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, CHANGE);

  Serial1.begin(xBeeBaud);
  
  #ifdef BatteryPowered
    setup_ZigBee(Serial1, sizeof(endpointClusters) / sizeof(EndpointCluster), true);
  #else
    setup_ZigBee(Serial1, sizeof(endpointClusters) / sizeof(EndpointCluster), false);
  #endif
  now = millis();
}


// Loop
// ----
void loop()
{
  //***************************************
  // loop() waits for inbound data from the ZigBee network and responds based on the packet's profile number
  // If the profile number in the received packet is 0x0000 then the packet is a ZigBee Device Object (ZDO) packet
  // If the profile number is not 0x0000 then the packet is a ZigBee Cluster Library (ZCL) packet
  // The packet is also printed to the screen formatted so you can see addressing and payload details.
  //***************************************

  #ifdef BatteryPowered
    sleepNow();
  #endif
  
  loop_ZigBee();

  // On Off cluster button check
  #ifdef OnOffButton
	  if ((digitalRead(buttonPin) == false) && ((millis()-now)>100) && (buttonReleased == true)) // Check if the button is being pressed
	  {
      #ifdef OnOffButtonMomentary
        digitalWrite(LEDPin,!pinState(LEDPin));                             // Toggle Output
      #else  
        digitalWrite(LEDPin,HIGH);                                          // Set high if non momentary operation
      #endif  
      
		  SendOnOffReport(1, pinState(LEDPin));                              // Let SmartThings know about it
    
		  buttonReleased = false;																								// Don't allow anymore toggling until button is released and pressed again

		  now = millis();                                                       // Save the current time for debouncing
	  }
	  if (digitalRead(buttonPin) == true)                                     // Check if the button has been released*/
	  {
      #ifndef OnOffButtonMomentary
        if (pinState(LEDPin))
        {
          digitalWrite(LEDPin,!pinState(LEDPin));                           // Toggle Output
          SendOnOffReport(1, pinState(LEDPin));                          // Let SmartThings know about it
        }
      #endif  
	    buttonReleased = true;
	  }             
  #endif
}

/*
void loop()
{
  //***************************************
  // loop() waits for inbound data from the ZigBee network and responds based on the packet's profile number
  // If the profile number in the received packet is 0x0000 then the packet is a ZigBee Device Object (ZDO) packet
  // If the profile number is not 0x0000 then the packet is a ZigBee Cluster Library (ZCL) packet
  // The packet is also printed to the screen formatted so you can see addressing and payload details.
  //***************************************

  int rxResult = 0;
  bool Sensor_Check = false; 

  
  if (BatteryPowered)
  {
    if (!xBeeIsAwake)
    {
      Serial.println(F("xBee is Sleeping"));
      digitalWrite(xBeeRTS, HIGH);
      Serial.println(F("Arduino Sleeping, waiting for awake signal"));
      sleepNow();
      if (xBeeIsAwake)
        Serial.println(F("XBee is Awake, Arduino Waking"));
      else   
        Serial.println(F("Arduino Waking, assume Arduino interrupt"));  
      digitalWrite(xBeeRTS, LOW);
      Sensor_LastCheckWake++;
    }
  }
  else
  {
    Serial.println();
    Serial.println(F("Waiting for packet"));
  }

  
 // do                                                                          // Begin polling the button and checking for incoming packets
 // {
    Sensor_Check = false;

    if ((BatteryPowered && (Sensor_LastCheckWake >= Sensor_CheckFreqWake)) ||
       (millis()-Sensor_LastCheckMillis >= Sensor_CheckFreqMillis) ||
       (Sensor_RequireRetry))
    {
      Sensor_Check = true;
      Sensor_LastCheckWake = 0;
      Sensor_LastCheckMillis = millis(); 
      Sensor_RequireRetry = false;
    }


    // Check for any unprocessed inboud messages. Process these first
    rxResult = CheckInboundPackets(false);

    // On Off cluster button check
    #ifdef OnOffCluster
      if ((digitalRead(buttonPin) == false) && ((millis()-now)>100) && (buttonReleased == true)) // Check if the button is being pressed
      {
        #ifdef OnOffClusterMomentary
          digitalWrite(LEDPin,!pinState(LEDPin));                             // Toggle Output
        #else  
          digitalWrite(LEDPin,HIGH);                                          // Set high if non momentary operation
        #endif  
        
        SendOnOffReport(1, pinState(LEDPin));                              // Let SmartThings know about it
      
        buttonReleased = false;                                               // Don't allow anymore toggling until button is released and pressed again

        now = millis();                                                       // Save the current time for debouncing
      }
      if (digitalRead(buttonPin) == true)                                     // Check if the button has been released
      {
        #ifndef OnOffClusterMomentary
          if (pinState(LEDPin))
          {
            digitalWrite(LEDPin,!pinState(LEDPin));                           // Toggle Output
            SendOnOffReport(1, pinState(LEDPin));                          // Let SmartThings know about it
          }
        #endif  
        buttonReleased = true;
      };             
    #endif

    if (Sensor_Check == true)
      rxResult = PollSensors();

//  } while (BatteryPowered && rxResult != -9);                                                   // Keep checking for packets until there is no timeout while checking
  

}
*/


bool get_OnOff(byte endPoint)
{
  if (endPoint == 1)
  {
    return pinState(LEDPin);
  }
  return false;
}

void set_OnOff(byte endPoint, bool On)
{
  if (endPoint == 1)
  {
    if (On)
    {
      digitalWrite(LEDPin, HIGH);                                               // Set High
      Serial.print(F("(OnOff) LED On (Pin "));
      Serial.print(LEDPin,DEC);
      Serial.print(F(" set to High)"));
    }
    else
    {
      digitalWrite(LEDPin, LOW); 
      Serial.print(F("(OnOff) LED Off (Pin "));
      Serial.print(LEDPin,DEC);
      Serial.print(F(" set to ground)"));
    }
  }
}

void toggle_OnOff(byte endPoint)
{
  if (endPoint == 1)
  {
    digitalWrite(LEDPin,!pinState(LEDPin));                                   // Toggle Output
    Serial.print(F("(OnOff) LED Toggled (Pin "));
    Serial.print(LEDPin,DEC);
    Serial.print(F(" toggled)"));  
  }
}

float get_Temperature(byte endPoint)
{
  float t = NAN;

  if (endPoint == 1)
  {
    #ifdef BMP280
      Serial.print(F("BMP280 Sensor"));
      t = bmp.readTemperature() * 10;
    #endif  
    
    #ifdef DHTTYPE
      Serial.print(F("DHT Sensor"));
      t = dht.readTemperature();
    #endif  
  }

  if (endPoint == 2)
  {
    #ifdef xBeeTemp
      Serial.print(F("XBee Sensor"));
      t = Get_xBeeTemp();
    //t = 0;
    #endif
  }

  Serial.print(F(", Getting Temp:"));
  Serial.println(t);
  return t;
}

float get_Pressure(byte endPoint)
{
  float p = NAN;
  
  #ifdef BMP280
    Serial.println(F("BMP280 Sensor"));
    p = bmp.readPressure() / 10.0;
  #endif  

  Serial.print(F(", Getting Pressure:"));
  Serial.println(p);
  return p;
}

float get_Humidity(byte endPoint)
{
  float h = NAN;
  
  #ifdef DHTTYPE
    Serial.print(F("DHT Sensor"));
    h = dht.readHumidity();
    if (isnan(h))
    {
      delay(2500);
      h = dht.readHumidity();
    }
  #endif 

  Serial.print(F(", Getting Humidity:"));
  Serial.println(h);
  return h;
   
}




