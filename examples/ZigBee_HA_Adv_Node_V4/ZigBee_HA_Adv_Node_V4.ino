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

// Define high level hardware
bool BatteryPowered = true;
//#define BMP280
#define DHTTYPE DHT11
//#define xBeeTemp

// Define available clusters
#define PowerConfigCluster
#define OnOffCluster
#define TemperatureCluster;
//#define PressureCluster;
#define HumidityCluster;
    
#include <ZigBeeAPI.h>
#include "ZigBee.h"
#include <SoftwareSerial.h>


// On Off setup
#ifdef OnOffCluster
  #define OnOffClusterMomentary
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

//#ifdef BatteryPowered
  #include <avr/sleep.h>
//#endif  

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
  int xBeeOnSleep = 3;
  int xBeeBaud=9600;    // The baud rate of the xBee must match this number (set with the xBee's BD command)
  int xBeeSleepRQ = 9;
  extern volatile bool xBeeIsAwake;

  
  #if !defined (__AVR_ATmega32U4__) && !defined (__MK20DX128__)
    SoftwareSerial Serial1(xBeeTx, xBeeRx);
  #endif  

 extern ZigBeeAPI zb;                


  #ifdef BatteryPowered
    const char Model[] = "Arduino xBee (Battery)";     // ZigBee Basic Device ModelIdentifier 0 to 32 byte char string P79 of ZCL
  #else
    const char Model[] = "Arduino xBee (Powered)";     // ZigBee Basic Device ModelIdentifier 0 to 32 byte char string P79 of ZCL
  #endif
  const char Manufacturer[] = "Arduino";   // ZigBee Basic Device ManufacturerName 0 to 32 bytes
  const char SWVersion[] = "1.0.0"; 
  byte AppVersion = 1;               // Application version
  byte HardwareVersion = 1;          // Hardware version 


//  int rxResult = 0;

  unsigned long Sensor_CheckFreqWake = 2; // Wake cycles for sleepy device, ms for non sleepy
  unsigned long Sensor_LastCheckWake = 0;
  unsigned long Sensor_CheckFreqMillis = 20000; // Wake cycles for sleepy device, ms for non sleepy
  unsigned long Sensor_LastCheckMillis = 0;

  #ifdef OnOffCluster
    boolean buttonReleased = true;
  #endif

  

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
  #ifdef OnOffCluster
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

  //***************************************
  // Setup xBee
  //***************************************  
  Serial1.begin(xBeeBaud);
  ConfigurexBee(Serial1);
  JoinNetwork(); 
  SetupAddresses();
  //ResetNetwork();
  Serial.println(F("xBee setup."));

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

  int rxResult = 0;
  bool Sensor_Check = false; 
  

  if (BatteryPowered)
  {
    if (!xBeeIsAwake)
    {
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

  
  do                                                                          // Begin polling the button and checking for incoming packets
  {
    Sensor_Check = false;

    if ((BatteryPowered) && (Sensor_LastCheckWake >= Sensor_CheckFreqWake))
        Sensor_Check = true;

    if ((millis()-Sensor_LastCheckMillis)>Sensor_CheckFreqMillis)
      Sensor_Check = true;

    if (Sensor_Check)
    { 
      Sensor_LastCheckWake = 0;
      Sensor_LastCheckMillis = millis();     
    }


    // Check for any unprocessed inboud messages. Process these first
  	rxResult = CheckInboundPackets();

    // On Off cluster button check
    #ifdef OnOffCluster
  	  if ((digitalRead(buttonPin) == false) && ((millis()-now)>100) && (buttonReleased == true)) // Check if the button is being pressed
  	  {
        #ifdef OnOffClusterMomentary
          digitalWrite(LEDPin,!pinState(LEDPin));                             // Toggle Output
        #else  
          digitalWrite(LEDPin,HIGH);                                          // Set high if non momentary operation
        #endif  
        
  		  SendOnOffReport(pinState(LEDPin));                              // Let SmartThings know about it
      
  		  buttonReleased = false;																								// Don't allow anymore toggling until button is released and pressed again

  		  now = millis();                                                       // Save the current time for debouncing
  	  }
  	  if (digitalRead(buttonPin) == true)                                     // Check if the button has been released*/
  	  {
        #ifndef OnOffClusterMomentary
          if (pinState(LEDPin))
          {
            digitalWrite(LEDPin,!pinState(LEDPin));                           // Toggle Output
            SendOnOffReport(pinState(LEDPin));                          // Let SmartThings know about it
          }
        #endif  
  	    buttonReleased = true;
  	  };             
    #endif

    #ifdef TemperatureCluster;
      if (Sensor_Check) SendTemperatureReport(get_Temperature());
    #endif 
    
    #ifdef PressureCluster
      if (Sensor_Check) SendPressureReport(get_Pressure());
    #endif  

    #ifdef HumidityCluster
      if (Sensor_Check) SendHumidityReport(get_Humidity());
    #endif
    

  } while (BatteryPowered && rxResult != -9);                                                   // Keep checking for packets until there is no timeout while checking
  
  if (BatteryPowered && !xBeeIsAwake) Serial.println(F("xBee is Sleeping"));
}


#ifdef TemperatureCluster
float get_Temperature()
{
  float t = NAN;
  #ifdef BMP280
    Serial.print(F("BMP280 Sensor"));
    t = bmp.readTemperature() * 10;
  #endif  
  
  #ifdef DHTTYPE
    Serial.print(F("DHT Sensor"));
    t = dht.readTemperature();
    if (isnan(t))
    {
      Serial.print(F(". Trying again"));
      delay(2500);
      t = dht.readTemperature();
    }
  #endif  

  #ifdef xBeeTemp
    Serial.print(F("XBee Sensor"));
    t = Get_xBeeTemp();
  #endif

  Serial.print(F(", Getting Temp:"));
  Serial.println(t);
  return t;
}
#endif

#ifdef PressureCluster
float get_Pressure()
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
#endif

#ifdef HumidityCluster
float get_Humidity()
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
#endif
    
int get_ClusterList(char *list)
{
  int clusterQty = 0;
  int charPtr=0;
  
  list[charPtr++] = 0x00;                                                        // Input cluster list 2 bytes each little endian. 0x0000 = Basic Cluster
  list[charPtr++] = 0x00;
  clusterQty++; 
  
  #ifdef PowerConfigCluster
    list[charPtr++] = 0x01;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
    list[charPtr++] = 0x00;
    clusterQty++; 
  #endif
  
  #ifdef OnOffCluster
    list[charPtr++] = 0x06;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
    list[charPtr++] = 0x00;
    clusterQty++; 
  #endif

  #ifdef TemperatureCluster
    list[charPtr++] = 0x02;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
    list[charPtr++] = 0x04;
    clusterQty++; 
  #endif

  #ifdef PressureCluster
    list[charPtr++] = 0x03;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
    list[charPtr++] = 0x04;
    clusterQty++; 
  #endif
  
  #ifdef HumidityCluster
    list[charPtr++] = 0x05;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
    list[charPtr++] = 0x04;
    clusterQty++; 
  #endif 

  return clusterQty;
}

void clstr_Basic(byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0000 Basic
{
  char Buf[16];
  
  //***************************************
  // ZCL Cluster 0x0000 Basic Cluster
  // Section 3.2 on page 78 of ZCL
  //***************************************
  Serial.println();
  Serial.print(F("Basic Cluster attribute ID "));
  Serial.print(attributeID,HEX);
  Serial.print(F(" "));
  
  if (cmdID == 0x00 && attributeID == 0x0001)                                 // Read Attribute 0x0001 ApplicationVersion
  {
    Serial.print(F("(Application Ver) "));
    Send20Response(AppVersion, 0x0001, seqNum);
    return;
  }
  if (cmdID == 0x00 && attributeID == 0x0003)                                 // Read Attribute 0x0003 HWVersion
  {
    Serial.print(F("(Hardware Ver) "));
    Send20Response(HardwareVersion, 0x0003, seqNum);
    return;
  }
  if (cmdID == 0x00 && attributeID == 0x0004)                                 // Read Attribute 0x0004 ManufacturerName
  {
    Serial.print(F("(Manufacturer) "));
    Send42Response(Manufacturer, 0x0004, seqNum);
    return;    
  }
  if (cmdID == 0x00 && attributeID == 0x0005)                                 // Read Attribute 0x0005 ModelIdentifier
  {
    Serial.print(F("(Model) "));
    Send42Response(Model, 0x0005, seqNum);
    return;    
  } 
  if (cmdID == 0x00 && attributeID == 0x0006)                                 // Read Attribute 0x0006 Datecode
  {
    Serial.print(F("(Datecode) "));
    formatDate(__DATE__, __TIME__, Buf);
    Send42Response(Buf, 0x0006, seqNum);
    return;    
  } 
  if (cmdID == 0x00 && attributeID == 0x0007)                                 // Read Attribute 0x0007 Power source
  {
    Serial.print(F("(Power Source) "));
    if (BatteryPowered) 
      Send30Response(0x03, 0x0007, seqNum);
    else
      Send30Response(0x04, 0x0007, seqNum);
    return;    
  } 
  if (cmdID == 0x00 && attributeID == 0x4000)                                 // Read Attribute 0x4000 SWVersion
  {
    Serial.print(F("(SW Version) "));
    Send42Response(SWVersion, 0x4000, seqNum);
    return;    
  }
  Serial.println(F("Invalid type/command/attribute!"));
}

#ifdef PowerConfigCluster
void clstr_PowerConfiguration(byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0402 Temp
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  Serial.println();
  Serial.print(F("Power Config Cluster attribute ID "));
  Serial.print(attributeID,HEX);
  Serial.print(F(" "));
  
  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x0020)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  { 
     Serial.print(F("(Battery Voltage) "));
     Send20Response(50, 0x0020, seqNum);
     return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}
#endif

#ifdef OnOffCluster
void clstr_OnOff(byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0006 On/Off
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  Serial.println();
  Serial.print(F("OnOff Cluster attribute ID "));
  Serial.print(attributeID,HEX);
  Serial.print(F(" "));
  
  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x00)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  {
    Serial.print(F("(OnOff) "));
    Send10Response(pinState(LEDPin), 0x0000, seqNum);
    return;
  }

  if (frmType == 0x01 && cmdID == 0x00 && attributeID == 0x00)                // Set device Off -- P126 Section 3.8.2.3 of ZCL
  {
    digitalWrite(LEDPin, LOW); 
    Serial.print(F("(OnOff) LED Off (Pin "));
    Serial.print(LEDPin,DEC);
    Serial.print(F(" set to ground)"));
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send default response back to originator of command
    return;
  }

  if (frmType == 0x01 && cmdID == 0x01 && attributeID == 0x00)                // Set device On
  {
    digitalWrite(LEDPin, HIGH);                                               // Set High
    Serial.print(F("(OnOff) LED On (Pin "));
    Serial.print(LEDPin,DEC);
    Serial.print(F(" set to High)"));
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send default response back to originator of command
    return;
  }

  if (frmType == 0x01 && cmdID == 0x02 && attributeID == 0x00)                // Toggle device
  {
    digitalWrite(LEDPin,!pinState(LEDPin));                                   // Toggle Output
    Serial.print(F("(OnOff) LED Toggled (Pin "));
    Serial.print(LEDPin,DEC);
    Serial.print(F(" toggled)"));
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send Default response back to originator of command
    return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}
#endif

#ifdef TemperatureCluster
void clstr_Temperature(byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0402 Temp
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  Serial.println();
  Serial.print(F("Temperature Cluster attribute ID "));
  Serial.print(attributeID,HEX);
  Serial.print(F(" "));
  
  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x00)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  {   
    Serial.print(F("(Temperature) "));
    Send29Response(get_Temperature() * 100, 0x0000, seqNum);
    return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}
#endif

#ifdef PressureCluster
void clstr_Pressure(byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0402 Temp
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  Serial.println();
  Serial.print(F("Pressure Cluster attribute ID "));
  Serial.print(attributeID,HEX);
  Serial.print(F(" "));
  
  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x00)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  { 
    Serial.print(F("(Pressure) "));
    Send29Response(get_Pressure(), 0x0000, seqNum);
    return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}
#endif

#ifdef HumidityCluster
void clstr_Humidity(byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0402 Temp
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  Serial.println();
  Serial.print(F("Humidity Cluster attribute ID "));
  Serial.print(attributeID,HEX);
  Serial.print(F(" "));

  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x00)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  { 
    Serial.print(F("(Humidity) "));
    Send21Response(get_Humidity() * 100, 0x0000, seqNum);
    return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}
#endif



