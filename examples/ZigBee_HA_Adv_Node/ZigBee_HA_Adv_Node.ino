

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
//#define BMP280
#define BatteryPowered
#define DHTTYPE DHT11

// Define clusters
#define OnOffCluster
#define TemperatureCluster;
//#define PressureCluster;
#define HumidityCluster;
    
#include <ZigBeeAPI.h>
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
  unsigned long TempSensor_CheckFreq = 5;
  unsigned long TempSensor_LastCheck = 0;
  bool TempSensor_Check = false;
#endif

#ifdef DHTTYPE
  #include "DHT.h"
  #define DHTPIN 6 
  #define TempSensorPowerPin 7
  DHT dht(DHTPIN, DHTTYPE);
  unsigned long TempSensor_CheckFreq = 1;
  unsigned long TempSensor_LastCheck = 0;
  bool TempSensor_Check = false;
#endif

#ifdef BatteryPowered
  #include <avr/sleep.h>
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

  
  // Async config
  int xBeeTx=4;     // This is the Arduino pin connected to the xBee's transmit pin. Not needed for Leonardo or Mega
  int xBeeRx=5;     // This is the Arduino pin connected to the xBee's receive pin. Not needed for Leonardo or Mega
  int xBeeRTS=8;    // This is the Arduino pin connected to the xBee's RTS pin.
  int xBeeBaud=9600;// The baud rate of the xBee must match this number (set with the xBee's BD command)
  int xBeeReset=12; // This is the Arduino pin connected to the xBee's reset pin, not needed if using soft reset (AT FR)
  #if !defined (__AVR_ATmega32U4__) && !defined (__MK20DX128__)
    SoftwareSerial Serial1(xBeeTx, xBeeRx);
  #endif                  
  ZigBeeAPI zb(Serial1);
  

  byte AppVersion = 1;               // Application version
  byte HardwareVersion = 1;          // Hardware version

  const char Model[] = "Arduino xBee";     // ZigBee Basic Device ModelIdentifier 0 to 32 byte char string P79 of ZCL
  const char Manufacturer[] = "Arduino";   // ZigBee Basic Device ManufacturerName 0 to 32 bytes
  const char Version[] = "1.0.0"; 
  

  #ifdef BatteryPowered
    const byte xBeeOnSleep = 3;
    bool xBeeIsAwake;
    const byte xBeeSleepRQ = 9;
  #endif

  const byte BufferSize = 75;
  byte Buffer[BufferSize];
  word LclNet=0;
  unsigned long LclIeeeLo = 0, LclIeeeHi =0, now=0;
  int rxResult = 0;


  #ifdef OnOffCluster
    boolean buttonReleased = true;
    //boolean buttonReleased = true;
    //boolean buttonIsPressed = false;
    //unsigned long buttonPressedTime = 0;
  #endif

  #ifdef TemperatureCluster
    int Temperature;
    int old_Temperature;
  #endif 

  #ifdef HumidityCluster
    int Humidity;
    int old_Humidity;
  #endif 

  #ifdef PressureCluster
    int Pressure;
    int old_Pressure;
  #endif 

  
bool pinState(byte pin) {return (0!=(*portOutputRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin)));};

void SendOnOffReport(boolean Value)
{
  //***************************************
  // Reports boolean value (1 or 0)
  //
  // Destinations are held in the xBee's Binding table
  //***************************************
  #ifdef BatteryPowered
    WakexBee();
  #endif
        
  memset(Buffer, 0, BufferSize);
  Buffer[0] = 0x18;                                                           // Frame Control 0x10 = direction is from server to client, disable default response P14 of ZCL
  Buffer[1] = 0x11;                                                           // Set the sequence number
  Buffer[2] = 0x0A;                                                           // Command Identifer 0x0A = Report attributes see Table 2.9 on page 16 of ZCL

  Buffer[3] = 0x00;                                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL. 0x0000 = OnOff.
  Buffer[4] = 0x00;

  Buffer[5] = 0x10;                                                           // Attribute Data Type 0x10 = Boolean enumeration, see table 2.16 on page 54 of ZCL

  Buffer[6] = Value;                                                          // Attribute Value (0 = off, 1 = on)

  zb.TX_Indirect(0x01, 0x0104, 0x0006, Buffer, 7);                            // TX_Indirect(sEP, Prfl, Clstr, BuffAdd, BuffSize)
  Serial.println(F("On Off packet sent"));
}

#ifdef TemperatureCluster
void SendTemperatureReport(int Value)
{
  #ifdef BatteryPowered
    WakexBee();
  #endif
  
  memset(Buffer, 0, BufferSize);
  Buffer[0] = 0x18;                                                           // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
  Buffer[1] = 0x11;                                                           // Set the sequence number
  Buffer[2] = 0x0A;                                                           // Command Identifer 0x0A = Report attributes see Table 2.9 on page 16 of ZCL

  Buffer[3] = 0x00;                                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL. 0x0000 = OnOff.
  Buffer[4] = 0x00;

  Buffer[5] = 0x29;                                                         // Attribute Data Type 0x10 = Boolean enumeration, see table 2.16 on page 54 of ZCL

  Buffer[6] = lowByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
  Buffer[7] = highByte(Value);                                                         // Attribute Value (0 = off, 1 = on)

  zb.TX_Indirect(0x01, 0x0104, 0x0402, Buffer, 8);                            // TX_Indirect(sEP, Prfl, Clstr, BuffAdd, BuffSize)
  Serial.println(F("Temperature packet sent"));
}
#endif

#ifdef PressureCluster
void SendPressureReport(int Value)
{
   #ifdef BatteryPowered
    WakexBee();
  #endif
  
  memset(Buffer, 0, BufferSize);
  Buffer[0] = 0x18;                                                           // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
  Buffer[1] = 0x11;                                                           // Set the sequence number
  Buffer[2] = 0x0A;                                                           // Command Identifer 0x0A = Report attributes see Table 2.9 on page 16 of ZCL

  Buffer[3] = 0x00;                                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL. 0x0000 = OnOff.
  Buffer[4] = 0x00;

  Buffer[5] = 0x29;                                                         // Attribute Data Type 0x10 = Boolean enumeration, see table 2.16 on page 54 of ZCL

  Buffer[6] = lowByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
  Buffer[7] = highByte(Value);                                                         // Attribute Value (0 = off, 1 = on)

  zb.TX_Indirect(0x01, 0x0104, 0x0403, Buffer, 8);                            // TX_Indirect(sEP, Prfl, Clstr, BuffAdd, BuffSize)
}
#endif

#ifdef HumidityCluster
void SendHumidityReport(int Value)
{
   #ifdef BatteryPowered
    WakexBee();
  #endif
  
  memset(Buffer, 0, BufferSize);
  Buffer[0] = 0x18;                                                           // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
  Buffer[1] = 0x11;                                                           // Set the sequence number
  Buffer[2] = 0x0A;                                                           // Command Identifer 0x0A = Report attributes see Table 2.9 on page 16 of ZCL

  Buffer[3] = 0x00;                                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL. 0x0000 = OnOff.
  Buffer[4] = 0x00;

  Buffer[5] = 0x29;                                                         // Attribute Data Type 0x10 = Boolean enumeration, see table 2.16 on page 54 of ZCL

  Buffer[6] = lowByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
  Buffer[7] = highByte(Value);                                                         // Attribute Value (0 = off, 1 = on)

  zb.TX_Indirect(0x01, 0x0104, 0x0405, Buffer, 8);                            // TX_Indirect(sEP, Prfl, Clstr, BuffAdd, BuffSize)
}
#endif

void Tx_Device_annce()                                                        // ZDO Cluster 0x0013 Device_annce
{
  //***************************************
  // Device_annce ZDO Cluster 0x0013 see page 109 of ZBSpec
  // This method announces that this device is now on the network and ready to receive packets
  // Called every time the Arduino reboots
  // Device announce is sent to the ZigBee Coordinator
  //***************************************
  memset(Buffer, 0 , BufferSize);
  Buffer[0] = 0x22;                                                           // Transaction seq number
  Buffer[1] = lowByte(LclNet);                                                // Network Address 2 bytes little endian Page 109 of ZBSpec
  Buffer[2] = highByte(LclNet);
  Buffer[3] = lowByte(LclIeeeLo);                                             // IEEE Address first 4 bytes little endian
  Buffer[4] = highByte(LclIeeeLo);
  Buffer[5] = lowByte(LclIeeeLo >> 16);
  Buffer[6] = highByte(LclIeeeLo >> 16);
  Buffer[7] = lowByte(LclIeeeHi);                                             // IEEE Address second 4 bytes little endian
  Buffer[8] = highByte(LclIeeeHi);
  Buffer[9] = lowByte(LclIeeeHi >> 16);
  Buffer[10] = highByte(LclIeeeHi >> 16);

  // MAC Capability Mask See page 83 of ZBSpec.
  #ifdef BatteryPowered
    Buffer[11] = 0x80;          // BATTERY MODE  
  #else                                             
    Buffer[11] = 0x8C;          // 0x8C = b10001100 = Mains powered device, Receiver on when idle, address not self-assigned
  #endif  
                                                                              
  zb.TX(0x00000000, 0x0000FFFF, 0xFFFD, 0, 0, 0, 0x0013, Buffer, 12);         // $FFFD = Broadcast to all non sleeping devices
}


void printByteData(uint8_t Byte)
{
  Serial.print((uint8_t)Byte >> 4, HEX);
  Serial.print((uint8_t)Byte & 0x0f, HEX);
}


void buttonPressed()
{ 
 /* if (!digitalRead(buttonPin)) {
    if (!buttonIsPressed && buttonReleased)
    { if (millis() > buttonPressedTime + 100)
      {
        buttonPressedTime = millis(); 
        Serial.println(F("Button pressed"));
        buttonIsPressed = true;
      }
    }
  } else {
    if (buttonIsPressed)
    {
      Serial.println(F("Button released"));
      buttonReleased = true;
    }
  }*/
}


#ifdef BatteryPowered
void xBeeAwakeChange()
{ 
  xBeeIsAwake = digitalRead(xBeeOnSleep);
  
  if (xBeeIsAwake) {
    Serial.println(F("xBee is Awake"));
  } else {
    Serial.println(F("xBee is Sleeping"));
  }
}

void WakexBee()
{
  if (!xBeeIsAwake) {
    Serial.println(F("Kicking xBee"));
    digitalWrite(xBeeSleepRQ,HIGH);  // Wake up xBee
    delay(10);
    digitalWrite(xBeeSleepRQ,LOW);  // Wake up xBee
    delay(50);
  }
}
#endif

void setup()
{
  // Configure xBee RTS pin
  pinMode(xBeeRTS, OUTPUT);
  digitalWrite(xBeeRTS, LOW);
          
  // On Off cluster setup
  #ifdef OnOffCluster
    pinMode(LEDPin, OUTPUT);              // Set LEDPin as Output
    pinMode(buttonPin, INPUT);            // Set Button pin Set as Input
    digitalWrite(buttonPin, HIGH);
    pinMode(10, OUTPUT);                  // Set this pin as outpt and low to (not enough GND pins)
    digitalWrite(10, LOW);
  #endif
  
  #ifdef BatteryPowered
    pinMode(xBeeOnSleep,INPUT);   // set Pin as Input (default)
    digitalWrite(xBeeOnSleep,HIGH);  // enable pullup resistor
    pinMode(xBeeSleepRQ,OUTPUT);   // set Pin as Input (default)
  #endif
 
 
  //***************************************
  // Start Serial Monitor Communications
  //***************************************
  Serial.begin(9600);
  Serial1.begin(xBeeBaud);
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


  bool status;

  // Temperature Sensor Power Pin setup
  #ifdef TempSensorPowerPin
    pinMode(TempSensorPowerPin, OUTPUT);         // Power pin for the BME
    digitalWrite(TempSensorPowerPin, HIGH);
  #endif

  
  #ifdef BMP280
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
    
  //***************************************
  // Un-Remark the next command to force xBee to leave Network and reset
  //***************************************
   //ResetNetwork();

  #ifdef BatteryPowered
    attachInterrupt(digitalPinToInterrupt(xBeeOnSleep), xBeeAwakeChange, CHANGE);
    delay(20);
    xBeeIsAwake = digitalRead(xBeeOnSleep);
  #endif

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, CHANGE);

  
  //***************************************
  // Loop until xBee is joined to a ZigBee network
  // See page 246 of XBee/XBee-PRO S2C ZigBee User Guide for an explanation of AI Network Join Status numbers
  // http://www.digi.com/resources/documentation/digidocs/pdfs/90002002.pdf
  //***************************************

  Serial.println();
  Serial.print(F("Network Join Status: "));
  zb.AT("AI");
  if (byte(zb._PktData()[0]) == 0x00)
  {                                                                           // If xBee is not joined print the AI error codes to screeen for reference
    Serial.println(F("Successfully joined a network"));
  }
  else
  {
    Serial.println();
    //Association Indicator (AI) definition strings
    Serial.println(F("0x00 - Successfully formed or joined a network. (Coordinators form a network, routers and end devices join a network.)"));
    Serial.println(F("0x21 - Scan found no PANs"));
    Serial.println(F("0x22 - Scan found no valid PANs based on current SC and ID settings"));
    Serial.println(F("0x23 - Valid Coordinator or Routers found, but they are not allowing joining (NJ expired)"));
    Serial.println(F("0x24 - No joinable beacons were found"));
    Serial.println(F("0x25 - Unexpected state, node should not be attempting to join at this time"));
    Serial.println(F("0x27 - Node Joining attempt failed (typically due to incompatible security settings)"));
    Serial.println(F("0x2A - Coordinator Start attempt failed"));
    Serial.println(F("0x2B - Checking for an existing coordinator"));
    Serial.println(F("0x2C - Attempt to leave the network failed"));
    Serial.println(F("0xAB - Attempted to join a device that did not respond."));
    Serial.println(F("0xAC - Secure join error - network security key received unsecured"));
    Serial.println(F("0xAD - Secure join error - network security key not received"));
    Serial.println(F("0xAF - Secure join error - joining device does not have the right preconfigured link key"));
    Serial.println(F("0xFF - Scanning for a ZigBee network (routers and end devices)"));
    Serial.println();
    Serial.print(F("Looping until network is joined: "));
    now=millis();
    while (byte(zb._PktData()[0]) != 0)                                       // Loop until xBee joins a valid network
    {
      if ((millis() - now) > 1000)
      {
        now=millis();
        if(zb.AT("AI"))
        {
          Serial.print(F(" "));
          Serial.write(0x0D);
          printByteData(byte(zb._PktData()[0]));
          printByteData(byte(zb._PktData()[1]));
        }
      }
      zb.RX(10);  // Continue checking for a reply while we are waiting...
    }
  }
  
  //***************************************
  // Read xBee's address settings and store in memory
  // The xBee's 64 bit IEEE address is in ROM on the xBee and will never change (SH & SL = IEEE Address)
  // The xBee's 16 bit network address (MY address) is set by the ZigBee coordinator and may change at any time
  //**************************************
  Serial.println(F("Setting up Addresses"));
  if (zb.AT("MY"))
  {
    LclNet = byte(zb._PktData()[0]);                                          // Set Network Address
    LclNet = (LclNet << 8) + byte(zb._PktData()[1]);                          // Set Network Address
  }

  if (zb.AT("SH"))
  {
    LclIeeeHi = byte(zb._PktData()[0]);                                       // Set IEEE Address Hi
    LclIeeeHi = (LclIeeeHi << 8) + byte(zb._PktData()[1]);
    LclIeeeHi = (LclIeeeHi << 8) + byte(zb._PktData()[2]);
    LclIeeeHi = (LclIeeeHi << 8) + byte(zb._PktData()[3]);
  }
  if(zb.AT("SL"))
  {
    LclIeeeLo = byte(zb._PktData()[0]);                                       // Set IEEE Address Lo
    LclIeeeLo = (LclIeeeLo << 8) + byte(zb._PktData()[1]);
    LclIeeeLo = (LclIeeeLo << 8) + byte(zb._PktData()[2]);
    LclIeeeLo = (LclIeeeLo << 8) + byte(zb._PktData()[3]);
  }
  delay(1000);                                                                // wait 1 sec
  Tx_Device_annce();

  //***************************************
  // Populate string variables with address information
  //**************************************
  Serial.println();
  Serial.print(F("IEEE Add: "));
  Serial.print(LclIeeeHi,HEX);
  Serial.print(F("-"));
  Serial.println(LclIeeeLo,HEX);
}

#ifdef BatteryPowered
void sleepNow()         // here we put the arduino to sleep
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
  sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
 
  Serial.println(F("Arduino Sleeping"));

  delay(100);
    
  sleep_mode();            // Put the device  to sleep!! THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
                            
  Serial.println(F("Arduino Waking"));   
}
#endif

void loop()
{
  //***************************************
  // loop() waits for inbound data from the ZigBee network and responds based on the packet's profile number
  // If the profile number in the received packet is 0x0000 then the packet is a ZigBee Device Object (ZDO) packet
  // If the profile number is not 0x0000 then the packet is a ZigBee Cluster Library (ZCL) packet
  // The packet is also printed to the screen formatted so you can see addressing and payload details.
  //***************************************
  Serial.println();
  Serial.println(F("Waiting for packet"));
  now = millis();
  
  #ifdef BMP280
    TempSensor_LastCheck = millis();
  #endif 
  
  do                                                                          // Begin polling the button and checking for incoming packets
  {
    #ifdef BatteryPowered
      if (!xBeeIsAwake)
      {
        digitalWrite(xBeeRTS, HIGH);
        sleepNow();
        digitalWrite(xBeeRTS, LOW);
        TempSensor_LastCheck++;
      }
    #endif  



    #ifdef BatteryPowered
      if (TempSensor_LastCheck >= TempSensor_CheckFreq)
      {
        TempSensor_LastCheck = 0;
        TempSensor_Check = true;
      } else {
        TempSensor_Check = false;
      }
    #else
      if ((millis()-TempSensor_LastCheck)>1000)
      {
        TempSensor_LastCheck = millis();
        TempSensor_Check = true;
      } else {
        TempSensor_Check = false;
      }
    #endif



  	rxResult = zb.RX(10);                                                     // Check for incoming packets for 10ms

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
    
    /*#ifdef OnOffCluster
      if (buttonIsPressed)
      { 
        #ifdef BatteryPowered
          WakexBee();
        #endif
                
        digitalWrite(LEDPin,!pinState(LEDPin));                                 // Toggle Output
        SendOnOffReport(pinState(LEDPin));                                // Let SmartThings know about it
      
        buttonIsPressed = false;                                                 // Don't allow anymore toggling until button is released and pressed again

        //now = millis();                                                         // Save the current time for debouncing
      }
    #endif */

      
    #ifdef BMP280
      if (TempSensor_Check)
      {
        Serial.println(F("Getting temp"));
        Temperature = bmp.readTemperature() * 10;
        Pressure = (int) (bmp.readPressure() / 10.0);
      }
    #endif  
    
    #ifdef DHTTYPE
      if (TempSensor_Check)
      {
        Serial.println(F("Getting temp"));
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t)) Temperature = (int) (t * 100.0);
        if (!isnan(h)) Humidity = (int) (h * 100.0);
      }
    #endif  
    
    #ifdef TemperatureCluster
    if (abs(Temperature - old_Temperature) >= 5)
    {
      Serial.print(F("New temperature is :"));
      Serial.println(Temperature);
      SendTemperatureReport(Temperature);
      old_Temperature = Temperature;
    }
    #endif

    #ifdef PressureCluster
    if (abs(Pressure - old_Pressure) >= 5)
    {
      Serial.print(F("New pressure is :"));
      Serial.println(Pressure);
      SendPressureReport(Pressure);
      old_Pressure = Pressure;
    }
    #endif

    #ifdef HumidityCluster
    if (abs(Humidity - old_Humidity) >= 5)
    {
      Serial.print(F("New humidity is :"));
      Serial.println(Humidity);
      SendHumidityReport(Humidity);
      old_Humidity = Humidity;
    }
    #endif
       
  } while (rxResult == -2);                                                   // Keep checking for packets until there is no timeout while checking
  if (rxResult == true)
  {
    Serial.println(F("Good packet received:"));
    Serial.print(F("\tIEEE Add: "));
    Serial.print(long(zb._PktIEEEAddHi()),HEX);
    Serial.print(long(zb._PktIEEEAddLo()),HEX);
    Serial.println();
  
    Serial.print(F("\tNet Add: "));
    Serial.print(long(zb._PktNetAdd()),HEX);
    Serial.println();
  
    Serial.print(F("\tDst EP: "));
    Serial.print(byte(zb._PktDEP()),HEX);
    Serial.println();
  
    Serial.print(F("\tSrc EP: "));
    Serial.print(byte(zb._PktSEP()),HEX);
    Serial.println();
  
    Serial.print(F("\tProfile ID: "));
    Serial.print(word(zb._PktProfile()),HEX);
    Serial.println();
  
    Serial.print(F("\tCluster ID: "));
    Serial.print(word(zb._PktCluster()),HEX);
    Serial.println();
  
    Serial.print(F("\tPayload Hex: "));
    Serial.print(F("->"));
    for (int x=0;  x < zb._PktDataSize(); x++)
    {
      printByteData(zb._PktData()[x]);
      if (x < zb._PktDataSize()-1)
      {
        Serial.print(F(" "));
      }
    }
    Serial.println(F("<-"));
  
    if (zb._PktProfile() != 0x00)
    {
      ZCLpkt();
    }
    else
    {
      ZDOpkt();
    }
  }
  else if (rxResult == -5)
  {
    Serial.println(F("Transmit Status frame received"));
  }
  else
  {
    Serial.print(F("Unknown packet ->"));
    int x=0;
    while(zb._ReadLog()[x] != '\0')
    {  
      Serial.print(char(zb._ReadLog()[x]));
      x++;
    }
    Serial.println(F("<-"));
    Serial.print(F("Pkt ->"));
    for (int i=0; i < zb._PktDataSize(); i++)
    {
      printByteData(byte(zb._PktData()[i]));
      if (i < zb._PktDataSize()-1)
      {
        Serial.print(F(" "));
      }
    }
    Serial.println(F("<-"));
  }
}

void ResetNetwork()
{
  Serial.println();
  Serial.print(F("Sending NR Command:"));
  Serial.print(zb.AT("NR"));
  delay(1000);
  Serial.print(zb.AT("&X"));
  delay(1000);
  Serial.print(zb.AT("FR"));
  delay(1000);
}

void LeaveNetwork()
{
  //***************************************
  // Force ZigBee to leave network and start
  // looking for a network to join
  //***************************************
  Serial.println();
  Serial.print(F("Sending Leave PAN Command:"));
  Serial.print(zb.ATbyte("CB", 0x04));
 // resetXB();
  delay(1000);
}

void resetXB()
{
  //***************************************
  // Do a hardware reset to xBee
  // Required afer a leave network command has been sent
  //***************************************
  Serial.println(zb.AT("FR"));
  pinMode(xBeeReset, OUTPUT);    // set xBeeReset pin to output
  digitalWrite(xBeeReset, LOW);  // drive xBeeReset pin low
  delay(1);                      // Reset requires 26 microseconds
  pinMode(xBeeReset, INPUT);     // set xBeeReset pin to input
}

//ZigBee Device Objects commands follow

void ZDOpkt()
{
  //***************************************
  // ZigBee Device Objects (ZDO) cluster numbers
  // ZDO are defined in the ZigBee Specification Document 053474r20
  // http://www.zigbee.org/wp-content/uploads/2014/11/docs-05-3474-20-0csg-zigbee-specification.pdf
  //***************************************
  switch (int(zb._PktCluster()))
  {
    case 0x0004:
      Simple_Desc_req();                                                      // Page 105 of ZBSpec
      break;
    case 0x0005:
      Active_EP_req();                                                        // Page 105 of ZBSpec
      break;
    default: Serial.print(F("** ZDOpkt received but there is no valid handler - "));
      Serial.println(int(zb._PktCluster()), HEX);
      break;
  }
}

void Simple_Desc_req()                                                        // ZDO Cluster 0x8004 Simple_Desc_rsp
{
  //***************************************
  // Simple_Desc_rsp Cluster 0x8004 on page 159 of ZBSpec
  // A Simple Description Request packet cluster 0x0004 is used to discover the configuration of an end point. The results of this command are
  // returned in a Simple_Desc_res packet cluster 0x8004 that contains the ZigBee Profile, ZigBee Device Type, and a list of inbound and
  // outbound clusters for the requested end point.  This command is sent once for each end point to discover all services available.
  //***************************************
  int packetSize;
  Serial.println(F("ZDOpkt Packet Received - Simple Descriptor cluster 0x0004. Responding with 0x8004"));
  
  byte epToRpt = byte(zb._PktData()[3]);                                      // End Point to report Simple Desc on

  Serial.println();
  Serial.print(F("Reporting Simple Desc for End Point "));
  Serial.print(epToRpt,HEX);

  switch (epToRpt) {
    case 0x01:                                                        // Report for end point 0x38
      memset(Buffer, 0 , BufferSize);
      Buffer[0] = zb._PktData()[0];                                             // Set Transaction Seq number to match inbound packet's seq number
      Buffer[1] = 0x00;                                                         // Status 0x00 = success Table 2.93 on page 159 of ZBSpec
      Buffer[2] = zb._PktData()[1];                                             // Set Network address little endian order
      Buffer[3] = zb._PktData()[2];
      Buffer[4] = 0x0E;                                                         // Length in bytes of the Simple Descriptor to Follow

      Buffer[5] = 0x01;                                                         // Endpoint of the simple descriptor Table 2.38 on page 88 of ZBSpec

      Buffer[6] = 0x04;                                                         // Application Profile ID 2 Bytes Little endian. 0x0104 = Home Automation Profile
      Buffer[7] = 0x01;
      Buffer[8] = 0x02;                                                         // Device type 2 Bytes Little endian, 0x0002 = On/Off Output see page 42 of ZigBee Home Automation Profile
      Buffer[9] = 0x00;

      Buffer[10] = 0x00;                                                        // App Dev Version 4bits + reserved 4bits

      Buffer[11] = 0x00;                                                        // Input cluster count in this case we only have 0x02 input clusters

      packetSize = 12;
      
      Buffer[packetSize++] = 0x00;                                                        // Input cluster list 2 bytes each little endian. 0x0000 = Basic Cluster
      Buffer[packetSize++] = 0x00;
      Buffer[11]++; 

      #ifdef OnOffCluster
        Buffer[packetSize++] = 0x06;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
        Buffer[packetSize++] = 0x00;
        Buffer[11]++; 
      #endif

      #ifdef TemperatureCluster
        Buffer[packetSize++] = 0x02;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
        Buffer[packetSize++] = 0x04;
        Buffer[11]++; 
      #endif

      #ifdef PressureCluster
        Buffer[packetSize++] = 0x03;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
        Buffer[packetSize++] = 0x04;
        Buffer[11]++; 
      #endif
      
      #ifdef HumidityCluster
        Buffer[packetSize++] = 0x05;                                                        // Output cluster 2 bytes each little endian. 0x0006 = On / Off Cluster
        Buffer[packetSize++] = 0x04;
        Buffer[11]++; 
      #endif 
             
      Buffer[packetSize++] = 0x00;                                                        // Output cluster list. No output clusters

      zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), 0, 0, 0, 0x8004, Buffer, packetSize);
    break;

      
    default:
      memset(Buffer, 0 , BufferSize);
      Buffer[0] = zb._PktData()[0];                                             // Set Transcation Seq number to match inbound packets seq number
      Buffer[1] = 0x82;                                                         // Status 0x82 = Invalid_EP page 212 of ZigBee Specification
      Buffer[2] = zb._PktData()[1];                                             // Set Network address little endian order
      Buffer[3] = zb._PktData()[2];
      Buffer[4] = 0x00;                                                         // Length in bytes of the Simple Descriptor to Follow

      zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), 0, 0, 0, 0x8004, Buffer, 5);
    break;
  }
}

void Active_EP_req()                                                          // ZDO Cluster 0x8005 Active_EP_rsp
{
  //***************************************
  // Active_EP_rsp Cluster 0x8005 see page 161 of ZBSpec
  // Active EP Request ZDO cluster 0x0005 is used to discover the end points of a ZigBee device.
  // The results of this request are returned in ZDO Cluster 0x8005, a packet that list the end point count and each end point number.
  //***************************************
  Serial.println(F("ZDOpkt Packet Received - Reporting Active End Points cluster 0x0005. Responding with 0x8005"));
  memset(Buffer, 0 , BufferSize);
  Buffer[0] = zb._PktData()[0];                                               // Set Transaction Seq number to match inbound packets seq number
  Buffer[1] = 0x00;                                                           // Status 0x00 = success Table 2.94 on page 161 of ZBSpec
  Buffer[2] = zb._PktData()[1];                                               // Set Network address little endian order
  Buffer[3] = zb._PktData()[2];
  Buffer[4] = 0x01;                                                           // Active EndPoint count only one in this case page 161 of ZBSpec
  Buffer[5] = 0x01;                                                           // EndPoint number
  zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), 0, 0, 0, 0x8005, Buffer, 6);
}

//ZigBee Cluster Library commands follow
void clstr_Basic()                                                            // Cluster 0x0000 Basic
{
  //***************************************
  // ZCL Cluster 0x0000 Basic Cluster
  // Section 3.2 on page 78 of ZCL
  //***************************************
  int StrLen = 0;
  byte seqNum = 0;
  byte cmdID = 0;
  word attributeID = 0;

  seqNum = byte(zb._PktData()[1]);                                            // Transaction seq number can be any value used in return packet to match a response to a request
  cmdID = byte(zb._PktData()[2]);                                             // Command ID Byte P16 of ZCL
  attributeID = byte(zb._PktData()[4]);                                       // Attribute ID Word(little endian) P126 of ZCL
  attributeID = (attributeID << 8) + byte(zb._PktData()[3]);

  Serial.println();
  Serial.print(F("Basic Cluster read attribute ID "));
  Serial.print(attributeID,HEX);

  if (cmdID == 0x00 && attributeID == 0x0001)                                 // Read Attribute 0x0001 ApplicationVersion
  {
    memset(Buffer, 0 , BufferSize);                                           // Clear response buffer
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x01;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x00;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x20;                                                         // Attribute Data Type 0x20 = Unsigned 8-bit integer, see Table 2.16 on page 55 of ZCL

    Buffer[7] = AppVersion;                                                   // Single byte value

    Serial.println();
    Serial.print(F("Read Application Ver, sending result"));
    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8);
  }
  if (cmdID == 0x00 && attributeID == 0x0003)                                 // Read Attribute 0x0003 HWVersion
  {
    memset(Buffer, 0 , BufferSize);                                           // Clear response buffer
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x03;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x00;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x20;                                                         // Attribute Data Type 0x20 = Unsigned 8-bit integer, see Table 2.16 on page 55 of ZCL

    Buffer[7] = HardwareVersion;                                              // Single byte value

    Serial.println();
    Serial.print(F("Read Hardware Ver, sending result"));
    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8);
  }
  if (cmdID == 0x00 && attributeID == 0x0004)                                 // Read Attribute 0x0004 ManufacturerName
  {
    StrLen = sizeof(Manufacturer)-1;
    memset(Buffer, 0 , BufferSize);                                           // Clear response buffer
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x04;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x00;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x42;                                                         // Attribute Data Type 0x42 = Character string, see Table 2.16 on page 56 of ZCL

    Buffer[7] = StrLen;                                                       // Character string size

    memcpy(&Buffer[8], &Manufacturer, StrLen);                                // Copy byte string array into buffer

    Serial.println();
    Serial.print(F("Read Manufacturer request, sending result"));
    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8 + StrLen);
  }
  if (cmdID == 0x00 && attributeID == 0x0005)                                 // Read Attribute 0x0005 ModelIdentifier
  {
    StrLen = sizeof(Model)-1;
    memset(Buffer, 0 , BufferSize);                                           // Clear response buffer
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x05;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x00;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x42;                                                         // Attribute Data Type 0x42 = Charcter string, see Table 2.16 on page 56 of ZCL

    Buffer[7] = StrLen;                                                       // Character string size

    memcpy(&Buffer[8], &Model, StrLen);                                       // Copy byte string array into buffer

    Serial.println();
    Serial.print(F("Read Model request, sending result"));
    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8 + StrLen);
  } 
  if (cmdID == 0x00 && attributeID == 0x4000)                                 // Read Attribute 0x4000 SW rsion
  {
    StrLen = sizeof(Model)-1;
    memset(Buffer, 0 , BufferSize);                                           // Clear response buffer
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x00;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x40;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x42;                                                         // Attribute Data Type 0x42 = Charcter string, see Table 2.16 on page 56 of ZCL

    Buffer[7] = StrLen;                                                       // Character string size

    memcpy(&Buffer[8], &Version, StrLen);                                       // Copy byte string array into buffer

    Serial.println();
    Serial.print(F("Read SW Version request, sending result"));
    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8 + StrLen);
  }
}

#ifdef OnOffCluster
void clstr_OnOff()                                                            // Cluster 0x0006 On/Off
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  byte frmType, seqNum, cmdID;
  word attributeID;
  frmType = byte(zb._PktData()[0]);                                           // Frame Type is bit 0 and 1 of Byte 0 P14 of ZCL
  frmType = frmType & 3;                                                      // Bitwise AND (&) with a mask to make sure we are looking at first two bits
  seqNum = byte(zb._PktData()[1]);                                            // Transaction seq number can be any value used in return packet to match a response to a request
  cmdID = byte(zb._PktData()[2]);                                             // Command ID Byte P16 of ZCL
  attributeID = byte(zb._PktData()[4]);                                       // Attribute ID Word(little endian) P126 of ZCL
  attributeID = (attributeID << 8) + byte(zb._PktData()[3]);                                       

  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x00)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  {
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x00;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x00;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x10;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL


    Buffer[7] = pinState(LEDPin);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Serial.println();
    Serial.print(F("Read attribute request, sending result. LED is "));
    if (pinState(LEDPin) == true)
    {
      Serial.print(F("on."));
    }
    else
    {
      Serial.print(F("off."));
    }
    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8);
  }

  if (frmType == 0x01 && cmdID == 0x00 && attributeID == 0x00)                // Set device Off -- P126 Section 3.8.2.3 of ZCL
  {
    digitalWrite(LEDPin, LOW);                                                // Turn LED off - 0V
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send default response back to originator of command
    Serial.println();
    Serial.print(F("LED Off (Pin "));
    Serial.print(LEDPin,DEC);
    Serial.print(F(" set to ground)"));
  }

  if (frmType == 0x01 && cmdID == 0x01 && attributeID == 0x00)                // Set device On
  {
    digitalWrite(LEDPin, HIGH);                                               // Set High
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send default response back to originator of command
    Serial.println();
    Serial.print(F("LED On (Pin "));
    Serial.print(LEDPin,DEC);
    Serial.print(F(" set to High)"));
  }

  if (frmType == 0x01 && cmdID == 0x02 && attributeID == 0x00)                // Toggle device
  {
    digitalWrite(LEDPin,!pinState(LEDPin));                                   // Toggle Output
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send Default response back to originator of command
    Serial.println();
    Serial.print(F("LED Toggle"));
  }
}
#endif

void ZCLpkt()
{
  //***************************************
  // ZigBee Cluster Library (ZCL) cluster numbers supported
  // ZCL are defined in the ZigBee Cluster Library Document 075123r04ZB
  // http://www.zigbee.org/wp-content/uploads/2014/11/docs-07-5123-04-zigbee-cluster-library-specification.pdf
  //***************************************
  switch (int(zb._PktCluster()))
  {
    case 0x0000:
      clstr_Basic();                                                          // Basic Cluster Page 78 of ZigBee Cluster Library
      break;
      
    #ifdef OnOffCluster  
    case 0x0006:
      clstr_OnOff();                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;
    #endif  

    #ifdef TemperatureCluster
    case 0x0402:
      clstr_Temperature();                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;
    #endif 

    #ifdef PressureCluster
    case 0x0403:
      clstr_Pressure();                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;
    #endif  

    #ifdef HumidityCluster
    case 0x0405:
      clstr_Humidity();                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;
    #endif  
    
    default: Serial.println(F("** ZCLpkt received but there is no valid handler! **"));
  }
}

#ifdef TemperatureCluster
void clstr_Temperature()                                                            // Cluster 0x0402 Temp
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  byte frmType, seqNum, cmdID;
  word attributeID;
  frmType = byte(zb._PktData()[0]);                                           // Frame Type is bit 0 and 1 of Byte 0 P14 of ZCL
  frmType = frmType & 3;                                                      // Bitwise AND (&) with a mask to make sure we are looking at first two bits
  seqNum = byte(zb._PktData()[1]);                                            // Transaction seq number can be any value used in return packet to match a response to a request
  cmdID = byte(zb._PktData()[2]);                                             // Command ID Byte P16 of ZCL
  attributeID = byte(zb._PktData()[4]);                                       // Attribute ID Word(little endian) P126 of ZCL
  attributeID = (attributeID << 8) + byte(zb._PktData()[3]);                                       

  Serial.print(F("Read temp attribute request."));
  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x00)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  {
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x00;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x00;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x29;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL


    Buffer[7] = lowByte(Temperature);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Buffer[8] = highByte(Temperature);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Serial.println();
    Serial.print(F("Read attribute request, sending result. Temp is "));
    Serial.println(Temperature/100.0);

    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 9);
  }
}
#endif

#ifdef PressureCluster
void clstr_Pressure()                                                            // Cluster 0x0402 Temp
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  byte frmType, seqNum, cmdID;
  word attributeID;
  frmType = byte(zb._PktData()[0]);                                           // Frame Type is bit 0 and 1 of Byte 0 P14 of ZCL
  frmType = frmType & 3;                                                      // Bitwise AND (&) with a mask to make sure we are looking at first two bits
  seqNum = byte(zb._PktData()[1]);                                            // Transaction seq number can be any value used in return packet to match a response to a request
  cmdID = byte(zb._PktData()[2]);                                             // Command ID Byte P16 of ZCL
  attributeID = byte(zb._PktData()[4]);                                       // Attribute ID Word(little endian) P126 of ZCL
  attributeID = (attributeID << 8) + byte(zb._PktData()[3]);                                       

  Serial.print(F("Read pressure attribute request."));
  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x00)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  {
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x00;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x00;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x29;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL


    Buffer[7] = lowByte(Pressure);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Buffer[8] = highByte(Pressure);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Serial.println();
    Serial.print(F("Read attribute request, sending result. Pressure is "));
    Serial.println(Pressure/10.0);

    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 9);
  }
}
#endif

#ifdef HumidityCluster
void clstr_Humidity()                                                            // Cluster 0x0402 Temp
{
  //***************************************
  // ZCL Cluster 0x0006 On/Off Cluster
  // Section 3.8 on page 125 of ZCL
  //***************************************
  byte frmType, seqNum, cmdID;
  word attributeID;
  frmType = byte(zb._PktData()[0]);                                           // Frame Type is bit 0 and 1 of Byte 0 P14 of ZCL
  frmType = frmType & 3;                                                      // Bitwise AND (&) with a mask to make sure we are looking at first two bits
  seqNum = byte(zb._PktData()[1]);                                            // Transaction seq number can be any value used in return packet to match a response to a request
  cmdID = byte(zb._PktData()[2]);                                             // Command ID Byte P16 of ZCL
  attributeID = byte(zb._PktData()[4]);                                       // Attribute ID Word(little endian) P126 of ZCL
  attributeID = (attributeID << 8) + byte(zb._PktData()[3]);                                       

  Serial.print(F("Read humidity attribute request."));
  if (frmType == 0x00 && cmdID == 0x00 && attributeID == 0x00)                // frmType = 0x00 (General Command Frame see Table 2.9 on P16 of ZCL)
  {
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = 0x00;                                                         // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = 0x00;

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x29;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL


    Buffer[7] = lowByte(Humidity);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Buffer[8] = highByte(Humidity);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Serial.println();
    Serial.print(F("Read attribute request, sending result. Humidity is "));
    Serial.println(Humidity/100.0);

    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 9);
  }
}
#endif

void sendDefaultResponse(byte CmdID, byte Status, byte EndPoint)
{
  //***************************************
  // Send Default Response Page 39 of ZigBee Cluster Library
  // CmdID = Byte size number representing the received command to which this command is a response see Table 2.9 on page 16 of ZigBee Cluster Library
  // Status = Byte size value specifies either Success (0x00) or the nature of the error see Table 2.17 on page 67 of ZigBee Cluster Library
  // EndPoint = Byte size value of the EndPoint this response is for
  //***************************************
  memset(Buffer, 0 , BufferSize);                                             // Clear Buffer
  Buffer[0] = 0x18;                                                           // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
  Buffer[1] = zb._PktData()[1];                                               // Set the sequence number to match the seq number in requesting packet
  Buffer[2] = 0x0B;                                                           // Command Identifer 0x0B = Default response see Table 2.9 on page 16 of ZCL

  Buffer[3] = CmdID;                                                          // Command Identifer to report on

  Buffer[4] = Status;                                                         // Status see Table 2.17 on page 67 of ZigBee Cluster Library

  Serial.println();
  Serial.print(F("Sending Default Response"));
  zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), EndPoint, zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 5);
}

//// * * * * * * * ZigBee Packet formation Notes * * * * * * *
////
//// ZigBee data packet (payload) syntax
////                                 --------------------------------------------- Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
////                                |     ---------------------------------------- Transaction seq num 0x66 = Should be the same as the transaction number requesting response
////                                |    |     ----------------------------------- Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL
////                                |    |    |      ----------------------------- Direction field 0x00: = attribute are reported P33 of ZCL
////                                |    |    |     |    ------------------------- Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
////                                |    |    |     |   |    |     --------------- Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL
////                                |    |    |     |   |    |    |     ---------- Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL
////                                |    |    |     |   |    |    |    |
////       Resp_OnOFF      Byte   0x18 0x66 0x01  0x00 0x00 0x00 0x10 0x00

////                                 --------------------------------------------- Transaction seq number
////                                |     ---------------------------------------- Status 0x00 = success Table 2.94 on page 161 of ZBSpec
////                                |    |     ----------------------------------- Network address of interest 2 bytes little endian order page 161 of ZBSpec
////                                |    |    |    |     ------------------------- Active EndPoint count page 161 of ZBSpec
////                                |    |    |    |    |     -------------------- Attribute EndPoint List page 161 of ZBSpec
////                                |    |    |    |    |    |
////       Active_EP_rsp   Byte    0x22 0x00 0x63 0x6B 0x01 0x38

////                                 ---------------------------------------------------------------------- Transaction seq number
////                                |     ----------------------------------------------------------------- Status 0x00 = success Table 2.93 on page 159 of ZBSpec
////                                |    |     ------------------------------------------------------------ Network address of interest 2 bytes little endian order page 161 of ZBSpec
////                                |    |    |    |     -------------------------------------------------- Length in bytes of the Simple Descriptor to Follow
////                                |    |    |    |    |     --------------------------------------------- Endpoint of the simple descriptor Table 2.38 on page 88 of ZBSpec
////                                |    |    |    |    |    |     ---------------------------------------- Application Profile ID 2 Bytes Little endian. 0x0104 = Home Automation Profile
////                                |    |    |    |    |    |    |    |     ------------------------------ Application device identifier 2 Bytes Little endian, 0x0002 = On/Off Output See Table 5.1 on page 17 of ZigBee Home Automation Profile
////                                |    |    |    |    |    |    |    |    |    |     -------------------- App Dev Version 4bits + reserved 4bits
////                                |    |    |    |    |    |    |    |    |    |    |     --------------- Input cluster count in this case we only have 0x01 input clusters
////                                |    |    |    |    |    |    |    |    |    |    |    |     ---------- Input cluster list 2 bytes each little endian. 0x0006 = OnOff Cluster Page 125 of ZCL
////                                |    |    |    |    |    |    |    |    |    |    |    |    |    |    - Output cluster list. No output clusers are supported
////                                |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
////                                |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
////                                |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
////      Simple_Desc_rsp Byte    0x22 0x00 0x63 0x6B 0x0A 0x38 0x04 0x01 0x02 0x00 0x00 0x01 0x06 0x00 0x00

////                                 ---------------------------------------------------------------------- Transaction seq number
////                                |     ----------------------------------------------------------------- Network Address 2 bytes little endian Page 109 of ZBSpec
////                                |    |    |     ------------------------------------------------------- IEEE Address 8 bytes little endian
////                                |    |    |    |    |    |    |    |    |    |    |    ---------------- MAC Capability Mask See page 83 of ZBSpec
////                                |    |    |    |    |    |    |    |    |    |    |    |
////      Device_annce    Byte    0x22 0x00 0x63 0x5A 0xE1 0x70 0x40 0x00 0xA2 0x13 0x00 0x8C