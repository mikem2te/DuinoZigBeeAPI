// ***************************************************************************************************
// ZigBee-Home Automation Communications          
// Author: Michael Morgan
//
// 25/01/2017
//
// Revision 1.0
//
// ZigBee-Home Automation Communications is based on an Arduino port written by
// Professor Mayhem of Yoyodyne Monkey Works, Inc. from a version of the same name
// written in the Propeller Spin language by John.Rucker(at)Solar-Current.com
//
// This version contains the following-
//  * Refactor of the ZigBee HA code in to separate h & cpp files to simplify the INO code
//    - The XBee / ZigBee code is moved from the INO
//    - To avoid having to develop cluster handling code in the INO, the library will call functions
//      defined in the INO to perform actions or get data (eg get_Temperature() etc) 
//  * Added various sensor code (temp, humidity etc) configurable using defines
//  * I improved definition of applicable clusters and endpoints
//  * Support for sleeping XBee's and Arduino's for battery powered devices
//
// It has been reported ZHA requires the S2C versions of the XBee; The S2C version provides
// the ZigBee binding needed for the Arduino to originate a message to coordinator. However,
// S2B Pro devices appear to work fine for attribute polling and reporting. It has been observed
// reporting the same cluster across different endpoints does not work (this may be a weakness of 
// the deConz coordinator.
// 
// There is a lot of configuration that is required to configure the node/XBee. It is designed so
// the majority of the setup is performed by configuring defines and variables at the top of the 
// INO
// ***************************************************************************************************

#include <ZigBeeAPI.h>
#include "ZigBee.h"
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

// --------------------------------------------------------------------------------------------------
// -- Is the device battery powered. Remark define for fully powered / non sleepy nodes            --
// --------------------------------------------------------------------------------------------------
//#define BatteryPowered


// --------------------------------------------------------------------------------------------------
// -- Configure name and versions of the node. Basic cluster details                               --
// --------------------------------------------------------------------------------------------------
#ifdef BatteryPowered
//  const char Model[] PROGMEM  = {"Arduino XBee (Battery)"};     // ZigBee Basic Device ModelIdentifier 0 to 32 byte char string P79 of ZCL
#else
//  const char Model[] PROGMEM  = {"Arduino XBee (Powered)"};     // ZigBee Basic Device ModelIdentifier 0 to 32 byte char string P79 of ZCL
#endif
const char Model[] PROGMEM  = {"Arduino"}; 
const char Manufacturer[] PROGMEM = {"Arduino"};                // ZigBee Basic Device ManufacturerName 0 to 32 bytes
const char SWVersion[] PROGMEM = {"1.0.0"};
byte AppVersion = 1;                                            // Application version
byte HardwareVersion = 1;                                       // Hardware version 



  
// --------------------------------------------------------------------------------------------------
// -- Clusters provided by this node where each line defines the endpoint and cluster              --
// -- Having the same cluster on more than one endpoint is not supported                           --
// --------------------------------------------------------------------------------------------------
EndpointCluster endpointClusters[]= { 
  {1, cluster_Basic},
  {1, cluster_PowerConfiguration},
  {1, cluster_Identity},
  {1, cluster_OnOff},
  {1, cluster_LevelControl},
  {1, cluster_ColorControl},
  {1, cluster_Temperature},
  {1, cluster_Pressure},
  {1, cluster_RelativeHumidity}
}; 


// --------------------------------------------------------------------------------------------------
// XBee pins and setup. See setup commentry below                                                  --
// --------------------------------------------------------------------------------------------------
byte XBeeTx=4;         // This is the Arduino pin connected to the XBee's transmit pin. Not needed for Leonardo or Mega
byte XBeeRx=5;         // This is the Arduino pin connected to the XBee's receive pin. Not needed for Leonardo or Mega
byte XBeeRTS=8;        // This is the Arduino pin connected to the XBee's RTS pin.
byte XBeeReset=12;     // This is the Arduino pin connected to the XBee's reset pin, not needed if using soft reset (AT FR)
byte XBeeOnSleep = 3;  // This is the Arduino pin connected to the XBee's on/sleep pin. Not needed if using a powered node
byte XBeeSleepRQ = 9;  // This is the Arduino pin connected to the XBee's sleep rq pin. Not needed if using a powered node
int  XBeeBaud=9600;    // The baud rate of the XBee must match this number (set with the XBee's BD command)


// --------------------------------------------------------------------------------------------------
// Configuration for sensor poll frequency                                                         --
// --------------------------------------------------------------------------------------------------
byte SensorCheck_FreqWake = 2;                        // Poll sensors every x wake cycles for sleepy devices
unsigned long SensorCheck_FreqMillis = 10000;         // Poll sensors every x millseconds for non sleepy devices
unsigned long SensorCheck_RetryMillis = 50000;        // Retry a failed poll after x milli seconds for non sleepy devices
unsigned long SensorStabilisationAfterWake = 1500;    // Wait x millseconds afer wake before polling sensors


// --------------------------------------------------------------------------------------------------
// Section below defines the sensors, switches and hardware connected to this node                 --
// --------------------------------------------------------------------------------------------------
// On/Off button
#define OnOffButton
#define OnOffButtonMomentary
const byte ButtonPin = 2;          // Pin that the button is attached to that will locally turn the LED on and off. Connect the button between GND and this pin
const byte LEDPin=LED_BUILTIN;     // Pin that is connected to an LED's anode (positive side), Make sure to use a current limiting resistor in series with the LED. Connect the other end of the LED to ground  


// Bosch BME280 / BMP280
// ---------------------
#define BME280
#define TempSensorPowerPin 7


// Silicon Labs Si7021
// -------------------
//#define Si7021
//#define TempSensorPowerPin 7

  
// DHT temperatue and humidity sensors. Types are DHT11 or DHT22
// -------------------------------------------------------------
//#define DHTTYPE DHT11
//#define DHTTYPE DHT22
//#define DHTPIN 6 
//#define TempSensorPowerPin 7

  
// Dallas one wire sensors 18B2x
// -----------------------------
//#define DS182x
//#define ONE_WIRE_BUS 6
//#define TempSensorPowerPin 7

  
// XBee built in temperature sensor. This suffers from self heating
// ----------------------------------------------------------------
//#define XBeeTemp


// --------------------------------------------------------------------------------------------------
// -- This completes the configuration of the device. Further changes below should not be required --
// --------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------
// -- Setup Notes                                                                                  --
// --------------------------------------------------------------------------------------------------
// Hardware requirements:
// Arduino, XBee shield, and Digi XBee. Tested on Arduino Uno, Duemilanove, Leonardo, and Mega 2560 using a Seeed Studio XBee Shield.
// Tested both with XBee Pro ZB SMT (Surface Mount) Digi part number XBP24CZ7PIS-004 and with the XBee S2 XB24-Z7WIT-004 TH (Through Hole).
// Configure the above XBee with the following settings: ZS = 2, NJ = 0x5a, EE = Enable, EO = 1,
// KY = 0X5A6967426565416C6C69616E63653039, BD = 9600, D6 = None, AP = true, AO = 3
// 
// This demo allows you to control a LED attached to the LEDPin from a public ZigBee Home Automation network or a button on the Arduino itself. 
// The demo is designed to work with a SmartThings hub available at SmartThings.com.
// A custom SmartThings device type is available at:
// https://github.com/JohnRucker/Nuts-Volts/blob/master/devicetypes/johnrucker/parallax-propeller-demo.src/parallax-propeller-demo.groovy
// Install the above device driver in the SmartThings developer section for your hub.
// Once installed SmartThings will recognize this device and will add a tile to their smart phone app when this device joins their Hub's network.
// This new tile will allow you to send on / off commands to this device turning the LED on and off.

// Conversation overview of the initial connection to the SmartThings home automation network.
//   1) At power up the XBee will look for a valid ZigBee Home Automation coordinator or router to join, if the network is allowing new devices to
//      join it will join. An Association number will be displayed on the Serial Monitor until the network is joined. To join a ZigBee Home Automation
//      network the XBee must be configured with the proper security key and security settings outlined above in the hardware configuration.
//   2) Once joined to a valid ZigBee Home Automation network a device announce packet (ZDO Cluster 0x00013) will be sent to the coordinator.
//   3) The coordinator should respond back with a request for active end points. This request will be received and a response report will be
//      sent back (ZDO Cluster 0x8005) to the coordinator telling it what end points are supported by this application.
//   4) Next the coordinator will request a description of each end point's configuration. This application will respond to this request with
//      a report of supported device types and clusters for each end point (ZDO Cluster 0x8004).
//   5) At this point the home automation network knows what this device is and how to talk to it.
//      loop() will continue to listen to ZigBee packets and will respond to packets that send the on or off command (Cluster 0x0006).
// 
// SoftwareSerial is not needed on Leonardo, or Mega. Attach the XBee to pins 0 & 1 of the Leonardo, or pins 19, & 18 of the Mega.
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


// --------------------------------------------------------------------------------------------------
// -- This initialises various libraries and varibles used                                         --
// --------------------------------------------------------------------------------------------------

// On Off setup
// ------------
#ifdef OnOffButton
  boolean buttonReleased = true;
#endif 


// BMP setup
// ---------
#ifdef BMP280
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp;
#endif

// BME setup
// ---------
#ifdef BME280
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme;
#endif

// DHT sensor setup
// ----------------
#ifdef DHTTYPE
  #include "DHT.h"
  DHT dht(DHTPIN, DHTTYPE);
#endif


// Silicon Labs Si7021 setup
// -------------------------
#ifdef Si7021
  #include "Adafruit_Si7021.h"
  Adafruit_Si7021 sensor = Adafruit_Si7021();
#endif


// Dallas one wire sensors 18B2x setup
// -----------------------------------
#ifdef DS182x
  #include <OneWire.h>
  #include <DallasTemperature.h>
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  DeviceAddress deviceAddress;
#endif


#ifdef AddressableLED
  #include <bitswap.h>
  #include <chipsets.h>
  #include <color.h>
  #include <colorpalettes.h>
  #include <colorutils.h>
  #include <controller.h>
  #include <cpp_compat.h>
  #include <dmx.h>
  #include <FastLED.h>
  #include <fastled_config.h>
  #include <fastled_delay.h>
  #include <fastled_progmem.h>
  #include <fastpin.h>
  #include <fastspi.h>
  #include <fastspi_bitbang.h>
  #include <fastspi_dma.h>
  #include <fastspi_nop.h>
  #include <fastspi_ref.h>
  #include <fastspi_types.h>
  #include <hsv2rgb.h>
  #include <led_sysdefs.h>
  #include <lib8tion.h>
  #include <noise.h>
  #include <pixelset.h>
  #include <pixeltypes.h>
  #include <platforms.h>
  #include <power_mgt.h>
  CRGB leds[5 * 5];
  #define NUM_LEDS 25
#endif

// XBee devices setup
// ------------------
#if !defined (__AVR_ATmega32U4__) && !defined (__MK20DX128__)
  SoftwareSerial Serial1(XBeeTx, XBeeRx);
#endif  

extern ZigBeeAPI zb;                
extern volatile bool XBeeIsAwake;


// --------------------------------------------------------------------------------------------------
// -- Interrupt Service Routines                                                                   --
// --------------------------------------------------------------------------------------------------
void buttonPressed()
{ 
}


// --------------------------------------------------------------------------------------------------
// -- Setup                                                                                        --
// --------------------------------------------------------------------------------------------------
void setup()
{
  //***************************************
  // Start Serial Monitor Communications
  //***************************************
  Serial.begin(115200);

  //while (!Serial)
  //{
  //  delay(1); // wait for serial port to connect. Needed for native USB port only
  //}
  
  Serial.println();
  Serial.println(F("On Line."));
  
  #if _DEBUG
    Serial.print(F("Compiled on: "));
    Serial.print(__DATE__);
    Serial.print(F(" "));
    Serial.println(__TIME__);
  #endif

  #ifdef AddressableLED
    LEDS.addLeds<WS2811,5,RGB>(leds,NUM_LEDS);
    LEDS.setBrightness(96);   
    for(int i = 0; i < 25; i++) {
      leds[1] = CHSV(2,255,100);
    }
    LEDS.show();
  #endif  
  
  // On Off cluster setup
  #ifdef OnOffButton
    pinMode(LEDPin, OUTPUT);              // Set LEDPin as Output
    pinMode(ButtonPin, INPUT);            // Set Button pin Set as Input
    digitalWrite(ButtonPin, HIGH);
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
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
      } else {
        Serial.println(F("A valid BMP280 sensor has been found!"));
      }
    } while (!status);
  #endif

  
  #ifdef BME280
    bool status; 
    do {
      status = bme.begin();  
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

  #ifdef DS182x
    sensors.begin();
    Serial.print(F("Found DS182x "));
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(F(" devices."));
    if (!sensors.getAddress(deviceAddress, 0)) Serial.println(F("Unable to find address for Device 0")); 
  #endif
    
  #ifdef OnOffButton
    attachInterrupt(digitalPinToInterrupt(ButtonPin), buttonPressed, CHANGE);
  #endif

  Serial1.begin(XBeeBaud);
  
  #ifdef BatteryPowered
    setup_ZigBee(Serial1, sizeof(endpointClusters) / sizeof(EndpointCluster), true);
  #else
    setup_ZigBee(Serial1, sizeof(endpointClusters) / sizeof(EndpointCluster), false);
  #endif
  now = millis();

  //LeaveNetwork();
}


// --------------------------------------------------------------------------------------------------
// -- Loop                                                                                         --
// --------------------------------------------------------------------------------------------------
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
	  if ((digitalRead(ButtonPin) == false) && ((millis()-now)>100) && (buttonReleased == true)) // Check if the button is being pressed
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
	  if (digitalRead(ButtonPin) == true)                                     // Check if the button has been released*/
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

bool get_OnOff(byte endPoint)
{
//  if (endPoint == 1)
  {
    #ifdef OnOffButton
      return pinState(LEDPin);
    #endif  
  }
  return false;
}

void set_OnOff(byte endPoint, bool On)
{
//  if (endPoint == 1)
  {
    #ifdef OnOffButton
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
    #endif  
  }
}

void clstr_LevelControlSetLevel(byte endPoint, byte level)
{
  Serial.print(F("Level:"));
  Serial.println(level);
}

void clstr_ColorControlSetHueSaturation(byte endPoint, byte hue, byte saturation)
{
  Serial.print(F("Hue:"));
  Serial.print(hue);
  Serial.print(F("  Saturation:"));
  Serial.println(saturation);  
}

void toggle_OnOff(byte endPoint)
{
//  if (endPoint == 1)
  {
    #ifdef OnOffButton
      digitalWrite(LEDPin,!pinState(LEDPin));                                   // Toggle Output
      Serial.print(F("(OnOff) LED Toggled (Pin "));
      Serial.print(LEDPin,DEC);
      Serial.print(F(" toggled)")); 
    #endif 
  }
}

float get_Temperature(byte endPoint)
{
  float t = NAN;

//  if (endPoint == 1)
  {
    #ifdef BMP280
      Serial.print(F("BMP280"));
      t = bmp.readTemperature() * 10;
    #endif
      
    #ifdef BME280
      Serial.print(F("BME280"));
      t = bme.readTemperature();
    #endif  
    
    #ifdef DHTTYPE
      Serial.print(F("DHT"));
      Serial.flush();
      if (XBeeIsAwake)
      {
        while (XBeeIsAwake) { } // DHT has poor voltage stability so wait until the XBee is off
        delay(500);
      }
      t = dht.readTemperature();
    #endif  

    #ifdef Si7021
      Serial.print(F("Si7021"));
      Serial.flush();
      t = sensor.readTemperature();
    #endif

    #ifdef DS182x
      sensors.requestTemperatures(); // Send the command to get temperatures
      t = sensors.getTempC(deviceAddress);
    #endif
          
    #ifdef XBeeTemp
      Serial.print(F("XBee"));
      t = Get_XBeeTemp(); 
    #endif
  }

  Serial.print(F("Sensor, Temp:"));
  Serial.println(t);
  return t;
}

float get_Pressure(byte endPoint)
{
  Serial.flush();
  float p = NAN;

  #ifdef BMP280
    Serial.print(F("BMP280"));
    p = bmp.readPressure() / 10.0;
  #endif  

  #ifdef BME280
    Serial.print(F("BME280"));
    p = bme.readPressure() / 100.0;
  #endif  
  
  Serial.print(F("Sensor, Pressure:"));
  Serial.println(p);
  return p;
}

float get_Humidity(byte endPoint)
{
  float h = NAN;

  #ifdef BME280
    Serial.print(F("BME280"));
    h = bme.readHumidity();
  #endif  
    
  #ifdef DHTTYPE
    Serial.print(F("DHT"));
    Serial.flush();
    if (XBeeIsAwake)
    {
      while (XBeeIsAwake) { } // DHT has poor voltage stability so wait until the XBee is off
      delay(500);
    }
    h = dht.readHumidity();
  #endif 

  #ifdef Si7021
    Serial.print(F("Si7021"));
    Serial.flush();
    h = sensor.readHumidity();
  #endif
    
  Serial.print(F("Sensor, Humidity:"));
  Serial.println(h);
  return h;
}



