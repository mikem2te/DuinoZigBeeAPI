#include "ZigBee.h"
#include <ZigBeeAPI.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
  
const char FuncNotImplemented[] = "** FUNCTION NOT IMPLEMENTED **";
const char mths[] = "Jan01Feb02Mar03Apr04May05Jun06Jul07Aug08Sep09Oct10Nov11Dec12";

extern EndpointCluster endpointClusters[];
byte endpointClusterCount; 

byte Buffer[BufferSize];
word LclNet = 0;
unsigned long LclIeeeLo = 0;
unsigned long LclIeeeHi = 0;
unsigned long now = 0;
volatile bool xBeeIsAwake = false;

extern int xBeeReset;
extern int xBeeSleepRQ;
extern int xBeeRTS;
extern int xBeeOnSleep;
extern int xBeeBaud;

extern int rxResult;
extern bool BatteryPowered;

unsigned long Sensor_LastCheckWake = 0;
unsigned long Sensor_LastCheckMillis = 0;
extern unsigned long Sensor_CheckFreqWake; // Wake cycles for sleepy device, ms for non sleepy
extern unsigned long Sensor_CheckFreqMillis; // Wake cycles for sleepy device, ms for non sleepy
   
ZigBeeAPI zb;

bool Sensor_RequireRetry;

int old_Temperature;
unsigned int old_Humidity;
int old_Pressure;

unsigned int Sensor_ReportFreq = 4; // Wake cycles for sleepy device, ms for non sleepy
unsigned int TempSensor_Report = 0;
unsigned int HumiditySensor_Report = 0;
unsigned int PressureSensor_Report = 0;

extern char Model[];
extern char Manufacturer[];
extern char SWVersion[];
extern byte AppVersion;
extern byte HardwareVersion;

// ----------------------------------
// Utility functions
// ----------------------------------
bool pinState(byte pin) {return (0!=(*portOutputRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin)));};

void printByteData(uint8_t Byte)
{
  Serial.print((uint8_t)Byte >> 4, HEX);
  Serial.print((uint8_t)Byte & 0x0f, HEX);
}

void formatDate(char const *date, char const *tm, char *buff)
{ 
  buff[0] = date[7];
  buff[1] = date[8];
  buff[2] = date[9];
  buff[3] = date[10];

  buff[6] = date[4];
  buff[7] = date[5];

  for (int i = 0; i < 55; i = i + 5)
  {
    if (date[0] == mths[i] && date[1] == mths[i+1] && date[2] == mths[i+2])
    {
      buff[4] = mths[i+3];
      buff[5] = mths[i+4];
    }
  }

  buff[8] = '.';
  buff[9] = tm[0];
  buff[10] = tm[1];
  buff[11] = tm[3];
  buff[12] = tm[4];
  buff[13] = tm[6];
  buff[14] = tm[7];
  buff[15] = 0;
}



// ----------------------------------
// xBee, Network and Reset management
// ----------------------------------
void ConfigurexBee(Stream& port)
{
  // Configure xBee RTS pin
  if (xBeeRTS > 0)
  {
    pinMode(xBeeRTS, OUTPUT);
    digitalWrite(xBeeRTS, LOW);
  }

  if (xBeeOnSleep > 0)
  {
    pinMode(xBeeOnSleep,INPUT);   // set Pin as Input (default)
    digitalWrite(xBeeOnSleep,HIGH);  // enable pullup resistor
  }
  
  if (xBeeSleepRQ > 0)
  {
    pinMode(xBeeSleepRQ,OUTPUT);   // set Pin as Input (default)
    digitalWrite(xBeeSleepRQ,LOW);  // Wake up xBee
  }
 
  zb.begin(port);

  if (BatteryPowered)
  {
    attachInterrupt(digitalPinToInterrupt(xBeeOnSleep), xBeeAwakeChange, CHANGE);
    delay(20);
    xBeeIsAwake = digitalRead(xBeeOnSleep);
  }
}

void xBeeAwakeChange()
{ 
  xBeeIsAwake = digitalRead(xBeeOnSleep);
  
  if (xBeeIsAwake) {
    //Serial.println(F("xBee is Awake"));
  } else {
    //Serial.println(F("xBee is Sleeping"));
  }
}

void sleepNow()         // here we put the arduino to sleep
{
  if (!xBeeIsAwake)
  {
    Serial.println(F("xBee is Sleeping"));
    digitalWrite(xBeeRTS, HIGH);
    Serial.println(F("Arduino Sleeping, waiting for awake signal"));
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
  sleep_enable();          // enables the sleep bit in the mcucr register so sleep is possible. just a safety pin
 
  //Serial.println(F("Arduino Sleeping, waiting for awake signal"));

  delay(100);
    
  sleep_mode();            // Put the device  to sleep!! THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
   
    if (xBeeIsAwake)
      Serial.println(F("XBee is Awake, Arduino Waking"));
      else   
        Serial.println(F("Arduino Waking, assume Arduino interrupt"));  
      digitalWrite(xBeeRTS, LOW);
      Sensor_LastCheckWake++;   
  }
  //if (xBeeIsAwake) Serial.println(F("xBee is Awake"));
             
  //Serial.println(F("Arduino Waking"));   
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

void Tx_Device_annce()                                                        // ZDO Cluster 0x0013 Device_annce
{
  //***************************************
  // Device_annce ZDO Cluster 0x0013 see page 109 of ZBSpec
  // This method announces that this device is now on the network and ready to receive packets
  // Called every time the Arduino reboots
  // Device announce is sent to the ZigBee Coordinator
  //***************************************
  Serial.println("Announcing Device");
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
  if (BatteryPowered)
    Buffer[11] = 0x80;          // BATTERY MODE  
  else                                             
    Buffer[11] = 0x8C;          // 0x8C = b10001100 = Mains powered device, Receiver on when idle, address not self-assigned
 
                                                                              
  zb.TX(0x00000000, 0x0000FFFF, 0xFFFD, 0, 0, 0, 0x0013, Buffer, 12);         // $FFFD = Broadcast to all non sleeping devices
}

void JoinNetwork()
{
  //***************************************
  // Loop until xBee is joined to a ZigBee network
  // See page 246 of XBee/XBee-PRO S2C ZigBee User Guide for an explanation of AI Network Join Status numbers
  // http://www.digi.com/resources/documentation/digidocs/pdfs/90002002.pdf
  //***************************************

  Serial.println();
  Serial.print(F("Network Join Status: "));
  
  if (BatteryPowered) WakexBee(true);
  
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
        if (BatteryPowered) WakexBee(true);
      
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
}

void SetupAddresses()
{
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

void WakexBee(bool force)
{
  if ((!xBeeIsAwake && BatteryPowered) || force)
  {
    Serial.println(F("Kicking xBee"));
    digitalWrite(xBeeSleepRQ,HIGH);  // Wake up xBee
    delay(10);
    digitalWrite(xBeeSleepRQ,LOW);  // Wake up xBee
    delay(50);
  }
}

float Get_xBeeTemp()
{
  if (BatteryPowered) WakexBee(true);
  
  if(zb.AT("TP"))
  {
    return (zb._PktData()[0] * 256 + zb._PktData()[1]);
  }
  else
  {
    int x=0;
    while(zb._ReadLog()[x] != '\0')
    {  
      Serial.print(char(zb._ReadLog()[x]));
      x++;
    }
    return NAN;
   }
}



// --------------------------
// Inbound packet processing
// --------------------------
// If RX not true it will =
//   -1 = Unknown Digi API Frame Type (Only 0x91 and 0x88 are supported)
//   -2 = Timeout waiting for data
//   -3 = RX Packet ID not known.
//   -4 = Bad checksum
//   -5 = Transmit Status frame received
//   -9 = xBee sleeping
int CheckInboundPackets(bool extendedTimeout)
{
  int rxResult = 0;

  if ((BatteryPowered) && (!xBeeIsAwake)) return -9;
  
  do
  {
    rxResult = zb.RX(extendedTimeout?500:20);                                                     // Check for incoming packets for 10ms

    // xBee packet received. Process
    if (rxResult != -2)
    {
      ProcessInboundPacket(rxResult);
    }
  } while (rxResult != -2);
  return rxResult;
}

void ProcessInboundPacket(int rxResult)
{
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
    Serial.println(F("Transmit Status frame received"));
  else if (rxResult == -9)
    Serial.println(F("XBee sleeping"));
  else
  {
    if (rxResult == -1)
      Serial.println(F("Unknown Digi API Frame Type ->"));
    else if (rxResult == -3)
      Serial.println(F("RX Packet ID not known ->"));
    else 
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

  

// -------------------------------------
// ZigBee Device Objects commands 
// -------------------------------------
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

int get_EndPointList(byte *list)
{
  int endPointCount = 0;
  //int charPtr=0;
  bool allreadyExists = false;
  
  for (int i = 0; i < endpointClusterCount; i++)
  {
      allreadyExists = false;
      
      for (int j = 0; j < endPointCount; j++)
      {
          if (list[j] == endpointClusters[i].endpoint) allreadyExists = true;
      }
      
      if (!allreadyExists)
      {
        list[endPointCount] = endpointClusters[i].endpoint;
        endPointCount++;
      }
  }
  Serial.print(F("Total number of endpoints:")); 
  Serial.println(endPointCount);
  return endPointCount;
}

int get_ClustersForEndPoint(byte endPoint, unsigned int *list)
{
  int clusterCount = 0;
  int charPtr=0;
  
  for (int i = 0; i < endpointClusterCount; i++)
  {
      if (endpointClusters[i].endpoint == endPoint)
      {
        list[clusterCount] = endpointClusters[i].cluster;
        clusterCount++;
      }
  }
  Serial.print(F("Total number of clusters for endpoint ")); 
  Serial.println(endPoint);
  Serial.print(F(":")); 
  Serial.println(clusterCount);
  return clusterCount;
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
  int clusterCount;
  unsigned int ClusterList[ClusterListSize];
    
  Serial.println(F("ZDOpkt Packet Received - Simple Descriptor cluster 0x0004. Responding with 0x8004"));
  
  byte epToRpt = byte(zb._PktData()[3]);                                      // End Point to report Simple Desc on

  clusterCount = get_ClustersForEndPoint(epToRpt, ClusterList);
    
  Serial.println();
  Serial.print(F("Reporting Simple Desc for End Point "));
  Serial.println (epToRpt,HEX);

  if (clusterCount > 0)
  {
                                                             // Report for end point 0x38

      memset(Buffer, 0 , BufferSize);
      
      Buffer[0] = zb._PktData()[0];                                             // Set Transaction Seq number to match inbound packet's seq number
      Buffer[1] = 0x00;                                                         // Status 0x00 = success Table 2.93 on page 159 of ZBSpec
      Buffer[2] = zb._PktData()[1];                                             // Set Network address little endian order
      Buffer[3] = zb._PktData()[2];
      Buffer[4] = 8 + clusterCount * 2;                                           // Length in bytes of the Simple Descriptor to Follow

      Buffer[5] = epToRpt;                                                         // Endpoint of the simple descriptor Table 2.38 on page 88 of ZBSpec

      Buffer[6] = 0x04;                                                         // Application Profile ID 2 Bytes Little endian. 0x0104 = Home Automation Profile
      Buffer[7] = 0x01;
      Buffer[8] = 0x02;                                                         // Device type 2 Bytes Little endian, 0x0002 = On/Off Output see page 42 of ZigBee Home Automation Profile
      Buffer[9] = 0x00;

      Buffer[10] = 0x00;                                                        // App Dev Version 4bits + reserved 4bits

      Buffer[11] = clusterCount;                                                  // Input cluster count in this case we only have 0x02 input clusters                                                       

      packetSize = 12;

      for (int i = 0; i < clusterCount * 2; i++)
      {
        Buffer[packetSize++] = lowByte(ClusterList[i]);
        Buffer[packetSize++] = highByte(ClusterList[i]);
      }
             
      Buffer[packetSize++] = 0x00;                                                        // Output cluster list. No output clusters

      zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), 0, 0, 0, 0x8004, Buffer, packetSize);
  }
   else
   {
      memset(Buffer, 0 , BufferSize);
      Buffer[0] = zb._PktData()[0];                                             // Set Transcation Seq number to match inbound packets seq number
      Buffer[1] = 0x82;                                                         // Status 0x82 = Invalid_EP page 212 of ZigBee Specification
      Buffer[2] = zb._PktData()[1];                                             // Set Network address little endian order
      Buffer[3] = zb._PktData()[2];
      Buffer[4] = 0x00;                                                         // Length in bytes of the Simple Descriptor to Follow

      zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), 0, 0, 0, 0x8004, Buffer, 5);
  }
}

void Active_EP_req()                                                          // ZDO Cluster 0x8005 Active_EP_rsp
{
  //***************************************
  // Active_EP_rsp Cluster 0x8005 see page 161 of ZBSpec
  // Active EP Request ZDO cluster 0x0005 is used to discover the end points of a ZigBee device.
  // The results of this request are returned in ZDO Cluster 0x8005, a packet that list the end point count and each end point number.
  //***************************************
  
  byte EndPointList[ClusterListSize];
    
  int endPointCount = get_EndPointList(EndPointList);
  
  Serial.println(F("ZDOpkt Packet Received - Reporting Active End Points cluster 0x0005. Responding with 0x8005"));
  memset(Buffer, 0 , BufferSize);
  Buffer[0] = zb._PktData()[0];                                               // Set Transaction Seq number to match inbound packets seq number
  Buffer[1] = 0x00;                                                           // Status 0x00 = success Table 2.94 on page 161 of ZBSpec
  Buffer[2] = zb._PktData()[1];                                               // Set Network address little endian order
  Buffer[3] = zb._PktData()[2];
  //Buffer[4] = 0x01;                                                           // Active EndPoint count only one in this case page 161 of ZBSpec
  //Buffer[5] = 0x01;                                                           // EndPoint number
  Buffer[4] = get_EndPointList(EndPointList);                                 // Active EndPoint count only one in this case page 161 of ZBSpec
  memcpy(&Buffer[5], EndPointList, endPointCount);                                       // Copy byte string array into buffer
  zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), 0, 0, 0, 0x8005, Buffer, 5 +  + endPointCount);
}



// --------------------------------------
// Zigbee Cluster Library commands
// --------------------------------------
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


void clstr_DeviceTemperature(byte endPoint, byte frmType, byte seqNum, byte cmdID, word attributeID) __attribute__((weak));
void clstr_DeviceTemperature(byte endPoint, byte frmType, byte seqNum, byte cmdID, word attributeID)
{
  Serial.println(FuncNotImplemented); 
}



bool get_OnOff(byte endPoint) __attribute__((weak));
bool get_OnOff(byte endPoint)
{
  Serial.println(FuncNotImplemented);  
}

void set_OnOff(byte endPoint, bool On) __attribute__((weak));
void set_OnOff(byte endPoint, bool On)
{
  Serial.println(FuncNotImplemented);  
}

void toggle_OnOff(byte endPoint) __attribute__((weak));
void toggle_OnOff(byte endPoint)
{
  Serial.println(FuncNotImplemented);  
}

void clstr_OnOff(byte endPoint, byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0006 On/Off
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
    Send10Response(get_OnOff(endPoint), 0x0000, seqNum);
    return;
  }

  if (frmType == 0x01 && cmdID == 0x00 && attributeID == 0x00)                // Set device Off -- P126 Section 3.8.2.3 of ZCL
  {
    set_OnOff(endPoint, false);
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send default response back to originator of command
    return;
  }

  if (frmType == 0x01 && cmdID == 0x01 && attributeID == 0x00)                // Set device On
  {
    set_OnOff(endPoint, true);
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send default response back to originator of command
    return;
  }

  if (frmType == 0x01 && cmdID == 0x02 && attributeID == 0x00)                // Toggle device
  {
     toggle_OnOff(endPoint);
    sendDefaultResponse(cmdID, 0x00, 0x01);                                   // Send Default response back to originator of command
    return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}



float get_Temperature(byte endPoint) __attribute__((weak));
float get_Temperature(byte endPoint)
{
  Serial.println(FuncNotImplemented);  
}

void clstr_Temperature(byte endPoint, byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0402 Temp
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
    Send29Response(get_Temperature(endPoint) * 100, 0x0000, seqNum);
    return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}



float get_Pressure(byte endPoint) __attribute__((weak));
float get_Pressure(byte endPoint)
{
  Serial.println(FuncNotImplemented); 
}

void clstr_Pressure(byte endPoint, byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0402 Temp
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
    Send29Response(get_Pressure(endPoint), 0x0000, seqNum);
    return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}



float get_Humidity(byte endPoint) __attribute__((weak));
float get_Humidity(byte endPoint)
{
  Serial.println(FuncNotImplemented); 
}

void clstr_Humidity(byte endPoint, byte frmType, byte seqNum, byte cmdID, word attributeID)                                                            // Cluster 0x0402 Temp
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
    Send21Response(get_Humidity(endPoint) * 100, 0x0000, seqNum);
    return;
  }
  Serial.println(F("Invalid type/command/attribute!"));
}


void ZCLpkt()
{
  //***************************************
  // ZigBee Cluster Library (ZCL) cluster numbers supported
  // ZCL are defined in the ZigBee Cluster Library Document 075123r04ZB
  // http://www.zigbee.org/wp-content/uploads/2014/11/docs-07-5123-04-zigbee-cluster-library-specification.pdf
  //***************************************
  byte frmType, seqNum, cmdID;
  word attributeID;
  frmType = byte(zb._PktData()[0]);                                           // Frame Type is bit 0 and 1 of Byte 0 P14 of ZCL
  frmType = frmType & 3;                                                      // Bitwise AND (&) with a mask to make sure we are looking at first two bits
  seqNum = byte(zb._PktData()[1]);                                            // Transaction seq number can be any value used in return packet to match a response to a request
  cmdID = byte(zb._PktData()[2]);                                             // Command ID Byte P16 of ZCL
  attributeID = byte(zb._PktData()[4]);                                       // Attribute ID Word(little endian) P126 of ZCL
  attributeID = (attributeID << 8) + byte(zb._PktData()[3]);       
  byte endPoint = zb._PktDEP();
  
  switch (int(zb._PktCluster()))
  {
    case 0x0000:
      clstr_Basic(frmType, seqNum, cmdID, attributeID);                                                          // Basic Cluster Page 78 of ZigBee Cluster Library
      break;
 
    case 0x0001:
      clstr_PowerConfiguration(frmType, seqNum, cmdID, attributeID);                                                          // Basic Cluster Page 78 of ZigBee Cluster Library
      break;
      
    case 0x0002:
      clstr_DeviceTemperature(endPoint, frmType, seqNum, cmdID, attributeID);                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;
        
    case 0x0006:
      clstr_OnOff(endPoint, frmType, seqNum, cmdID, attributeID);                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;

    case 0x0402:
      clstr_Temperature(endPoint, frmType, seqNum, cmdID, attributeID);                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;

    case 0x0403:
      clstr_Pressure(endPoint, frmType, seqNum, cmdID, attributeID);                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;

    case 0x0405:
      clstr_Humidity(endPoint, frmType, seqNum, cmdID, attributeID);                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
      break;
    
    default: Serial.println(F("** ZCLpkt received but there is no valid handler! **"));
  }
}



// ------------------------
// ZigBee Cluster responses
// ------------------------
void Send10Response(bool Value, int attribute, byte seqNum)   
{
    if (BatteryPowered) WakexBee(false);
  
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = highByte(attribute);

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x10;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL

    Buffer[7] = Value;                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
 
    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8);
    Serial.print(F("Boolean response sent:"));
    if (Value == true) Serial.println(F("True")); else Serial.println(F("False"));
}

void Send20Response(byte Value, int attribute, byte seqNum)   
{
    if (BatteryPowered) WakexBee(false);
  
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = highByte(attribute);

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x20;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL

    Buffer[7] = Value;                                              // Single byte value

    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 9);
    Serial.print(F("uint8 response sent:"));
    Serial.println(Value);
}

void Send21Response(unsigned int Value, int attribute, byte seqNum)   
{
    if (BatteryPowered) WakexBee(false);
  
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = highByte(attribute);

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x21;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL

    Buffer[7] = lowByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Buffer[8] = highByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin

    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 9);
    Serial.print(F("uint16 response sent:"));
    Serial.println(Value);
}

void Send29Response(int Value, int attribute, byte seqNum)   
{
    if (BatteryPowered) WakexBee(WakexBee);
  
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = highByte(attribute);

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x29;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL

    Buffer[7] = lowByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
    Buffer[8] = highByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin

    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 9);
    Serial.print(F("int16 response sent:"));
    Serial.println(Value);
}

void Send30Response(int Value, int attribute, byte seqNum)   
{
    if (BatteryPowered) WakexBee(false);
  
    memset(Buffer, 0 , BufferSize);                                           // Read attributes
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = highByte(attribute);

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x30;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL

    Buffer[7] = lowByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin

    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8);
    Serial.print(F("enum8 response sent:"));
    Serial.println(Value);
}

void Send42Response(char * Value, int attribute, byte seqNum)   
{ 
    if (BatteryPowered) WakexBee(false);
  
    int StrLen = 0;
    
    StrLen = strlen(Value); //-1;

    memset(Buffer, 0 , BufferSize);                                           // Clear response buffer
    Buffer[0] = 0x18;                                                         // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
    Buffer[1] = seqNum;                                                       // Set the sequence number to match the seq number in requesting packet
    Buffer[2] = 0x01;                                                         // Command Identifer 0x01 = Read attribute response see Table 2.9 on page 16 of ZCL

    Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
    Buffer[4] = highByte(attribute);

    Buffer[5] = 0x00;                                                         // Status 00 = Success

    Buffer[6] = 0x42;                                                         // Attribute Data Type 0x42 = Charcter string, see Table 2.16 on page 56 of ZCL

    Buffer[7] = StrLen;                                                       // Character string size

    memcpy(&Buffer[8], Value, StrLen);                                       // Copy byte string array into buffer

    zb.TX(zb._PktIEEEAddHi(), zb._PktIEEEAddLo(), zb._PktNetAdd(), zb._PktDEP(), zb._PktSEP(), zb._PktProfile(), zb._PktCluster(), Buffer, 8 + StrLen);
    Serial.print(F("string response sent:"));
    Serial.println(Value);
}

void sendDefaultResponse(byte CmdID, byte Status, byte EndPoint)
{
  //***************************************
  // Send Default Response Page 39 of ZigBee Cluster Library
  // CmdID = Byte size number representing the received command to which this command is a response see Table 2.9 on page 16 of ZigBee Cluster Library
  // Status = Byte size value specifies either Success (0x00) or the nature of the error see Table 2.17 on page 67 of ZigBee Cluster Library
  // EndPoint = Byte size value of the EndPoint this response is for
  //***************************************
  if (BatteryPowered) WakexBee(false);
  
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



// ------------------------
// ZigBee Cluster reports
// ------------------------
void SendOnOffReport(byte endPoint, boolean Value)
{
  Send10Report(endPoint, Value, 0x0006, 0x0000);
}

void SendTemperatureReport(byte endPoint, float Value)
{
  if (Value == NAN) return;
  
  int Temperature = (int) (Value * 100.0);
  
  if ((abs(Temperature - old_Temperature) >= 5) || (TempSensor_Report >= Sensor_ReportFreq))
  {  
    Send29Report(endPoint, Temperature, 0x0402, 0x0000);
    old_Temperature = Temperature;
    TempSensor_Report = 0;
  }
  
  TempSensor_Report++;
}

void SendPressureReport(byte endPoint, float Value)
{
  if (Value == NAN) return;
  
  int Pressure = (int) (Value * 100.0);
     
  if ((abs(Pressure - old_Pressure) >= 5) || (PressureSensor_Report >= Sensor_ReportFreq))
  {
    Send29Report(endPoint, Pressure, 0x0403, 0x0000);
    old_Pressure = Pressure;
    PressureSensor_Report = 0;
  }
  
  PressureSensor_Report++;
}

void SendHumidityReport(byte endPoint, float Value)
{
  if (Value == NAN) return;
  
  unsigned int Humidity = (int) (Value * 100.0);
  
  if ((abs(Humidity - old_Humidity) >= 5) || (HumiditySensor_Report >= Sensor_ReportFreq))
  {
    Send21Report(endPoint, Humidity, 0x0405, 0x0000);
    old_Humidity = Humidity;
    HumiditySensor_Report = 0;
  }
  
  HumiditySensor_Report++;
}



// ----------------------  
// ZigBee Cluster reports
// ----------------------
void Send10Report(byte endPoint, int Value, int cluster, int attribute)  
{
  //***************************************
  // Reports boolean value (1 or 0)
  //
  // Destinations are held in the xBee's Binding table
  //***************************************
  if (BatteryPowered) WakexBee(false);
        
  memset(Buffer, 0, BufferSize);
  Buffer[0] = 0x18;                                                           // Frame Control 0x10 = direction is from server to client, disable default response P14 of ZCL
  Buffer[1] = 0x11;                                                           // Set the sequence number
  Buffer[2] = 0x0A;                                                           // Command Identifer 0x0A = Report attributes see Table 2.9 on page 16 of ZCL

  Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
  Buffer[4] = highByte(attribute);

  Buffer[5] = 0x10;                                                           // Attribute Data Type 0x10 = Boolean enumeration, see table 2.16 on page 54 of ZCL

  Buffer[6] = Value;                                                          // Attribute Value (0 = off, 1 = on)

  zb.TX_Indirect(endPoint, 0x0104, cluster, Buffer, 7);                            // TX_Indirect(sEP, Prfl, Clstr, BuffAdd, BuffSize)
  Serial.print(F("Boolean report sent:"));
  if (Value == true) Serial.println(F("True")); else Serial.println(F("False"));
}

void Send21Report(byte endPoint, unsigned int Value, int cluster, int attribute)   
{ 
  if (BatteryPowered) WakexBee(false);
  
  memset(Buffer, 0 , BufferSize);                                           // Read attributes
  Buffer[0] = 0x18;                                                           // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
  Buffer[1] = 0x11;                                                           // Set the sequence number
  Buffer[2] = 0x0A;                                                           // Command Identifer 0x0A = Report attributes see Table 2.9 on page 16 of ZCL

  Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
  Buffer[4] = highByte(attribute);

  Buffer[5] = 0x21;                                                         // Attribute Data Type 0x10 = Boolean, see Table 2.16 on page 54 of ZCL

  Buffer[6] = lowByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
  Buffer[7] = highByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin

  zb.TX_Indirect(endPoint, 0x0104, cluster, Buffer, 8);                            // TX_Indirect(sEP, Prfl, Clstr, BuffAdd, BuffSize)
  Serial.print(F("uint16 report sent:"));
  Serial.println(Value);
}

void Send29Report(byte endPoint, int Value, int cluster, int attribute)   
{
  if (BatteryPowered) WakexBee(false);

  memset(Buffer, 0, BufferSize);
  Buffer[0] = 0x18;                                                           // Frame Control 0x18 = direction is from server to client, disable default response P14 of ZCL
  Buffer[1] = 0x11;                                                           // Set the sequence number
  Buffer[2] = 0x0A;                                                           // Command Identifer 0x0A = Report attributes see Table 2.9 on page 16 of ZCL

  Buffer[3] = lowByte(attribute);                                           // Attribute Identifier (2 bytes) field being reported see figure 2.24 on page 36 of ZCL
  Buffer[4] = highByte(attribute);

  Buffer[5] = 0x29;                                                         // Attribute Data Type 0x10 = Boolean enumeration, see table 2.16 on page 54 of ZCL

  Buffer[6] = lowByte(Value);                                             // Attribute Data Field in this case 0x00 = Off, see Table 3.40 on page 126 of ZCL. Set the on / off status based on pin
  Buffer[7] = highByte(Value);                                                         // Attribute Value (0 = off, 1 = on)

  zb.TX_Indirect(endPoint, 0x0104, cluster, Buffer, 8);                            // TX_Indirect(sEP, Prfl, Clstr, BuffAdd, BuffSize)
  Serial.print(F("int16 report sent:"));
  Serial.println(Value);
}



void setup_ZigBee(Stream& port, byte _endpointClusterCount)
{
  //***************************************
  // Setup xBee
  //***************************************  
  ConfigurexBee(port);
  JoinNetwork(); 
  SetupAddresses();
  //ResetNetwork();
  Serial.println(F("XBee setup."));
  
  endpointClusterCount = _endpointClusterCount;
  
  // Display number of endpoints
  char EndPointList[ClusterListSize];
  get_EndPointList(EndPointList); 
  
  // Dispaly number of clusters
  Serial.print(F("Total number of clusters:")); 
  Serial.println(endpointClusterCount);
  
  CheckInboundPackets(false);
  
  PollSensors();
}

void loop_ZigBee()
{
  int rxResult = 0;
  bool Sensor_Check = false; 

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

  if (Sensor_Check == true)
    rxResult = PollSensors();
}

int PollSensors()
{
  int rxResult = 0;
   
  for (int i = 0; i < endpointClusterCount; i++)
  {
    switch (endpointClusters[i].cluster)
    {
      case 0x0001:
        //clstr_PowerConfiguration(frmType, seqNum, cmdID, attributeID);                                                          // Basic Cluster Page 78 of ZigBee Cluster Library
        break;
      
      case 0x0002:
        //clstr_DeviceTemperature(endPoint, frmType, seqNum, cmdID, attributeID);                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
        break;
        
      case 0x0006:
        //clstr_OnOff(endPoint, frmType, seqNum, cmdID, attributeID);                                                          // On/Off Cluster Page 125 of ZigBee Cluster Library
        break;

      case 0x0402:
      {
        float t = get_Temperature(endpointClusters[i].endpoint);
        if (!isnan(t))
        {
          SendTemperatureReport(endpointClusters[i].endpoint, t);
          rxResult = CheckInboundPackets(true);
        }
        else Sensor_RequireRetry = true; 
      }
      break;

      case 0x0403:
      {
        float p = get_Pressure(endpointClusters[i].endpoint);
        if (!isnan(p))
        {
          SendPressureReport(endpointClusters[i].endpoint, p);
          rxResult = CheckInboundPackets(true);
        }
        else Sensor_RequireRetry = true;
      }
      break;

      case 0x0405:
      {
        float h = get_Humidity(endpointClusters[i].endpoint);
        if (!isnan(h))
        {
          SendHumidityReport(endpointClusters[i].endpoint, h);
          rxResult = CheckInboundPackets(true);
        }
        else Sensor_RequireRetry = true;
      }
      break;
    }
  }
  return rxResult;
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
