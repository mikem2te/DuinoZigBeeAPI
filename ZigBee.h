#ifndef ZigBee_h
#define ZigBee_h

#include <Arduino.h>
#include <ZigBeeAPI.h>
#include <SoftwareSerial.h>

#define cluster_Basic 0x0000
#define cluster_PowerConfiguration 0x0001
#define cluster_DeviceTemperature 0x0002
#define cluster_Identity 0x0003
#define cluster_Groups 0x0004
#define cluster_Scenes 0x0005
#define cluster_OnOff 0x0006
#define cluster_OnOffConfiguration 0x0007
#define cluster_LevelControl 0x0008
#define cluster_Alarms 0x0009
#define cluster_Time 0x000a
#define cluster_RSSI 0x000b
#define cluster_IOValue 0x000c
#define cluster_PowerProfile 0x001a
#define cluster_PollControl 0x0020

#define cluster_ShadeConfiguration 0x0100
#define cluster_DoorLock 0x0101
#define cluster_WindowCovering 0x0102

#define cluster_ColorControl 0x0300
#define cluster_BallastConfiguration 0x0301

#define cluster_IlluminanceMeasurement 0x0400
#define cluster_IlluminanceLevelSensing 0x0401
#define cluster_Temperature 0x0402
#define cluster_Pressure 0x0403
#define cluster_FlowMeasurement 0x0404
#define cluster_RelativeHumidity 0x0405
#define cluster_OccupancySensing 0x0406

#define cluster_IASZone 0x0500
#define cluster_IASACE 0x0501
#define cluster_IASWD 0x0502

#define cluster_ElectricalMeasurement 0x0b04
#define cluster_Diagnostics 0x0b05


extern uint16_t Light_Update_Interval;

extern volatile byte         clstr_LevelControl_Command;
extern volatile float        clstr_LevelControl_CurrentLevel;
extern volatile byte         clstr_LevelControl_Level;
extern volatile unsigned int clstr_LevelControl_RemainingTime;
extern volatile float        clstr_LevelControl_Gradient;

extern volatile byte     clstr_ColorControl_ColourMode;
extern volatile uint16_t clstr_ColorControl_RemainingTime;

extern volatile float    clstr_ColorControl_A_Current;
extern volatile uint16_t clstr_ColorControl_A;
extern volatile float    clstr_ColorControl_A_Gradient;

extern volatile float    clstr_ColorControl_B_Current;
extern volatile uint16_t clstr_ColorControl_B;
extern volatile float    clstr_ColorControl_B_Gradient;

//extern float        clstr_ColorControl_B_Current;


struct EndpointCluster {
   byte endpoint;
   unsigned int cluster;
};

extern const char Model[] PROGMEM; 
extern const char Manufacturer[] PROGMEM;
extern const char SWVersion[] PROGMEM;
 
const byte ClusterListSize = 40;
const byte BufferSize = 75;

extern byte Buffer[BufferSize];
extern word LclNet;
extern unsigned long LclIeeeLo, LclIeeeHi;
extern unsigned long now;

// Utility functions
bool pinState(byte pin);
void printByteData(uint8_t Byte);
//void PrintHex(int data, int length);
void formatDate(char const *date, char const *tm, char *buff);


// XBee, Network and Reset management
void setup_ZigBee(Stream& port, byte _endpointClusterCount, bool _BatteryPowered);
void loop_ZigBee();

void ConfigureXBee(Stream& port);
void XBeeAwakeChange();
void sleepNow();
void ResetNetwork();
void LeaveNetwork();
void resetXB();
void Tx_Device_annce();
byte GetJoinStatus();
void JoinNetwork();
void SetupAddresses();
void WakeXBee(bool force);
float Get_XBeeTemp();


// Inbound packet processing
int CheckInboundPackets(bool extendedTimeout);
void ProcessInboundPacket(int rxResult);


// ZigBee Device Objects commands 
void ZDOpkt();
void Simple_Desc_req();
void Active_EP_req();
int get_EndPointList(char *list);
int get_ClustersForEndPoint(byte endPoint, unsigned int *list);

// Zigbee Cluster Library commands
void ZCLpkt();


void clstr_DeviceTemperature(byte endPoint, byte frmType, byte seqNum, byte cmdID, uint16_t attributeID) __attribute__((weak));

bool get_OnOff(byte endPoint) __attribute__((weak));
void set_OnOff(byte endPoint, bool On) __attribute__((weak));
bool toggle_OnOff(byte endPoint) __attribute__((weak));

void clstr_ColorControl(byte endPoint, byte frmType, byte seqNum, byte cmdID, word attributeID);

float get_Temperature(byte endPoint) __attribute__((weak));
float get_Pressure(byte endPoint) __attribute__((weak));
float get_Humidity(byte endPoint) __attribute__((weak));


void Send10Response(bool Value, int attribute, byte seqNum); 
void Send19Response(unsigned int Value, int attribute, byte seqNum); 
void Send20Response(byte Value, int attribute, byte seqNum);  
void Send21Response(unsigned int Value, int attribute, byte seqNum);   
void Send29Response(int Value, int attribute, byte seqNum);  
void Send30Response(int Value, int attribute, byte seqNum); 
void Send42Response(char * Value, int attribute, byte seqNum); 
void sendDefaultResponse(byte CmdID, byte Status, byte EndPoint);

void SendOnOffReport(byte endPoint, boolean Value);
void SendTemperatureReport(byte endPoint, float Value);
void SendPressureReport(byte endPoint, float Value);
void SendHumidityReport(byte endPoint, float Value);

void Send10Report(byte endPoint, int Value, int cluster, int attribute);
void Send21Report(byte endPoint, unsigned int Value, int cluster, int attribute);  
void Send29Report(byte endPoint, int Value, int cluster, int attribute);

void clstr_ColorControlSetColour(byte endPoint, byte ColourMode, float hue, float saturation, float level) __attribute__((weak));

int PollSensors();


#endif

