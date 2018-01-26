#ifndef ZigBee_h
#define ZigBee_h

#include <Arduino.h>
#include <ZigBeeAPI.h>
#include <SoftwareSerial.h>

#define cluster_Basic 0x0000
#define cluster_PowerConfiguration 0x0001
#define cluster_OnOff 0x0006
#define cluster_Temperature 0x0402
#define cluster_Pressure 0x0403
#define cluster_RelativeHumidity 0x0405


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
int get_EndPointList(byte *list);
int get_ClustersForEndPoint(byte endPoint, unsigned int *list);

// Zigbee Cluster Library commands
void ZCLpkt();


void Send10Response(bool Value, int attribute, byte seqNum);  
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

int PollSensors();


#endif

