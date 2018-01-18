#ifndef ZigBee_h
#define ZigBee_h

#include <Arduino.h>
#include <ZigBeeAPI.h>
#include <SoftwareSerial.h>

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


// xBee, Network and Reset management
void ConfigurexBee(Stream& port);
void xBeeAwakeChange();
void sleepNow();
void ResetNetwork();
void LeaveNetwork();
void resetXB();
void Tx_Device_annce();
void JoinNetwork();
void SetupAddresses();
void WakexBee();
float Get_xBeeTemp();


// Inbound packet processing
int CheckInboundPackets();
void ProcessInboundPacket(int rxResult);


// ZigBee Device Objects commands 
void ZDOpkt();
void Simple_Desc_req();
void Active_EP_req();

// Zigbee Cluster Library commands
void ZCLpkt();


void Send10Response(bool Value, int attribute, byte seqNum);  
void Send20Response(byte Value, int attribute, byte seqNum);  
void Send21Response(unsigned int Value, int attribute, byte seqNum);   
void Send29Response(int Value, int attribute, byte seqNum);  
void Send30Response(int Value, int attribute, byte seqNum); 
void Send42Response(char * Value, int attribute, byte seqNum); 
void sendDefaultResponse(byte CmdID, byte Status, byte EndPoint);

void SendOnOffReport(boolean Value);
void SendTemperatureReport(float Value);
void SendPressureReport(float Value);
void SendHumidityReport(float Value);

void Send10Report(int Value, int cluster, int attribute);
void Send21Report(unsigned int Value, int cluster, int attribute);  
void Send29Report(int Value, int cluster, int attribute);

#endif

