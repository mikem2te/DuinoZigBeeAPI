#ifndef ZigBeeAPI_h
#define ZigBeeAPI_h

#include <Arduino.h>
#include <SoftwareSerial.h>

const byte xBuffSize = 128;
const byte strBuffSize = 128;

class ZigBeeAPI : public Stream
{
public:
	ZigBeeAPI(Stream& port);
	void begin(Stream &serial);
  void setSerial(Stream &serial);
  int available();
  int read();
  size_t write(uint8_t val);
  void flush();
  int peek();
	byte* _PktData();
	boolean AT(const char *CmdStrPtr);
	boolean ATbyte(const char *CmdStrPtr, byte ValueByte);
	boolean ATqueue(const char *CmdStrPtr, const uint8_t ByteArryPtr[], word ArrySize);
	//void begin (byte xBeeRx, byte xBeeTx, uint16_t xBeeBaud);
	void RxFlush();
	boolean TX(long DAddHi, long DAddLo, word NetAdd, byte sEP, byte dEP, word Prfl, word Clstr, const uint8_t *BuffAdd, word BuffSize);
	boolean TX_Indirect(byte sEP, word Prfl, word Clstr, const char BuffAdd[], word BuffSize);
	int RX(int TimeMS);
	int _PktDataSize();
  long _PktIEEEAddHi();
  long _PktIEEEAddLo();
  long _PktNetAdd();
  byte _PktDEP();
  byte _PktSEP();
  word _PktProfile();
  word _PktCluster();
  byte* _ReadLog();
  
private:
	Stream *_port;
};

#endif