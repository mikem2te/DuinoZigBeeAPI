//***********************************************
// ZigBeeAPI - ZigBee Arduino library for Digi xBee ZB            
// Author: Professor Mayhem  
// Yoyodyne Monkey Works, Inc.
//
//
// 2/15/2017
//
// Revision 1.0
//
// ZigBeeAPI is an Arduino port from a version
// of the same name written in the Propeller Spin language
// by John.Rucker(at)Solar-Current.com
//***********************************************
#include <ZigBeeAPI.h>
#define _DEBUG 0

ZigBeeAPI::ZigBeeAPI() {};
ZigBeeAPI::ZigBeeAPI(Stream& port) : _port(&port){};
void ZigBeeAPI::begin(Stream &serial) {_port = &serial;}
void ZigBeeAPI::setSerial(Stream &serial) {	_port = &serial;}
int ZigBeeAPI::available() {return _port->available();}
int ZigBeeAPI::read() {return _port->read();}
size_t ZigBeeAPI::write(uint8_t val) {_port->write(val);}
void ZigBeeAPI::flush() {_port->flush();}
int ZigBeeAPI::peek() {return _port->peek();}
	
//xBee Configuration

//Info for last recived packet:
long S_IAddHi = 0;             //IEEE Source address High
long S_IAddLo = 0;             //IEEE Source address Lo
word S_NAdd = 0;               //Network Address of inbound packet
byte S_SEP = 0;                //Source endpoint
byte S_DEP = 0;                //Destination endpoint
word S_Profile = 0;            //Profile ID
word S_Cluster = 0;            //Cluster ID
int x = 0;

//Async config
byte xBuff[xBuffSize];
char strBuff[strBuffSize];
word xBuffPayloadSize = 0;

#if _DEBUG
void PrintHex(uint8_t *data, uint8_t length)            // prints 8-bit data in hex
{
  for (int i=0; i<=length; i++)
  {
    Serial.print((uint8_t)data[i] >> 4, HEX);
    Serial.print((uint8_t)data[i] & 0x0f, HEX);
    if (i <length) Serial.print(" ");
  }
  return;
}
#endif

boolean CheckChkSum(int xBuffStart, int xBuffEnd)
{
  //***************************************
  // Check for valid checksum
  // return true if OK
  //***************************************
  int TmpChkSum=0;
  for (int x=xBuffStart; x <= xBuffEnd; x++)
  {
    TmpChkSum = TmpChkSum + xBuff[min(x,xBuffSize)];
  }
  if (0xFF == lowByte(TmpChkSum))
    return true;
  else
    return false;
}

word GetChkSum(int xBuffStart, int xBuffEnd)
{
  //***************************************
  // Calculate checksum and return it
  //***************************************
  word TmpChkSum = 0;
  for (int x=xBuffStart; x <= xBuffEnd; x++)
  {
    TmpChkSum = TmpChkSum + xBuff[x];
  }
  return 0xFFFF - lowByte(TmpChkSum);
}

void LOG(const char LogTxtAdd[])
{
  //***************************************
  // Copy LogTxt to the strBuff buffer so it can be passed
  // when user calls _ReadLog
  //***************************************
  memset(strBuff,0,strBuffSize);
  memcpy(&strBuff[0], LogTxtAdd, min(strBuffSize, strlen(LogTxtAdd)+1));
  #if _DEBUG
	  Serial.println();
	  Serial.print(F("*LOG*:  "));
    for (int i=0; i <= min(strBuffSize, strlen(LogTxtAdd));i++)
    {
      Serial.print(char(strBuff[i]));
    }
    Serial.println();
  #endif
  return;
}
void ZigBeeAPI::RxFlush()
{
  //***************************************
  // Flush receive buffer
  //***************************************
  while(available()) read();
  flush();
}

boolean ZigBeeAPI::TX(long DAddHi, long DAddLo, word NetAdd, byte sEP, byte dEP, word Prfl, word Clstr, const uint8_t BuffAdd[], word BuffSize)
{
  //***************************************
  // Send StrPtr to DAddHi DAddLo and net address.
  // DAddHi(Long) = high 32 bits of 64 bit IEEE destination
  // DaddLo(Long) = low 32 bits of 64 bit IEEE destination
  //   set DAddHi = 0x0000_0000 and DaddLo = 0x0000_0000 to send packet to coordinator
  //   set DAddHi = 0x0000_0000 and DaddLo = 0x0000_FFFF to broadcast
  // NetAdd(Word) = 16 bit network address of destination.
  //   set NetAdd to 0xFFFE = if destination network address unknown
  //   set NetAdd to 0xFFFC = broadcast to all routers
  //   set NetAdd to 0xFFFD = broadcast to all non-sleeping devices
  //   set NetAdd to 0xFFFF = broadcast to all devices
  // sEP(Byte) = Source End Point
  // dEP(Byte) = Destination End Point
  // Prfl(Word) = Profile ID
  // Clstr(Word) = ZigBee Cluster number
  // BuffAdd(memory pointer) = memory pointer of byte aligned payload data to be sent
  // BuffSize(Word) = size of payload data. Must be equal to or less than xBuffSize - 20 bytes to or it will be truncated.
  // returns true = Success
  //***************************************
  word Size = 0;
  memset(xBuff, 0, xBuffSize);
  Size = 20 + min(BuffSize,(xBuffSize-20));             // Packet size is all data but chksum, start delimiter, and Length.
  //Create API Packet
  xBuff[0] = 0x7E;                                      // 7E = Start Delimiter P106 of XBee/XBee-PRO ZB RF Modules. Doc# 90000976_P
  xBuff[1] = highByte(Size);                            // Set Length of packet
  xBuff[2] = lowByte(Size);
  xBuff[3] = 0x11;                                      // APID 11 = Explicit Addressing
  xBuff[4] = 0x00;                                      // Frame ID 0 = no response
  xBuff[5] = highByte(DAddHi >> 16);                    // Set 64 bit IEEE Address
  xBuff[6] = lowByte(DAddHi >> 16);
  xBuff[7] = highByte(DAddHi);
  xBuff[8] = lowByte(DAddHi);
  xBuff[9] = highByte(DAddLo >> 16);
  xBuff[10] = lowByte(DAddLo >> 16);
  xBuff[11] = highByte(DAddLo);
  xBuff[12] = lowByte(DAddLo);

  xBuff[13] = highByte(NetAdd);                         // Set 16 bit Network Address
  xBuff[14] = lowByte(NetAdd);

  xBuff[15] = sEP;                                      // Set Source Endpoint

  xBuff[16] = dEP;                                      // Set Destination Endpoint

  xBuff[17] = highByte(Clstr);                          // Cluster ID
  xBuff[18] = lowByte(Clstr);

  xBuff[19] = highByte(Prfl);                           // Profile ID
  xBuff[20] = lowByte(Prfl);

  xBuff[21] = 0x00;                                     // Broadcast Radius. Maximum number of hops = 00 = use network default

  xBuff[22] = 0x00;                                     // Transmit Options Bitfield

  memcpy(&xBuff[23], BuffAdd, min(BuffSize,100));       // Copy Payload buffer into packet
  xBuff[min((23 + BuffSize),xBuffSize)] = GetChkSum(3, Size + 3);   // Find the Chksum and place it in last byte of API Packet
  
  #if _DEBUG
    Serial.print("<");
    PrintHex(xBuff,20 + min(BuffSize,(xBuffSize-20))+3);
    Serial.println();
  #endif
  
  //Send API Packet
  for (int x=0; x <= (Size + 3); x++)                   // Transmit packet byte at a time
    {
    write(xBuff[min(x,xBuffSize)]);
    }
  return true;                                          // Check for Ack and return result
}
boolean ZigBeeAPI::TX_Indirect(byte sEP, word Prfl, word Clstr, const char BuffAdd[], word BuffSize)
{
  //***************************************
  // TX_Indrect is used to send to nodes that are in the Bind Table of the xBee. Only supported on the SMT version of the xBee Pro
  // sEP(Byte) = Source End Point
  // Prfl(Word) = Profile ID
  // Clstr(Word) = ZigBee Clstr number
  // BuffAdd(memory pointer) = memory pointer of byte aligned payload data to be sent
  // BuffSize(Word) = size of payload data. Must be equal to or less than #xBuffSize - 20 bytes to or it will be truncated.
  // returns true=Success
  //***************************************
  word Size = 0;
  memset(xBuff, 0, xBuffSize);
  Size = 20 + (min(BuffSize,(xBuffSize-20)));           // Packet size is all data but chksum, start delimiter, and Length.
  //Create API Packet
  xBuff[0] = 0x7E;                                      // 7E = Start Delimiter P106 of XBee/XBee-PRO ZB RF Modules. Doc# 90000976_P
  xBuff[1] = highByte(Size);                            // Set Length of packet
  xBuff[2] = lowByte(Size);
  xBuff[3] = 0x11;                                      // APID 11 = Explicit Addressing
  xBuff[4] = 0x01;                                      // Frame ID 0 = no response
  xBuff[5] = 0xFF;                                      // Set 64 bit IEEE Address to all 0xFFFFFFFF since this is an indrect message the real address will be looked
  xBuff[6] = 0xFF;                                      // up in the xBee's binding table and put in the place of the 0xFFFFF
  xBuff[7] = 0xFF;
  xBuff[8] = 0xFF;
  xBuff[9] = 0xFF;
  xBuff[10] = 0xFF;
  xBuff[11] = 0xFF;
  xBuff[12] = 0xFF;

  xBuff[13] = 0xFF;                                     // Set 16 bit Network Address is set to 0xFFFE (unknown)
  xBuff[14] = 0xFE;

  xBuff[15] = sEP;                                      // Set Source Endpoint

  xBuff[16] = 0xFF;                                     // Destination end point will be looked up in the bind table

  xBuff[17] = highByte(Clstr);                          // Cluster ID
  xBuff[18] = lowByte(Clstr);

  xBuff[19] = highByte(Prfl);                           // Profile ID
  xBuff[20] = lowByte(Prfl);

  xBuff[21] = 0x00;                                     // Broadcast Radius. Maximum number of hops = 00 = use network default

  xBuff[22] = 0x04;                                     // Transmit Options Bitfield 0x04 = indirect addressing

  memcpy(&xBuff[23], BuffAdd, min(BuffSize,100));       // Copy Payload buffer into packet

  xBuff[min((23 + BuffSize),xBuffSize)] = GetChkSum(3, Size + 3);   // Find the Chksum and place it in last byte of API Packet
  
  #if _DEBUG
    Serial.print("<");
    PrintHex(xBuff,20 + min(BuffSize,(xBuffSize-20))+3);
    Serial.println();
  #endif
  
  //Send API Packet
  for (int x=0; x <= (Size + 3); x++)                   // Transmit packet byte at a time
  {
    write(xBuff[min(x,xBuffSize)]);
  }
  return true;
}

int ZigBeeAPI::RX(int TimeMS)
{
 //***************************************
 // Waits for packet and copies data to xBuff.
 // TimeMS = number of milliseconds for first byte to be received. If TimeMS = 0 this method will wait forever for data.
 // returns true=if valid packet (good checksum)
 // If RX returns true, call: _PktData to access a string pointer to packet data
 //                           _PktDataSize to get the size of the packet data buffer
 //                           _PktIEEEAddHi to see upper address of xBee that sent the packet
 //                           _PktIEEEAddLo to see lower address of xBee that sent the packet
 //                           _PktNetAdd to see the network assigned address of the xbee that sent the packet
 //                           _PktProfile, _PktCluster, _PktDEP, _PktSEP to access the zigbee profile, cluster and end nodes of the data packet
 // If RX not true it will =
 //   -1 = Unknown Digi API Frame Type (Only 0x91 and 0x88 are supported)
 //   -2 = Timeout waiting for data
 //   -3 = RX Packet ID not known.
 //   -4 = Bad checksum
 //***************************************
  int PktSize=0;
  byte TmpReadBuff[3];
  unsigned long start=0;
  if (TimeMS == 0)
  {
    while(available()<1) {}                             // Read with no time limit
    TmpReadBuff[0] = read();
  }
  else
  {
    start = millis();
    while (int((millis() - start)) < TimeMS && available()<1){}    // Read byte and wait TimeMS
    x = read();
    if (x == -1)                                        // -1 = error no data recieved within TimeMS
    {
      return -2;                                        // Return error timeout waiting for data
    }
    else
    {
      TmpReadBuff[0] = x;                               // Read was good. Set 1st byte of tmpReadBuff and continue
    }
  }
  for (int i=1; i <= 2; i++)
  {
    while(available() < 1){}
    TmpReadBuff[i] = read();                            // Read next 2 bytes into temp long
  }

  if (TmpReadBuff[0] == 0x7E)                           // Check for Start Delimiter
  {
    memset(xBuff, 0, xBuffSize);                        // Clear buffer
    xBuff[0]=TmpReadBuff[0];                            // Fill first 3 bytes with data from TmpReadBuff
    xBuff[1]=TmpReadBuff[1];
    xBuff[2]=TmpReadBuff[2];
    PktSize = xBuff[2];                                 // Set PktSize to the size of the packet
  }
  else
  {
    LOG("Err RX Pkt ID");
    x = 3;
    while(available() > 0 && x < xBuffSize )
    {
      xBuff[x] = read();
      x++;
    }
    #if _DEBUG
      Serial.print(">");
      PrintHex(TmpReadBuff,2);
      Serial.println(" ");
      PrintHex(xBuff[3],x);
      Serial.println();
    #endif
    return -3;                                          // Return Error 3, Error RX Pkt ID
  }
  
  #if _DEBUG
    Serial.print(">");
    PrintHex(TmpReadBuff,2);
  #endif
  for (int i=3; i <= min((PktSize + 3), xBuffSize); i++)// Read all packet data & checksum into xBuff
  {
  	if (TimeMS == 0)
    {
    	while(available() < 1){}                          // Read with no time limits
    }
    else
	  {
		  start = millis();
		  while (int((millis() - start)) < TimeMS && available()<1){}    // Wait for TimeMS 
		}
		x = read();
	  if (x == -1) return -2;                             // Return error timeout waiting for data
    xBuff[min(i,xBuffSize)] = x;
    #if _DEBUG
      Serial.print(" ");
      Serial.print((uint8_t)xBuff[min(i,xBuffSize)] >> 4, HEX);
      Serial.print((uint8_t)xBuff[min(i,xBuffSize)] & 0x0f, HEX);
    #endif
  }
  #if _DEBUG
    Serial.println();
    Serial.println(F("Packet fully read."));
  #endif
  if (!CheckChkSum(3,PktSize + 3))
  {
    xBuffPayloadSize = PktSize + 3;
    LOG("Err RX ChkSum");
    return -4;                                          // Return Error 4 if checksum is bad
  }
  if (xBuff[3] == 0x8A)                                 // Modem Status Frame
  {
  	if (xBuff[4] == 0) LOG("Hardware reset");
  	if (xBuff[4] == 1) LOG("Watchdog timer reset");
  	if (xBuff[4] == 2) LOG("Joined network (routers and end devices)");
  	if (xBuff[4] == 3) LOG("Disassociated");
  	if (xBuff[4] == 6) LOG("Coordinator started");
  	if (xBuff[4] == 7) LOG("Network security key was updated");
  	if (xBuff[4] == 0x0D) LOG("Voltage supply limit exceeded");
  	if (xBuff[4] == 0x11) LOG("Modem configuration changed while join in progress");
  	if (xBuff[4] >= 0x80) LOG("Ember ZigBee stack error");
    return false;
  }
  else if (xBuff[3] == 0x91)                            // API Identifier: 0x91 = ZigBee Explicit Rx Indicator
  {
    S_IAddHi = xBuff[4];                                // Set SourceAddHi to upper 32 bit address
    S_IAddHi = (S_IAddHi << 8) + xBuff[5];
    S_IAddHi = (S_IAddHi << 8) + xBuff[6];
    S_IAddHi = (S_IAddHi << 8) + xBuff[7];

    S_IAddLo = xBuff[8];                                // Set SourceAddLo to lower 32 bit address
    S_IAddLo = (S_IAddLo << 8) + xBuff[9];
    S_IAddLo = (S_IAddLo << 8) + xBuff[10];
    S_IAddLo = (S_IAddLo << 8) + xBuff[11];

    S_NAdd = xBuff[13];                                 // Set Source Network Address
    S_NAdd = (S_NAdd << 8) + xBuff[12];

    S_SEP=xBuff[14];                                    // Set Source and Destination end points
    S_DEP=xBuff[15];

    S_Cluster = xBuff[16];                              // Set Cluster ID
    S_Cluster = (S_Cluster << 8) + xBuff[17];

    S_Profile = xBuff[18];                              // Set Profile ID
    S_Profile = (S_Profile << 8) + xBuff[19];

    PktSize = PktSize - 18;                             // Set PktSize = to size of remaining data
    xBuffPayloadSize = PktSize - 1;
    x = 21;                                             // Skip the Options Byte not used by our object
    memcpy(&xBuff[0], &xBuff[x], PktSize);              // Move packet data frame to begining of xBuff
    memset(xBuff+PktSize, 0, xBuffSize-PktSize);        // Fill remaining bytes with 0
    return true;
  }
  else if (xBuff[3] == 0x88)                            // API Identifier: 0x88 = AT Frame Type
  {
    if (xBuff[7] == 0x00 && PktSize > 0x05)             // Byte 7 is command status for AT command Response 00 = OK
    {
      x = PktSize - 5;                                  // Set x to the data size in bytes
    	memcpy(&xBuff[0], &xBuff[8], x);                  // Move packet data frame to beginning of xBuff
      memset(xBuff+x, 0, xBuffSize-x);                  // Fill remaining bytes with 0
      return true;
    }
    else if (xBuff[7] == 0x00)                          // ATbyte commands have no data to return
    {
      return true;
    }
    else
    {
      LOG("Err RX AT Cmd");
    }
  }
  else if (xBuff[3] == 0xA3)                            // "Many-to-One" route request
  {
  	LOG("\"Many-to-One\" route request");
  	xBuffPayloadSize = PktSize + 3;
	return false;
  }
  else if (xBuff[3] == 0x8B)                            // "Transmit Status frame" frame
  {
    //LOG("\"Transmit Status frame\" frame");
    xBuffPayloadSize = PktSize + 3;
    return -5;
  }
  else
  {
  	LOG("Err Unknown RX API ID");
  }
  xBuffPayloadSize = PktSize + 3;
  return -1;
}

boolean ZigBeeAPI::AT(const char CmdStrPtr[])
{
  //***************************************
  // Send two letter AT command to xBee
  // CmdStrPtr(memory pointer) is the address of the two character AT command
  // returns true or false
  // if return = true, read _PktData to get a pointer to a buffer with the result.
  //***************************************
  memset(xBuff, 0, xBuffSize);
  //Create API Packet
  xBuff[0] = 0x7E;                                      // 7E = Start Delimiter P106 of XBee/XBee-PRO ZB RF Modules. Doc# 90000976_P
  xBuff[1] = 0x00;                                      // Set Length of packet (MSB). Packet size is all data but chksum, start delimiter, and Length.
  xBuff[2] = 0x04;                                      // Set Length of packet (LSB).
  xBuff[3] = 0x08;                                      // APID 08 = AT Command
  xBuff[4] = 0x21;                                      // Frame ID 0 = no response
  xBuff[5] = CmdStrPtr[0];                              // First byte of AT command
  xBuff[6] = CmdStrPtr[1];                              // Second byte of AT command
  xBuff[7] = GetChkSum(3, 7);                           // Find the Chksum and place it in last byte of API Packet

  //Send API Packet
  RxFlush();
  #if _DEBUG
    Serial.println();
    Serial.print(F("AT: "));
    Serial.print(char(CmdStrPtr[0]));
  	Serial.println(char(CmdStrPtr[1]));
    Serial.print("<");
    PrintHex(xBuff,7);
    Serial.println();
  #endif
  for (int i=0; i <= 7; i++)                            // Transmit packet byte at a time
  {
    write(xBuff[i]);
  }

  x = RX(1000);																				  // Check for AT response and return result
  while (x < 1 && (xBuff[3] == 0x8A))                   // Check for AT response again if cause of failure is Modem Status frames
  {
  	xBuff[3]=0;
  	x=RX(1000);
  }
  if (x == 1) return true; else return false;
}

boolean ZigBeeAPI::ATbyte(const char *CmdStrPtr, byte ValueByte)
{
  //***************************************
  // Send two letter AT command plus a one byte value to xBee
  // CmdStrPtr(memory pointer) is the address of the two character AT command
  // returns true or false
  // if return = true, read _PktData to get a pointer to a buffer with the result.
  //***************************************
  byte Size = 5;                                        // Packet size is all data but chksum, start delimiter, and Length.
  memset(xBuff, 0, xBuffSize);
  //Create API Packet
  xBuff[0] = 0x7E;                                      // 7E = Start Delimiter P106 of XBee/XBee-PRO ZB RF Modules. Doc# 90000976_P
  xBuff[1] = highByte(Size);                            // Set Length of packet
  xBuff[2] = lowByte(Size);
  xBuff[3] = 0x08;                                      // APID 08 = AT Command
  xBuff[4] = 0x21;                                      // Frame ID 0 = no response

  xBuff[5] = CmdStrPtr[0];                              // First byte of AT command
  xBuff[6] = CmdStrPtr[1];                              // Second byte of AT command

  xBuff[7] = ValueByte;                                 // Set Value

  xBuff[8] = GetChkSum(3, Size + 3);                    // Find the Chksum and place it in last byte of API Packet

  //Send API Packet
  RxFlush();
  #if _DEBUG
    Serial.println();
    Serial.print(F("AT: "));
    Serial.print(char(CmdStrPtr[0]));
  	Serial.println(char(CmdStrPtr[1]));
    Serial.print("<");
    PrintHex(xBuff,8);
    Serial.println();
  #endif
  for (int i=0; i <= (Size + 3); i++)                   // Transmit packet byte at a time
  {
    write(xBuff[i]);
  }
 if (RX(1000) == 1) return true; else return false;     // Check for AT response and return result
}

boolean ZigBeeAPI::ATqueue(const char *CmdStrPtr, const uint8_t ByteArryPtr[], word ArrySize)
{
  //***************************************
  // Send two letter AT command plus data in a ByteArray to xBee queued.  Must be followed with a WR and AC command
  // CmdStrPtr(memory pointer) is the address of the two character AT command
  // ByteArryPtr is the address of a byte array to send
  // ArrySize is the size of the ByteArryPtr
  // Returns true or false
  // if Return = true, read _PktData to get a pointer to a buffer with the result
  //***************************************
  int Size = 4 + ArrySize;                              // Packet size is all data but checksum, start delimiter, and length
  memset(xBuff, 0, xBuffSize);
  // Create API Packet
  xBuff[0] = 0x7E;                                      // 7E = Start Delimiter P106 of XBee/XBee-PRO ZB RF Modules. Doc# 90000976_P
  xBuff[1] = highByte(Size);                            // Set Length of packet
  xBuff[2] = lowByte(Size);
  xBuff[3] = 0x09;                                      // APID 09 Queue Parameter Value 
  xBuff[4] = 0x00;                                      // Frame ID 0 = no response

  xBuff[5] = CmdStrPtr[0];                              // First byte of AT command
  xBuff[6] = CmdStrPtr[1];                              // Second byte of AT command

  ArrySize--;
  for (int i=7; i <= (7 + ArrySize); i++)
  {
    xBuff[i] = ByteArryPtr[ArrySize-(i-7)];             // Set Value
  } 
  xBuff[7 + ArrySize] = GetChkSum(3, Size + 3);         // Find the checksum and place it in the last byte of API Packet
  
  //Send API Packet
  RxFlush();
  #if _DEBUG
    Serial.println();
    Serial.print(F("AT: "));
    Serial.print(char(CmdStrPtr[0]));
  	Serial.println(char(CmdStrPtr[1]));
    Serial.print("<");
    PrintHex(xBuff,7 + ArrySize);
    Serial.println();
  #endif
  for (int i=0; i <= (Size + 3); i++)
  {                           
    write(xBuff[i]);                                    // Transmit packet byte at a time
  }
  return true;
}

byte * ZigBeeAPI::_PktData()
{
  //***************************************
  // Returns a pointer to a buffer containing packet data
  // Max size will never exceed 101 bytes
  // CAUTION! xBuff is used in other methods and will be overwritten. Therefore the
  // data in this buffer should be copied to a local buffer.
  //***************************************
  return xBuff;
}

int ZigBeeAPI::_PktDataSize()
{
  //***************************************
  // Returns the size of the _PktData
  // Max size will never exceed 101 bytes
  // CAUTION! xBuff is used in other methods and will be overwritten. Therefore the
  // data in this buffer should be copied to a local buffer.
  //***************************************
  return xBuffPayloadSize + 1;
}

long ZigBeeAPI::_PktIEEEAddHi()
{
  //***************************************
  // Returns a long containing the upper half of the IEEE address of last packet
  //***************************************
  return S_IAddHi;
}

long ZigBeeAPI::_PktIEEEAddLo()
{
  //***************************************
  // Returns a long containing the lower half of the IEEE address of last packet
  //***************************************
  return S_IAddLo;
}

long ZigBeeAPI::_PktNetAdd()
{
  //***************************************
  // Returns a long containing the network address of last packet
  //***************************************
  return S_NAdd;
}

byte ZigBeeAPI::_PktDEP()
{
  //***************************************
  // Returns a byte containing the Destination End Point of last packet
  //***************************************
  return S_DEP;
}

byte ZigBeeAPI::_PktSEP()
{
  //***************************************
  // Returns a byte containing the Source End Point of last packet
  //***************************************
  return S_SEP;
}

word ZigBeeAPI::_PktProfile()
{
  //***************************************
  // Returns a word containing the Profile ID of last packet
  //***************************************
  return S_Profile;
}

word ZigBeeAPI::_PktCluster()
{
  //***************************************
  // Returns a word containing the Cluster ID of last packet
  //***************************************
  return S_Cluster;
}

byte * ZigBeeAPI::_ReadLog()
{
  //***************************************
  // Returns a pointer to the last logged item
  //***************************************
  return strBuff;
}