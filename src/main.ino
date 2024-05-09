#include <Arduino.h>
#include "Adafruit_BNO08x_RVC.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP
{
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 5;
};

ConfigIP networkAddress; // 3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = {0, 0, 0, 0}; // This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;           // port of this module
unsigned int AOGNtripPort = 2233;     // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888; // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;  // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];     // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     // Out port 5544
EthernetUDP Eth_udpNtrip;     // In port 2233
EthernetUDP Eth_udpAutoSteer; // In & Out Port 8888

IPAddress Eth_ipDestination;

bool Autosteer_running = true; // Auto set off in autosteer setup
bool Ethernet_running = false; // Auto set on in ethernet setup

void EthernetStart();
void udpNtrip();
void ReceiveUdp();

#define SerialAOG Serial                //AgIO USB conection
HardwareSerial* SerialGPS = &Serial7;   //UM982 com1
HardwareSerial* SerialRVC = &Serial5;   //RVC port
const int32_t baudGPS = 460800;         //UM982 connection speed
const int32_t baudAOG = 115200;         //USB connection speed
constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t RVCrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t RVCtxbuffer[serial_buffer_size];    //Extra serial tx buffer

#define EventOUT 33 //Trigger UM982 event

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;

bool resRVC;
double rvcCount = 0;

void setup() {
  delay(500);                         //Small delay so serial can monitor start up
  pinMode(EventOUT, OUTPUT);
  digitalWrite(EventOUT, LOW);

  delay(10);
  Serial.begin(baudAOG);
  delay(10);
  Serial.println("Start setup");

  SerialGPS->begin(baudGPS);
  SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  SerialRVC->begin(baudAOG);
  SerialRVC->addMemoryForRead(RVCrxbuffer, serial_buffer_size);
  SerialRVC->addMemoryForWrite(RVCtxbuffer, serial_buffer_size);

  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();

  if (!rvc.begin(SerialRVC))
  { // connect to the sensor over hardware serial
    Serial.println("Could not find RVC BNO08x!");
    while (1)
      delay(10);
  }
  else
  {
    Serial.println("RVC BNO08x found!");
  }

  Serial.println("Setup complete");

}

void loop() {
  BNO08x_RVC_Data rvcData;
  // Serial.println("--Begin Record--");
  // Serial.print(micros());
  // Serial.println(" :Begin Micros");

  // digitalWrite(EventOUT, HIGH); // Send begining of pulse to UM982. Triggers on rising edge.
  // Serial.print(micros());
  // Serial.println(" : Send UM982 Event Pulse");
  // delay(7);
  // digitalWrite(EventOUT, LOW); //End of UM982 event pulse.

  // Serial.print(micros());
  // Serial.println(" :End Micros");
  // Serial.println("--End Record--");
  // Serial.println();

  if ( rvc.read(&rvcData) )
  {
    //delay(100);
    digitalWrite(EventOUT, HIGH); // Send begining of pulse to UM982. Triggers on rising edge.
    digitalWrite(EventOUT, LOW); //End of UM982 event pulse.
    //Serial.println(rvcCount);
    rvcCount = 0;
  }
  rvcCount ++;

  while (SerialGPS->available())
  {
    char incoming = SerialGPS->read();
    //UMparser << incoming;
    //Serial.println(incoming);
    switch (incoming) 
    {
        case '#':
        msgBuf[msgBufLen] = incoming;
        msgBufLen ++;
        gotDollar = true;
        break;
        case '\r':
        //msgBuf[msgBufLen] = incoming;
        //msgBufLen ++;
        gotCR = true;
        gotDollar = false;
        break;
        case '\n':
        //msgBuf[msgBufLen] = incoming;
        //msgBufLen ++;
        gotLF = true;
        gotDollar = false;
        break;
        default:
        if (gotDollar)
            {
            msgBuf[msgBufLen] = incoming;
            msgBufLen ++;
            }
        break;
    }

    if (gotCR && gotLF)
    {
      Serial.print(msgBuf);
      Serial.print(","); Serial.print(rvcData.yaw);
      Serial.print(","); Serial.print(rvcData.pitch);
      Serial.print(","); Serial.println(rvcData.roll);
      gotCR = false;
      gotLF = false;
      gotDollar = false;
      memset( msgBuf, 0, 254 );
      msgBufLen = 0;
    }
  }

  udpNtrip();
}
