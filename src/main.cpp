#include <Arduino.h>

#define SerialAOG Serial                //AgIO USB conection
HardwareSerial* SerialGPS = &Serial7;   //UM982 com1
const int32_t baudGPS = 460800;         //UM982 connection speed
const int32_t baudAOG = 115200;         //USB connection speed
constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer

#define EventOUT 23 //Trigger UM982 event

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

  Serial.println("Setup complete");

}

void loop() {

  Serial.println("--Begin Record--");
  Serial.print(micros());
  Serial.println(" :Begin Micros");

  digitalWrite(EventOUT, HIGH); // Send begining of pulse to UM982. Triggers on rising edge.
  Serial.print(micros());
  Serial.println(" : Send UM982 Event Pulse");
  delay(7);
  digitalWrite(EventOUT, LOW); //End of UM982 event pulse.

  Serial.print(micros());
  Serial.println(" :End Micros");
  Serial.println("--End Record--");
  Serial.println();

}
