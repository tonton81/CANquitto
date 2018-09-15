#include <IFCT.h>
#include <CANquitto.h>

void setup() {
  pinMode(2, OUTPUT); // for the transceiver enable pin
  Can0.setBaudRate(1000000);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Node.onReceive(myCallback);
  Node.begin(123);
  Can0.intervalTimer(); // enable queue system and run callback in background.
}

void loop() {
  // put your main code here, to run repeatedly:
  char buf[] = "Hello World! Hello Teensy! Hello IFCT!";
  if (Node.write((const uint8_t*)buf, sizeof(buf), 112, 11, 2000, 3000)) digitalWrite(13, !digitalRead(13));
  delay(100);
}

void myCallback(const uint8_t* buffer, uint16_t length, AsyncCQ info) {
  Serial.print("PAYLOAD!!: ");
  for ( uint32_t i = 0; i < length; i++ ) {
    Serial.print(buffer[i]); Serial.print(" ");
  } Serial.println();
  Serial.print("From Node: ");
  Serial.println(info.node);

  Serial.print("Length: ");
  Serial.println(length);

  Serial.print("Millis(): ");
  Serial.println(millis());
}
