#include "RF24.h"
#include "motor.h"
#include "printf.h"
#include <SPI.h>

// Motor pins
#define MTR0f 5
#define MTR0b 4
#define MTR0e 2
#define MTR1f 7
#define MTR1b 6
#define MTR1e 3

#define SL 13
#define SR 12
#define SML 11
#define SM 10
#define SMR 9

Car self = Car(MTR0f, MTR1f, MTR0b, MTR1b, MTR0e, MTR1e);

// NRF24 pins
#define CE_PIN 48
#define CSN_PIN 53

RF24 radio(CE_PIN, CSN_PIN);

// Addresses
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // this car uses address[1]
bool role = false;    // false = RX by default
int16_t payload[9];

void setup() {
  Serial.begin(9600);
  delay(100);

  if (!radio.begin()) {
    Serial.println(F("nRF24 hardware not responding!"));
    while (true) {
    }
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(payload));

  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);

  if (role)
    radio.stopListening();
  else
    radio.startListening();

  printf_begin();
  radio.printPrettyDetails();

  Serial.println(F("Car ready. Press T to transmit, R to receive"));
}

void loop() {
  if (radio.available()) {
    radio.read(&payload, sizeof(payload));
    Serial.print(payload[0]);
    Serial.println(payload[1]);

    self.drive(payload[0], payload[1]);
  }
  delay(10);
}
