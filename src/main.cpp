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

#define SL 9
#define SR 13
#define SML 10
#define SM 11
#define SMR 12

Car self = Car(MTR0f, MTR1f, MTR0b, MTR1b, MTR0e, MTR1e);
LineFollower lineFollower = LineFollower(self, SL, SML, SM, SMR, SR);

bool isLineFollower = false;

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

  lineFollower.init();

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
    int last6 = payload[6];

    if (!isLineFollower) {
      radio.read(&payload, sizeof(payload));
      Serial.print(payload[0]);
      Serial.print(", ");
      Serial.print(payload[1]);
      Serial.print(", ");
      Serial.print(payload[2]);
      Serial.print(", ");
      Serial.print(payload[3]);
      Serial.print(", ");
      Serial.print(payload[4]);
      Serial.print(", ");
      Serial.print(payload[5]);
      Serial.print(", ");
      Serial.print(payload[6]);
      Serial.print(", ");
      Serial.print(payload[7]);
      Serial.print(", ");
      Serial.println(payload[8]);
    }

    if (last6 != payload[6] && payload[6] == 0)
      isLineFollower = !isLineFollower;

    if (isLineFollower)
      lineFollower.run();
    else
      self.drive(payload[0], payload[1]);
  }
  delay(10);
}
