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

Car self = Car(MTR0f, MTR1f, MTR0b, MTR1b, MTR0e, MTR1e);

// NRF24 pins
#define CE_PIN 48
#define CSN_PIN 53

RF24 radio(CE_PIN, CSN_PIN);

// Addresses
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // this car uses address[1]
bool role = false;    // false = RX by default
float payload = 0.0;

void setup() {
  Serial.begin(9600);
  delay(100);

  if (!radio.begin()) {
    Serial.println(F("nRF24 hardware not responding!"));
    while (1) {
    }
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(payload));

  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);

  if (!role)
    radio.startListening();

  printf_begin();
  radio.printPrettyDetails();

  Serial.println(F("Car ready. Press T to transmit, R to receive"));
}

void loop() {
  // --- Joystick control ---
  int y = analogRead(A0);
  int x = analogRead(A1);
  self.drive(x, y);

  // --- RF24 TX/RX ---
  if (role) {
    unsigned long start_timer = micros();
    bool ok = radio.write(&payload, sizeof(payload));
    unsigned long end_timer = micros();

    if (ok) {
      Serial.print(F("TX OK  | Time: "));
      Serial.print(end_timer - start_timer);
      Serial.print(F("us | Payload: "));
      Serial.println(payload);
      payload += 0.01;
    } else {
      Serial.println(F("TX FAIL"));
    }

    delay(1000);
  } else {
    uint8_t pipe;
    if (radio.available(&pipe)) {
      radio.read(&payload, sizeof(payload));
      Serial.print(F("RX "));
      Serial.print(payload);
      Serial.print(F(" on pipe "));
      Serial.println(pipe);
    }
  }

  // --- Serial mode switching ---
  if (Serial.available()) {
    char c = toupper(Serial.read());

    if (c == 'T' && !role) {
      role = true;
      radio.stopListening();
      Serial.println("### CAR NOW IN TX MODE ###");
    }
    if (c == 'R' && role) {
      role = false;
      radio.startListening();
      Serial.println("### CAR NOW IN RX MODE ###");
    }
  }
}
