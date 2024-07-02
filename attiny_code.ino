/*
 * ATtiny85-Servo-Heartbeat-Controller
 * 
 * Author: Farhang Naderi
 * Email: farhang.naderi@uri.edu
 *
 * Year: 2024
 * 
 * Description:
 * This code runs on an ATtiny85 microcontroller to control the power to a servo rail based on
 * heartbeat signals received via I2C communication. If a heartbeat signal is not received within
 * a specified timeout period, the servo rail is powered down, and the ATtiny85 enters a low power
 * sleep mode. When a heartbeat signal is received, the servo rail is powered on, and the ATtiny85
 * remains active.
 */

#include <TinyWireS.h>
#include <avr/sleep.h>

const int powerControlPin = 0; // Pin to control power to the servo rail
const int heartbeatTimeout = 3000; // Timeout period in milliseconds
volatile bool heartbeatReceived = false;
unsigned long lastHeartbeatTime = 0;

void setup() {
  pinMode(powerControlPin, OUTPUT);
  digitalWrite(powerControlPin, HIGH); // Initially, power on the servo rail

  TinyWireS.begin(0x08); // Set I2C address for the ATtiny85
  TinyWireS.onReceive(receiveEvent); // Register the receive event handler

  // Set PB1 as output
  DDRB |= (1 << PB1);
}

void loop() {
  TinyWireS_stop_check(); // Call this in your loop for proper TinyWireS functionality

  if (millis() - lastHeartbeatTime > heartbeatTimeout) {
    digitalWrite(powerControlPin, LOW); // No heartbeat received, power down the servo rail
    PORTB |= (1 << PB1); // Set PB1 high

    // Enter sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
  } else {
    digitalWrite(powerControlPin, HIGH); // Heartbeat received, power on the servo rail
    PORTB &= ~(1 << PB1); // Set PB1 low
  }

  delay(100); // Sleep for a short period to save power
}

// I2C receive event handler
void receiveEvent(uint8_t howMany) {
  while (TinyWireS.available()) {
    char c = TinyWireS.read();
    if (c == 'H') { // Assuming 'H' is the heartbeat signal
      heartbeatReceived = true;
      lastHeartbeatTime = millis();
    }
  }
}