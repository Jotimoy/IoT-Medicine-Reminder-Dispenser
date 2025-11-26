#include "arduino_secrets.h"
#include "thingProperties.h"

#define LED_PIN D5   // External LED (change if needed)

void setup() {
  Serial.begin(9600);
  delay(1500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update();
}

// This is auto-linked with the Cloud variable
void onLedStateChange() {
  if (ledState) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED OFF");
  }
}
