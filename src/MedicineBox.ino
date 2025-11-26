#include "arduino_secrets.h"
/* 
  IoT-Based Automatic Medicine Reminder & Dispenser
  Board: NodeMCU 1.0 (ESP-12E, ESP8266)

  Cloud variables (already created in your Thing):
    int  reminderCode;   // READ ONLY   (0=none, 1=breakfast, 2=lunch, 3=dinner)
    int  remoteBox;      // READ/WRITE  (0=none, 1=breakfast, 2=lunch, 3=dinner)
    bool buzzerControl;  // READ/WRITE  (button to play buzzer)

  Callbacks (enable in Thing for RW vars):
    onRemoteBoxChange()
    onBuzzerControlChange()
*/

#include "thingProperties.h"

#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ----------------- LCD -----------------
#define LCD_ADDRESS 0x27       // change to 0x3F if your LCD uses that
#define LCD_COLS    16
#define LCD_ROWS    2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// ----------------- Hardware RTC (DS3231) -----------------
// IMPORTANT: name is NOT "rtc" to avoid conflict with ArduinoIoTCloud internal rtc
RTC_DS3231 ds3231;

// ----------------- Pins -----------------
const int SERVO1_PIN = D5;     // Breakfast box
const int SERVO2_PIN = D6;     // Lunch box
const int SERVO3_PIN = D7;     // Dinner box
const int BUZZER_PIN = D3;     // Passive buzzer

Servo servo1;
Servo servo2;
Servo servo3;

// ----------------- Servo positions -----------------
const int SERVO_CLOSED_ANGLE = 0;    // adjust to your box
const int SERVO_OPEN_ANGLE   = 90;   // adjust to open position

// ----------------- Fixed reminder times (24h) -----------------
const int BF_HOUR    = 8;
const int BF_MIN     = 0;

const int LUNCH_HOUR = 13;
const int LUNCH_MIN  = 30;

const int DIN_HOUR   = 20;
const int DIN_MIN    = 0;

// Track last date when each reminder was executed (yyyymmdd)
int lastBFDate      = 0;
int lastLunchDate   = 0;
int lastDinnerDate  = 0;

// For 1-second timing
unsigned long lastTick = 0;

// ======================================================
// Helper functions
// ======================================================

int getDateInt(const DateTime &t) {
  return t.year() * 10000 + t.month() * 100 + t.day();
}

// Rhythmic buzzer pattern
void playRhythmicBuzzer() {
  // simple "ta-ta-taa" pattern
  for (int i = 0; i < 2; i++) {
    tone(BUZZER_PIN, 2000);  // high tone
    delay(200);
    noTone(BUZZER_PIN);
    delay(150);

    tone(BUZZER_PIN, 1500);  // lower tone
    delay(200);
    noTone(BUZZER_PIN);
    delay(150);
  }

  // final long beep
  tone(BUZZER_PIN, 2500);
  delay(500);
  noTone(BUZZER_PIN);
}

// Open a box for 5 minutes, while keeping Cloud connection alive
void openBoxForFiveMinutes(Servo &s, int pin) {
  s.attach(pin);
  s.write(SERVO_OPEN_ANGLE);
  delay(2000);  // move to open

  unsigned long start = millis();
  while (millis() - start < 300000UL) { // 300000 ms = 5 minutes
    ArduinoCloud.update();   // keep IoT connection
    delay(100);
  }

  s.write(SERVO_CLOSED_ANGLE);
  delay(1000);
  s.detach();
}

// Handle a time-based reminder: sets reminderCode, buzzer, servo
void handleReminderTime(
  int targetHour,
  int targetMinute,
  int &lastDate,
  int codeForThisMeal,      // 1=breakfast, 2=lunch, 3=dinner
  Servo &servo,
  int servoPin,
  const char *label,
  const DateTime &now
) {
  int today = getDateInt(now);

  // Trigger when hour & minute match and within first 10 seconds
  if (now.hour() == targetHour && now.minute() == targetMinute && now.second() < 10) {
    if (lastDate != today) {
      Serial.print("Time-based reminder: ");
      Serial.println(label);

      // Set reminderCode (Cloud trigger will send notification)
      reminderCode = codeForThisMeal;
      ArduinoCloud.update();   // push value immediately

      // LCD message
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(label);
      lcd.setCursor(0, 1);
      lcd.print("Take medicine!");

      // buzzer + open box
      playRhythmicBuzzer();
      openBoxForFiveMinutes(servo, servoPin);

      lastDate = today;
    }
  } else {
    // reset reminderCode back to 0 when not in this reminder window
    if (reminderCode == codeForThisMeal) {
      reminderCode = 0;
    }
  }
}

void showTimeOnLCD(const DateTime &now) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time ");
  char buf[17];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  lcd.print(buf);

  lcd.setCursor(0, 1);
  lcd.print("8:00 13:30 20:00");
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  // Initialize serial
  Serial.begin(9600);
  delay(1500);

  // Initialize properties and Cloud
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  // ---- Hardware init ----

  // I2C (NodeMCU: D2 = SDA, D1 = SCL)
  Wire.begin(D2, D1);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Med Box IoT");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  // RTC DS3231
  if (!ds3231.begin()) {
    Serial.println("Couldn't find DS3231 RTC");
    lcd.clear();
    lcd.print("RTC ERROR!");
    while (1) {
      delay(1000);
    }
  }

  if (ds3231.lostPower()) {
    Serial.println("RTC lost power! Setting time once.");
    // Set time ONCE using compile time, then comment this line and re-upload:
    // ds3231.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // Initial servo positions (closed)
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  servo1.write(SERVO_CLOSED_ANGLE);
  servo2.write(SERVO_CLOSED_ANGLE);
  servo3.write(SERVO_CLOSED_ANGLE);
  delay(1000);

  servo1.detach();
  servo2.detach();
  servo3.detach();

  reminderCode = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IoT Connected");
  lcd.setCursor(0, 1);
  lcd.print("Ready");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  ArduinoCloud.update();    // keep IoT connection alive

  unsigned long nowMs = millis();
  if (nowMs - lastTick >= 1000) {  // 1 second tick
    lastTick = nowMs;

    DateTime now = ds3231.now();

    // Show time on LCD
    showTimeOnLCD(now);

    // Time-based reminders
    handleReminderTime(
      BF_HOUR, BF_MIN,
      lastBFDate,
      1,                // code for breakfast
      servo1,
      SERVO1_PIN,
      "Breakfast",
      now
    );

    handleReminderTime(
      LUNCH_HOUR, LUNCH_MIN,
      lastLunchDate,
      2,                // code for lunch
      servo2,
      SERVO2_PIN,
      "Lunch",
      now
    );

    handleReminderTime(
      DIN_HOUR, DIN_MIN,
      lastDinnerDate,
      3,                // code for dinner
      servo3,
      SERVO3_PIN,
      "Dinner",
      now
    );
  }
}

/*
  Since remoteBox is READ_WRITE, onRemoteBoxChange() is
  executed every time a new value is received from IoT Cloud.

  remoteBox:
    0 = none
    1 = open Breakfast box
    2 = open Lunch box
    3 = open Dinner box
*/
void onRemoteBoxChange()  {
  Serial.print("App: remoteBox = ");
  Serial.println(remoteBox);

  if (remoteBox == 1) {
    Serial.println("Open Breakfast Box (manual)");
    openBoxForFiveMinutes(servo1, SERVO1_PIN);
  } else if (remoteBox == 2) {
    Serial.println("Open Lunch Box (manual)");
    openBoxForFiveMinutes(servo2, SERVO2_PIN);
  } else if (remoteBox == 3) {
    Serial.println("Open Dinner Box (manual)");
    openBoxForFiveMinutes(servo3, SERVO3_PIN);
  }

  // Reset to 0 so next press works again
  remoteBox = 0;
}

/*
  Since buzzerControl is READ_WRITE, onBuzzerControlChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBuzzerControlChange()  {
  if (buzzerControl) {
    Serial.println("App: Play buzzer");
    playRhythmicBuzzer();
    // Reset back to false so the button can be used again
    buzzerControl = false;
  } else {
    // If later you use a continuous tone, this stops it
    noTone(BUZZER_PIN);
  }
}
