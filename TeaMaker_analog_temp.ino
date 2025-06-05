#include <LiquidCrystal.h>

// Motor positions (kept at top to ensure global scope)
enum MotorPosition {
  MOTOR_STOP = 0,
  MOTOR_UP = 1,
  MOTOR_DOWN = 2
};
MotorPosition currentMotorPosition = MOTOR_STOP;

// Pin definitions
#define TMP36_PIN A0          // TMP36 analog pin
#define BUTTON_PIN 3          // Start button pin
#define PRESET_BUTTON_PIN 2   // Tea preset button pin
#define BLUE_LED_PIN 4        // Blue LED pin
#define RED_LED_PIN 5         // Red LED pin
#define MOTOR_IN1 A2          // L293D Input 1
#define MOTOR_IN2 A3          // L293D Input 2
#define MOTOR_EN A4           // L293D Enable pin (PWM)
#define BUZZER_PIN A1         // Piezo buzzer pin

// LCD pin definitions for LCM1602C (16x2 HD44780-compatible LCD)
#define LCD_RS 12
#define LCD_EN 11
#define LCD_D4 10
#define LCD_D5 9
#define LCD_D6 8
#define LCD_D7 7
#define LCD_BACKLIGHT 13      // Backlight anode control (PWM)
// Note: LCM1602C VO pin (contrast) requires a 10kΩ potentiometer to GND/5V.
//       Backlight (A/K pins) uses pin 13 with a 220Ω resistor for anode (A).

// LCD setup (RS, EN, D4, D5, D6, D7)
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Tea preset definitions
enum TeaType {
  BLACK_TEA = 0,
  GREEN_TEA = 1
};

// Tea preset parameters
struct TeaPreset {
  const char* name;
  float tempHigh;
  float tempLow;
  unsigned long waitTime;
};

TeaPreset teaPresets[] = {
  {"Black Tea", 100.0, 90.0, 360000},  // Black tea: 90-100°C, 6 minutes
  {"Green Tea", 90.0, 75.0, 240000}    // Green tea: 75-90°C, 4 minutes
};

// TMP36 calibration constants
#define AREF_VOLTAGE 5.0      // Arduino reference voltage
#define ADC_RESOLUTION 1024   // 10-bit ADC resolution

// Timing constants (in milliseconds)
#define DEBOUNCE_DELAY 50         // Button debounce delay
#define MAIN_LOOP_DELAY 100       // Main loop delay
#define HIGH_TEMP_WAIT 60000      // Wait time when temp > high threshold (1 minute)
#define MOTOR_RUN_TIME 3000       // Motor run time (3 seconds)
#define LED_BLINK_DELAY 500       // LED on/off time for blinking
#define TEMP_READING_DELAY 10     // Delay between temperature readings
#define STEEP_UPDATE_INTERVAL 1000  // LCD update interval during steeping (1 second)
#define COOLDOWN_UPDATE_INTERVAL 60000 // LCD update interval during cooldown (1 minute)

// Motor control constants
#define MOTOR_SPEED 255           // Motor speed (0-255)
#define MOTOR_UP_TIME 2000        // Time to run motor UP (2 seconds)
#define MOTOR_DOWN_TIME 2000      // Time to run motor DOWN (2 seconds)
#define MOTOR_HOLD_TIME 3000      // Time to hold position (3 seconds)

// LED and buzzer constants
#define LED_BLINK_COUNT 5         // Number of times to blink LEDs
#define BUZZER_FREQUENCY 1000     // Buzzer frequency in Hz
#define BUZZER_DURATION 200       // Buzzer beep duration
#define BUZZER_PAUSE 300          // Pause between buzzer beeps
#define BUZZER_BEEP_COUNT 3       // Number of beeps at end

// Temperature sensor constants
#define TEMP_READINGS_COUNT 10    // Number of readings to average
#define TMP36_OFFSET_VOLTAGE 0.5  // TMP36 offset voltage (500mV at 0°C)
#define TMP36_SCALE_FACTOR 100.0  // TMP36 scale factor (10mV/°C)

// Global variables
bool systemRunning = false;
bool buttonPressed = false;
bool presetButtonPressed = false;
float currentTemperature = 0.0;
TeaType currentTeaType = BLACK_TEA;

void moveMotorToPosition(int position) {
  digitalWrite(MOTOR_EN, HIGH); // Enable motor
  
  switch (position) {
    case MOTOR_UP:
      Serial.println("Moving motor UP");
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      analogWrite(MOTOR_EN, MOTOR_SPEED);
      delay(MOTOR_UP_TIME);
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, LOW);
      currentMotorPosition = MOTOR_UP;
      Serial.println("Motor reached UP position");
      break;
      
    case MOTOR_DOWN:
      Serial.println("Moving motor DOWN");
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
      analogWrite(MOTOR_EN, MOTOR_SPEED);
      delay(MOTOR_DOWN_TIME);
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, LOW);
      currentMotorPosition = MOTOR_DOWN;
      Serial.println("Motor reached DOWN position");
      break;
      
    case MOTOR_STOP:
    default:
      Serial.println("Stopping motor");
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, LOW);
      digitalWrite(MOTOR_EN, LOW);
      currentMotorPosition = MOTOR_STOP;
      break;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(LCD_BACKLIGHT, OUTPUT);
  analogWrite(LCD_BACKLIGHT, 128); // Set backlight to medium brightness (PWM)
  lcd.begin(16, 2);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PRESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  moveMotorToPosition(MOTOR_UP);
  updateLCDStatus("Starting...", "System Ready");
  delay(2000);
  updateLCDTeaType();
  Serial.println("Temperature Control System Ready (TMP36)");
  Serial.println("Press the start button to begin...");
}

void loop() {
  if (digitalRead(PRESET_BUTTON_PIN) == LOW && !presetButtonPressed) {
    delay(DEBOUNCE_DELAY);
    if (digitalRead(PRESET_BUTTON_PIN) == LOW) {
      presetButtonPressed = true;
      if (!systemRunning) {
        currentTeaType = (currentTeaType == BLACK_TEA) ? GREEN_TEA : BLACK_TEA;
        updateLCDTeaType();
        Serial.print("Tea type changed to: ");
        Serial.println(teaPresets[currentTeaType].name);
      }
    }
  }
  
  if (digitalRead(PRESET_BUTTON_PIN) == HIGH) {
    presetButtonPressed = false;
  }
  
  if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
    delay(DEBOUNCE_DELAY);
    if (digitalRead(BUTTON_PIN) == LOW) {
      buttonPressed = true;
      systemRunning = true;
      Serial.println("System started!");
      Serial.print("Using preset: ");
      Serial.println(teaPresets[currentTeaType].name);
      mainProcess();
    }
  }
  
  if (digitalRead(BUTTON_PIN) == HIGH) {
    buttonPressed = false;
  }
  
  delay(MAIN_LOOP_DELAY);
}

void mainProcess() {
  TeaPreset currentPreset = teaPresets[currentTeaType];
  updateLCDStatus("Processing...", "Reading temp...");
  
  while (systemRunning) {
    currentTemperature = measureTemperature();
    Serial.print("Current temperature: ");
    Serial.print(currentTemperature);
    Serial.println("°C");
    
    String tempLine = "Temp: " + String(currentTemperature, 1) + "C";
    updateLCDStatus(currentPreset.name, tempLine);
    
    if (currentTemperature > currentPreset.tempHigh) {
      Serial.print("Temperature above ");
      Serial.print(currentPreset.tempHigh);
      Serial.println("°C - Waiting 1 minute");
      updateLCDStatus(currentPreset.name, "Too hot! Wait...");
      waitWithCountdown(HIGH_TEMP_WAIT, "Cool down: ");
      continue;
    }
    else if (currentTemperature < currentPreset.tempLow) {
      Serial.print("Temperature below ");
      Serial.print(currentPreset.tempLow);
      Serial.println("°C - Blinking blue LED");
      updateLCDStatus(currentPreset.name, "Too cold!");
      blinkBlueLED();
      break;
    }
    else {
      Serial.println("Temperature in range - Starting actuator sequence");
      Serial.println("Moving motor to DOWN position for 3 seconds");
      updateLCDStatus(currentPreset.name, "Motor DOWN");
      moveMotorToPosition(MOTOR_DOWN);
      delay(MOTOR_HOLD_TIME);
      
      Serial.print("Waiting ");
      Serial.print(currentPreset.waitTime / 60000);
      Serial.println(" minutes...");
      waitWithCountdown(currentPreset.waitTime, "Steeping: ");
      
      Serial.println("Moving motor to UP position for 3 seconds");
      updateLCDStatus(currentPreset.name, "Motor UP");
      moveMotorToPosition(MOTOR_UP);
      delay(MOTOR_HOLD_TIME);
      
      Serial.println("Blinking red LED 5 times");
      updateLCDStatus(currentPreset.name, "Complete!");
      blinkRedLED();
      break;
    }
  }
  
  endProcess();
}

void waitWithCountdown(unsigned long waitTime, const char* prefix) {
  unsigned long startTime = millis();
  unsigned long lastUpdate = 0;
  unsigned long updateInterval = (strcmp(prefix, "Steeping: ") == 0) ? STEEP_UPDATE_INTERVAL : COOLDOWN_UPDATE_INTERVAL;
  
  while (millis() - startTime < waitTime) {
    if (millis() - lastUpdate >= updateInterval) {
      unsigned long remaining = waitTime - (millis() - startTime);
      unsigned long minutes = remaining / 60000;
      unsigned long seconds = (remaining % 60000) / 1000;
      currentTemperature = measureTemperature(); // Measure temperature for each update
      Serial.print("Current temperature: ");
      Serial.print(currentTemperature);
      Serial.println("°C");
      
      // Format time as MM:SS
      String timeStr = String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);
      // Format temperature to 1 decimal place
      String tempStr = String(currentTemperature, 1) + "C";
      // Combine time and temperature, ensuring it fits 16 characters
      String countdownLine = timeStr + " " + tempStr;
      if (countdownLine.length() > 16) {
        countdownLine = timeStr; // Fallback to time only if too long
      }
      
      updateLCDStatus(teaPresets[currentTeaType].name, countdownLine);
      lastUpdate = millis();
    }
    delay(100); // Small delay to prevent excessive processing
  }
}

void updateLCDStatus(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

void updateLCDTeaType() {
  TeaPreset currentPreset = teaPresets[currentTeaType];
  String tempRange = String(currentPreset.tempLow, 0) + "-" + String(currentPreset.tempHigh, 0) + "C";
  updateLCDStatus(currentPreset.name, tempRange);
}

float measureTemperature() {
  long totalReading = 0;
  for (int i = 0; i < TEMP_READINGS_COUNT; i++) {
    totalReading += analogRead(TMP36_PIN);
    delay(TEMP_READING_DELAY);
  }
  float averageReading = totalReading / (float)TEMP_READINGS_COUNT;
  float voltage = (averageReading * AREF_VOLTAGE) / ADC_RESOLUTION;
  float temperatureC = (voltage - TMP36_OFFSET_VOLTAGE) * TMP36_SCALE_FACTOR;
  Serial.print("ADC Reading: ");
  Serial.print(averageReading);
  Serial.print(", Voltage: ");
  Serial.print(voltage);
  Serial.print("V, ");
  temperatureC=78;
  return temperatureC;
}

void blinkBlueLED() {
  for (int i = 0; i < LED_BLINK_COUNT; i++) {
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(LED_BLINK_DELAY);
    digitalWrite(BLUE_LED_PIN, LOW);
    delay(LED_BLINK_DELAY);
  }
}

void blinkRedLED() {
  for (int i = 0; i < LED_BLINK_COUNT; i++) {
    digitalWrite(RED_LED_PIN, HIGH);
    delay(LED_BLINK_DELAY);
    digitalWrite(RED_LED_PIN, LOW);
    delay(LED_BLINK_DELAY);
  }
}

void makeBeep() {
  for (int i = 0; i < BUZZER_BEEP_COUNT; i++) {
    tone(BUZZER_PIN, BUZZER_FREQUENCY, BUZZER_DURATION);
    delay(BUZZER_PAUSE);
  }
}

void endProcess() {
  Serial.println("Process completed!");
  updateLCDStatus("Process Done!", "Press to restart");
  makeBeep();
  systemRunning = false;
  buttonPressed = false;
  delay(3000);
  updateLCDTeaType();
  Serial.println("Press the start button to restart the system...");
}