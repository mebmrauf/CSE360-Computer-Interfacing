/* 
 * ===============================================================
 * MOTORCYCLE SMART SECURITY SYSTEM - Arduino Uno
 * Feature 1: Dual-Stage Motion Alarm with ADXL345
 * ===============================================================
 * 
 * This is PHASE 1: Testing Dual-Stage Motion Alarm
 * - Motion detection via ADXL345 accelerometer
 * - Stage 1: Soft vibration/beep for < 3 seconds
 * - Stage 2: Loud siren for continuous motion
 * 
 * Current Pin Usage:
 * - A4 (SDA), A5 (SCL): ADXL345 & RTC I2C
 * - D9 (~PWM): Active Buzzer
 * - D10 (~PWM): Vibration Motor
 * - D4, D5, D6: Buttons (for future control)
 */

#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <RTClib.h>
#include <math.h>

// ===== ADXL345 I2C Configuration =====
#define ADXL345_ADDRESS 0x53  // Default I2C address (if ALT address pin is floating/GND)
#define DATAX0 0x32           // X-axis data register
#define POWER_CTL 0x2D        // Power control register
#define DATA_FORMAT 0x31      // Data format register
#define INT_ENABLE 0x2E       // Interrupt enable register

// ===== Pin Definitions =====
#define BUZZER_PIN 9          // PWM pin for active buzzer
#define MOTOR_PIN 10          // PWM pin for vibration motor
#define BUTTON1_PIN 4         // Disarm button
#define BUTTON2_PIN 5         // Mode button
#define BUTTON3_PIN 6         // Secret button
#define RIDE_KEY_PIN 3        // Key input (LOW = inserted, HIGH = removed)
#define REED_SWITCH_PIN 2     // Reed switch (LOW = closed/moving, HIGH = open/parked)
#define GPS_RX_PIN A1         // GPS TX -> Arduino A1 (software serial RX)
#define GPS_TX_PIN A2         // GPS RX <- Arduino A2 (software serial TX)
#define KILL_SWITCH_PIN A0    // Relay control for ignition kill-switch

// ===== GSM Configuration =====
#define GSM_RX_PIN 7          // GSM TX -> Arduino D7
#define GSM_TX_PIN 8          // GSM RX <- Arduino D8
SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN);

// CRITICAL: Replace with your actual phone number including country code
const char MASTER_PHONE[] = "+88017XXXXXXXX";  // Example: "+8801712345678"

// ===== Incoming SMS Polling Variables =====
unsigned long lastSmsCheckTime = 0;
const unsigned long SMS_CHECK_INTERVAL = 15000; // Check for incoming SMS every 15 seconds

// ===== Reed Switch Variables =====
bool reedSwitchMovementDetected = false;  // Movement detected via reed switch
unsigned long lastReedSwitchChangeTime = 0;
bool lastReedSwitchState = HIGH;  // Last known state (HIGH = open/parked)
const unsigned long REED_SWITCH_DEBOUNCE = 500;  // 500ms debounce
bool reedSwitchAlertSent = false;  // Track if alert already sent for this movement event

// ===== Geofencing Variables =====
bool geofenceActive = false;           // Geofence is armed
int32_t initialLatE5 = 0;              // Initial latitude when armed (E5 format)
int32_t initialLonE5 = 0;              // Initial longitude when armed (E5 format)
unsigned long lastGeofenceCheckTime = 0;
const unsigned long GEOFENCE_CHECK_INTERVAL = 30000;  // Check geofence every 30 seconds
const float GEOFENCE_RADIUS_METERS = 10.0;            // 10-meter radius

// ===== Alarm Configuration =====
#define STAGE1_THRESHOLD 250   // Acceleration threshold for stage 1 (mg) - Light tap
#define STAGE2_THRESHOLD 450   // Acceleration threshold for stage 2 (mg) - Sustained motion
#define STAGE1_DURATION 3000   // Duration for Stage 1 alarm (3 seconds)
#define STAGE2_DURATION 5000   // Duration for Stage 2 alarm (5 seconds)
#define STAGE1_CHECK_INTERVAL 2000  // Check for escalation every 2 seconds
#define STAGE2_VIBRATION_PULSE 500  // Vibrate pulse every 500ms
#define MOTION_DEBOUNCE 200    // Debounce time for motion events

// ===== Lockout Configuration =====
#define LOCKOUT_TRIGGER_COUNT 3  // Lockout after 3 alarms
#define LOCKOUT_WINDOW 300000  // 5 minutes = 300,000 ms
#define SECRET_SEQUENCE_TIMEOUT 5000  // 5 seconds to complete sequence
#define BUTTON1_CODE 1
#define BUTTON2_CODE 2
#define BUTTON3_CODE 3

// ===== LED Pin Definitions =====
#define RED_LED_PIN 11        // Red LED (D11) - Tens digit
#define GREEN_LED_PIN 12      // Green LED (D12) - Ones digit
#define BLUE_LED_PIN 13       // Blue LED (D13) - Status indicator

// ===== Ride Mode Configuration =====
#define ENGINE_SIMULATION_DURATION 300000  // 5 minutes simulated riding time

// ===== State Machine Enums =====
enum SystemState {
  DISARMED,       // Engine runnable, monitoring for auto-arm
  ARMED,          // Monitoring for motion
  WARNING,        // Stage 1: Soft vibe/beep for < 3s
  ALARM,          // Stage 2: Loud siren
  LOCKOUT         // Triggered >3x/5m. Ignores sensors
};

// ===== Global Variables =====
SystemState currentState = DISARMED;
SystemState previousState = DISARMED;

unsigned long stage1StartTime = 0;
unsigned long stage2StartTime = 0;
unsigned long lastMotionTime = 0;
unsigned long lastStage1CheckTime = 0;
unsigned long lastStage2VibrationTime = 0;
bool motionDetected = false;
bool inStage1 = false;
bool inStage2 = false;
int baselineMagnitude = 1000;  // Baseline at rest (approx 1g)

// Track one tamper log entry per complete alarm incident
bool tamperEventActive = false;
unsigned long tamperEventStartTimeMs = 0;

// ===== Lockout Variables =====
unsigned long lastAlarmTriggerTime = 0;  // Time of last alarm trigger
int alarmTriggerCount = 0;  // Count of alarms in current window
bool inLockout = false;  // Lockout mode flag
byte secretSequence[3];  // Stores pressed button sequence
byte secretSequenceIndex = 0;  // Current index in sequence
unsigned long secretSequenceStartTime = 0;  // Timer for sequence input

// ===== Admin Button Sequences (outside lockout) =====
byte adminSequence[3];
byte adminSequenceIndex = 0;
unsigned long adminSequenceStartTime = 0;

// ===== Ride Mode Variables =====
bool inRideMode = false;  // Ride mode flag (disables motion detection)

// ===== Engine Simulation Variables =====
bool engineSimulated = false;  // Simulated engine running state
unsigned long engineSimStartTime = 0;  // When engine simulation started
unsigned long lastEngineCheckTime = 0;  // For periodic engine status

// ===== RTC Variables =====
RTC_DS3231 rtc;

// ===== GPS Variables =====
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
bool hasGpsFix = false;
int32_t latestLatE5 = 0;
int32_t latestLonE5 = 0;
unsigned long lastGpsFixTimeMs = 0;
unsigned long lastGpsDiagPrintMs = 0;

int32_t toScaledE5(double value) {
  if (value >= 0.0) {
    return (int32_t)(value * 100000.0 + 0.5);
  }
  return (int32_t)(value * 100000.0 - 0.5);
}

void pollGpsStream() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    latestLatE5 = toScaledE5(gps.location.lat());
    latestLonE5 = toScaledE5(gps.location.lng());
    hasGpsFix = true;
    lastGpsFixTimeMs = millis();
  }

  unsigned long now = millis();
  if ((now - lastGpsDiagPrintMs) >= 30000UL) {
    lastGpsDiagPrintMs = now;
    if (hasGpsFix) {
      Serial.print(F("GPS OK: "));
      Serial.println(latestLatE5 / 100000.0, 5);
    }
  }
}

// ===== EEPROM Event Structure =====
struct TamperEvent {
  uint32_t timestamp;   // Seconds since boot (placeholder until RTC wired)
  uint8_t duration;     // Duration in seconds (0-255)
  int32_t latitudeE5;   // Latitude in degrees * 100000
  int32_t longitudeE5;  // Longitude in degrees * 100000
};

// ===== EEPROM API Prototypes =====
void logTamperEvent(uint32_t timestamp, uint8_t duration, int32_t latitudeE5, int32_t longitudeE5);
uint16_t getTotalStoredEvents();
TamperEvent readTamperEvent(uint16_t eventIndex);
void clearTamperLog();
void startLedBlinkSequence();
void manageNonBlockingLedBlink();

// ===== EEPROM Ring Buffer Configuration =====
#define EEPROM_WRITE_OFFSET_ADDR 0        // Address 0-1: Current write offset (uint16_t)
#define EEPROM_EVENT_COUNT_ADDR 2         // Address 2-3: Total events written counter (uint16_t)
#define EEPROM_BUFFER_START_ADDR 4        // Ring buffer data starts at address 4
#define EEPROM_EVENT_SIZE sizeof(TamperEvent)
#define EEPROM_AVAILABLE 1024             // Arduino Uno has 1024 bytes EEPROM
#define EEPROM_BUFFER_SIZE (EEPROM_AVAILABLE - EEPROM_BUFFER_START_ADDR)  // 1020 bytes available
#define MAX_EVENTS (EEPROM_BUFFER_SIZE / EEPROM_EVENT_SIZE)

// ===== Non-Blocking LED Blink Variables =====
unsigned long lastLedBlinkTime = 0;    // Track blink timing
byte currentLedBlinkPhase = 0;          // Which digit/LED is blinking (0=tens red, 1=ones green)
byte blinkCount = 0;                    // Count blinks for current digit
byte targetBlinks = 0;                  // How many times to blink
bool ledBlinkInProgress = false;        // Is LED blink sequence active
unsigned long ledBlinkStartTime = 0;    // When blink sequence started

// ===== Tamper Event Helpers =====
void startTamperEventIfNeeded(unsigned long nowMs);
void finalizeTamperEvent();
void dumpAllTamperLogsToSerial();
void indicateLogsOnGreenLed();
void syncRideModeFromKeyInput();
void applyRideModeState(bool enable);
void validateEepromLayout();
void manageKillSwitch();

// ===== Reed Switch Helpers =====
void checkReedSwitch();

// ===== Geofencing Helpers =====
float calculateGpsDistance(int32_t lat1E5, int32_t lon1E5, int32_t lat2E5, int32_t lon2E5);
void checkGeofence();

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  delay(100);
  
  Serial.println(F("=== Motorcycle Security System BOOTING ==="));
  Serial.println(F("Feature 1: Dual-Stage Motion Alarm"));
  Serial.println(F(""));
  
  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(RIDE_KEY_PIN, INPUT_PULLUP);
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
  pinMode(KILL_SWITCH_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  // CRITICAL: Turn off all outputs immediately
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(KILL_SWITCH_PIN, LOW);  // NC relay default: closed circuit (ignition path allowed)
  noTone(BUZZER_PIN);
  analogWrite(MOTOR_PIN, 0);
  
  // Initialize I2C
  Wire.begin();
  delay(100);

  // Initialize RTC module
  if (!rtc.begin()) {
    Serial.println(F("RTC ERR"));
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize GPS serial (NEO module)
  gpsSerial.begin(9600);
  
  // Initialize GSM serial
  gsmSerial.begin(9600);
  
  // Set default listener to GPS
  gpsSerial.listen();
  
  // Initialize ADXL345
  if (!initADXL345()) {
    Serial.println(F("ADXL ERR"));
  }

  validateEepromLayout();
  setState(ARMED);
  
}

// ===== MAIN LOOP =====
void loop() {
  // Continuously parse GPS data stream.
  pollGpsStream();

  // NEW: Check for incoming SMS commands periodically
  manageIncomingSMS();

  // Check button inputs
  checkButtons();

  // Ride mode follows D3 key input (LOW = on, HIGH = off)
  syncRideModeFromKeyInput();

  // Apply kill-switch relay output policy.
  manageKillSwitch();
  
  // Manage auto-arm and lockout logic
  manageAutoArmAndLockout();
  
  // Check reed switch for movement while armed
  if (currentState == ARMED) {
    checkReedSwitch();
  }
  
  // Check geofence if active
  if (geofenceActive) {
    checkGeofence();
  }
  
  // Attempt to activate geofence if armed but not yet active (waiting for GPS fix)
  if (currentState == ARMED && !geofenceActive && hasGpsFix) {
    geofenceActive = true;
    initialLatE5 = latestLatE5;
    initialLonE5 = latestLonE5;
    lastGeofenceCheckTime = millis();
    Serial.print(F("[Geofence] ACTIVATED (delayed, waiting for GPS): "));
    Serial.print(initialLatE5 / 100000.0, 5);
    Serial.print(F(", "));
    Serial.println(initialLonE5 / 100000.0, 5);
  }
  
  // Manage ride mode engine simulation
  manageEngineSimulation();
  
 // feature 1: dual-stage motion alarm
  //Read accelerometer data and detect motion
  if (currentState == ARMED) {
    checkMotionSensor();
  }
  
  //Manage alarm states and outputs
  manageAlarmStates();
  //feature 2: auto-arm and disarm based on engine status (future)
  
  // Debug: Print system status
  printSystemStatus();
  
  // Manage non-blocking LED blink sequence for event log
  manageNonBlockingLedBlink();
  
  delay(100);  // Update every 100ms
}

// ===== ADXL345 INITIALIZATION =====
bool initADXL345() {
  // Check if device is present on I2C bus
  Wire.beginTransmission(ADXL345_ADDRESS);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  
  // Power control: wake up the device
  writeADXL345(POWER_CTL, 0x08);
  delay(10);
  
  // Data format: full resolution, ±16g range
  writeADXL345(DATA_FORMAT, 0x0B);
  delay(10);
  
  // Interrupt enable: disable all interrupts (we're polling for now)
  writeADXL345(INT_ENABLE, 0x00);
  
  return true;
}

// ===== I2C READ/WRITE HELPERS =====
void writeADXL345(byte reg, byte value) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte readADXL345(byte reg) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDRESS, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// ===== READ ADXL345 ACCELERATION DATA =====
void getAcceleration(int16_t& x, int16_t& y, int16_t& z) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(DATAX0);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDRESS, 6);
  
  byte xLow, xHigh, yLow, yHigh, zLow, zHigh;
  if (Wire.available() >= 6) {
    xLow = Wire.read();
    xHigh = Wire.read();
    yLow = Wire.read();
    yHigh = Wire.read();
    zLow = Wire.read();
    zHigh = Wire.read();
    
    // Convert to 16-bit signed values (two's complement)
    x = (int16_t)(xHigh << 8 | xLow);
    y = (int16_t)(yHigh << 8 | yLow);
    z = (int16_t)(zHigh << 8 | zLow);
    
    // Convert to mg (each LSB = 4mg in full resolution)
    x = (x * 4);
    y = (y * 4);
    z = (z * 4);
  }
}

// ===== MOTION SENSOR CHECK =====
void checkMotionSensor() {
  // SKIP motion detection if in RIDE_MODE
  if (inRideMode) {
    return;  // Don't check for motion while riding
  }
  
  int16_t accelX, accelY, accelZ;
  getAcceleration(accelX, accelY, accelZ);
  
  // Calculate proper vector magnitude: sqrt(x^2 + y^2 + z^2)
  int32_t x_sq = (int32_t)accelX * accelX;
  int32_t y_sq = (int32_t)accelY * accelY;
  int32_t z_sq = (int32_t)accelZ * accelZ;
  int32_t magnitude_sq = x_sq + y_sq + z_sq;
  int16_t magnitude = sqrt(magnitude_sq);
  
  // Dynamic acceleration (motion relative to baseline gravity)
  int16_t dynamicAccel = magnitude - baselineMagnitude;
  
  // Check if motion exceeds threshold
  if (dynamicAccel > STAGE1_THRESHOLD) {
    if (millis() - lastMotionTime > MOTION_DEBOUNCE) {
      motionDetected = true;
      lastMotionTime = millis();
      
      Serial.print(F("MOTION: "));
      Serial.println(dynamicAccel);
    }
  } else {
    motionDetected = false;
  }
}

// ===== ALARM STATE MANAGEMENT =====
void manageAlarmStates() {
  unsigned long now = millis();
  
  // ===== ONLY TRIGGER NEW ALARMS ON MOTION DETECTION =====
  if (motionDetected && !inStage1 && !inStage2) {
    // No alarm is active - check if we should start one
    int16_t currentDynamic = getCurrentDynamicAccel();
    
    if (currentDynamic > STAGE2_THRESHOLD) {
      // Heavy motion detected - go straight to Stage 2
      recordAlarmTrigger();  // Track for lockout
      startTamperEventIfNeeded(now);
      inStage2 = true;
      stage2StartTime = now;
      lastStage2VibrationTime = now;
      Serial.println(F("→ Stage 2: LOUD ALARM (heavy motion detected) - 5 sec"));
    } else {
      // Light motion - enter Stage 1
      recordAlarmTrigger();  // Track for lockout
      startTamperEventIfNeeded(now);
      inStage1 = true;
      stage1StartTime = now;
      lastStage1CheckTime = now;
      Serial.println(F("→ Stage 1: BEEP ALARM (light motion detected) - 3 sec"));
    }
  }
  
  // ===== CHECK FOR ESCALATION DURING STAGE 1 =====
  if (inStage1 && !inStage2 && motionDetected) {
    // Check every 2 seconds if motion escalated to Stage 2
    if ((now - lastStage1CheckTime) >= STAGE1_CHECK_INTERVAL) {
      lastStage1CheckTime = now;
      int16_t currentDynamic = getCurrentDynamicAccel();
      
      Serial.print(F("Accel: "));
      Serial.println(currentDynamic);
      
      if (currentDynamic > STAGE2_THRESHOLD) {
        inStage1 = false;
        inStage2 = true;
        stage2StartTime = now;
        lastStage2VibrationTime = now;
        Serial.println(F("→ Stage 2: ESCALATING TO LOUD ALARM - 5 sec"));
      }
    }
  }
  
  // ===== CHECK FOR STAGE 1 TIMEOUT (LET IT RUN FULL 3 SECONDS) =====
  if (inStage1) {
    if ((now - stage1StartTime) >= STAGE1_DURATION) {
      Serial.println(F("✓ Stage 1 complete (3 sec). Resetting alarm."));
      inStage1 = false;
      noTone(BUZZER_PIN);
      analogWrite(MOTOR_PIN, 0);
      finalizeTamperEvent();
    }
  }
  
  // ===== CHECK FOR STAGE 2 TIMEOUT (LET IT RUN FULL 5 SECONDS) =====
  if (inStage2) {
    if ((now - stage2StartTime) >= STAGE2_DURATION) {
      Serial.println(F("✓ Stage 2 complete (5 sec). Resetting alarm."));
      inStage2 = false;
      noTone(BUZZER_PIN);
      analogWrite(MOTOR_PIN, 0);
      finalizeTamperEvent();
    }
  }
  
  // ===== EXECUTE ALARM OUTPUTS =====
  if (inStage1) {
    // Stage 1: Continuous beep for 3 seconds, NO vibration
    tone(BUZZER_PIN, 1000);  // Continuous 1kHz beep
    analogWrite(MOTOR_PIN, 0);  // NO vibration
  } else if (inStage2) {
    // Stage 2: Alternating tone pattern (beeeee boooooo) + pulsed vibration
    unsigned long elapsedInStage2 = now - stage2StartTime;
    unsigned long cyclePosition = elapsedInStage2 % 1000;  // 1 second cycle
    
    if (cyclePosition < 500) {
      // First 500ms: high pitch (beeee)
      tone(BUZZER_PIN, 2000);
    } else {
      // Next 500ms: low pitch (boooo)
      tone(BUZZER_PIN, 1000);
    }
    
    unsigned long vibrationCycle = elapsedInStage2 % 1000;
    if (vibrationCycle < 250) {
      analogWrite(MOTOR_PIN, 255);  // Vibrate for first 250ms
    } else {
      analogWrite(MOTOR_PIN, 0);    // Off for next 250ms
    }
  } else if (currentState == DISARMED) {
    // Ensure outputs are off when disarmed
    noTone(BUZZER_PIN);
    analogWrite(MOTOR_PIN, 0);
  }
}

// ===== HELPER: Get Current Dynamic Acceleration =====
int16_t getCurrentDynamicAccel() {
  int16_t accelX, accelY, accelZ;
  getAcceleration(accelX, accelY, accelZ);
  
  int32_t x_sq = (int32_t)accelX * accelX;
  int32_t y_sq = (int32_t)accelY * accelY;
  int32_t z_sq = (int32_t)accelZ * accelZ;
  int32_t magnitude_sq = x_sq + y_sq + z_sq;
  int16_t magnitude = sqrt(magnitude_sq);
  
  return (magnitude - baselineMagnitude);
}

// ===== BUTTON HANDLING WITH SECRET SEQUENCE SUPPORT =====
void checkButtons() {
  unsigned long now = millis();
  
  // Reset secret sequence if timeout
  if (secretSequenceIndex > 0 && (now - secretSequenceStartTime) > SECRET_SEQUENCE_TIMEOUT) {
    Serial.println(F("[Sequence] Timeout - resetting sequence"));
    secretSequenceIndex = 0;
  }

  // Reset admin sequence if timeout
  if (adminSequenceIndex > 0 && (now - adminSequenceStartTime) > SECRET_SEQUENCE_TIMEOUT) {
    Serial.println(F("[Admin Sequence] Timeout - resetting sequence"));
    adminSequenceIndex = 0;
  }
  
  // Button 1: Sequence input
  if (digitalRead(BUTTON1_PIN) == LOW) {
    delay(20);
    if (digitalRead(BUTTON1_PIN) == LOW) {
      if (inLockout) {
        // In lockout: part of secret sequence
        addToSecretSequence(BUTTON1_CODE);
      }

      if (!inLockout) {
        addToAdminSequence(BUTTON1_CODE);
      }
      delay(300);
    }
  }
  
  // Button 2: Sequence input
  if (digitalRead(BUTTON2_PIN) == LOW) {
    delay(20);
    if (digitalRead(BUTTON2_PIN) == LOW) {
      if (inLockout) {
        // In lockout: part of secret sequence
        addToSecretSequence(BUTTON2_CODE);
      }

      if (!inLockout) {
        addToAdminSequence(BUTTON2_CODE);
      }
      delay(300);
    }
  }
  
  // Button 3: Sequence input
  if (digitalRead(BUTTON3_PIN) == LOW) {
    delay(20);
    if (digitalRead(BUTTON3_PIN) == LOW) {
      if (inLockout) {
        // In lockout: part of secret sequence
        addToSecretSequence(BUTTON3_CODE);
      }

      if (!inLockout) {
        addToAdminSequence(BUTTON3_CODE);
      }
      delay(300);
    }
  }
}

// ===== HELPER: Add Button to Admin Sequence =====
void addToAdminSequence(byte buttonCode) {
  if (adminSequenceIndex == 0) {
    adminSequenceStartTime = millis();
  }

  adminSequence[adminSequenceIndex] = buttonCode;
  adminSequenceIndex++;

  Serial.print(F("Btn: "));
  Serial.println(buttonCode);

  if (adminSequenceIndex >= 3) {
    byte s0 = adminSequence[0];
    byte s1 = adminSequence[1];
    byte s2 = adminSequence[2];

    if (s0 == BUTTON1_CODE && s1 == BUTTON1_CODE && s2 == BUTTON1_CODE) {
      Serial.println(F("[Admin Sequence] ✓ 1+1+1 detected: system disarm"));
      setState(DISARMED);
    } else if (s0 == BUTTON2_CODE && s1 == BUTTON2_CODE && s2 == BUTTON2_CODE) {
      Serial.println(F("[Admin Sequence] ✓ 2+2+2 detected: system arm"));
      setState(ARMED);
    } else if (s0 == BUTTON3_CODE && s1 == BUTTON3_CODE && s2 == BUTTON3_CODE) {
      Serial.println(F("[Admin Sequence] ✓ 3+3+3 detected: test alarm"));
      testAlarm();
    } else if (s0 == BUTTON2_CODE && s1 == BUTTON2_CODE && s2 == BUTTON3_CODE) {
      Serial.println(F("[Admin Sequence] ✓ 2+2+3 detected: dumping tamper logs"));
      dumpAllTamperLogsToSerial();
    } else if (s0 == BUTTON1_CODE && s1 == BUTTON3_CODE && s2 == BUTTON3_CODE) {
      Serial.println(F("[Admin Sequence] ✓ 1+3+3 detected: clearing all tamper logs"));
      clearTamperLog();
    } else if (s0 == BUTTON1_CODE && s1 == BUTTON2_CODE && s2 == BUTTON2_CODE) {
      Serial.println(F("[Admin Sequence] ✓ 1+2+2 detected: checking log presence"));
      indicateLogsOnGreenLed();
    } else {
      Serial.println(F("[Admin Sequence] ✗ Unknown sequence"));
    }

    adminSequenceIndex = 0;
  }
}

// ===== HELPER: Add Button to Secret Sequence =====
void addToSecretSequence(byte buttonCode) {
  if (secretSequenceIndex == 0) {
    secretSequenceStartTime = millis();
  }
  
  secretSequence[secretSequenceIndex] = buttonCode;
  secretSequenceIndex++;
  
  Serial.print(F("SEQ: "));
  Serial.println(buttonCode);
  
  // Check if sequence is complete (3 buttons)
  if (secretSequenceIndex >= 3) {
    checkSecretSequence();
  }
}

// ===== HELPER: Check if Secret Sequence is Correct =====
void checkSecretSequence() {
  // Secret sequence: Button 1 -> Button 2 -> Button 3
  byte correctSequence[] = {BUTTON1_CODE, BUTTON2_CODE, BUTTON3_CODE};
  bool sequenceCorrect = true;
  
  for (int i = 0; i < 3; i++) {
    if (secretSequence[i] != correctSequence[i]) {
      sequenceCorrect = false;
      break;
    }
  }
  
  if (sequenceCorrect) {
    Serial.println(F("SEQ OK"));
    inLockout = false;
    alarmTriggerCount = 0;
    digitalWrite(KILL_SWITCH_PIN, LOW);  // Exit lockout: close relay circuit
    setState(DISARMED);

    // On successful unlock, show tamper log summary and dump all events.
    dumpAllTamperLogsToSerial();

    // Automatically display event count via red/green LED blink protocol.
    startLedBlinkSequence();

    // Confirm with beeps
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2000, 100);
  } else {
    Serial.println(F("[Sequence] ✗ INCORRECT! Sequence reset."));
    // Warn with buzzer
    tone(BUZZER_PIN, 500, 200);
  }
  
  secretSequenceIndex = 0;
}

// ===== TEST ALARM SEQUENCE =====
void testAlarm() {
  Serial.println(F("Testing Stage 1 (3 seconds)..."));
  inStage1 = true;
  stage1StartTime = millis();
  
  while (millis() - stage1StartTime < STAGE1_DURATION) {
    analogWrite(MOTOR_PIN, 150);
    tone(BUZZER_PIN, 1000, 100);
    delay(200);
  }
  
  Serial.println(F("Testing Stage 2 (3 seconds)..."));
  inStage1 = false;
  inStage2 = true;
  stage1StartTime = millis();
  
  while (millis() - stage1StartTime < STAGE1_DURATION) {
    analogWrite(MOTOR_PIN, 255);
    tone(BUZZER_PIN, 2000);
    delay(100);
  }
  
  // Reset
  noTone(BUZZER_PIN);
  analogWrite(MOTOR_PIN, 0);
  inStage2 = false;
  Serial.println(F("Test complete!"));
}

// ===== STATE CHANGE HANDLER =====
void setState(SystemState newState) {
  if (newState != currentState) {
    previousState = currentState;
    currentState = newState;

    if (tamperEventActive && (inStage1 || inStage2)) {
      finalizeTamperEvent();
    }
    
    // Reset alarm states on state change
    inStage1 = false;
    inStage2 = false;
    noTone(BUZZER_PIN);
    analogWrite(MOTOR_PIN, 0);
    
    // LED indicators for state change
    if (newState == ARMED) {
      // Blink RED LED 2 times for ARMED
      for (int i = 0; i < 2; i++) {
        digitalWrite(RED_LED_PIN, HIGH);
        delay(200);
        digitalWrite(RED_LED_PIN, LOW);
        delay(200);
      }
    } else if (newState == DISARMED) {
      // Blink GREEN LED 2 times for DISARMED
      for (int i = 0; i < 2; i++) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        delay(200);
        digitalWrite(GREEN_LED_PIN, LOW);
        delay(200);
      }
      
      // Disable geofence when disarmed
      geofenceActive = false;
      
      // Reset reed switch tracking when disarmed
      reedSwitchMovementDetected = false;
      reedSwitchAlertSent = false;
    }
    
    // Enable geofence when armed (only if GPS fix available)
    if (newState == ARMED && hasGpsFix) {
      geofenceActive = true;
      initialLatE5 = latestLatE5;
      initialLonE5 = latestLonE5;
      lastGeofenceCheckTime = millis();
      Serial.print(F("[Geofence] ACTIVATED at initial location: "));
      Serial.print(initialLatE5 / 100000.0, 5);
      Serial.print(F(", "));
      Serial.println(initialLonE5 / 100000.0, 5);
    } else if (newState == ARMED && !hasGpsFix) {
      Serial.println(F("[Geofence] Cannot activate - waiting for GPS fix"));
    }
  }
}

void startTamperEventIfNeeded(unsigned long nowMs) {
  if (!tamperEventActive) {
    tamperEventActive = true;
    tamperEventStartTimeMs = nowMs;
  }
}

void finalizeTamperEvent() {
  if (!tamperEventActive) {
    return;
  }

  unsigned long now = millis();
  uint32_t durationSeconds32 = (now - tamperEventStartTimeMs) / 1000UL;
  if (durationSeconds32 == 0) {
    durationSeconds32 = 1;
  }
  if (durationSeconds32 > 255) {
    durationSeconds32 = 255;
  }

  // Fetch real-world time from the RTC
  DateTime currentTime = rtc.now();
  uint32_t timestampSeconds = currentTime.unixtime();

  int32_t latE5ForLog = 0;
  int32_t lonE5ForLog = 0;
  if (hasGpsFix) {
    latE5ForLog = latestLatE5;
    lonE5ForLog = latestLonE5;
  }

  logTamperEvent(timestampSeconds, (uint8_t)durationSeconds32, latE5ForLog, lonE5ForLog);

  if (!hasGpsFix) {
    if (gps.charsProcessed() < 10) {
      Serial.println(F("[GPS] Logged 0,0 because no GPS serial data is being received"));
    } else {
      Serial.println(F("[GPS] Logged 0,0 because GPS has no satellite fix yet"));
    }
  }

  tamperEventActive = false;
  tamperEventStartTimeMs = 0;
}

void dumpAllTamperLogsToSerial() {
  uint16_t eventCount = getTotalStoredEvents();
  Serial.print(F("Logs: "));
  Serial.println(eventCount);

  for (uint16_t i = 0; i < eventCount; i++) {
    TamperEvent event = readTamperEvent(i);
    Serial.print(F("#"));
    Serial.print(i + 1);
    Serial.print(F(" | Time="));
    Serial.print(event.timestamp);
    Serial.print(F(" | Duration="));
    Serial.print(event.duration);
    Serial.print(F("s | Lat="));
    Serial.print(event.latitudeE5 / 100000.0, 5);
    Serial.print(F(" | Lon="));
    Serial.println(event.longitudeE5 / 100000.0, 5);
  }
}

void validateEepromLayout() {
  uint16_t writeOffset = 0;
  uint16_t totalEvents = 0;
  EEPROM.get(EEPROM_WRITE_OFFSET_ADDR, writeOffset);
  EEPROM.get(EEPROM_EVENT_COUNT_ADDR, totalEvents);

  bool offsetInvalid = (writeOffset >= EEPROM_BUFFER_SIZE) || ((writeOffset % EEPROM_EVENT_SIZE) != 0);
  bool countInvalid = (totalEvents > 60000);

  if (offsetInvalid || countInvalid) {
    clearTamperLog();
    Serial.println(F("EEPROM RESET"));
  }
}

void manageKillSwitch() {
  // NC relay behavior: HIGH opens (cuts ignition), LOW closes (allows circuit).
  digitalWrite(KILL_SWITCH_PIN, inLockout ? HIGH : LOW);
}

void indicateLogsOnGreenLed() {
  uint16_t eventCount = getTotalStoredEvents();
  if (eventCount == 0) {
    return;
  }
  for (byte i = 0; i < 3; i++) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(180);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(180);
  }
}

// ===== STATE MACHINE UPDATE =====
void updateStateMachine() {
  // Currently using simple polling - ready for future enhancements
  // (deep sleep, RTC integration, GSM alerts, etc.)
}

// ===== AUTO-ARM & LOCKOUT MANAGEMENT =====
void manageAutoArmAndLockout() {
  unsigned long now = millis();
  
  // ===== LOCKOUT WINDOW CLEANUP =====
  if (alarmTriggerCount > 0 && (now - lastAlarmTriggerTime) > LOCKOUT_WINDOW) {
    alarmTriggerCount = 0;
  }
}

// ===== HELPER: Record Alarm Trigger =====
void recordAlarmTrigger() {
  unsigned long now = millis();
  
  if (alarmTriggerCount == 0) {
    lastAlarmTriggerTime = now;
    alarmTriggerCount = 1;
    Serial.println(F("[Trigger] Count: 1/3 in 5-min window"));
  } else if ((now - lastAlarmTriggerTime) <= LOCKOUT_WINDOW) {
    alarmTriggerCount++;
Serial.print(F("Trigger: "));
      Serial.println(alarmTriggerCount);
    
    if (alarmTriggerCount >= LOCKOUT_TRIGGER_COUNT) {
      enterLockoutMode();
    }
  } else {
    lastAlarmTriggerTime = now;
    alarmTriggerCount = 1;
    Serial.println(F("[Trigger] New window. Count: 1/3"));
  }
}

// ===== HELPER: Enter Lockout Mode =====
void enterLockoutMode() {
  inLockout = true;
  secretSequenceIndex = 0;
  applyRideModeState(false);  // Ride mode must not run during lockout.
  digitalWrite(KILL_SWITCH_PIN, HIGH);  // Engage kill-switch (open circuit)
  Serial.println(F("\n[LOCKOUT]"));
  Serial.println(F("Btn: 1->2->3"));
  
  // Send lockout SMS alert
  sendLockoutAlertSMS();
  
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1000, 200);
    delay(300);
  }
  noTone(BUZZER_PIN);
}

// ===== RIDE MODE INPUT HANDLING =====
void syncRideModeFromKeyInput() {
  if (inLockout) {
    if (inRideMode) {
      applyRideModeState(false);
    }
    return;
  }

  bool keyInserted = (digitalRead(RIDE_KEY_PIN) == LOW);
  applyRideModeState(keyInserted);
}

void applyRideModeState(bool enable) {
  if (inRideMode == enable) {
    return;
  }

  inRideMode = enable;

  if (enable) {
    Serial.println(F("RIDE MODE ON"));
    
    // Start engine simulation
    engineSimulated = true;
    engineSimStartTime = millis();
    
    // Confirmation beeps
    tone(BUZZER_PIN, 1500, 150);
    delay(200);
    tone(BUZZER_PIN, 1500, 150);
  } else {
    Serial.println(F("RIDE MODE OFF"));
    
    // Stop engine simulation
    engineSimulated = false;
    
    // Confirmation beep
    tone(BUZZER_PIN, 1000, 200);
  }
}

// ===== ENGINE SIMULATION MANAGEMENT =====
void manageEngineSimulation() {
  unsigned long now = millis();
  
  if (engineSimulated) {
    // Print engine status every 10 seconds

  }
}

// ===== REED SWITCH MOVEMENT DETECTION =====
void checkReedSwitch() {
  unsigned long now = millis();
  bool currentReedState = digitalRead(REED_SWITCH_PIN);  // HIGH = open/parked, LOW = closed/moving
  
  // Debounce: check if state changed and enough time has passed
  if (currentReedState != lastReedSwitchState) {
    if ((now - lastReedSwitchChangeTime) >= REED_SWITCH_DEBOUNCE) {
      lastReedSwitchState = currentReedState;
      lastReedSwitchChangeTime = now;
      
      if (currentReedState == LOW) {
        Serial.println(F("REED: MOVED"));
        reedSwitchMovementDetected = true;
        reedSwitchAlertSent = false;
      } else {
        Serial.println(F("REED: PARKED"));
        reedSwitchMovementDetected = false;
        reedSwitchAlertSent = false;
      }
    }
  } else {
    lastReedSwitchChangeTime = now;  // Keep resetting debounce timer while state is stable
  }
  
  // Trigger LOCKOUT MODE immediately when movement detected while armed
  if (reedSwitchMovementDetected && !reedSwitchAlertSent && currentState == ARMED) {
    reedSwitchAlertSent = true;
    Serial.println(F("[Reed] LOCKOUT"));
    enterLockoutMode();
  }
}

// ===== SYSTEM STATUS DISPLAY =====
void printSystemStatus() {
  static unsigned long lastPrintTime = 0;
  unsigned long now = millis();
  
  if ((now - lastPrintTime) >= 5000) {
    lastPrintTime = now;
    
    Serial.print(F("[")); 
    switch (currentState) {
      case DISARMED: Serial.print(F("D")); break;
      case ARMED: Serial.print(F("A")); break;
      case WARNING: Serial.print(F("W")); break;
      case ALARM: Serial.print(F("AL")); break;
      case LOCKOUT: Serial.print(F("L")); break;
      default: Serial.print(F("?"));
    }
    Serial.print(F("] T:"));
    Serial.print(alarmTriggerCount);
    if (inRideMode) Serial.print(F(" RIDE"));
    if (digitalRead(KILL_SWITCH_PIN) == HIGH) Serial.print(F(" RELAY-CUT"));
    Serial.println();
  }
}

// ===== EEPROM RING BUFFER FUNCTIONS =====

// Write a tamper event to EEPROM ring buffer
void logTamperEvent(uint32_t timestamp, uint8_t duration, int32_t latitudeE5, int32_t longitudeE5) {
  // Read current write offset
  uint16_t writeOffset = 0;
  EEPROM.get(EEPROM_WRITE_OFFSET_ADDR, writeOffset);
  
  // Create event struct
  TamperEvent event;
  event.timestamp = timestamp;
  event.duration = duration;
  event.latitudeE5 = latitudeE5;
  event.longitudeE5 = longitudeE5;
  
  // Calculate actual EEPROM address
  uint16_t eepromAddr = EEPROM_BUFFER_START_ADDR + writeOffset;
  
  // Write event to EEPROM
  EEPROM.put(eepromAddr, event);
  
  // Update write offset for next event
  writeOffset = (writeOffset + EEPROM_EVENT_SIZE) % EEPROM_BUFFER_SIZE;
  EEPROM.put(EEPROM_WRITE_OFFSET_ADDR, writeOffset);
  
  // Increment total events counter
  uint16_t totalEvents = 0;
  EEPROM.get(EEPROM_EVENT_COUNT_ADDR, totalEvents);
  totalEvents++;
  EEPROM.put(EEPROM_EVENT_COUNT_ADDR, totalEvents);
  
  Serial.print(F("[EEPROM] Event logged. Total events: "));
  Serial.println(totalEvents);
}

// Get total number of stored events
uint16_t getTotalStoredEvents() {
  uint16_t totalEvents = 0;
  EEPROM.get(EEPROM_EVENT_COUNT_ADDR, totalEvents);
  
  // Cap at MAX_EVENTS since buffer is circular
  if (totalEvents > MAX_EVENTS) {
    totalEvents = MAX_EVENTS;
  }
  
  return totalEvents;
}

// Read a specific event from EEPROM by index (0 = oldest, totalEvents-1 = newest)
TamperEvent readTamperEvent(uint16_t eventIndex) {
  TamperEvent event = {0, 0, 0, 0};
  
  uint16_t totalEvents = getTotalStoredEvents();
  if (eventIndex >= totalEvents) {
    return event;  // Return empty event
  }
  
  // Calculate which physical offset this event is at
  // If totalEvents < MAX_EVENTS, oldest is at offset 0
  // If totalEvents >= MAX_EVENTS, oldest is at current write offset
  uint16_t writeOffset = 0;
  EEPROM.get(EEPROM_WRITE_OFFSET_ADDR, writeOffset);
  
  uint16_t physicalIndex;
  if (totalEvents >= MAX_EVENTS) {
    // Buffer has wrapped - oldest event is at current writeOffset
    physicalIndex = (writeOffset + (eventIndex * EEPROM_EVENT_SIZE)) % EEPROM_BUFFER_SIZE;
  } else {
    // Buffer hasn't wrapped yet - oldest is at index 0
    physicalIndex = eventIndex * EEPROM_EVENT_SIZE;
  }
  
  uint16_t eepromAddr = EEPROM_BUFFER_START_ADDR + physicalIndex;
  EEPROM.get(eepromAddr, event);
  
  return event;
}

// Clear all EEPROM tamper log data
void clearTamperLog() {
  // Reset write offset and event counter
  uint16_t zeroOffset = 0;
  EEPROM.put(EEPROM_WRITE_OFFSET_ADDR, zeroOffset);
  EEPROM.put(EEPROM_EVENT_COUNT_ADDR, zeroOffset);
  
  Serial.println(F("[EEPROM] Tamper log cleared"));
}

// ===== NON-BLOCKING LED BLINK FUNCTION =====

// Start LED blink sequence to display event count
// tens digit blinks on Red LED, ones digit blinks on Green LED
void startLedBlinkSequence() {
  if (ledBlinkInProgress) {
    return;  // Already blinking
  }
  
  uint16_t totalEvents = getTotalStoredEvents();
  
  Serial.print(F("LED: "));
  Serial.println(totalEvents);
  
  ledBlinkInProgress = true;
  ledBlinkStartTime = millis();
  currentLedBlinkPhase = 0;  // Start with tens digit on Red LED
  blinkCount = 0;
  targetBlinks = (totalEvents / 10) % 10;  // Tens digit (0-9)
  if (targetBlinks == 0) targetBlinks = 10;  // Blink 10 times for 0
  
  lastLedBlinkTime = millis();
}

// Non-blocking LED blink manager - call this in main loop
void manageNonBlockingLedBlink() {
  if (!ledBlinkInProgress) {
    return;
  }
  
  unsigned long now = millis();
  unsigned long blinkDuration = 200;  // On or off for 200ms
  unsigned long pauseBetweenDigits = 1500;  // 1.5 second pause between tens and ones
  
  // Check if sequence timeout (max 20 seconds)
  if ((now - ledBlinkStartTime) > 20000) {
    ledBlinkInProgress = false;
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    return;
  }
  
  // Check if time to toggle LED
  if ((now - lastLedBlinkTime) >= blinkDuration) {
    lastLedBlinkTime = now;
    
    if (currentLedBlinkPhase == 0) {
      // Tens digit phase (Red LED)
      // Toggle Red LED
      digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
      
      // Increment blink count
      if (digitalRead(RED_LED_PIN) == LOW) {
        blinkCount++;
        
        if (blinkCount >= targetBlinks) {
          // Done with tens digit, move to ones digit
          currentLedBlinkPhase = 1;
          blinkCount = 0;
          
          uint16_t totalEvents = getTotalStoredEvents();
          targetBlinks = totalEvents % 10;  // Ones digit (0-9)
          if (targetBlinks == 0) targetBlinks = 10;  // Blink 10 times for 0
          
          lastLedBlinkTime = now + pauseBetweenDigits;  // Pause before ones digit
          Serial.print(F("[LED] Tens complete, starting ones digit ("));
          Serial.print(targetBlinks);
          Serial.println(F(" blinks)"));
        }
      }
    } else {
      // Ones digit phase (Green LED)
      // Toggle Green LED
      digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
      
      // Increment blink count
      if (digitalRead(GREEN_LED_PIN) == LOW) {
        blinkCount++;
        
        if (blinkCount >= targetBlinks) {
          ledBlinkInProgress = false;
          digitalWrite(RED_LED_PIN, LOW);
          digitalWrite(GREEN_LED_PIN, LOW);
        }
      }
    }
  }
}

// ===== GSM SMS TRANSMISSION (FIXED VERSION) =====
void sendAlertSMS(String alertMessage) {
Serial.println(F("[GSM] SMS..."));
  
  gsmSerial.listen();
  while(gsmSerial.available()) { gsmSerial.read(); }
  
  gsmSerial.println(F("AT+CMGF=1")); 
  delay(500); 
  
  gsmSerial.print(F("AT+CMGS=\""));
  gsmSerial.print(MASTER_PHONE);
  gsmSerial.println(F("\""));
  delay(1000); 
  
  gsmSerial.print(alertMessage);
  delay(100);
  
  // CRITICAL FIX: Must use .write(26) instead of .println((char)26)
  gsmSerial.write(26); 
  
  delay(4000);
  gpsSerial.listen();
}

// ===== SMS ALERT HELPER FUNCTIONS =====

// Send lockout mode triggered alert
void sendLockoutAlertSMS() {
  sendAlertSMS(F("CRITICAL: Bike attacked! Ignition killed. Lockout engaged. Reply 'UNLOCK' to disarm."));
}

// Send reed switch movement detection alert
void sendMovementAlertSMS() {
  String movementAlert = F("ALERT: Bike moved while parked! Someone is stealing it! Loc: ");
  
  if (hasGpsFix) {
    movementAlert += F("http://maps.google.com/?q=");
    movementAlert += String(latestLatE5 / 100000.0, 5);
    movementAlert += F(",");
    movementAlert += String(latestLonE5 / 100000.0, 5);
  } else {
    movementAlert += F("[GPS Fix Unavailable]");
  }
  
  sendAlertSMS(movementAlert);
}

// Send geofence breach alert
void sendGeofenceAlertSMS(float distanceMeters) {
  String geofenceAlert = F("GEOFENCE BREACH: Bike moved ");
  geofenceAlert += String(distanceMeters, 1);
  geofenceAlert += F("m from parking location! Lockout engaged. Loc: http://maps.google.com/?q=");
  geofenceAlert += String(latestLatE5 / 100000.0, 5);
  geofenceAlert += F(",");
  geofenceAlert += String(latestLonE5 / 100000.0, 5);
  
  sendAlertSMS(geofenceAlert);
}

// ===== SMS COMMAND POLLING =====
void manageIncomingSMS() {
  unsigned long now = millis();
  if (now - lastSmsCheckTime >= SMS_CHECK_INTERVAL) {
    lastSmsCheckTime = now;
    checkCommandSMS();
  }
}

void checkCommandSMS() {
  // 1. Suspend GPS and switch to GSM
  gsmSerial.listen();
  
  // 2. Clear out any junk in the buffer
  while(gsmSerial.available()) { gsmSerial.read(); }
  
  // 3. Set text mode and request unread messages
  gsmSerial.println(F("AT+CMGF=1"));
  delay(200);
  gsmSerial.println(F("AT+CMGL=\"REC UNREAD\""));
  
  // 4. Capture the response into a String (Memory safe limit)
  String response = "";
  response.reserve(200); 
  unsigned long startTime = millis();
  
  // Wait up to 2 seconds for the module to dump the text messages
  while(millis() - startTime < 2000) {
    while (gsmSerial.available()) {
      char c = gsmSerial.read();
      response += c;
    }
  }
  
  // 5. Parse the response
  if (response.length() > 5) {
    response.toUpperCase();
    
    if (response.indexOf(MASTER_PHONE) != -1 && response.indexOf("STATUS") != -1) {
      Serial.println(F("STATUS OK"));
      sendStatusLogSMS();
    }
    
    // 6. Delete all messages from SIM memory
    gsmSerial.println(F("AT+CMGD=1,4")); 
    delay(500);
  }
  
  // 7. Switch back to GPS tracking
  gpsSerial.listen();
}

// ===== GEOFENCING FUNCTIONS =====
float calculateGpsDistance(int32_t lat1E5, int32_t lon1E5, int32_t lat2E5, int32_t lon2E5) {
  // Convert E5 coordinates to degrees
  float lat1 = lat1E5 / 100000.0;
  float lon1 = lon1E5 / 100000.0;
  float lat2 = lat2E5 / 100000.0;
  float lon2 = lon2E5 / 100000.0;
  
  // Simple approximation for small distances
  // 1 degree latitude ≈ 111,000 meters
  float latDeltaMeters = (lat2 - lat1) * 111000.0;
  
  // 1 degree longitude ≈ 111,000 * cos(latitude) meters
  // Use average latitude for cosine calculation
  float avgLat = (lat1 + lat2) / 2.0;
  float lonDeltaMeters = (lon2 - lon1) * 111000.0 * cos(avgLat * 0.01745329); // 0.01745329 = pi/180
  
  // Pythagorean distance
  float distance = sqrt(latDeltaMeters * latDeltaMeters + lonDeltaMeters * lonDeltaMeters);
  return distance;
}

void checkGeofence() {
  unsigned long now = millis();
  
  // Check geofence every 30 seconds
  if (now - lastGeofenceCheckTime >= GEOFENCE_CHECK_INTERVAL) {
    lastGeofenceCheckTime = now;
    
    if (!hasGpsFix) {
      return;
    }
    
    // Calculate distance from initial location
    float distanceMeters = calculateGpsDistance(initialLatE5, initialLonE5, latestLatE5, latestLonE5);
    
Serial.print(F("GEO: "));
      Serial.print(distanceMeters, 1);
      Serial.println(F("m"));
    
    // If distance exceeds 5-meter radius, trigger lockout
    if (distanceMeters > GEOFENCE_RADIUS_METERS) {
      Serial.print(F("GEO BREACH: "));
      Serial.print(distanceMeters, 1);
      Serial.println(F("m"));
      
      // Send geofence breach alert and trigger lockout
      sendGeofenceAlertSMS(distanceMeters);
      geofenceActive = false;  // Disable further checks
      enterLockoutMode();
    }
  }
}

// ===== BUILD & SEND STATUS REPLY =====
void sendStatusLogSMS() {
  String payload = F("Bike Status: ");
  
  // Add current state
  switch (currentState) {
    case DISARMED: payload += F("DISARMED"); break;
    case ARMED: payload += F("ARMED"); break;
    case WARNING: payload += F("WARNING"); break;
    case ALARM: payload += F("ALARM"); break;
    case LOCKOUT: payload += F("LOCKOUT"); break;
    default: payload += F("UNKNOWN");
  }
  
  // Add EEPROM Log Data
  uint16_t totalEvents = getTotalStoredEvents();
  payload += F("\nSaved Logs: ");
  payload += String(totalEvents);
  
  if (totalEvents > 0) {
    // Fetch the most recent tamper event
    TamperEvent lastEvent = readTamperEvent(totalEvents - 1);
    payload += F("\nLast Event: ");
    payload += String(lastEvent.duration);
    payload += F("s\nLoc: http://maps.google.com/?q=");
    payload += String(lastEvent.latitudeE5 / 100000.0, 5);
    payload += F(",");
    payload += String(lastEvent.longitudeE5 / 100000.0, 5);
  } else {
    // If no logs, just send current location
    if (hasGpsFix) {
      payload += F("\nCurrent Loc: http://maps.google.com/?q=");
      payload += String(latestLatE5 / 100000.0, 5);
      payload += F(",");
      payload += String(latestLonE5 / 100000.0, 5);
    } else {
      payload += F("\n[GPS Fix Unavailable]");
    }
  }
  
  // Send the compiled payload
  sendAlertSMS(payload);
}