#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_VL53L0X.h>

String input = "";

#define NINJA_ESP32_VERSION "NINJA_ESP32_V2_SIMPLE"
#define DEFAULT_MOTOR_MIN_PWM 150
#define DEFAULT_MOTOR_MAX_PWM 255

// =============================
// Motor safety watchdog
// =============================
unsigned long lastDriveCommandMs = 0;
unsigned long commandTimeoutMs = 300;

bool watchdogEnabled = true;
bool motorsAreActive = false;
bool watchdogStopPrinted = false;

// =============================
// Motor tuning settings
// =============================
int motorMinPwm = DEFAULT_MOTOR_MIN_PWM;
int motorMaxPwm = DEFAULT_MOTOR_MAX_PWM;

int leftTrim = 0;
int rightTrim = 0;

// 0 = disabled.
// If > 0, motor PWM moves gradually toward target.
int motorRampStep = 0;

int currentLeftPwm = 0;
int currentRightPwm = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("NINJA ESP32 READY");
  Serial.print("TEL VERSION ");
  Serial.println(NINJA_ESP32_VERSION);
  Serial.println("Starting NinjaCode...");

  setupMotors();
  setupGripperTilt();
  setupVLX();
  setupVoltageMonitor();

  stopMotors();
  startPosition();

  printHelp();
}

void loop() {
  handleSerialInput();

  updateMotorWatchdog();

  updateVLX();
  updateVoltageMonitor();
  updateEating();
}