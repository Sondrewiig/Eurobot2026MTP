#define BATTERY_ADC_PIN 36

const float R1 = 17910.0;
const float R2 = 5570.0;

const float VOLTAGE_DIVIDER_MULTIPLIER = (R1 + R2) / R2;

// Calibrated from your measurement:
// multimeter 11.08 V / ESP32 old reading 10.56 V = 1.049
const float VOLTAGE_CALIBRATION_FACTOR = 1.049;

const float LOW_BATTERY_LIMIT_V = 10.20;
const float CRITICAL_BATTERY_LIMIT_V = 9.90;

const float ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX = 4095;

bool voltageContinuous = false;
unsigned long lastVoltageRead = 0;
const unsigned long voltageInterval = 1000;

void setupVoltageMonitor() {
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  Serial.println("ACK VOLTAGE MONITOR READY");
}

float readBatteryVoltage() {
  const int samples = 10;
  long adcSum = 0;

  for (int i = 0; i < samples; i++) {
    adcSum += analogRead(BATTERY_ADC_PIN);
    delay(2);
  }

  float adcRaw = adcSum / (float)samples;
  float adcVoltage = (adcRaw / ADC_MAX) * ADC_REF_VOLTAGE;
  float batteryVoltage = adcVoltage * VOLTAGE_DIVIDER_MULTIPLIER;
  batteryVoltage *= VOLTAGE_CALIBRATION_FACTOR;

  return batteryVoltage;
}

void updateVoltageMonitor() {
  if (voltageContinuous && millis() - lastVoltageRead >= voltageInterval) {
    lastVoltageRead = millis();
    printBatteryVoltage();
  }
}

void printBatteryVoltage() {
  float voltage = readBatteryVoltage();

  Serial.printf("TEL BAT %.2f\n", voltage);

  if (voltage <= LOW_BATTERY_LIMIT_V) {
    Serial.printf("TEL WARNING BATTERY_LOW %.2f\n", voltage);
  }

  if (voltage <= CRITICAL_BATTERY_LIMIT_V) {
    Serial.printf("TEL WARNING BATTERY_CRITICAL %.2f\n", voltage);
  }
}

void toggleVoltageMonitor() {
  voltageContinuous = !voltageContinuous;

  if (voltageContinuous) {
    Serial.println("ACK VOLTAGE STREAM STARTED");
  } else {
    Serial.println("ACK VOLTAGE STREAM STOPPED");
  }
}
