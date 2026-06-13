#define TOF1_XSHUT 23
#define TOF2_XSHUT 17
#define TOF3_XSHUT 16
#define TOF4_XSHUT 13
#define TOF5_XSHUT 33
#define TOF6_XSHUT 32

#define I2C_SDA 21
#define I2C_SCL 22

#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31
#define TOF3_ADDR 0x32
#define TOF4_ADDR 0x33
#define TOF5_ADDR 0x34
#define TOF6_ADDR 0x35

Adafruit_VL53L0X tof1;
Adafruit_VL53L0X tof2;
Adafruit_VL53L0X tof3;
Adafruit_VL53L0X tof4;
Adafruit_VL53L0X tof5;
Adafruit_VL53L0X tof6;

bool tofOk[6] = {false, false, false, false, false, false};

bool vlxContinuous = false;
unsigned long lastVLXRead = 0;
const unsigned long vlxInterval = 500;

const int WALL_LIMIT_MM = 120;
const int CRATE_POSITION_LIMIT_MM = 80;

void setupVLX() {
  pinMode(TOF1_XSHUT, OUTPUT);
  pinMode(TOF2_XSHUT, OUTPUT);
  pinMode(TOF3_XSHUT, OUTPUT);
  pinMode(TOF4_XSHUT, OUTPUT);
  pinMode(TOF5_XSHUT, OUTPUT);
  pinMode(TOF6_XSHUT, OUTPUT);

  digitalWrite(TOF1_XSHUT, LOW);
  digitalWrite(TOF2_XSHUT, LOW);
  digitalWrite(TOF3_XSHUT, LOW);
  digitalWrite(TOF4_XSHUT, LOW);
  digitalWrite(TOF5_XSHUT, LOW);
  digitalWrite(TOF6_XSHUT, LOW);

  delay(50);

  Wire.begin(I2C_SDA, I2C_SCL);

  tofOk[0] = initVLX(tof1, TOF1_XSHUT, TOF1_ADDR, "VLX1");
  tofOk[1] = initVLX(tof2, TOF2_XSHUT, TOF2_ADDR, "VLX2");
  tofOk[2] = initVLX(tof3, TOF3_XSHUT, TOF3_ADDR, "VLX3");
  tofOk[3] = initVLX(tof4, TOF4_XSHUT, TOF4_ADDR, "VLX4");
  tofOk[4] = initVLX(tof5, TOF5_XSHUT, TOF5_ADDR, "VLX5");
  tofOk[5] = initVLX(tof6, TOF6_XSHUT, TOF6_ADDR, "VLX6");

  Serial.println("ACK VLX SETUP DONE");
}

void updateVLX() {
  if (vlxContinuous && millis() - lastVLXRead >= vlxInterval) {
    lastVLXRead = millis();
    printVLXStatus();
  }
}

void toggleVLX() {
  vlxContinuous = !vlxContinuous;

  if (vlxContinuous) {
    Serial.println("ACK VLX STREAM STARTED");
  } else {
    Serial.println("ACK VLX STREAM STOPPED");
  }
}

bool initVLX(Adafruit_VL53L0X &sensor, int xshutPin, uint8_t address, const char *name) {
  digitalWrite(xshutPin, HIGH);
  delay(150);

  if (!sensor.begin(0x29, false, &Wire)) {
    Serial.printf("ERR %s NOT_FOUND\n", name);
    return false;
  }

  sensor.setAddress(address);
  delay(50);

  Serial.printf("ACK %s OK ADDR 0x%X\n", name, address);
  return true;
}

int readVLXDistance(Adafruit_VL53L0X &sensor, bool ok) {
  if (!ok) {
    return -2;
  }

  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  int distance = measure.RangeMilliMeter;

  if (measure.RangeStatus == 4 || distance <= 10 || distance > 2000) {
    return -1;
  }

  return distance;
}

void printVLXStatus() {
  int d1 = readVLXDistance(tof1, tofOk[0]);
  int d2 = readVLXDistance(tof2, tofOk[1]);
  int d3 = readVLXDistance(tof3, tofOk[2]);
  int d4 = readVLXDistance(tof4, tofOk[3]);
  int d5 = readVLXDistance(tof5, tofOk[4]);
  int d6 = readVLXDistance(tof6, tofOk[5]);

  Serial.printf("TEL VLX %d %d %d %d %d %d\n", d1, d2, d3, d4, d5, d6);

  printVLXReading("VLX1", d1);
  printVLXReading("VLX2", d2);
  printVLXReading("VLX3", d3);
  printVLXReading("VLX4", d4);
  printVLXReading("VLX5", d5);
  printVLXReading("VLX6", d6);

  if (d5 > 0 && d5 < WALL_LIMIT_MM) {
    Serial.println("TEL WARNING LEFT_WALL_TOO_CLOSE");
  }

  if (d6 > 0 && d6 < WALL_LIMIT_MM) {
    Serial.println("TEL WARNING RIGHT_WALL_TOO_CLOSE");
  }

  if (
    d1 > 0 && d1 < CRATE_POSITION_LIMIT_MM &&
    d2 > 0 && d2 < CRATE_POSITION_LIMIT_MM &&
    d3 > 0 && d3 < CRATE_POSITION_LIMIT_MM &&
    d4 > 0 && d4 < CRATE_POSITION_LIMIT_MM
  ) {
    Serial.println("TEL CRATES_IN_POSITION");
  }
}

void printVLXReading(const char *name, int distance) {
  if (distance == -2) {
    Serial.printf("%s: not connected\n", name);
  } else if (distance == -1) {
    Serial.printf("%s: invalid / out of range\n", name);
  } else {
    Serial.printf("%s: %d mm\n", name, distance);
  }
}