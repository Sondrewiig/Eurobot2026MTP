#define GRIPPER_SERVO_PIN 19
#define TILT_SERVO_PIN 18

Servo tiltServo;
Servo gripperServo;

int tiltPos = 9;
int gripPos = 0;

// Adjust these angles to your real mechanism.
const int TILT_START = 9;
const int TILT_UP = 70;
const int TILT_DOWN = 9;
const int TILT_NEUTRAL = 70;

const int GRIPPER_OPEN = 0;
const int GRIPPER_ONE_CRATE = 163;
const int GRIPPER_TWO_CRATES = 56;
const int GRIPPER_CLOSED = 100;
const int GRIPPER_NEUTRAL = 0;

bool eatingMode = false;

unsigned long lastEatingMove = 0;
const int eatingInterval = 600;

bool eatingState = false;

const int SERVO_STEP_DELAY = 15;
const int EATING_GRIP_STEP_DELAY = 5;
const int TILT_EATING = 20;

void setupGripperTilt() {
  tiltServo.attach(TILT_SERVO_PIN, 500, 2500);
  gripperServo.attach(GRIPPER_SERVO_PIN, 500, 2500);

  tiltServo.write(tiltPos);
  gripperServo.write(gripPos);

  Serial.println("ACK SERVOS READY");
}

void setTilt(int angle) {
  angle = constrain(angle, 0, 180);

  slowServoMove(tiltServo, tiltPos, angle, SERVO_STEP_DELAY);

  Serial.printf("TEL TILT %d\n", angle);
}

void setGripper(int angle) {
  angle = constrain(angle, 0, 180);

  slowServoMove(gripperServo, gripPos, angle, SERVO_STEP_DELAY);

  Serial.printf("TEL GRIP %d\n", angle);
}

void startPosition() {
  setTilt(TILT_START);
  setGripper(GRIPPER_OPEN);
}

void neutralPosition() {
  setTilt(TILT_NEUTRAL);
  setGripper(GRIPPER_NEUTRAL);
}

void eating() {
  eatingMode = true;
  lastEatingMove = 0;
  Serial.println("TEL EATING STARTED");
}

void stopEating() {
  eatingMode = false;
  Serial.println("TEL EATING STOPPED");
}

void updateEating() {
  if (!eatingMode) {
    return;
  }

  if (millis() - lastEatingMove < eatingInterval) {
    return;
  }

  lastEatingMove = millis();

  eatingState = !eatingState;
  slowServoMove(tiltServo, tiltPos, TILT_EATING, SERVO_STEP_DELAY);

  if (eatingState) {
    slowServoMove(gripperServo, gripPos, GRIPPER_CLOSED, EATING_GRIP_STEP_DELAY);
  } else {
    slowServoMove(gripperServo, gripPos, GRIPPER_OPEN, EATING_GRIP_STEP_DELAY);
  }
}

void twoCrates() {
  setGripper(GRIPPER_TWO_CRATES);
}

void oneCrate() {
  setGripper(GRIPPER_ONE_CRATE);
}

void tiltUp() {
  setTilt(TILT_UP);
}

void tiltDown() {
  setTilt(TILT_DOWN);
}

void releaseGripper() {
  setGripper(GRIPPER_OPEN);
}

void slowServoMove(Servo &servo, int &currentPos, int targetPos, int stepDelay) {
  targetPos = constrain(targetPos, 0, 180);

  if (targetPos > currentPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      servo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      servo.write(pos);
      delay(stepDelay);
    }
  }

  currentPos = targetPos;
}