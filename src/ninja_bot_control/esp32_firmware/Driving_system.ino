#define LEFT_PWM  27
#define LEFT_DIR  14

#define RIGHT_PWM 25
#define RIGHT_DIR 26

const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// Change these if a motor spins the wrong way.
#define LEFT_INVERT  false
#define RIGHT_INVERT true

const int DEFAULT_SPEED = 180;
const int TURN_SPEED = 160;

void setupMotors() {
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // ESP32 Arduino core 3.x LEDC API.
  // ledcAttach(pin, frequency, resolution)
  bool leftOk = ledcAttach(LEFT_PWM, PWM_FREQ, PWM_RES);
  bool rightOk = ledcAttach(RIGHT_PWM, PWM_FREQ, PWM_RES);

  if (!leftOk) {
    Serial.println("ERR LEFT_PWM LEDC attach failed");
  }

  if (!rightOk) {
    Serial.println("ERR RIGHT_PWM LEDC attach failed");
  }

  stopMotors();

  Serial.println("ACK MOTORS READY");
}

void markDriveCommand() {
  lastDriveCommandMs = millis();
  watchdogStopPrinted = false;
}

void updateMotorWatchdog() {
  if (!watchdogEnabled) {
    return;
  }

  if (!motorsAreActive) {
    return;
  }

  if (millis() - lastDriveCommandMs > commandTimeoutMs) {
    stopMotorsQuiet();

    if (!watchdogStopPrinted) {
      Serial.println("ERR WATCHDOG motor command timeout");
      watchdogStopPrinted = true;
    }
  }
}

int applyMotorLimits(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm == 0) {
    return 0;
  }

  int sign = pwm > 0 ? 1 : -1;
  int magnitude = abs(pwm);

  if (motorMaxPwm > 0 && magnitude > motorMaxPwm) {
    magnitude = motorMaxPwm;
  }

  if (motorMinPwm > 0 && magnitude > 0 && magnitude < motorMinPwm) {
    magnitude = motorMinPwm;
  }

  return sign * magnitude;
}

int rampToward(int current, int target) {
  if (motorRampStep <= 0) {
    return target;
  }

  if (current < target) {
    current += motorRampStep;

    if (current > target) {
      current = target;
    }
  }

  else if (current > target) {
    current -= motorRampStep;

    if (current < target) {
      current = target;
    }
  }

  return current;
}

void setMotorRaw(int motor, int speed) {
  speed = constrain(speed, -255, 255);

  if (motor == 1) {
    currentLeftPwm = speed;

    if (speed == 0) {
      ledcWrite(LEFT_PWM, 0);
    } else {
      bool forward = speed > 0;

      if (LEFT_INVERT) {
        forward = !forward;
      }

      digitalWrite(LEFT_DIR, forward ? HIGH : LOW);
      ledcWrite(LEFT_PWM, abs(speed));
    }
  }

  else if (motor == 2) {
    currentRightPwm = speed;

    if (speed == 0) {
      ledcWrite(RIGHT_PWM, 0);
    } else {
      bool forward = speed > 0;

      if (RIGHT_INVERT) {
        forward = !forward;
      }

      digitalWrite(RIGHT_DIR, forward ? HIGH : LOW);
      ledcWrite(RIGHT_PWM, abs(speed));
    }
  }

  motorsAreActive = (currentLeftPwm != 0 || currentRightPwm != 0);
}

void setMotorsTuned(int leftCommand, int rightCommand) {
  int leftTarget = leftCommand;
  int rightTarget = rightCommand;

  if (leftTarget != 0) {
    leftTarget += leftTrim;
  }

  if (rightTarget != 0) {
    rightTarget += rightTrim;
  }

  leftTarget = applyMotorLimits(leftTarget);
  rightTarget = applyMotorLimits(rightTarget);

  int leftOutput = rampToward(currentLeftPwm, leftTarget);
  int rightOutput = rampToward(currentRightPwm, rightTarget);

  setMotorRaw(1, leftOutput);
  setMotorRaw(2, rightOutput);

  Serial.printf("TEL MOTORS OUT %d %d TARGET %d %d\n",
                leftOutput,
                rightOutput,
                leftTarget,
                rightTarget);
}

void driveForward() {
  setMotorsTuned(DEFAULT_SPEED, DEFAULT_SPEED);
  Serial.println("ACK FORWARD");
}

void driveBackward() {
  setMotorsTuned(-DEFAULT_SPEED, -DEFAULT_SPEED);
  Serial.println("ACK BACKWARD");
}

void turnLeft() {
  setMotorsTuned(-TURN_SPEED, TURN_SPEED);
  Serial.println("ACK LEFT");
}

void turnRight() {
  setMotorsTuned(TURN_SPEED, -TURN_SPEED);
  Serial.println("ACK RIGHT");
}

void reverseLeft() {
  setMotorsTuned(-TURN_SPEED / 2, -TURN_SPEED);
  Serial.println("ACK REVERSELEFT");
}

void reverseRight() {
  setMotorsTuned(-TURN_SPEED, -TURN_SPEED / 2);
  Serial.println("ACK REVERSERIGHT");
}

void stopMotorsQuiet() {
  ledcWrite(LEFT_PWM, 0);
  ledcWrite(RIGHT_PWM, 0);

  currentLeftPwm = 0;
  currentRightPwm = 0;

  motorsAreActive = false;
}

void stopMotors() {
  stopMotorsQuiet();
  Serial.println("ACK STOP");
}

void printMotorSettings() {
  Serial.println();
  Serial.println("TEL SETTINGS");
  Serial.printf("TEL WATCHDOG %s\n", watchdogEnabled ? "ON" : "OFF");
  Serial.printf("TEL TIMEOUT_MS %lu\n", commandTimeoutMs);
  Serial.printf("TEL MINPWM %d\n", motorMinPwm);
  Serial.printf("TEL MAXPWM %d\n", motorMaxPwm);
  Serial.printf("TEL TRIM %d %d\n", leftTrim, rightTrim);
  Serial.printf("TEL RAMP %d\n", motorRampStep);
  Serial.printf("TEL CURRENT_PWM %d %d\n", currentLeftPwm, currentRightPwm);
  Serial.println();
}

void resetMotorSettings() {
  motorMinPwm = DEFAULT_MOTOR_MIN_PWM;
  motorMaxPwm = DEFAULT_MOTOR_MAX_PWM;

  leftTrim = 0;
  rightTrim = 0;

  motorRampStep = 0;

  watchdogEnabled = true;
  commandTimeoutMs = 300;

  stopMotorsQuiet();
}

void runMotorTest(int motor, int startPwm, int endPwm, int stepPwm) {
  if (motor != 1 && motor != 2) {
    Serial.println("ERR motor must be 1 or 2");
    return;
  }

  if (stepPwm == 0) {
    Serial.println("ERR step cannot be 0");
    return;
  }

  if (startPwm < endPwm && stepPwm < 0) {
    stepPwm = -stepPwm;
  }

  if (startPwm > endPwm && stepPwm > 0) {
    stepPwm = -stepPwm;
  }

  Serial.printf("ACK TESTMOTOR motor=%d start=%d end=%d step=%d\n",
                motor,
                startPwm,
                endPwm,
                stepPwm);

  int pwm = startPwm;

  while (true) {
    pwm = constrain(pwm, -255, 255);

    markDriveCommand();
    setMotorRaw(motor, pwm);

    Serial.printf("TEL TESTMOTOR %d PWM %d\n", motor, pwm);

    delay(700);

    if (pwm == endPwm) {
      break;
    }

    pwm += stepPwm;

    if (stepPwm > 0 && pwm > endPwm) {
      pwm = endPwm;
    }

    if (stepPwm < 0 && pwm < endPwm) {
      pwm = endPwm;
    }
  }

  stopMotors();
  Serial.println("ACK TESTMOTOR DONE");
}