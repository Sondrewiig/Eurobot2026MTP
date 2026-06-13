extern bool eatingMode;

int parseInts(String text, int startIndex, int *out, int maxCount) {
  String tail = text.substring(startIndex);
  tail.trim();

  char buffer[120];
  tail.toCharArray(buffer, sizeof(buffer));

  int count = 0;
  char *token = strtok(buffer, " ");

  while (token != NULL && count < maxCount) {
    out[count] = atoi(token);
    count++;
    token = strtok(NULL, " ");
  }

  return count;
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        handleCommand(input);
        input = "";
      }
    } else {
      input += c;

      if (input.length() > 120) {
        input = "";
        Serial.println("ERR input too long");
      }
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) {
    return;
  }

  Serial.print("RX ");
  Serial.println(cmd);

  if (cmd != "eating" && eatingMode) {
    stopEating();
  }

  // =============================
  // Basic commands
  // =============================

  if (cmd == "ping") {
    Serial.println("ACK PING");
  }

  else if (cmd == "version") {
    Serial.print("TEL VERSION ");
    Serial.println(NINJA_ESP32_VERSION);
  }

  else if (cmd == "help") {
    printHelp();
  }

  else if (cmd == "settings") {
    printMotorSettings();
  }

  else if (cmd == "resetsettings") {
    resetMotorSettings();
    Serial.println("ACK RESETSETTINGS");
    printMotorSettings();
  }

  // =============================
  // Motor tuning commands
  // =============================

  else if (cmd.startsWith("trim ")) {
    int values[2];
    int count = parseInts(cmd, 5, values, 2);

    if (count == 2) {
      leftTrim = constrain(values[0], -100, 100);
      rightTrim = constrain(values[1], -100, 100);

      Serial.printf("ACK TRIM %d %d\n", leftTrim, rightTrim);
    } else {
      Serial.println("ERR use: trim <left_trim> <right_trim>");
    }
  }

  else if (cmd.startsWith("minpwm ")) {
    int value = cmd.substring(7).toInt();
    motorMinPwm = constrain(value, 0, 255);

    if (motorMinPwm > motorMaxPwm) {
      motorMinPwm = motorMaxPwm;
    }

    Serial.printf("ACK MINPWM %d\n", motorMinPwm);
  }

  else if (cmd.startsWith("maxpwm ")) {
    int value = cmd.substring(7).toInt();
    motorMaxPwm = constrain(value, 0, 255);

    if (motorMaxPwm < motorMinPwm) {
      motorMaxPwm = motorMinPwm;
    }

    Serial.printf("ACK MAXPWM %d\n", motorMaxPwm);
  }

  else if (cmd.startsWith("ramp ")) {
    int value = cmd.substring(5).toInt();
    motorRampStep = constrain(value, 0, 255);

    Serial.printf("ACK RAMP %d\n", motorRampStep);
  }

  else if (cmd == "watchdog on") {
    watchdogEnabled = true;
    watchdogStopPrinted = false;

    Serial.println("ACK WATCHDOG ON");
  }

  else if (cmd == "watchdog off") {
    watchdogEnabled = false;
    watchdogStopPrinted = false;

    Serial.println("ACK WATCHDOG OFF");
  }

  else if (cmd.startsWith("timeout ")) {
    int value = cmd.substring(8).toInt();
    commandTimeoutMs = constrain(value, 50, 5000);

    Serial.printf("ACK TIMEOUT %lu\n", commandTimeoutMs);
  }

  else if (cmd.startsWith("testmotor ")) {
    int values[4];
    int count = parseInts(cmd, 10, values, 4);

    if (count == 4) {
      int motor = values[0];
      int startPwm = values[1];
      int endPwm = values[2];
      int stepPwm = values[3];

      runMotorTest(motor, startPwm, endPwm, stepPwm);
    } else {
      Serial.println("ERR use: testmotor <motor> <start> <end> <step>");
      Serial.println("Example: testmotor 1 30 120 10");
    }
  }

  // =============================
  // Motor drive commands
  // =============================

  else if (cmd == "forward") {
    markDriveCommand();
    driveForward();
  }

  else if (cmd == "backward") {
    markDriveCommand();
    driveBackward();
  }

  else if (cmd == "left") {
    markDriveCommand();
    turnLeft();
  }

  else if (cmd == "right") {
    markDriveCommand();
    turnRight();
  }

  else if (cmd == "reverseleft") {
    markDriveCommand();
    reverseLeft();
  }

  else if (cmd == "reverseright") {
    markDriveCommand();
    reverseRight();
  }

  else if (cmd == "stop") {
    stopMotors();
  }

  // Direct raw motor test.
  // These bypass trim/minpwm/maxpwm/ramp.
  else if (cmd.startsWith("m1 ")) {
    int speed = cmd.substring(3).toInt();

    markDriveCommand();
    setMotorRaw(1, speed);

    Serial.printf("ACK M1 RAW %d\n", speed);
  }

  else if (cmd.startsWith("m2 ")) {
    int speed = cmd.substring(3).toInt();

    markDriveCommand();
    setMotorRaw(2, speed);

    Serial.printf("ACK M2 RAW %d\n", speed);
  }

  // Main tuned command used by ROS2 bridge.
  // Applies trim, minpwm, maxpwm, and ramp.
  else if (cmd.startsWith("motors ")) {
    int values[2];
    int count = parseInts(cmd, 7, values, 2);

    if (count == 2) {
      int left = values[0];
      int right = values[1];

      markDriveCommand();
      setMotorsTuned(left, right);

      Serial.printf("ACK MOTORS CMD %d %d\n", left, right);
    } else {
      Serial.println("ERR use: motors <left> <right>");
    }
  }

  // =============================
  // Gripper and tilt commands
  // =============================

  else if (cmd == "startposition") {
    startPosition();
    Serial.println("ACK STARTPOSITION");
  }

  else if (cmd == "neutralposition") {
    neutralPosition();
    Serial.println("ACK NEUTRALPOSITION");
  }

  else if (cmd == "eating") {
    eating();
    Serial.println("ACK EATING");
  }

  else if (cmd == "stopeating") {
    stopEating();
    Serial.println("ACK STOPEATING");
  }

  else if (cmd == "twocrates") {
    twoCrates();
    Serial.println("ACK TWOCRATES");
  }

  else if (cmd == "onecrate") {
    oneCrate();
    Serial.println("ACK ONECRATE");
  }

  else if (cmd == "tiltup") {
    tiltUp();
    Serial.println("ACK TILTUP");
  }

  else if (cmd == "tiltdown") {
    tiltDown();
    Serial.println("ACK TILTDOWN");
  }

  else if (cmd == "release") {
    releaseGripper();
    Serial.println("ACK RELEASE");
  }

  else if (cmd.startsWith("tilt ")) {
    int angle = cmd.substring(5).toInt();

    setTilt(angle);

    Serial.printf("ACK TILT %d\n", angle);
  }

  else if (cmd.startsWith("grip ")) {
    int angle = cmd.substring(5).toInt();

    setGripper(angle);

    Serial.printf("ACK GRIP %d\n", angle);
  }

  // =============================
  // VLX / ToF commands
  // =============================

  else if (cmd == "vlx") {
    toggleVLX();
  }

  else if (cmd == "vlxstatus") {
    printVLXStatus();
  }

  // =============================
  // Voltage commands
  // =============================

  else if (cmd == "voltage") {
    printBatteryVoltage();
  }

  else if (cmd == "voltagestream") {
    toggleVoltageMonitor();
  }

  else {
    Serial.println("ERR unknown command. Type help.");
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Available commands:");

  Serial.println();
  Serial.println("Connection:");
  Serial.println("  ping");
  Serial.println("  version");
  Serial.println("  help");
  Serial.println("  settings");
  Serial.println("  resetsettings");

  Serial.println();
  Serial.println("Motor tuning:");
  Serial.println("  trim <left_trim> <right_trim>");
  Serial.println("  minpwm <0-255>");
  Serial.println("  maxpwm <0-255>");
  Serial.println("  ramp <0-255>");
  Serial.println("  watchdog on");
  Serial.println("  watchdog off");
  Serial.println("  timeout <50-5000>");
  Serial.println("  testmotor <motor> <start> <end> <step>");

  Serial.println();
  Serial.println("Motors:");
  Serial.println("  forward");
  Serial.println("  backward");
  Serial.println("  left");
  Serial.println("  right");
  Serial.println("  reverseleft");
  Serial.println("  reverseright");
  Serial.println("  stop");
  Serial.println("  m1 <-255 to 255>      raw single motor");
  Serial.println("  m2 <-255 to 255>      raw single motor");
  Serial.println("  motors <left> <right> tuned dual motor");

  Serial.println();
  Serial.println("Gripper/Tilt:");
  Serial.println("  startposition");
  Serial.println("  neutralposition");
  Serial.println("  eating");
  Serial.println("  stopeating");
  Serial.println("  twocrates");
  Serial.println("  onecrate");
  Serial.println("  tiltup");
  Serial.println("  tiltdown");
  Serial.println("  release");
  Serial.println("  tilt <0-180>");
  Serial.println("  grip <0-180>");

  Serial.println();
  Serial.println("VLX:");
  Serial.println("  vlx");
  Serial.println("  vlxstatus");

  Serial.println();
  Serial.println("Voltage:");
  Serial.println("  voltage");
  Serial.println("  voltagestream");

  Serial.println();
}