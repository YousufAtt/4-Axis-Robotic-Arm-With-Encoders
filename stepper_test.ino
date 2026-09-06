#include <Wire.h>
#include <EEPROM.h>

// ============================================================
// AXIS CONFIGURATION
// ============================================================

struct AxisConfig {
  const char* name;
  byte stepPin;
  byte dirPin;
  byte enablePin;
  byte muxChannel;

  // Motor rotations per joint rotation: 9 / 2 = 4.5
  float gearReduction;

  // Change to false if positive commands move the wrong way.
  bool dirHighIncreasesAngle;
};

AxisConfig axes[] = {
  // Name         STEP DIR EN  MUX  Ratio      Direction
  {"YAW_X",       54, 55, 38, 0,   9.0 / 2.0, true},
  {"SHOULDER_Y",  60, 61, 56, 1,   9.0 / 2.0, true},
  {"ELBOW_Z",     46, 48, 62, 2,   9.0 / 2.0, true}
};

const byte AXIS_COUNT = 3;
byte selectedAxis = 0;

// ============================================================
// LIMIT SWITCHES
// ============================================================
//
// C/COM -> RAMPS -
// NC    -> RAMPS S
// +     -> unused
//
// LOW  = clear
// HIGH = pressed, disconnected, or broken
// ============================================================

struct LimitConfig {
  const char* name;
  byte pin;
};

LimitConfig limits[] = {
  {"X_MIN", 3},
  {"X_MAX", 2},
  {"Y_MIN", 14},
  {"Y_MAX", 15}
};

const byte LIMIT_COUNT = 4;
const byte LIMIT_TRIGGERED_STATE = HIGH;

int previousLimitState[LIMIT_COUNT] = {
  -1, -1, -1, -1
};

bool limitStopLatched = false;

// ============================================================
// TCA9548A AND AS5600
// ============================================================

const byte TCA_ADDR = 0x70;
const byte AS5600_ADDR = 0x36;

const byte AS5600_ANGLE_REG = 0x0E;
const byte AS5600_STATUS_REG = 0x0B;

// Encoder position tracking
int currentRaw[AXIS_COUNT] = {0, 0, 0};
int previousRaw[AXIS_COUNT] = {0, 0, 0};

long accumulatedMotorCounts[AXIS_COUNT] = {
  0, 0, 0
};

bool encoderInitialized[AXIS_COUNT] = {
  false, false, false
};

// ============================================================
// EEPROM CALIBRATION
// ============================================================

struct CalibrationData {
  unsigned int marker;
  unsigned int zeroRaw[AXIS_COUNT];
  byte validMask;
};

CalibrationData calibration;

const int EEPROM_ADDRESS = 0;
const unsigned int EEPROM_MARKER = 9202;

// ============================================================
// MOVEMENT SETTINGS
// ============================================================

// Larger value means slower movement.
const unsigned long STEP_INTERVAL_US = 3000;

// This tolerance is measured at the joint, not motor.
float jointToleranceDegrees = 1.0;

float targetJointDegrees = 0.0;

bool motionActive = false;

unsigned long previousStepTime = 0;
unsigned long previousStatusTime = 0;

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  Wire.begin();
  Wire.setClock(100000);

  for (byte i = 0; i < AXIS_COUNT; i++) {
    pinMode(axes[i].stepPin, OUTPUT);
    pinMode(axes[i].dirPin, OUTPUT);
    pinMode(axes[i].enablePin, OUTPUT);

    digitalWrite(axes[i].stepPin, LOW);
    digitalWrite(axes[i].dirPin, LOW);
    digitalWrite(axes[i].enablePin, HIGH);
  }

  for (byte i = 0; i < LIMIT_COUNT; i++) {
    pinMode(limits[i].pin, INPUT_PULLUP);
  }

  loadCalibration();

  delay(500);

  for (byte i = 0; i < AXIS_COUNT; i++) {
    initializeEncoder(i);
  }

  enableSelectedMotor();

  Serial.println();
  Serial.println("Gear-aware robotic arm controller");
  Serial.println("Gear reduction: 9:2 = 4.5:1");
  Serial.println();
  Serial.println("Encoder channels:");
  Serial.println("  Yaw      = SD0 / SC0");
  Serial.println("  Shoulder = SD1 / SC1");
  Serial.println("  Elbow    = SD2 / SC2");
  Serial.println();

  printLimits();
  printAllEncoders();

  if (anyLimitTriggered()) {
    limitStopLatched = true;

    Serial.println(
      "WARNING: A limit switch is triggered."
    );

    printTriggeredLimits();
  }

  for (byte i = 0; i < LIMIT_COUNT; i++) {
    previousLimitState[i] =
      digitalRead(limits[i].pin);
  }

  setTargetToCurrentPosition();
  printHelp();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  if (motionActive && anyLimitTriggered()) {
    emergencyLimitStop();
  }

  reportLimitChanges();
  handleSerial();

  if (motionActive) {
    updatePositionControl();
  }

  if (millis() - previousStatusTime >= 500) {
    previousStatusTime = millis();
    printLiveStatus();
  }
}

// ============================================================
// MULTIPLEXER
// ============================================================

bool selectMuxChannel(byte channel) {
  if (channel > 7) {
    return false;
  }

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);

  return Wire.endTransmission() == 0;
}

// ============================================================
// AS5600 READING
// ============================================================

bool readAS5600Raw(byte axis, int &raw) {
  if (!selectMuxChannel(axes[axis].muxChannel)) {
    return false;
  }

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_REG);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  byte received = Wire.requestFrom(
    (byte)AS5600_ADDR,
    (byte)2
  );

  if (received != 2 || Wire.available() < 2) {
    return false;
  }

  int highByte = Wire.read();
  int lowByte = Wire.read();

  raw = ((highByte & 0x0F) << 8) | lowByte;

  return true;
}

bool readMagnetDetected(byte axis, bool &detected) {
  if (!selectMuxChannel(axes[axis].muxChannel)) {
    return false;
  }

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_STATUS_REG);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  byte received = Wire.requestFrom(
    (byte)AS5600_ADDR,
    (byte)1
  );

  if (received != 1 || !Wire.available()) {
    return false;
  }

  byte status = Wire.read();

  detected = (status & 0x20) != 0;

  return true;
}

// Convert an AS5600 wrap into a signed change.
int wrappedEncoderDelta(int delta) {
  if (delta > 2048) {
    delta -= 4096;
  }

  if (delta < -2048) {
    delta += 4096;
  }

  return delta;
}

bool axisCalibrated(byte axis) {
  return calibration.validMask & (1 << axis);
}

bool initializeEncoder(byte axis) {
  int raw;

  if (!readAS5600Raw(axis, raw)) {
    encoderInitialized[axis] = false;
    return false;
  }

  currentRaw[axis] = raw;
  previousRaw[axis] = raw;

  if (axisCalibrated(axis)) {
    int difference =
      raw - calibration.zeroRaw[axis];

    accumulatedMotorCounts[axis] =
      wrappedEncoderDelta(difference);
  } else {
    accumulatedMotorCounts[axis] = 0;
  }

  encoderInitialized[axis] = true;

  return true;
}

bool updateEncoder(byte axis) {
  int raw;

  if (!readAS5600Raw(axis, raw)) {
    return false;
  }

  if (!encoderInitialized[axis]) {
    currentRaw[axis] = raw;
    previousRaw[axis] = raw;
    accumulatedMotorCounts[axis] = 0;
    encoderInitialized[axis] = true;

    return true;
  }

  int delta = raw - previousRaw[axis];

  delta = wrappedEncoderDelta(delta);

  accumulatedMotorCounts[axis] += delta;

  previousRaw[axis] = raw;
  currentRaw[axis] = raw;

  return true;
}

float getMotorAngle(byte axis) {
  return accumulatedMotorCounts[axis] *
         360.0 / 4096.0;
}

float getJointAngle(byte axis) {
  return getMotorAngle(axis) /
         axes[axis].gearReduction;
}

bool updateAndGetJointAngle(
  byte axis,
  float &jointAngle
) {
  if (!updateEncoder(axis)) {
    return false;
  }

  jointAngle = getJointAngle(axis);

  return true;
}

// ============================================================
// ENCODER DISPLAY
// ============================================================

void printOneEncoder(byte axis) {
  bool magnetDetected;

  Serial.print("  ");
  Serial.print(axes[axis].name);

  Serial.print(" | CH");
  Serial.print(axes[axis].muxChannel);

  if (!updateEncoder(axis)) {
    Serial.println(" | I2C READ FAILED");
    return;
  }

  Serial.print(" | Raw: ");
  Serial.print(currentRaw[axis]);

  Serial.print(" | Motor: ");
  Serial.print(getMotorAngle(axis), 2);
  Serial.print(" deg");

  Serial.print(" | Joint: ");
  Serial.print(getJointAngle(axis), 2);
  Serial.print(" deg");

  Serial.print(" | Ratio: ");
  Serial.print(axes[axis].gearReduction, 2);
  Serial.print(":1");

  Serial.print(" | Calibration: ");

  if (axisCalibrated(axis)) {
    Serial.print("SAVED");
  } else {
    Serial.print("NOT SET");
  }

  if (readMagnetDetected(axis, magnetDetected)) {
    Serial.print(" | Magnet: ");

    if (magnetDetected) {
      Serial.println("DETECTED");
    } else {
      Serial.println("NOT DETECTED");
    }
  } else {
    Serial.println(" | Magnet status failed");
  }
}

void printAllEncoders() {
  Serial.println("Encoder measurements:");

  for (byte i = 0; i < AXIS_COUNT; i++) {
    printOneEncoder(i);
  }

  Serial.println();
}

// ============================================================
// LIMIT SWITCHES
// ============================================================

bool isLimitTriggered(byte pin) {
  return digitalRead(pin) ==
         LIMIT_TRIGGERED_STATE;
}

bool anyLimitTriggered() {
  for (byte i = 0; i < LIMIT_COUNT; i++) {
    if (isLimitTriggered(limits[i].pin)) {
      return true;
    }
  }

  return false;
}

void printOneLimit(byte index) {
  int raw = digitalRead(limits[index].pin);

  Serial.print("  ");
  Serial.print(limits[index].name);
  Serial.print(" pin ");
  Serial.print(limits[index].pin);
  Serial.print(" | raw=");
  Serial.print(raw);
  Serial.print(" | ");

  if (raw == LIMIT_TRIGGERED_STATE) {
    Serial.println("TRIGGERED");
  } else {
    Serial.println("CLEAR");
  }
}

void printLimits() {
  Serial.println("Limit switch states:");

  for (byte i = 0; i < LIMIT_COUNT; i++) {
    printOneLimit(i);
  }

  Serial.println();
}

void printTriggeredLimits() {
  Serial.println("Triggered switches:");

  bool found = false;

  for (byte i = 0; i < LIMIT_COUNT; i++) {
    if (isLimitTriggered(limits[i].pin)) {
      found = true;

      Serial.print("  ");
      Serial.println(limits[i].name);
    }
  }

  if (!found) {
    Serial.println("  None");
  }
}

void reportLimitChanges() {
  for (byte i = 0; i < LIMIT_COUNT; i++) {
    int currentState =
      digitalRead(limits[i].pin);

    if (
      currentState !=
      previousLimitState[i]
    ) {
      previousLimitState[i] =
        currentState;

      printOneLimit(i);
    }
  }
}

void emergencyLimitStop() {
  motionActive = false;
  limitStopLatched = true;

  for (byte i = 0; i < AXIS_COUNT; i++) {
    digitalWrite(axes[i].stepPin, LOW);
  }

  // Keep the selected motor holding.
  digitalWrite(
    axes[selectedAxis].enablePin,
    LOW
  );

  Serial.println();
  Serial.println("LIMIT STOP: MOTION STOPPED");

  printTriggeredLimits();

  Serial.println(
    "Release all switches and enter: reset"
  );
}

void resetLimitLatch() {
  if (anyLimitTriggered()) {
    Serial.println(
      "Cannot reset: a switch remains triggered."
    );

    printTriggeredLimits();
    return;
  }

  limitStopLatched = false;
  enableSelectedMotor();

  Serial.println("Limit-stop latch cleared.");
}

// ============================================================
// EEPROM CALIBRATION
// ============================================================

void loadCalibration() {
  EEPROM.get(EEPROM_ADDRESS, calibration);

  bool invalid =
    calibration.marker != EEPROM_MARKER;

  for (byte i = 0; i < AXIS_COUNT; i++) {
    if (calibration.zeroRaw[i] > 4095) {
      invalid = true;
    }
  }

  if (invalid) {
    calibration.marker = EEPROM_MARKER;
    calibration.validMask = 0;

    for (byte i = 0; i < AXIS_COUNT; i++) {
      calibration.zeroRaw[i] = 0;
    }

    saveCalibration();
  }
}

void saveCalibration() {
  EEPROM.put(EEPROM_ADDRESS, calibration);
}

void calibrateSelectedAxis() {
  bool magnetDetected;
  int raw;

  motionActive = false;

  if (!readMagnetDetected(
        selectedAxis,
        magnetDetected
      )) {
    Serial.println(
      "Calibration failed: encoder not responding."
    );

    return;
  }

  if (!magnetDetected) {
    Serial.println(
      "Calibration failed: magnet not detected."
    );

    return;
  }

  if (!readAS5600Raw(selectedAxis, raw)) {
    Serial.println(
      "Calibration failed: angle read failed."
    );

    return;
  }

  calibration.zeroRaw[selectedAxis] = raw;
  calibration.validMask |= (1 << selectedAxis);

  saveCalibration();

  currentRaw[selectedAxis] = raw;
  previousRaw[selectedAxis] = raw;

  accumulatedMotorCounts[selectedAxis] = 0;
  encoderInitialized[selectedAxis] = true;

  targetJointDegrees = 0.0;

  Serial.print("Calibrated ");
  Serial.print(axes[selectedAxis].name);
  Serial.println(" at joint zero.");

  Serial.print("Saved raw zero: ");
  Serial.println(raw);
}

void printCalibration() {
  Serial.println("Saved calibration:");

  for (byte i = 0; i < AXIS_COUNT; i++) {
    Serial.print("  ");
    Serial.print(axes[i].name);
    Serial.print(" | Zero raw: ");
    Serial.print(calibration.zeroRaw[i]);
    Serial.print(" | ");

    if (axisCalibrated(i)) {
      Serial.println("VALID");
    } else {
      Serial.println("NOT CALIBRATED");
    }
  }
}

// ============================================================
// MOTOR ENABLE AND SELECTION
// ============================================================

void enableSelectedMotor() {
  for (byte i = 0; i < AXIS_COUNT; i++) {
    digitalWrite(axes[i].enablePin, HIGH);
  }

  digitalWrite(
    axes[selectedAxis].enablePin,
    LOW
  );
}

void disableAllMotors() {
  motionActive = false;

  for (byte i = 0; i < AXIS_COUNT; i++) {
    digitalWrite(axes[i].stepPin, LOW);
    digitalWrite(axes[i].enablePin, HIGH);
  }

  Serial.println("All motors disabled.");
}

void selectAxis(byte axis) {
  motionActive = false;
  selectedAxis = axis;

  enableSelectedMotor();

  if (!encoderInitialized[selectedAxis]) {
    initializeEncoder(selectedAxis);
  }

  setTargetToCurrentPosition();

  Serial.print("Selected axis: ");
  Serial.print(axes[selectedAxis].name);

  Serial.print(" | Mux channel: ");
  Serial.print(axes[selectedAxis].muxChannel);

  Serial.print(" | Gear reduction: ");
  Serial.print(
    axes[selectedAxis].gearReduction,
    2
  );

  Serial.println(":1");
}

// ============================================================
// POSITION CONTROL
// ============================================================

void setTargetToCurrentPosition() {
  float currentJointAngle;

  if (updateAndGetJointAngle(
        selectedAxis,
        currentJointAngle
      )) {
    targetJointDegrees =
      currentJointAngle;
  }

  motionActive = false;
}

void startJointMove(float requestedAngle) {
  if (limitStopLatched) {
    Serial.println(
      "Movement locked by a limit stop."
    );

    Serial.println(
      "Release switches and enter: reset"
    );

    return;
  }

  if (anyLimitTriggered()) {
    limitStopLatched = true;

    Serial.println(
      "Cannot move: a limit is triggered."
    );

    printTriggeredLimits();
    return;
  }

  if (!axisCalibrated(selectedAxis)) {
    Serial.println(
      "Cannot move: selected axis is not calibrated."
    );

    Serial.println(
      "Place it at zero and enter: cal"
    );

    return;
  }

  bool magnetDetected;

  if (!readMagnetDetected(
        selectedAxis,
        magnetDetected
      ) || !magnetDetected) {
    Serial.println(
      "Cannot move: AS5600 magnet is not detected."
    );

    return;
  }

  float currentJointAngle;

  if (!updateAndGetJointAngle(
        selectedAxis,
        currentJointAngle
      )) {
    Serial.println(
      "Cannot move: encoder read failed."
    );

    return;
  }

  targetJointDegrees = requestedAngle;

  previousStepTime = micros();
  motionActive = true;

  enableSelectedMotor();

  Serial.print("Joint target: ");
  Serial.print(targetJointDegrees, 2);
  Serial.println(" deg");

  Serial.print("Required motor angle from zero: ");
  Serial.print(
    targetJointDegrees *
    axes[selectedAxis].gearReduction,
    2
  );

  Serial.println(" deg");
}

void updatePositionControl() {
  if (anyLimitTriggered()) {
    emergencyLimitStop();
    return;
  }

  float currentJointAngle;

  if (!updateAndGetJointAngle(
        selectedAxis,
        currentJointAngle
      )) {
    motionActive = false;

    Serial.println(
      "ENCODER FAULT: Motion stopped."
    );

    return;
  }

  float error =
    targetJointDegrees -
    currentJointAngle;

  if (
    error <= jointToleranceDegrees &&
    error >= -jointToleranceDegrees
  ) {
    motionActive = false;

    digitalWrite(
      axes[selectedAxis].stepPin,
      LOW
    );

    Serial.print("Target reached: ");
    Serial.print(currentJointAngle, 2);
    Serial.println(" joint degrees");

    return;
  }

  bool directionLevel;

  if (error > 0) {
    directionLevel =
      axes[selectedAxis]
        .dirHighIncreasesAngle;
  } else {
    directionLevel =
      !axes[selectedAxis]
         .dirHighIncreasesAngle;
  }

  digitalWrite(
    axes[selectedAxis].dirPin,
    directionLevel ? HIGH : LOW
  );

  unsigned long currentTime = micros();

  if (
    currentTime - previousStepTime >=
    STEP_INTERVAL_US
  ) {
    previousStepTime = currentTime;

    digitalWrite(
      axes[selectedAxis].stepPin,
      HIGH
    );

    delayMicroseconds(3);

    digitalWrite(
      axes[selectedAxis].stepPin,
      LOW
    );
  }
}

void stopMotor() {
  setTargetToCurrentPosition();

  digitalWrite(
    axes[selectedAxis].stepPin,
    LOW
  );

  enableSelectedMotor();

  Serial.println(
    "Motion stopped. Motor remains holding."
  );
}

// ============================================================
// STATUS
// ============================================================

void printLiveStatus() {
  float jointAngle;

  Serial.print("Axis: ");
  Serial.print(axes[selectedAxis].name);

  if (!updateAndGetJointAngle(
        selectedAxis,
        jointAngle
      )) {
    Serial.println(" | Encoder read failed");
    return;
  }

  Serial.print(" | Motor: ");
  Serial.print(
    getMotorAngle(selectedAxis),
    2
  );

  Serial.print(" deg | Joint: ");
  Serial.print(jointAngle, 2);

  Serial.print(" deg | Target: ");
  Serial.print(targetJointDegrees, 2);

  Serial.print(" deg | Motion: ");

  if (motionActive) {
    Serial.print("RUNNING");
  } else {
    Serial.print("STOPPED");
  }

  Serial.print(" | Limit lock: ");

  if (limitStopLatched) {
    Serial.println("YES");
  } else {
    Serial.println("NO");
  }
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String command =
    Serial.readStringUntil('\n');

  command.trim();
  command.toLowerCase();

  if (
    command == "axis yaw" ||
    command == "axis x"
  ) {
    selectAxis(0);
  }
  else if (
    command == "axis shoulder" ||
    command == "axis y"
  ) {
    selectAxis(1);
  }
  else if (
    command == "axis elbow" ||
    command == "axis z"
  ) {
    selectAxis(2);
  }
  else if (command == "cal") {
    calibrateSelectedAxis();
  }
  else if (command == "raw") {
    printOneEncoder(selectedAxis);
  }
  else if (command == "encoders") {
    printAllEncoders();
  }
  else if (command == "limits") {
    printLimits();
  }
  else if (command == "reset") {
    resetLimitLatch();
  }
  else if (command == "stop") {
    stopMotor();
  }
  else if (command == "enable") {
    enableSelectedMotor();
    Serial.println("Selected motor enabled.");
  }
  else if (command == "disable") {
    disableAllMotors();
  }
  else if (command == "status") {
    printCalibration();
    printAllEncoders();
    printLimits();
  }
  else if (command.startsWith("goto ")) {
    float jointTarget =
      command.substring(5).toFloat();

    startJointMove(jointTarget);
  }
  else if (command == "help") {
    printHelp();
  }
  else {
    Serial.println(
      "Unknown command. Enter: help"
    );
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  axis yaw        select yaw on CH0");
  Serial.println("  axis shoulder   select shoulder on CH1");
  Serial.println("  axis elbow      select elbow on CH2");
  Serial.println("  raw             selected encoder measurement");
  Serial.println("  encoders        all encoder measurements");
  Serial.println("  cal             define current joint position as 0");
  Serial.println("  goto 45         move joint to 45 degrees");
  Serial.println("  limits          display limit switches");
  Serial.println("  reset           clear limit-stop latch");
  Serial.println("  stop            stop and hold");
  Serial.println("  enable          enable selected motor");
  Serial.println("  disable         disable all motors");
  Serial.println("  status          complete system status");
  Serial.println("  help            show commands");
  Serial.println();
}