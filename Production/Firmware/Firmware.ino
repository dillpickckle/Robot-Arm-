
// --- Stepper pins (J1 base, J2 shoulder, J3 elbow) ---
const int STEP_PIN[3] = { 2, 4, 6 };
const int DIR_PIN[3]  = { 3, 5, 9 };
const int ENABLE_PIN  = 10;          // tie all three TMC2209 EN pins here

// --- Limit switch pins (one per stepper joint) ---
const int LIMIT_PIN[3] = { 20, 21, 22 };


const float MOTOR_STEPS   = 200.0;    
const float MICROSTEPS    = 16.0;     
const float GEAR_RATIO[3] = { 20.0, 5.0, 5.0 };

const float STEPS_PER_DEG[3] = {
  MOTOR_STEPS * MICROSTEPS * GEAR_RATIO[0] / 360.0f,   // J1 = 177.8 per degree
  MOTOR_STEPS * MICROSTEPS * GEAR_RATIO[1] / 360.0f,   // J2 =  44.4
  MOTOR_STEPS * MICROSTEPS * GEAR_RATIO[2] / 360.0f   // J3 =  44.4
};

// If a joint turns the wrong way, flip its sign to -1.
const int DIR_SIGN[3] = { 1, 1, 1 };

// Speed, in degrees per second at the joint. Start slow.
const float SPEED_DPS[3] = { 30.0, 20.0, 30.0 };

// --- Homing ---
// Which way each joint has to move to find its limit switch (+1 or -1).
const int   HOME_DIR[3] = { -1, -1, -1 };
// The joint angle the arm is AT when the switch clicks. MEASURE THESE.
const float HOME_DEG[3] = { -165.0, -15.0, -145.0 };

// --- Bus servos (on Serial1: pin 1 = TX to URT-1 RXD, pin 0 = RX from TXD) ---
const int SERVO_ID_J4      = 4;
const int SERVO_ID_J5      = 5;
const int SERVO_ID_GRIPPER = 6;

// STS3215: 4096 encoder counts over 360 degrees. 2048 is the middle.
// Put the raw count you measure at "zero degrees" here (use the 'raw' command).
const int SERVO_ZERO[3]  = { 2048, 2048, 2048 };   // J4, J5, gripper
const int SERVO_SIGN[3]  = { 1, 1, 1 };
const int SERVO_SPEED    = 1200;                   // counts per second

// Gripper raw counts. Measure these with the 'raw' command too.
const int GRIP_OPEN   = 2048;
const int GRIP_CLOSED = 1400;

// Don't let the arm drive itself into its own frame.
const float MIN_DEG[5] = { -170, -20, -150, -95, -160 };
const float MAX_DEG[5] = {  170, 110,   10,  95,  160 };




long  stepPos[3]    = { 0, 0, 0 };     // where we believe we are
long  stepTarget[3] = { 0, 0, 0 };     // where we are going
unsigned long lastStepUs[3] = { 0, 0, 0 };
unsigned long stepIntervalUs[3];       // microseconds between pulses
bool  motorsOn = false;

long degToSteps(int axis, float deg) { return (long)(deg * STEPS_PER_DEG[axis]); }
float stepsToDeg(int axis, long s)   { return (float)s / STEPS_PER_DEG[axis]; }

bool limitPressed(int axis) {
  // Switches are wired to GND with the pin pulled up, using the NC contact:
  // HIGH means pressed (or a broken wire, which is also a good reason to stop).
  return digitalRead(LIMIT_PIN[axis]) == HIGH;
}

void setMotors(bool on) {
  motorsOn = on;
  digitalWrite(ENABLE_PIN, on ? LOW : HIGH);   // TMC2209 EN is active LOW
  if (!on) for (int i = 0; i < 3; i++) stepTarget[i] = stepPos[i];
}

// Send exactly one pulse to one motor.
void pulse(int axis, int dir) {
  digitalWrite(DIR_PIN[axis], (dir * DIR_SIGN[axis]) > 0 ? HIGH : LOW);
  delayMicroseconds(2);                        // let DIR settle before stepping
  digitalWrite(STEP_PIN[axis], HIGH);
  delayMicroseconds(3);
  digitalWrite(STEP_PIN[axis], LOW);
  stepPos[axis] += dir;
}


void runSteppers() {
  if (!motorsOn) return;
  unsigned long now = micros();

  for (int i = 0; i < 3; i++) {
    if (stepPos[i] == stepTarget[i]) continue;
    if (now - lastStepUs[i] < stepIntervalUs[i]) continue;
    lastStepUs[i] = now;

    int dir = (stepTarget[i] > stepPos[i]) ? 1 : -1;

    // Never drive further into a switch that is already pressed.
    if (dir == HOME_DIR[i] && limitPressed(i)) {
      stepTarget[i] = stepPos[i];
      continue;
    }
    pulse(i, dir);
  }
}

bool steppersBusy() {
  for (int i = 0; i < 3; i++) if (stepPos[i] != stepTarget[i]) return true;
  return false;
}


void homeAll() {
  if (!motorsOn) { Serial.println("turn the motors on first ('on')"); return; }
  Serial.println("homing, keep clear...");

  for (int i = 0; i < 3; i++) {
    Serial.print("  J"); Serial.print(i + 1); Serial.print(" ... ");

    unsigned long start = millis();
    // Slow, deliberate approach: 4 degrees per second.
    unsigned long interval = (unsigned long)(1000000.0 / (4.0 * STEPS_PER_DEG[i]));

    while (!limitPressed(i)) {
      if (millis() - start > 20000) {            // 20 second giving-up point
        Serial.println("TIMED OUT - check the switch and HOME_DIR");
        setMotors(false);
        return;
      }
      pulse(i, HOME_DIR[i]);
      delayMicroseconds(interval);
    }

    // This is the whole point: the step count now means something real.
    stepPos[i]    = degToSteps(i, HOME_DEG[i]);
    stepTarget[i] = stepPos[i];
    Serial.println("ok");
    delay(300);
  }
  Serial.println("homed.");
}



const int SERVO_REG_GOAL_POS = 42;
const int SERVO_REG_SPEED    = 46;
const int SERVO_REG_POS      = 56;

void servoWrite(int id, int reg, const uint8_t *data, int len) {
  uint8_t packet[16];
  int n = 0;
  packet[n++] = 0xFF;
  packet[n++] = 0xFF;
  packet[n++] = id;
  packet[n++] = len + 3;         // params (reg + data) + instruction + checksum
  packet[n++] = 0x03;            // 0x03 = WRITE
  packet[n++] = reg;
  for (int i = 0; i < len; i++) packet[n++] = data[i];

  int sum = 0;
  for (int i = 2; i < n; i++) sum += packet[i];
  packet[n++] = ~sum;

  while (Serial1.available()) Serial1.read();   // drop anything stale
  Serial1.write(packet, n);
  Serial1.flush();
}

// Tell a servo to go to a raw encoder count (0..4095).
void servoGoTo(int id, int counts, int speed) {
  if (counts < 0)    counts = 0;
  if (counts > 4095) counts = 4095;
  // Registers 42..47 in one write: goal position, goal time, goal speed.
  uint8_t d[6] = {
    (uint8_t)(counts & 0xFF), (uint8_t)(counts >> 8),
    0, 0,
    (uint8_t)(speed & 0xFF),  (uint8_t)(speed >> 8)
  };
  servoWrite(id, SERVO_REG_GOAL_POS, d, 6);
  delay(2);                                     // let the bus settle
}


int servoReadPos(int id) {
  uint8_t packet[8];
  int n = 0;
  packet[n++] = 0xFF; packet[n++] = 0xFF;
  packet[n++] = id;
  packet[n++] = 4;                              // 2 params + instruction + checksum
  packet[n++] = 0x02;                           // 0x02 = READ
  packet[n++] = SERVO_REG_POS;
  packet[n++] = 2;                              // read 2 bytes
  int sum = 0;
  for (int i = 2; i < n; i++) sum += packet[i];
  packet[n++] = ~sum;

  while (Serial1.available()) Serial1.read();
  Serial1.write(packet, n);
  Serial1.flush();


  unsigned long start = millis();
  uint8_t buf[16];
  int got = 0;
  while (millis() - start < 20) {
    if (!Serial1.available()) continue;
    uint8_t b = Serial1.read();
    if (got < 2) {
      buf[got] = b;
      got = (b == 0xFF) ? got + 1 : 0;          // wait for 0xFF 0xFF
    } else {
      buf[got++] = b;
      // header(2) + id + len + error + 2 data + checksum = 8 bytes
      if (got >= 8) {
        if (buf[2] != id) { got = 0; continue; }   // that was our own echo
        return buf[5] | (buf[6] << 8);             // low byte first
      }
    }
  }
  return -1;
}

int servoDegToCounts(int slot, float deg) {
  return SERVO_ZERO[slot] + (int)(deg * SERVO_SIGN[slot] * 4096.0 / 360.0);
}

float servoCountsToDeg(int slot, int counts) {
  return (counts - SERVO_ZERO[slot]) * SERVO_SIGN[slot] * 360.0 / 4096.0;
}




float servoTargetDeg[2] = { 0, 0 };     // J4, J5

void moveJoint(int joint, float deg) {   // joint is 1..5
  int idx = joint - 1;
  if (deg < MIN_DEG[idx] || deg > MAX_DEG[idx]) {
    Serial.println("outside the joint limits");
    return;
  }
  if (!motorsOn) { Serial.println("turn the motors on first ('on')"); return; }

  if (joint <= 3) {
    stepTarget[idx] = degToSteps(idx, deg);
  } else if (joint == 4) {
    servoTargetDeg[0] = deg;
    servoGoTo(SERVO_ID_J4, servoDegToCounts(0, deg), SERVO_SPEED);
  } else {
    servoTargetDeg[1] = deg;
    servoGoTo(SERVO_ID_J5, servoDegToCounts(1, deg), SERVO_SPEED);
  }
  Serial.println("moving");
}

void printPositions() {
  Serial.print("steppers:  ");
  for (int i = 0; i < 3; i++) {
    Serial.print("J"); Serial.print(i + 1); Serial.print(" ");
    Serial.print(stepsToDeg(i, stepPos[i]), 1); Serial.print("  ");
  }
  Serial.println();

  Serial.print("servos:    ");
  int p4 = servoReadPos(SERVO_ID_J4);
  int p5 = servoReadPos(SERVO_ID_J5);
  Serial.print("J4 ");
  if (p4 < 0) Serial.print("no reply"); else Serial.print(servoCountsToDeg(0, p4), 1);
  Serial.print("  J5 ");
  if (p5 < 0) Serial.print("no reply"); else Serial.print(servoCountsToDeg(1, p5), 1);
  Serial.println();

  Serial.print("switches:  ");
  for (int i = 0; i < 3; i++) { Serial.print(limitPressed(i)); Serial.print(" "); }
  Serial.println();
}

void printRaw() {
  int ids[3] = { SERVO_ID_J4, SERVO_ID_J5, SERVO_ID_GRIPPER };
  for (int i = 0; i < 3; i++) {
    int p = servoReadPos(ids[i]);
    Serial.print("servo id "); Serial.print(ids[i]); Serial.print(" raw count: ");
    if (p < 0) Serial.println("no reply"); else Serial.println(p);
  }
}

void handleCommand(char *line) {
  // Split into a word and (maybe) a number.
  char *sp = strchr(line, ' ');
  float value = 0;
  bool hasValue = false;
  if (sp) { *sp = 0; value = atof(sp + 1); hasValue = true; }

  if (!strcmp(line, "on"))        { setMotors(true);  Serial.println("motors on"); }
  else if (!strcmp(line, "off"))  { setMotors(false); Serial.println("motors off"); }
  else if (!strcmp(line, "home")) { homeAll(); }
  else if (!strcmp(line, "pos"))  { printPositions(); }
  else if (!strcmp(line, "raw"))  { printRaw(); }
  else if (!strcmp(line, "grip")) {
    if (!hasValue) { Serial.println("usage: grip 0-100"); return; }
    if (value < 0) value = 0;
    if (value > 100) value = 100;
    int counts = GRIP_CLOSED + (int)((GRIP_OPEN - GRIP_CLOSED) * value / 100.0);
    servoGoTo(SERVO_ID_GRIPPER, counts, SERVO_SPEED);
    Serial.println("gripper moving");
  }
  else if (line[0] == 'j' && line[1] >= '1' && line[1] <= '5' && line[2] == 0) {
    if (!hasValue) { Serial.println("usage: j1 45"); return; }
    moveJoint(line[1] - '0', value);
  }
  else {
    Serial.println("commands: on  off  home  j1..j5 <deg>  grip <0-100>  pos  raw");
  }
}

// Collect characters until Enter, then act on the line.
char cmdBuf[40];
int  cmdLen = 0;

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = 0;
      if (cmdLen > 0) handleCommand(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}



void setup() {
  for (int i = 0; i < 3; i++) {
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i],  OUTPUT);
    pinMode(LIMIT_PIN[i], INPUT_PULLUP);
    digitalWrite(STEP_PIN[i], LOW);

    // Work out the delay between step pulses from the speed setting.
    stepIntervalUs[i] = (unsigned long)(1000000.0 / (SPEED_DPS[i] * STEPS_PER_DEG[i]));
  }

  pinMode(ENABLE_PIN, OUTPUT);
  setMotors(false);                     // always boot with the motors off

  Serial.begin(115200);
  Serial1.begin(1000000);               // the servo bus, via the URT-1 adapter
  delay(500);

  Serial.println();
  Serial.println("5-DOF arm - basic firmware");
  Serial.println("motors are off, the arm is limp. nothing will move yet.");
  Serial.println("commands: on  off  home  j1..j5 <deg>  grip <0-100>  pos  raw");
  Serial.println();
}

void loop() {
  runSteppers();     // must be called constantly, this is what moves the motors
  readSerial();
}
