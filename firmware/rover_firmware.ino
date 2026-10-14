/* ===========================================================================
 *  Warehouse Inspection Rover — low-level firmware
 *  Arduino Uno / Nano / ESP32  +  L298N motor driver  +  HC-SR04 ultrasonic
 *
 *  Capstone Project-1 · Autonomous Warehouse Rover
 *
 *  This board does three jobs and nothing else:
 *    1. drive the motors as the Jetson tells it to
 *    2. measure distance ahead with the ultrasonic sensor
 *    3. report odometry, range and battery back up the USB cable
 *
 *  All the thinking — where to go, when to inspect, what the strip colour
 *  means — happens on the Jetson in tools/rover_client.py. Keep it that way:
 *  this sketch must stay small enough to be obviously correct.
 *
 *  ---------------------------------------------------------------------
 *  SERIAL PROTOCOL  (115200 baud, newline terminated, both directions)
 *
 *  Commands IN, from the Jetson:
 *      F<mm/s>    drive forward at this speed, e.g. F450
 *      T<deg>     turn in place by this many degrees, + = left
 *      S          stop immediately
 *      ?          send one telemetry line now
 *
 *  Lines OUT, to the Jetson:
 *      P <dx> <dy> <dtheta>   odometry delta since the last report:
 *                             metres forward, metres sideways (always 0 on
 *                             a differential drive), degrees turned
 *      U <cm>                 front ultrasonic range in centimetres;
 *                             -1 means no echo came back
 *      B <volts>              battery pack voltage
 *
 *  ---------------------------------------------------------------------
 *  SAFETY
 *
 *  The board refuses to drive forward when the ultrasonic reads closer than
 *  SAFETY_STOP_CM. That is deliberate belt-and-braces: the Jetson also stops,
 *  and the twin also stops it, but a stop that depends on a USB cable, a
 *  Python process and a Wi-Fi link is not a stop. Keep this check here even
 *  though it looks redundant.
 *
 *  SAFETY_STOP_CM is a few centimetres TIGHTER than the twin's
 *  obstacle_stop_distance (0.40 m in config/route.yaml), so in normal running
 *  the Jetson stops first and this is only a backstop. If you change one,
 *  change the other.
 * ======================================================================= */

// ---------------------------------------------------------------- pins ----
// L298N motor driver
const int ENA = 5;    // left  PWM   (must be a PWM pin)
const int IN1 = 7;
const int IN2 = 8;
const int ENB = 6;    // right PWM   (must be a PWM pin)
const int IN3 = 9;
const int IN4 = 11;

// HC-SR04
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// Wheel encoders (interrupt-capable pins: 2 and 3 on an Uno/Nano)
const int ENC_LEFT  = 2;
const int ENC_RIGHT = 3;

// Battery sense: pack through a divider into an analog pin
const int BATT_PIN = A0;

// ------------------------------------------------------- robot geometry ---
// MEASURE THESE ON YOUR OWN ROVER. Everything the dashboard draws depends on
// them: get them wrong and the rover's reported position drifts away from the
// route, which the twin will show as "off route by N cm".
const float WHEEL_DIAMETER_M = 0.065;   // metres
const int   TICKS_PER_REV    = 20;      // slotted-disc encoder slots
const float WHEEL_BASE_M     = 0.145;   // distance between the two wheels

const float METRES_PER_TICK = (PI * WHEEL_DIAMETER_M) / TICKS_PER_REV;

// --------------------------------------------------------------- tuning ---
const int   SAFETY_STOP_CM   = 30;      // hard stop; keep below the twin's 40
const int   MIN_PWM          = 60;      // below this the motors just buzz
const int   MAX_PWM          = 255;
const float MAX_SPEED_MMS    = 600.0;   // speed at full PWM — measure it
const unsigned long REPORT_MS = 100;    // telemetry rate, 10 Hz
const unsigned long PING_MS   = 60;     // ultrasonic rate; below 50 ms the
                                        // previous echo bleeds into the next
const float BATT_DIVIDER     = 3.0;     // (R1+R2)/R2 of your divider
const float ADC_REF_V        = 5.0;     // 3.3 on an ESP32

// ---------------------------------------------------------------- state ---
volatile long ticksLeft = 0;
volatile long ticksRight = 0;
long lastTicksLeft = 0, lastTicksRight = 0;

int  commandedPwm = 0;        // signed: negative drives backwards
int  turnPwm = 0;             // signed: positive turns left
long rangeCm = -1;
unsigned long lastReport = 0, lastPing = 0;
String inBuf = "";

void onLeftTick()  { ticksLeft++; }
void onRightTick() { ticksRight++; }

// ====================================================================== //
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(ENC_LEFT, INPUT_PULLUP); pinMode(ENC_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT),  onLeftTick,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), onRightTick, RISING);

  stopMotors();
  Serial.println("# rover firmware ready");
}

// ====================================================================== //
void loop() {
  readCommands();

  if (millis() - lastPing >= PING_MS) {
    lastPing = millis();
    rangeCm = measureDistance();
  }

  // Belt-and-braces: never drive forward into something close.
  if (commandedPwm > 0 && rangeCm >= 0 && rangeCm < SAFETY_STOP_CM) {
    commandedPwm = 0;
    stopMotors();
  }

  applyMotors();

  if (millis() - lastReport >= REPORT_MS) {
    lastReport = millis();
    sendTelemetry();
  }
}

// ------------------------------------------------------------- commands ---
void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inBuf.length()) handleCommand(inBuf);
      inBuf = "";
    } else if (inBuf.length() < 24) {
      inBuf += c;
    }
  }
}

void handleCommand(const String &cmd) {
  char verb = cmd.charAt(0);
  float value = cmd.length() > 1 ? cmd.substring(1).toFloat() : 0.0;

  switch (verb) {
    case 'F': {                       // forward at mm/s
      float frac = constrain(value / MAX_SPEED_MMS, -1.0, 1.0);
      commandedPwm = (int)(frac * MAX_PWM);
      if (commandedPwm > 0 && commandedPwm < MIN_PWM) commandedPwm = MIN_PWM;
      turnPwm = 0;
      break;
    }
    case 'T': {                       // turn in place, degrees
      // Short, fixed-power nudges: the Jetson closes the loop on heading, so
      // the board only has to turn roughly the right way.
      turnPwm = (value > 0 ? 1 : -1) * max(MIN_PWM, (int)(abs(value) * 2));
      turnPwm = constrain(turnPwm, -MAX_PWM, MAX_PWM);
      commandedPwm = 0;
      break;
    }
    case 'S':
      commandedPwm = 0; turnPwm = 0; stopMotors();
      break;
    case '?':
      sendTelemetry();
      break;
  }
}

// --------------------------------------------------------------- motors ---
void applyMotors() {
  if (turnPwm != 0) {
    // Turn in place: wheels in opposite directions. Positive = left.
    bool left = turnPwm > 0;
    int p = abs(turnPwm);
    setSide(IN1, IN2, ENA, left ? -p : p);
    setSide(IN3, IN4, ENB, left ?  p : -p);
  } else if (commandedPwm != 0) {
    setSide(IN1, IN2, ENA, commandedPwm);
    setSide(IN3, IN4, ENB, commandedPwm);
  } else {
    stopMotors();
  }
}

void setSide(int inA, int inB, int en, int pwm) {
  if (pwm >= 0) { digitalWrite(inA, HIGH); digitalWrite(inB, LOW); }
  else          { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  analogWrite(en, constrain(abs(pwm), 0, MAX_PWM));
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);    analogWrite(ENB, 0);
}

// ----------------------------------------------------------- ultrasonic ---
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 25 ms ceiling is about 4 m there and back — the sensor's useful limit.
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (us == 0) return -1;              // no echo: nothing in range
  long cm = (long)(us / 58);           // speed of sound, there and back
  if (cm < 2 || cm > 400) return -1;   // outside the HC-SR04's honest range
  return cm;
}

// ------------------------------------------------------------ telemetry ---
void sendTelemetry() {
  noInterrupts();
  long l = ticksLeft, r = ticksRight;
  interrupts();

  long dl = l - lastTicksLeft;
  long dr = r - lastTicksRight;
  lastTicksLeft = l; lastTicksRight = r;

  // Unsigned encoders cannot tell forward from backward, so take the sign
  // from what we commanded. Fit quadrature encoders if you need better.
  float dirDrive = commandedPwm >= 0 ? 1.0 : -1.0;
  float distL = dl * METRES_PER_TICK * (turnPwm > 0 ? -1.0 : (turnPwm < 0 ? 1.0 : dirDrive));
  float distR = dr * METRES_PER_TICK * (turnPwm > 0 ?  1.0 : (turnPwm < 0 ? -1.0 : dirDrive));

  float forward = (distL + distR) / 2.0;
  float dTheta  = (distR - distL) / WHEEL_BASE_M * 180.0 / PI;   // degrees

  Serial.print("P "); Serial.print(forward, 4);
  Serial.print(" 0.0000 ");  Serial.println(dTheta, 3);

  Serial.print("U "); Serial.println(rangeCm);

  float volts = analogRead(BATT_PIN) * (ADC_REF_V / 1023.0) * BATT_DIVIDER;
  Serial.print("B "); Serial.println(volts, 2);
}
