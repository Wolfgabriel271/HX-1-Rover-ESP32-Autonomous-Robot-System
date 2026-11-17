
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_SH110X.h>

// ── Timing constants ──────────────────────────────────────────
#define SENSOR_INTERVAL  80      // ms between sensor reads
#define OLED_INTERVAL    200     // ms between OLED refreshes
#define BLINK_MS         400     // ms per blink half-cycle
#define ECHO_TIMEOUT     25000   // us (~425cm max range)
#define USB_BUF_MAX      64      // max USB buffer length

// ── Motor pins ────────────────────────────────────────────────
#define ENA  25
#define IN1  26
#define IN2  27
#define ENB  13
#define IN3  14
#define IN4  12

// ── Motor PWM channels (ledc) ─────────────────────────────────
#define CH_ENA  0
#define CH_ENB  1

// ── Light pins ────────────────────────────────────────────────
#define SIG_LEFT   4
#define SIG_RIGHT  5

// ── Ultrasonic pins ───────────────────────────────────────────
#define TRIG_F  32
#define ECHO_F  33
#define TRIG_L  16
#define ECHO_L  17
#define TRIG_R  15
#define ECHO_R  23

// ── IR sensor pins ────────────────────────────────────────────
#define IR_LEFT   18
#define IR_RIGHT  19

// ── Servo / OLED ─────────────────────────────────────────────
#define SERVOMIN   150
#define SERVOMAX   600
#define SERVO_CNT  6
#define SCREEN_W   128
#define SCREEN_H    64
#define OLED_ADDR  0x3C

const int SERVO_LO[SERVO_CNT] = {30, 30, 30, 30, 30, 30};
const int SERVO_HI[SERVO_CNT] = {150,150,150,150,150,150};

// ── Objects ───────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_SH1106G display(SCREEN_W, SCREEN_H, &Wire, -1);

// ── State ─────────────────────────────────────────────────────
float servoPos[SERVO_CNT];
float servoTarget[SERVO_CNT];

int  speedLeft  = 0, speedRight = 0;
int  blinkMode  = 0;
bool blinkState = false;
float slewStick = 2.0f;
float slewHold  = 0.6f;

int  distFront = 999, distLeft = 999, distRight = 999;
bool irLeft = false, irRight = false;

String usbBuffer = "";

unsigned long lastSensor = 0;
unsigned long lastOLED   = 0;
unsigned long lastBlink  = 0;

char currentMode[12] = "MANUAL";

// ── Motor control (ledc for reliable PWM on ESP32) ────────────
void setMotors(int left, int right) {
  speedLeft  = constrain(left,  -255, 255);
  speedRight = constrain(right, -255, 255);

  // Direction pins
  digitalWrite(IN1, speedLeft  > 0 ? HIGH : LOW);
  digitalWrite(IN2, speedLeft  < 0 ? HIGH : LOW);
  digitalWrite(IN3, speedRight > 0 ? HIGH : LOW);
  digitalWrite(IN4, speedRight < 0 ? HIGH : LOW);

  // Speed via ledc
  ledcWrite(CH_ENA, abs(speedLeft));
  ledcWrite(CH_ENB, abs(speedRight));
}

void stopMotors() { setMotors(0, 0); }

// ── Ultrasonic — non-blocking staggered reads ─────────────────
// Each sensor reads independently every SENSOR_INTERVAL ms
// staggered by 1/3 each to prevent cross-talk
int readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT);
  if (dur == 0) return 999;
  int cm = (int)(dur * 0.034f / 2.0f);
  return (cm > 400) ? 999 : cm;
}

// Round-robin sensor index: 0=Front 1=Left 2=Right
// One sensor fired per SENSOR_INTERVAL tick — no cross-talk, no blocking overrun
static uint8_t sensorTurn = 0;
static unsigned long lastSensorTick = 0;

void updateSensors() {
  unsigned long now = millis();
  if (now - lastSensorTick < SENSOR_INTERVAL) return;
  lastSensorTick = now;

  // Always read IR (fast digital read — no timing issue)
  irLeft  = (digitalRead(IR_LEFT)  == LOW);
  irRight = (digitalRead(IR_RIGHT) == LOW);

  // Read one ultrasonic per tick, rotating
  switch (sensorTurn) {
    case 0: distFront = readUltrasonic(TRIG_F, ECHO_F); break;
    case 1: distLeft  = readUltrasonic(TRIG_L, ECHO_L); break;
    case 2: distRight = readUltrasonic(TRIG_R, ECHO_R); break;
  }
  sensorTurn = (sensorTurn + 1) % 3;
}

void sendSensorData() {
  if (millis() - lastSensor < SENSOR_INTERVAL) return;
  lastSensor = millis();

  Serial.print("{\"F\":");  Serial.print(distFront);
  Serial.print(",\"L\":");  Serial.print(distLeft);
  Serial.print(",\"R\":");  Serial.print(distRight);
  Serial.print(",\"IL\":"); Serial.print(irLeft  ? 1 : 0);
  Serial.print(",\"IR\":"); Serial.print(irRight ? 1 : 0);
  Serial.print(",\"SL\":"); Serial.print(speedLeft);
  Serial.print(",\"SR\":"); Serial.print(speedRight);
  Serial.println("}");
}

// ── Servos ────────────────────────────────────────────────────
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setTarget(int ch, float angle) {
  servoTarget[ch] = constrain(angle,
    (float)SERVO_LO[ch], (float)SERVO_HI[ch]);
}

void slewServo(int ch, float rate) {
  float diff = servoTarget[ch] - servoPos[ch];
  if (fabsf(diff) < 0.5f) return;
  if      (diff >  rate) servoPos[ch] += rate;
  else if (diff < -rate) servoPos[ch] -= rate;
  else                   servoPos[ch]  = servoTarget[ch];
  pwm.setPWM(ch, 0, angleToPulse((int)servoPos[ch]));
}

void updateServos() {
  slewServo(0, slewStick);
  slewServo(1, slewStick);
  slewServo(2, slewHold);
  slewServo(3, slewHold);
  slewServo(4, slewHold);
  slewServo(5, slewHold);
}

// ── Lights — proper millis() blink ────────────────────────────
void updateLights() {
  if (millis() - lastBlink >= BLINK_MS) {
    lastBlink  = millis();
    blinkState = !blinkState;
  }
  switch (blinkMode) {
    case 0:
      digitalWrite(SIG_LEFT, LOW);
      digitalWrite(SIG_RIGHT, LOW);
      break;
    case 1:  // Left blinker
      digitalWrite(SIG_LEFT,  blinkState ? HIGH : LOW);
      digitalWrite(SIG_RIGHT, LOW);
      break;
    case 2:  // Right blinker
      digitalWrite(SIG_LEFT,  LOW);
      digitalWrite(SIG_RIGHT, blinkState ? HIGH : LOW);
      break;
    case 3:  // Hazard — both blink together
      digitalWrite(SIG_LEFT,  blinkState ? HIGH : LOW);
      digitalWrite(SIG_RIGHT, blinkState ? HIGH : LOW);
      break;
  }
}

// ── USB Serial parser ─────────────────────────────────────────
void parseUSBCommand(const String& cmd) {

  if (cmd == "STOP") {
    stopMotors();
    return;
  }

  if (cmd.startsWith("DRIVE:")) {
    int comma = cmd.indexOf(',', 6);
    if (comma < 0) return;
    setMotors(cmd.substring(6, comma).toInt(),
              cmd.substring(comma + 1).toInt());
    return;
  }

  if (cmd.startsWith("SERVO:")) {
    int comma = cmd.indexOf(',', 6);
    if (comma < 0) return;
    int   ch  = cmd.substring(6, comma).toInt();
    float ang = cmd.substring(comma + 1).toFloat();
    if (ch >= 0 && ch < SERVO_CNT) setTarget(ch, ang);
    return;
  }

  if (cmd.startsWith("LIGHT:")) {
    blinkMode = constrain(cmd.substring(6).toInt(), 0, 3);
    return;
  }

  if (cmd.startsWith("ARMSPEED:")) {
    float spd = constrain(cmd.substring(9).toFloat(), 0.2f, 15.0f);
    slewStick = spd;
    slewHold  = spd * 0.3f;
    return;
  }

  if (cmd.startsWith("MODE:")) {
    String m = cmd.substring(5);
    m.toCharArray(currentMode, sizeof(currentMode));
    return;
  }
}

void readUSBSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      usbBuffer.trim();
      if (usbBuffer.length() > 0) parseUSBCommand(usbBuffer);
      usbBuffer = "";
    } else if (c != '\r') {
      // Cap buffer length to prevent unbounded growth from line noise
      if (usbBuffer.length() < USB_BUF_MAX) usbBuffer += c;
      else usbBuffer = "";  // overflow — discard and start fresh
    }
  }
}

// ── OLED ──────────────────────────────────────────────────────
void updateOLED() {
  if (millis() - lastOLED < OLED_INTERVAL) return;
  lastOLED = millis();

  const char* blinkNames[] = {"OFF","<L","R>","HAZ"};

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Row 0 — Mode
  display.setCursor(0, 0);
  display.print("MODE: ");
  display.print(currentMode);

  // Row 1 — Distances
  display.setCursor(0, 11);
  display.print("F:"); display.print(distFront);
  display.print(" L:"); display.print(distLeft);
  display.print(" R:"); display.print(distRight);

  // Row 2 — IR + light mode
  display.setCursor(0, 21);
  display.print("IRL:"); display.print(irLeft  ? "ON " : "OFF");
  display.print(" IRR:"); display.print(irRight ? "ON " : "OFF");
  display.print(" "); display.print(blinkNames[blinkMode]);

  // Row 3 — Motors
  display.setCursor(0, 31);
  display.print("MTR L:"); display.print(speedLeft);
  display.print(" R:");    display.print(speedRight);

  // Row 4 — Servos S1-S3
  display.setCursor(0, 41);
  display.print("S1:"); display.print((int)servoPos[0]);
  display.print(" S2:"); display.print((int)servoPos[1]);
  display.print(" S3:"); display.print((int)servoPos[2]);

  // Row 5 — Servos S4-S6
  display.drawFastHLine(0, 52, 128, SH110X_WHITE);
  display.setCursor(0, 54);
  display.print("S4:"); display.print((int)servoPos[3]);
  display.print(" S5:"); display.print((int)servoPos[4]);
  display.print(" S6:"); display.print((int)servoPos[5]);

  display.display();
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Motor direction pins
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  // Motor speed via ledc (more reliable than analogWrite on ESP32)
  ledcSetup(CH_ENA, 5000, 8);   // 5kHz, 8-bit (0-255)
  ledcSetup(CH_ENB, 5000, 8);
  ledcAttachPin(ENA, CH_ENA);
  ledcAttachPin(ENB, CH_ENB);
  stopMotors();

  // Lights
  pinMode(SIG_LEFT, OUTPUT);  digitalWrite(SIG_LEFT, LOW);
  pinMode(SIG_RIGHT,OUTPUT);  digitalWrite(SIG_RIGHT,LOW);

  // Ultrasonic
  pinMode(TRIG_F,OUTPUT); pinMode(ECHO_F,INPUT);
  pinMode(TRIG_L,OUTPUT); pinMode(ECHO_L,INPUT);
  pinMode(TRIG_R,OUTPUT); pinMode(ECHO_R,INPUT);

  // IR (input — module has onboard pullup)
  pinMode(IR_LEFT,  INPUT);
  pinMode(IR_RIGHT, INPUT);

  // I2C
  Wire.begin(21, 22);

  // PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  // Center all servos
  for (int i = 0; i < SERVO_CNT; i++) {
    servoPos[i] = servoTarget[i] = 90;
    pwm.setPWM(i, 0, angleToPulse(90));
  }

  // OLED
  display.begin(OLED_ADDR, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(20, 10); display.println("ROBOT READY");
  display.setCursor(10, 24); display.println("Waiting for");
  display.setCursor(10, 34); display.println("Orange Pi...");
  display.setCursor(10, 48); display.println("USB: 115200 baud");
  display.display();
  delay(1500);

  Serial.println("[ESP32] Ready");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  readUSBSerial();   // receive commands from Orange Pi
  updateSensors();   // staggered non-blocking sensor reads
  sendSensorData();  // send JSON to Orange Pi
  updateServos();    // slew servos toward targets
  updateLights();    // blink lights
  updateOLED();      // refresh display
  // NO delay() — everything uses millis() timers
}
