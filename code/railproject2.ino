#include <Servo.h>

/* ----- Number of gate systems ----- */
const uint8_t NUM_GATES = 2;

/* ----- Pin assignments for each gate ----- */
const uint8_t TRIG_PIN[NUM_GATES]  = {10, 6};
const uint8_t ECHO_PIN[NUM_GATES]  = {9, 5};
const uint8_t SERVO_PIN[NUM_GATES] = {3, 11};
const uint8_t BUZZER_PIN[NUM_GATES] = {7, 4};

/* ----- Behaviour parameters ----- */
const unsigned int DIST_THRESHOLD_CM = 5;
const unsigned long WARNING_DURATION_MS = 200;   // 1 second before closing
const unsigned long SENSOR_READ_INTERVAL_MS = 60;
const unsigned long MIN_RETRIGGER_GAP_MS = 200;  // anti-retrigger

/* Servo behavior */
const uint8_t OPEN_ANGLE = 0;
const uint8_t CLOSED_ANGLE = 90;
const uint8_t SERVO_STEP_DEG = 2;
const unsigned long SERVO_STEP_INTERVAL_MS = 10;

/* Gate remains closed until object has been absent for this duration */
const unsigned long TRAIN_GONE_DELAY_MS = 500;

/* Buzzer pattern */
const unsigned long BUZZER_ON_MS = 150;
const unsigned long BUZZER_OFF_MS = 350;

/* ----- Gate State Machine ----- */
enum GateState {
  STATE_OPEN,
  STATE_WARNING,
  STATE_CLOSING,
  STATE_CLOSED,
  STATE_OPENING
};

/* ----- Per-Gate Variables ----- */
GateState gateState[NUM_GATES];

Servo gateServo[NUM_GATES];

unsigned long lastSensorRead[NUM_GATES];
unsigned long lastTriggerTime[NUM_GATES];
unsigned long warningStartTime[NUM_GATES];
unsigned long closedStartTime[NUM_GATES];
unsigned long lastServoStepTime[NUM_GATES];
unsigned long lastBuzzerToggle[NUM_GATES];

int currentServoAngle[NUM_GATES];
bool buzzerOn[NUM_GATES];

/* --------------------------------------------- */
/* ----- Ultrasonic: read one sensor (cm) ------ */
/* --------------------------------------------- */
unsigned int readUltrasonicCM(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long duration = pulseIn(echo, HIGH, 30000UL);
  if (duration == 0) return 400;
  return (unsigned int)(duration / 58.2);
}

/* --------------------------------------------- */
/* ----- Move servo smoothly per gate ---------- */
/* --------------------------------------------- */
void stepServoTowards(int gate, int targetAngle) {
  unsigned long now = millis();
  if (now - lastServoStepTime[gate] < SERVO_STEP_INTERVAL_MS) return;
  lastServoStepTime[gate] = now;

  if (currentServoAngle[gate] == targetAngle) return;

  if (currentServoAngle[gate] < targetAngle)
    currentServoAngle[gate] = min(targetAngle, currentServoAngle[gate] + SERVO_STEP_DEG);
  else
    currentServoAngle[gate] = max(targetAngle, currentServoAngle[gate] - SERVO_STEP_DEG);

  gateServo[gate].write(currentServoAngle[gate]);
}

/* --------------------------------------------- */
/* ----- Buzzer pattern per gate --------------- */
/* --------------------------------------------- */
void handleBuzzerPattern(int gate) {
  unsigned long now = millis();

  if (buzzerOn[gate]) {
    if (now - lastBuzzerToggle[gate] >= BUZZER_ON_MS) {
      buzzerOn[gate] = false;
      digitalWrite(BUZZER_PIN[gate], LOW);
      lastBuzzerToggle[gate] = now;
    }
  } else {
    if (now - lastBuzzerToggle[gate] >= BUZZER_OFF_MS) {
      buzzerOn[gate] = true;
      digitalWrite(BUZZER_PIN[gate], HIGH);
      lastBuzzerToggle[gate] = now;
    }
  }
}

/* --------------------------------------------- */
/* ----- Setup --------------------------------- */
/* --------------------------------------------- */
void setup() {
  Serial.begin(9600);

  for (int g = 0; g < NUM_GATES; g++) {
    pinMode(TRIG_PIN[g], OUTPUT);
    pinMode(ECHO_PIN[g], INPUT);
    pinMode(BUZZER_PIN[g], OUTPUT);
    digitalWrite(BUZZER_PIN[g], LOW);

    gateServo[g].attach(SERVO_PIN[g]);
    gateServo[g].write(OPEN_ANGLE);

    currentServoAngle[g] = OPEN_ANGLE;
    buzzerOn[g] = false;
    gateState[g] = STATE_OPEN;

    lastSensorRead[g] = lastTriggerTime[g] = 0;
  }

  Serial.println(F("Dual Gate System Initialized"));
}

/* --------------------------------------------- */
/* ----- Main Loop ----------------------------- */
/* --------------------------------------------- */
void loop() {
  unsigned long now = millis();

  for (int g = 0; g < NUM_GATES; g++) {

    /* --- Read distance periodically --- */
    static unsigned int dist[NUM_GATES];

    if (now - lastSensorRead[g] >= SENSOR_READ_INTERVAL_MS) {
      lastSensorRead[g] = now;

      dist[g] = readUltrasonicCM(TRIG_PIN[g], ECHO_PIN[g]);
      Serial.print("Gate ");
      Serial.print(g);
      Serial.print(" Distance: ");
      Serial.println(dist[g]);

      if (gateState[g] == STATE_OPEN && dist[g] <= DIST_THRESHOLD_CM) {
        if (now - lastTriggerTime[g] >= MIN_RETRIGGER_GAP_MS) {
          Serial.print("Gate "); Serial.print(g); Serial.println(" -> WARNING");
          gateState[g] = STATE_WARNING;
          warningStartTime[g] = now;
          buzzerOn[g] = true;
          digitalWrite(BUZZER_PIN[g], HIGH);
          lastBuzzerToggle[g] = now;
        }
      }
    }

    /* ------ Gate State Machine ------- */
    switch (gateState[g]) {

      case STATE_OPEN:
        stepServoTowards(g, OPEN_ANGLE);
        digitalWrite(BUZZER_PIN[g], LOW);
        buzzerOn[g] = false;
        break;

      case STATE_WARNING:
        handleBuzzerPattern(g);
        if (now - warningStartTime[g] >= WARNING_DURATION_MS) {
          Serial.print("Gate "); Serial.print(g); Serial.println(" -> CLOSING");
          gateState[g] = STATE_CLOSING;
          buzzerOn[g] = false;
          digitalWrite(BUZZER_PIN[g], LOW);
        }
        break;

      case STATE_CLOSING:
        stepServoTowards(g, CLOSED_ANGLE);
        if (currentServoAngle[g] == CLOSED_ANGLE) {
          Serial.print("Gate "); Serial.print(g); Serial.println(" CLOSED");
          gateState[g] = STATE_CLOSED;
          closedStartTime[g] = now;
          buzzerOn[g] = true;
          digitalWrite(BUZZER_PIN[g], HIGH);
          lastBuzzerToggle[g] = now;
        }
        break;

      case STATE_CLOSED:
        handleBuzzerPattern(g);

        /* Train still detected? Reset timer */
        if (dist[g] <= DIST_THRESHOLD_CM) {
          closedStartTime[g] = now;  // train still present
        }

        /* Train gone for FULL 5 sec? */
        if (now - closedStartTime[g] >= TRAIN_GONE_DELAY_MS) {
          Serial.print("Gate "); Serial.print(g); Serial.println(" -> OPENING");
          gateState[g] = STATE_OPENING;
          buzzerOn[g] = false;
          digitalWrite(BUZZER_PIN[g], LOW);
          lastTriggerTime[g] = now;
        }
        break;

      case STATE_OPENING:
        stepServoTowards(g, OPEN_ANGLE);
        if (currentServoAngle[g] == OPEN_ANGLE) {
          Serial.print("Gate "); Serial.print(g); Serial.println(" OPEN");
          gateState[g] = STATE_OPEN;
          lastTriggerTime[g] = now;
        }
        break;
    }
  }
}
