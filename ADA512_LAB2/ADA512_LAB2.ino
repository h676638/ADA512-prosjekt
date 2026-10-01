/*  ADA512 LAB 2 - control of profile temperature. PROGRAM FRAMEWORK.
 *
 *  Names in the program and symbols of the lectures (Lecture 1, slide 17):
 *    theta  = c    controlled variable, rise above ambient T - T_amb [K]
 *    thetaZ = r    set point [K]           e = r - c   error [K]
 *    u             control variable = duty cycle (D in Lecture 4) [0...1]
 *    DELTA_T = dT  settled rise at u = 1, LAB 1 [K]
 *    T_P           time constant, LAB 1 [s]     L_LOOP = L  delay [s]
 *    KR, TI, KI    controller gain K_R, integral time T_i, K_I = K_R/T_i
 *    T_M           switching window (modulation period, Lecture 4) [s]
 *
 *  Given: acquisition, time-proportional output (switching window T_M),
 *  approach and phases F0-F3, performance indices, logging, commands.
 *
 *  Written by the group, in section 8, function controller():
 *    method 2   on-off                  Task 3
 *    method 6   proportional            Task 4
 *    method 9   PI                      Task 5
 *    method 10  PI with anti-windup     Task 5
 *  Each of the four blocks is marked TODO. Until a block is written the
 *  program refuses to start that method (command R).
 *
 *  Set once in the program, from LAB 1 (sections 3 and 4):
 *    DELTA_T, T_P, K_CAL; CTRL_CH = 2 (NTC3) at every station.
 *
 *  Set at run time over the serial port - no new upload between methods.
 *  One command per line, ended with Enter, typed in the terminal that logs the
 *  data (PuTTY, CoolTerm; CR or LF both accepted).
 *    A          measure ambient temperature: 180 s, heater off, cold station
 *    T 22.41    enter the ambient temperature [C] by hand, e.g. after an upload
 *    M 6        select the method
 *    KR 0.25    controller gain K_R [1/K]            methods 6, 7, 9, 10
 *    TI 63      integral time T_i [s]                methods 9, 10
 *    KI 0.008   integral gain K_I = K_R/T_i          method 8
 *    H 0.5      half hysteresis band h [K]           methods 3, 4, 5
 *    L 20       loop delay L [s]                     method 12
 *    ?          print the settings
 *    R          start the protocol (unattended, about 26 min)
 *    S          stop, heater off
 *  Settings cannot be changed during a run. After an upload or a reset all
 *  settings return to the values in section 2 and T_amb is empty: enter it
 *  again with T, using the value printed by command A on the same day.
 *  T_amb is measured once per day of measurements, before the first heating.
 *
 *  Protocol: approach to r = 5 K and F0 300 s, both held by a fixed controller
 *  u = u0 + KR_HOLD*e, the same for every method. The approach ends in the
 *  band 5 +/- 0.2 K, or with a warning when c has settled outside it or after
 *  20 min. Then F1 r = 10 K 400 s,
 *  F2 r = 7 K 240 s, F3 r = 7 K 480 s, with the selected method.
 *  Logged every 1 s: t_s,phase,r,T1,T2,T3,c,e,u,heater,I_term
 *  I_term = KR*ctrl.I/TI, the integral contribution (methods 9 to 11).
 *  Summary line "# Fx,..." at the end of each controlled phase.
 */

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

// ============ 1. METHOD SELECTION (default at power-up; command M) ============
//  1 open loop            5 floating            9  PI
//  2 on-off               6 proportional        10 PI with anti-windup
//  3 on-off + hysteresis  7 P with manual reset 11 (PI-D, after Task 5)
//  4 three-position       8 integral            12 two-stage
int method = 2;

// ============ 2. CONTROLLER SETTINGS (defaults; commands KR, TI, KI, H, L) ============
float H_BAND = 0.5f;               // half hysteresis band [K]        methods 3, 4, 5
constexpr float R_RATE = 0.0011f;  // slewing rate [1/s]              method 5
float KR = 0.0f;                   // controller gain K_R [1/K]       methods 6, 7, 9, 10 - Task 3
float TI = 0.0f;                   // integral time T_i [s]           methods 9, 10       - Task 3
float KI = 0.0f;                   // integral gain [1/(K*s)] = KR/TI method 8            - Task 3
float TD = 0.0f;                   // derivative time [s]             method 11
float L_LOOP = 0.0f;               // loop delay [s]                  method 12           - Task 3

// ============ 3. PLANT CONSTANTS (LAB 1) ============
constexpr float DELTA_T = 16.6f;  // settled rise at u = 1, sensor at 120 mm [K] - group 6, LAB 1 Task 6
constexpr float T_P = 307.0f;     // time constant, sensor at 120 mm [s]      - group 6, LAB 1 Task 6
                                  // (LAB 1 delay of this sensor: L = 18.0 s)
constexpr float P_EL = 1.92f;     // electrical heater power [W], 12 V and 75 ohm (LAB 1 data)

// ============ 4. MEASUREMENT CHAIN (LAB 1) ============
constexpr uint8_t HEATER_PIN = 4;
constexpr uint8_t FAN_PIN = 5;  // used by method 4 only
// Group 6 station: the touch test of LAB 1 put A0 at 120 mm and A2 at 40 mm, the
// reverse of the order this program assumes. The pins are listed in reverse so that
// index 0 = 40 mm, 1 = 80 mm, 2 = 120 mm, CTRL_CH = 2 stays as specified, and the
// columns T1, T2, T3 of the log are NTC1, NTC2, NTC3 of the instruction.
constexpr uint8_t SENSOR_PINS[3] = { A2, A1, A0 };
constexpr uint8_t CTRL_CH = 2;  // 0=NTC1 40mm, 1=NTC2 80mm, 2=NTC3 120mm - LAB 2: NTC3 at every station

constexpr uint8_t ADC_BITS = 14;
constexpr uint32_t ADC_MAX = (1UL << ADC_BITS) - 1UL;
constexpr uint16_t AVG_COUNT = 100;

constexpr float R_FIXED = 10000.0f;
constexpr float NTC_R0 = 10000.0f;
constexpr float NTC_T0 = 298.15f;
constexpr float NTC_B = 3984.0f;

float K_CAL[3] = { 0.97898f, 1.03903f, 0.98416f };  // A2 (40 mm), A1 (80 mm), A0 (120 mm)

// ============ 5. MEASUREMENT PROTOCOL ============
constexpr float T_S = 10.0f;                    // controller period [s]
constexpr float T_M = 1.0f;                     // switching window [s] - Lecture 4, slide 38
constexpr float BAND = 0.5f;                    // settling band [K]
constexpr float T_ESS = 120.0f;                 // e_ss = mean e over the last T_ESS s of a phase
constexpr uint32_t APPROACH_MAX_S = 1200;       // approach to 5 K must finish within 20 min
constexpr float APPROACH_BAND = 0.2f;           // approach ends when |e| <= 0.2 K
constexpr float KR_HOLD = 0.2f;                 // fixed controller of approach and F0 [1/K],
                                                // u = u0 + KR_HOLD*e; 0.2-0.3 K_u, well inside
                                                // the stable range
constexpr uint32_t APPROACH_SETTLE_MS = 60000;  // approach also ends when c has settled:
constexpr float APPROACH_SETTLE_K = 0.05f;      // changed less than 0.05 K in 60 s

struct Phase {
  const char *name;
  float thetaZ;
  uint32_t dur_s;
  bool fixedHold;
};

const Phase PHASES[] = {
  { "F0", 5.0f, 300, true },    // after the approach: held at 5 K by u0 + KR_HOLD*e
  { "F1", 10.0f, 400, false },  // step up    5 -> 10 K
  { "F2", 7.0f, 240, false },   // step down 10 ->  7 K
  { "F3", 7.0f, 480, false },   // steady-state observation
};
constexpr uint8_t N_PHASES = sizeof(PHASES) / sizeof(PHASES[0]);

// ============ 6. PERFORMANCE INDICES ============
struct Metrics {
  float iae;       // sum |e|*T_s               [K*s]
  float ise;       // sum e^2*T_s               [K^2*s]
  float tv;        // sum |u[k]-u[k-1]|         [-]
  float uSum;      // sum u, for energy         [-]
  float thetaMax;  // highest theta in phase    [K]
  float thetaMin;  // lowest theta in phase     [K]
  uint32_t nOn;    // heater switch-ons         [-]
  float tLastOut;  // last instant outside band [s from phase start]
  bool outAtEnd;   // last sample outside the band: t5 not reached
  float eTail;     // sum e over the last T_ESS s  [K]
  uint32_t nTail;  // samples in eTail             [-]
};
Metrics M;

void metricsReset() {
  M.iae = M.ise = M.tv = M.uSum = M.eTail = 0.0f;
  M.nTail = 0;
  M.thetaMax = -1.0e9f;
  M.thetaMin = 1.0e9f;
  M.nOn = 0;
  M.tLastOut = 0.0f;
  M.outAtEnd = false;
}

void metricsUpdate(float e, float theta, float u, float uPrev, float tInPhase,
                   float durS) {
  if (tInPhase >= durS - T_ESS) {
    M.eTail += e;
    M.nTail++;
  }
  M.iae += fabsf(e) * T_S;
  M.ise += e * e * T_S;
  M.tv += fabsf(u - uPrev);
  M.uSum += u;
  if (theta > M.thetaMax) M.thetaMax = theta;
  if (theta < M.thetaMin) M.thetaMin = theta;
  M.outAtEnd = (fabsf(e) > BAND);
  if (M.outAtEnd) M.tLastOut = tInPhase;  // t_5 = last exit from the band
}

// ============ 7. TIME-PROPORTIONAL OUTPUT ============
// The heater has two states. Intermediate power is produced by switching it on
// for u*T_M in every window T_M. The profile averages this itself, since T_P >> T_M.
uint32_t windowStartMs = 0;
bool heaterState = false;
bool fanState = false;

void applyHeater(float u) {
  float dt = (millis() - windowStartMs) / 1000.0f;
  if (dt >= T_M) {
    windowStartMs = millis();
    dt = 0.0f;
  }
  bool on = (dt < u * T_M);
  if (on && !heaterState) M.nOn++;  // count switch-ons, not all transitions
  heaterState = on;
  digitalWrite(HEATER_PIN, on ? HIGH : LOW);
}

void applyFan(bool on) {
  fanState = on;
  digitalWrite(FAN_PIN, on ? HIGH : LOW);
}

// ============ 8. CONTROLLER ============
struct CtrlState {
  float u;
  float I;
  float thetaPrev;
};
CtrlState ctrl = { 0.0f, 0.0f, NAN };

void controllerReset(float u_init) {
  ctrl.u = u_init;
  ctrl.I = 0.0f;
  ctrl.thetaPrev = NAN;
}

// Set by every TODO block that has not been written yet. Delete the line
// "todoHit = true;" from a block once the block is written.
bool todoHit = false;

inline float clamp01(float x) {
  return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

// steady duty predicted by the plant model: u0 = r / DELTA_T
inline float u0_of(float thetaZ) {
  return clamp01(thetaZ / DELTA_T);
}

// Called every T_S seconds with the error e = r - c, the controlled variable
// c (theta) and the set point r (thetaZ). Returns u, 0...1.
float controller(float e, float theta, float thetaZ) {
  switch (method) {

    case 1:
      {  // open loop
        (void)e;
        (void)theta;
        ctrl.u = u0_of(thetaZ);
        break;
      }

    case 2:
      {  // on-off                    - Task 3
        (void)theta;
        (void)thetaZ;
        // Heater fully on below the set point, fully off at or above it.
        ctrl.u = (e > 0.0f) ? 1.0f : 0.0f;
        break;
      }
    case 3:
      {  // on-off with hysteresis
        (void)theta;
        (void)thetaZ;
        if (e > H_BAND) ctrl.u = 1.0f;
        else if (e < -H_BAND) ctrl.u = 0.0f;
        // inside the band: ctrl.u unchanged
        break;
      }

    case 4:
      {  // three-position: heat / off / fan
        (void)theta;
        (void)thetaZ;
        if (e > H_BAND) {
          ctrl.u = 1.0f;
          applyFan(false);
        } else if (e < -H_BAND) {
          ctrl.u = 0.0f;
          applyFan(true);
        } else {
          ctrl.u = 0.0f;
          applyFan(false);
        }
        break;
      }

    case 5:
      {  // single-speed floating
        (void)theta;
        (void)thetaZ;
        if (e > H_BAND) ctrl.u += R_RATE * T_S;
        else if (e < -H_BAND) ctrl.u -= R_RATE * T_S;
        ctrl.u = clamp01(ctrl.u);
        break;
      }

    case 6:
      {  // proportional              - Task 4
        (void)theta;
        (void)thetaZ;
        // Pure P control, u = KR*e, no u0 (method 7 adds it). Clamped to 0...1:
        // a negative e gives u = 0, since the heater cannot cool.
        ctrl.u = clamp01(KR * e);
        break;
      }
    case 7:
      {  // P with manual reset
        (void)theta;
        ctrl.u = clamp01(u0_of(thetaZ) + KR * e);
        break;
      }

    case 8:
      {  // integral
        (void)theta;
        (void)thetaZ;
        ctrl.u = clamp01(ctrl.u + KI * e * T_S);
        break;
      }

    case 9:
      {  // PI, no protection         - Task 5
        (void)theta;
        (void)thetaZ;
        // u = KR*(e + I/TI); I = integral of e [K*s], rectangular rule, one step T_S.
        float I_new = ctrl.I + e * T_S;
        float v = KR * (e + I_new / TI);  // command before clamping
        ctrl.I = I_new;                   // always integrate: wind-up allowed
        ctrl.u = clamp01(v);
        break;
      }
    case 10:
      {  // PI with anti-windup       - Task 5
        (void)theta;
        (void)thetaZ;
        // As method 9, one line changed: conditional integration.
        float I_new = ctrl.I + e * T_S;
        float v = KR * (e + I_new / TI);             // command before clamping
        if (v >= 0.0f && v <= 1.0f) ctrl.I = I_new;  // keep the update only if u is not saturated
        ctrl.u = clamp01(v);
        break;
      }
    case 12:
      {  // two-stage, near time-optimal
        (void)e;
        float s_g = (DELTA_T - thetaZ) / T_P;    // heating slope [K/s]
        float theta_sw = thetaZ - s_g * L_LOOP;  // switch early by the loop delay
        ctrl.u = (theta < theta_sw) ? 1.0f : u0_of(thetaZ);
        break;
      }

    default:
      ctrl.u = 0.0f;
      break;
  }
  return ctrl.u;
}

// methods that exist in this program
bool methodAvailable(int m) {
  return (m >= 1 && m <= 10) || m == 12;
}

// integral contribution to u, logged as I_term (Task 5); 0 without an integral
float iTerm() {
  if ((method >= 9 && method <= 11) && TI > 0.0f) return KR * ctrl.I / TI;
  return 0.0f;
}

// ============ 9. ACQUISITION (LAB 1) ============
float readAverageAdc(uint8_t pin) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < AVG_COUNT; ++i) sum += analogRead(pin);
  return (float)sum / AVG_COUNT;
}

bool adcToTemperature(float adc, uint8_t ch, float &tC) {
  if (adc <= 0.5f || adc >= (float)ADC_MAX - 0.5f) return false;
  float r = R_FIXED * adc / ((float)ADC_MAX - adc);
  r *= K_CAL[ch];  // multiplying correction, before eq. B
  float invT = 1.0f / NTC_T0 + logf(r / NTC_R0) / NTC_B;
  if (!isfinite(invT) || invT <= 0.0f) return false;
  tC = 1.0f / invT - 273.15f;
  return isfinite(tC);
}

// ============ 10. SEQUENCER, COMMANDS AND MAIN LOOP ============
enum Mode { IDLE,
            AMBIENT,
            RUN };
Mode mode = IDLE;
float T_amb = NAN;  // 01.10.26

uint8_t phaseIdx = 0;
bool approach = false;     // before F0: bring c to 5 K
float approachRefC = NAN;  // c at the start of the current 60 s settle window
uint32_t approachRefMs = 0;
uint32_t phaseStartMs = 0, runStartMs = 0;
uint32_t nextCtrlMs = 0, nextLogMs = 0, ambientEndMs = 0;
float ambSum = 0.0f;
uint32_t ambN = 0;
float uCmd = 0.0f, uPrevCmd = 0.0f;

void printSettings(bool hash = true) {
  if (hash) Serial.print(F("# "));
  Serial.print(F("method="));
  Serial.print(method);
  Serial.print(F(",KR="));
  Serial.print(KR, 4);
  Serial.print(F(",TI="));
  Serial.print(TI, 1);
  Serial.print(F(",KI="));
  Serial.print(KI, 5);
  Serial.print(F(",H="));
  Serial.print(H_BAND, 2);
  Serial.print(F(",L="));
  Serial.print(L_LOOP, 1);
  Serial.print(F(",DELTA_T="));
  Serial.print(DELTA_T, 2);
  Serial.print(F(",T_P="));
  Serial.print(T_P, 0);
  Serial.print(F(",CTRL_CH="));
  Serial.print(CTRL_CH);
  Serial.print(F(",T_amb="));
  Serial.println(T_amb, 3);
}

void printSummary(const Phase &p, float tInPhase) {
  float n_h = (tInPhase > 0.0f) ? M.nOn * 3600.0f / tInPhase : 0.0f;
  float E = P_EL * T_S * M.uSum;                         // energy delivered [J]
  float c_pp = M.thetaMax - M.thetaMin;                  // peak-to-peak swing [K]
  float c_p = M.thetaMax - p.thetaZ;                     // overshoot [K]; negative: none
  float e_ss = (M.nTail > 0) ? M.eTail / M.nTail : NAN;  // steady-state error [K]

  Serial.print(F("# "));
  Serial.print(p.name);
  Serial.print(F(",method="));
  Serial.print(method);
  Serial.print(F(",r="));
  Serial.print(p.thetaZ, 2);
  Serial.print(F(",t_phase="));
  Serial.print(tInPhase, 0);
  Serial.print(F(",e_ss="));
  Serial.print(e_ss, 3);
  Serial.print(F(",IAE="));
  Serial.print(M.iae, 2);
  Serial.print(F(",ISE="));
  Serial.print(M.ise, 2);
  Serial.print(F(",TV="));
  Serial.print(M.tv, 3);
  Serial.print(F(",n_on="));
  Serial.print(M.nOn);
  Serial.print(F(",n_h="));
  Serial.print(n_h, 1);
  Serial.print(F(",t5="));
  if (M.outAtEnd) Serial.print(F("not_reached"));
  else Serial.print(M.tLastOut, 0);
  Serial.print(F(",c_p="));
  Serial.print(c_p, 3);
  Serial.print(F(",c_pp="));
  Serial.print(c_pp, 3);
  Serial.print(F(",E="));
  Serial.println(E, 1);
}

// settings the selected method needs; false and a message if one is missing
bool settingsReady() {
  if ((method == 6 || method == 7 || method == 9 || method == 10 || method == 11)
      && KR <= 0.0f) {
    Serial.println(F("# ERROR: KR not set (command KR)"));
    return false;
  }
  if ((method == 9 || method == 10 || method == 11)
      && TI <= 0.0f) {
    Serial.println(F("# ERROR: TI not set (command TI)"));
    return false;
  }
  if (method == 8 && KI <= 0.0f) {
    Serial.println(F("# ERROR: KI not set (command KI)"));
    return false;
  }
  if (method == 11 && TD <= 0.0f) {
    Serial.println(F("# ERROR: TD not set (section 2)"));
    return false;
  }
  if (method == 12 && L_LOOP <= 0.0f) {
    Serial.println(F("# ERROR: L not set (command L)"));
    return false;
  }
  return true;
}

void startRun() {
  if (isnan(T_amb)) {
    Serial.println(F("# ERROR: T_amb unknown - command A (cold station) or T"));
    return;
  }
  if (!methodAvailable(method)) {
    Serial.println(F("# ERROR: no such method (command M)"));
    return;
  }
  if (!settingsReady()) return;
  todoHit = false;
  controller(0.0f, PHASES[0].thetaZ, PHASES[0].thetaZ);  // dry call: written?
  if (todoHit) {
    Serial.print(F("# ERROR: method "));
    Serial.print(method);
    Serial.println(F(" not written yet (TODO in section 8)"));
    return;
  }
  mode = RUN;
  phaseIdx = 0;
  approach = true;
  approachRefC = NAN;
  runStartMs = phaseStartMs = windowStartMs = millis();
  nextCtrlMs = nextLogMs = millis();
  controllerReset(u0_of(PHASES[0].thetaZ));
  uPrevCmd = uCmd = 0.0f;
  metricsReset();
  Serial.print(F("# START,"));
  printSettings(false);
  Serial.println(F("t_s,phase,r,T1,T2,T3,c,e,u,heater,I_term"));
}

void stopRun(const __FlashStringHelper *why) {
  mode = IDLE;
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  Serial.println(why);
}

// one command line; returns nothing, prints the reply
void handleLine(char *s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == 0) return;
  char cmd[3] = { 0, 0, 0 };
  uint8_t n = 0;
  while (n < 2 && ((*s >= 'A' && *s <= 'Z') || (*s >= 'a' && *s <= 'z') || *s == '?')) {
    cmd[n++] = (char)toupper(*s);
    s++;
  }
  while (*s == ' ' || *s == '=' || *s == '\t') s++;
  bool hasArg = (*s != 0);
  float v = hasArg ? (float)atof(s) : NAN;

  if (!strcmp(cmd, "S")) {
    stopRun(F("# STOP"));
    return;
  }
  if (!strcmp(cmd, "?")) {
    printSettings();
    return;
  }
  if (mode == RUN || mode == AMBIENT) {
    Serial.println(F("# ERROR: busy - wait for # END or send S"));
    return;
  }
  if (!strcmp(cmd, "R")) {
    startRun();
    return;
  }
  if (!strcmp(cmd, "A")) {
    mode = AMBIENT;
    ambSum = 0.0f;
    ambN = 0;
    ambientEndMs = millis() + 180000UL;
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    Serial.println(F("# measuring ambient, 180 s, heater off"));
    return;
  }
  if (!hasArg || !isfinite(v)) {
    Serial.println(F("# ERROR: command needs a number, e.g. KR 0.25"));
    return;
  }
  if (!strcmp(cmd, "T") && v > -10.0f && v < 45.0f) T_amb = v;
  else if (!strcmp(cmd, "M") && methodAvailable((int)v)) method = (int)v;
  else if (!strcmp(cmd, "KR") && v > 0.0f && v < 20.0f) KR = v;
  else if (!strcmp(cmd, "TI") && v > 0.0f && v < 10000.0f) TI = v;
  else if (!strcmp(cmd, "KI") && v > 0.0f && v < 1.0f) KI = v;
  else if (!strcmp(cmd, "H") && v >= 0.0f && v <= 5.0f) H_BAND = v;
  else if (!strcmp(cmd, "L") && v > 0.0f && v < 600.0f) L_LOOP = v;
  else {
    Serial.println(F("# ERROR: unknown command or value out of range"));
    return;
  }
  printSettings();
}

char lineBuf[40];
uint8_t lineLen = 0;

void readCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      lineBuf[lineLen] = 0;
      if (lineLen > 0) handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  analogReadResolution(ADC_BITS);
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.println(F("# ADA512 LAB2. One command per line: A, T, M, KR, TI, KI, H, L, ?, R, S"));
  printSettings();
}

void loop() {
  readCommands();

  // --- read temperatures ---
  float T[3];
  bool ok[3];
  for (uint8_t i = 0; i < 3; ++i)
    ok[i] = adcToTemperature(readAverageAdc(SENSOR_PINS[i]), i, T[i]);

  // --- ambient measurement mode ---
  if (mode == AMBIENT) {
    if (ok[CTRL_CH]) {
      ambSum += T[CTRL_CH];
      ambN++;
    }
    if ((int32_t)(millis() - ambientEndMs) >= 0) {
      T_amb = (ambN > 0) ? ambSum / ambN : NAN;
      mode = IDLE;
      Serial.print(F("# T_amb="));
      Serial.print(T_amb, 3);
      Serial.print(F(" C, samples="));
      Serial.print(ambN);
      Serial.println(F(" - note it: after an upload enter it with T"));
    }
    return;
  }

  if (mode != RUN) {
    digitalWrite(HEATER_PIN, LOW);
    return;
  }

  // --- controlled variables ---
  const Phase &p = PHASES[phaseIdx];
  float theta = ok[CTRL_CH] ? (T[CTRL_CH] - T_amb) : NAN;
  float e = p.thetaZ - theta;
  float tInPhase = (millis() - phaseStartMs) / 1000.0f;
  float tRun = (millis() - runStartMs) / 1000.0f;

  // --- controller, every T_S ---
  if ((int32_t)(millis() - nextCtrlMs) >= 0) {
    nextCtrlMs += (uint32_t)(T_S * 1000.0f);
    uPrevCmd = uCmd;
    if (isnan(theta)) {
      uCmd = 0.0f;  // sensor fault: switch off
    } else if (p.fixedHold) {
      // approach and F0: fixed controller of the program, the same for every
      // method, so that every run enters F1 from c = 5 K
      uCmd = clamp01(u0_of(p.thetaZ) + KR_HOLD * e);
      if (approach) {
        // ends in the band, or when c has settled outside it (the fixed
        // controller is proportional and may leave a small offset), or after
        // 20 min; the run always goes on to F0
        bool inBand = (fabsf(e) <= APPROACH_BAND);
        bool settled = false;
        if (isnan(approachRefC)) {
          approachRefC = theta;
          approachRefMs = millis();
        } else if (millis() - approachRefMs >= APPROACH_SETTLE_MS) {
          settled = (fabsf(theta - approachRefC) < APPROACH_SETTLE_K);
          approachRefC = theta;
          approachRefMs = millis();
        }
        bool timeout = (tInPhase > (float)APPROACH_MAX_S);
        if (inBand || settled || timeout) {
          approach = false;
          phaseStartMs = millis();
          tInPhase = 0.0f;
          if (inBand) {
            Serial.print(F("# APPROACH done at t_s="));
            Serial.println(tRun, 0);
          } else {
            Serial.print(F("# WARNING: approach ended at c="));
            Serial.print(theta, 2);
            Serial.print(settled ? F(" K, settled outside 5 +/- 0.2 K")
                                 : F(" K, after 20 min"));
            Serial.print(F(" - F1 starts from the last F0 line of the log; t_s="));
            Serial.println(tRun, 0);
          }
        }
      }
    } else {
      uCmd = controller(e, theta, p.thetaZ);
      if (!isnan(theta))
        metricsUpdate(e, theta, uCmd, uPrevCmd, tInPhase, (float)p.dur_s);
    }
  }

  applyHeater(uCmd);

  // --- logging, every 1 s ---
  if ((int32_t)(millis() - nextLogMs) >= 0) {
    nextLogMs += 1000UL;
    Serial.print(tRun, 1);
    Serial.print(',');
    Serial.print(approach ? "AP" : p.name);
    Serial.print(',');
    Serial.print(p.thetaZ, 2);
    Serial.print(',');
    for (uint8_t i = 0; i < 3; ++i) {
      if (ok[i]) Serial.print(T[i], 4);
      else Serial.print(F("invalid"));
      Serial.print(',');
    }
    if (ok[CTRL_CH]) {
      Serial.print(theta, 4);
      Serial.print(',');
      Serial.print(e, 4);
    } else {
      Serial.print(F("invalid,invalid"));
    }
    Serial.print(',');
    Serial.print(uCmd, 4);
    Serial.print(',');
    Serial.print(heaterState ? 1 : 0);
    Serial.print(',');
    Serial.println(iTerm(), 4);
  }

  // --- advance to next phase ---
  if (!approach && tInPhase >= (float)p.dur_s) {
    if (!p.fixedHold) printSummary(p, tInPhase);
    phaseIdx++;
    if (phaseIdx >= N_PHASES) {
      stopRun(F("# END"));
      return;
    }
    phaseStartMs = millis();
    metricsReset();
    // F1 is the comparison phase: every method enters it from the same
    // controller state (integral cleared). No reset before F2 and F3 -
    // there the controller carries its own history on.
    if (phaseIdx == 1) controllerReset(uCmd);
  }
}
