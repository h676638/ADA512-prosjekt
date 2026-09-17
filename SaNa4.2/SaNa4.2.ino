// ADA512 Lab 1 - acquisition, calibration, temperature and logging (Tasks 3-4)
//
// Serial commands (end with newline):
//   heater on | heater off | fan on | fan off
//   cal    - collect 120 rest records, report drift, k_i and channel spread
//   noise  - averaging study for Task 3(d); do NOT run during a heating record
//
// Log record (Task 4c), one per period:
//   t_ms, dk_us, R_40, R_80, R_120, T_40, T_80, T_120, heater
// Resistances are RAW (uncorrected); temperatures use K_CAL * R.
// Lines starting with '#' are comments/events.

// ---------------- pins ----------------
const int HEATER = 3;   // lab sheet: heater switched from D3 -- check where your wire goes
const int FAN    = 2;   // not used in Lab 1 -- check before Lab 2

const int NUM_SENSORS = 3;
const int SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2};
const int POS_MM[NUM_SENSORS]      = {40, 80, 120};   // set from the Task 1 touch test

// ---------------- measuring chain ----------------
const float R1      = 10000.0;   // divider resistor, 5 V side
const float R25     = 10000.0;
const float T25_K   = 298.15;
const float BETA    = 3984.0;
const float ADC_MAX = 16383.0;   // 14 bit

// Valid window = the NTC's rated range, -40..125 degC.
const float ADC_SHORT = 552.0;    // below: R < 349 ohm  -> input shorted
const float ADC_OPEN  = 15997.0;  // above: R > 415 kohm -> input open
const int   ADC_FLOAT = 300;      // max-min within one block above this -> floating input
                                  // (set well above the spread seen in the noise test)

// ---------------- acquisition ----------------
const unsigned long TS_US = 1000000UL;   // Ts = 1 s
const int NUM_SAMPLES = 100;             // justify with the "noise" command

const float K_CAL[NUM_SENSORS] = {1.0, 1.0, 1.0};   // paste from "# k_i =" and re-flash

enum { ST_OK, ST_SHORT, ST_OPEN, ST_FLOAT };

unsigned long nextTick_us = 0;
unsigned long prevTick_us = 0;
bool firstCycle = true;
bool heaterOn = false;

// ---------------- calibration set ----------------
const int NUM_CAL  = 120;   // 2 min at Ts = 1 s
const int DRIFT_WIN = 10;   // records averaged at each end for the drift check
float calR[NUM_SENSORS][NUM_CAL];
int calCount = -1;          // -1 = not collecting

// ---------------- serial input ----------------
char cmdBuf[32];
int cmdLen = 0;

// =================================================================
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  analogReadResolution(14);   // without this the R4 reads 10 bit

  pinMode(HEATER, OUTPUT);    // without OUTPUT only a weak pull-up -> no heating
  pinMode(FAN, OUTPUT);
  digitalWrite(HEATER, LOW);
  digitalWrite(FAN, LOW);

  // configuration line, so the log documents how it was made
  Serial.print("# Ts_us="); Serial.print(TS_US);
  Serial.print(" N="); Serial.print(NUM_SAMPLES);
  Serial.print(" B="); Serial.print(BETA, 0);
  Serial.print(" K_CAL=");
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(K_CAL[i], 5); Serial.print(" "); }
  Serial.println();

  Serial.print("t_ms,dk_us");
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(",R_"); Serial.print(POS_MM[i]); }
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(",T_"); Serial.print(POS_MM[i]); }
  Serial.println(",heater");

  nextTick_us = micros() + TS_US;
}

// =================================================================
void loop() {
  unsigned long now = micros();

  if ((long)(now - nextTick_us) >= 0) {
    // ---- Task 3(a,b): fixed period, period error ----
    unsigned long tk = now;
    long dk_us = firstCycle ? 0 : (long)(tk - prevTick_us) - (long)TS_US;
    prevTick_us = tk;
    firstCycle = false;

    nextTick_us += TS_US;                       // schedule does not depend on execution time
    if ((long)(now - nextTick_us) >= 0) {       // fell a whole period behind (e.g. noise test)
      nextTick_us = now + TS_US;                // resync instead of bursting
    }
    unsigned long t_ms = millis();

    // ---- Task 3(c) + 4(a,b): read, check, convert ----
    float R[NUM_SENSORS], T[NUM_SENSORS];
    int st[NUM_SENSORS];
    bool allValid = true;

    for (int i = 0; i < NUM_SENSORS; i++) {
      float adc;
      st[i] = readChannel(SENSOR_PINS[i], NUM_SAMPLES, adc);
      if (st[i] == ST_OK) {
        R[i] = resistanceFromADC(adc);                    // raw, logged
        T[i] = tempFromResistance(K_CAL[i] * R[i]);       // corrected before conversion
      } else {
        allValid = false;
      }
    }

    // ---- Task 3(e): calibration set ----
    if (calCount >= 0) {
      if (!allValid) {
        Serial.println("# cal aborted: invalid channel");
        calCount = -1;
      } else {
        for (int i = 0; i < NUM_SENSORS; i++) calR[i][calCount] = R[i];
        calCount++;
        if (calCount == NUM_CAL) {
          reportCalibration();
          calCount = -1;
        }
      }
    }

    // ---- Task 4(c): log record ----
    Serial.print(t_ms);
    Serial.print(",");
    Serial.print(dk_us);
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(",");
      if (st[i] == ST_OK) Serial.print(R[i], 1);
      else                Serial.print("nan");
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(",");
      if      (st[i] == ST_OK)    Serial.print(T[i], 3);
      else if (st[i] == ST_SHORT) Serial.print("SHORT");
      else if (st[i] == ST_OPEN)  Serial.print("OPEN");
      else                        Serial.print("FLOAT");
    }
    Serial.print(",");
    Serial.println(heaterOn ? 1 : 0);
  }

  pollSerial();   // non-blocking, does not disturb the period
}

// =================================================================
// Averages n readings; classifies the input.
int readChannel(int pin, int n, float &mean) {
  long sum = 0;
  int lo = 32767, hi = -1;
  for (int k = 0; k < n; k++) {
    int v = analogRead(pin);
    sum += v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  mean = (float)sum / n;
  if (mean < ADC_SHORT)       return ST_SHORT;
  if (mean > ADC_OPEN)        return ST_OPEN;
  if (hi - lo > ADC_FLOAT)    return ST_FLOAT;
  return ST_OK;
}

// n/(N-1) = R/(R+R1)  ->  R = R1 * n / (N-1-n)
float resistanceFromADC(float n) {
  return R1 * n / (ADC_MAX - n);
}

// inverted B equation
float tempFromResistance(float R) {
  return 1.0 / (1.0 / T25_K + log(R / R25) / BETA) - 273.15;
}

float tempFromADC(float n) {
  return tempFromResistance(resistanceFromADC(n));
}

// =================================================================
void reportCalibration() {
  float mean[NUM_SENSORS], first[NUM_SENSORS], last[NUM_SENSORS];
  float all = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    mean[i] = first[i] = last[i] = 0;
    for (int k = 0; k < NUM_CAL; k++) mean[i] += calR[i][k];
    for (int k = 0; k < DRIFT_WIN; k++) {
      first[i] += calR[i][k];
      last[i]  += calR[i][NUM_CAL - DRIFT_WIN + k];
    }
    mean[i]  /= NUM_CAL;
    first[i] /= DRIFT_WIN;
    last[i]  /= DRIFT_WIN;
    all += mean[i] / NUM_SENSORS;
  }

  // condition: no channel moves more than 0.02 K over two minutes
  bool stable = true;
  Serial.print("# drift_K =");
  for (int i = 0; i < NUM_SENSORS; i++) {
    float d = tempFromResistance(last[i]) - tempFromResistance(first[i]);
    if (fabs(d) > 0.02) stable = false;
    Serial.print(" "); Serial.print(d, 4);
  }
  Serial.println(stable ? "  (stable)" : "  (NOT stable - do not use these k_i)");

  // new factors
  Serial.print("# k_i =");
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(" "); Serial.print(all / mean[i], 5); }
  Serial.println();

  // spread without correction and with the factors currently flashed
  float tRaw[NUM_SENSORS], tCur[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    tRaw[i] = tempFromResistance(mean[i]);
    tCur[i] = tempFromResistance(K_CAL[i] * mean[i]);
  }
  printTemps("# T_raw_C =", tRaw);
  printTemps("# T_with_flashed_K_CAL_C =", tCur);
}

void printTemps(const char *label, float t[]) {
  float lo = t[0], hi = t[0];
  Serial.print(label);
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(" "); Serial.print(t[i], 3);
    if (t[i] < lo) lo = t[i];
    if (t[i] > hi) hi = t[i];
  }
  Serial.print("  spread_K = ");
  Serial.println(hi - lo, 3);
}

// =================================================================
// Task 3(d): standard deviation of the block mean, in K, for several N.
// Falls as 1/sqrt(N) while noise is random; the N where it stops falling
// justifies NUM_SAMPLES. Also prints the resolution (K per ADC count).
void noiseStudy() {
  const int NS[] = {1, 4, 16, 64, 256};
  const int NN = 5;
  const int BLOCKS = 30;

  Serial.println("# noise: channel, K_per_count, then std_K for N = 1 4 16 64 256");
  for (int i = 0; i < NUM_SENSORS; i++) {
    float ref;
    readChannel(SENSOR_PINS[i], 64, ref);
    float kPerCount = fabs(tempFromADC(ref + 1.0) - tempFromADC(ref));

    Serial.print("# noise "); Serial.print(POS_MM[i]); Serial.print("mm ");
    Serial.print(kPerCount, 5);

    for (int j = 0; j < NN; j++) {
      double s = 0, s2 = 0;               // deviations from ref: avoids float cancellation
      for (int b = 0; b < BLOCKS; b++) {
        float m;
        readChannel(SENSOR_PINS[i], NS[j], m);
        double d = m - ref;
        s += d; s2 += d * d;
      }
      double mu = s / BLOCKS;
      double var = (s2 - BLOCKS * mu * mu) / (BLOCKS - 1);
      if (var < 0) var = 0;
      Serial.print(" "); Serial.print(sqrt(var) * kPerCount, 5);
    }
    Serial.println();
  }
}

// =================================================================
void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

void handleCommand(const char *cmd) {
  if (strcmp(cmd, "heater on") == 0) {
    digitalWrite(HEATER, HIGH); heaterOn = true;
    Serial.print("# heater ON at t_ms="); Serial.println(millis());
  } else if (strcmp(cmd, "heater off") == 0) {
    digitalWrite(HEATER, LOW); heaterOn = false;
    Serial.print("# heater OFF at t_ms="); Serial.println(millis());
  } else if (strcmp(cmd, "fan on") == 0) {
    digitalWrite(FAN, HIGH);
    Serial.println("# fan ON");
  } else if (strcmp(cmd, "fan off") == 0) {
    digitalWrite(FAN, LOW);
    Serial.println("# fan OFF");
  } else if (strcmp(cmd, "cal") == 0) {
    if (heaterOn) Serial.println("# warning: heater is on");
    calCount = 0;
    Serial.println("# calibration set started (2 min)");
  } else if (strcmp(cmd, "noise") == 0) {
    noiseStudy();
  } else {
    Serial.print("# unknown command: "); Serial.println(cmd);
  }
}
