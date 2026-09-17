const int SENSOR_PINS[] = {A0, A1, A2};
const int NUM_SENSORS = 3;

const int FAN = 2;                                  // Heater resistor control pin
const int HEATER = 4;                               // Fan control pin

const float SERIES_RESISTOR = 10000.0;              // Reference resistor value in ohms
const float NOMINAL_RESISTANCE = 10000.0;           // Nominal thermistor resistance at 25°C
const float NOMINAL_TEMPERATURE = 25.0;             // Nominal temperature in °C
const float BETA_COEFFICIENT = 3984.0;              // Beta coefficient for thermistor
const float ADC_MAX = 16383.0;                      // 14-bit ADC max value

const long INTERVAL = 1000;                         // Sampling interval in milliseconds
const int NUM_SAMPLES = 100;                        // Number of ADC samples to average

unsigned long previousMillis = 0;

const int NUM_CAL = 120;                            // records in the calibration set (2 min at 1 s)
float calR[NUM_SENSORS][NUM_CAL];
int   calCount = -1;                                // -1 = not collecting
const float K_CAL[NUM_SENSORS] = {0.98416, 1.03903 0.97898};   // fill in from the "# k_i" line    

void setup() {
  Serial.begin(9600);
  analogReadResolution(14);                         // Set ADC resolution to 14-bit

  pinMode(HEATER, OUTPUT);
  pinMode(FAN, OUTPUT);

  // Optionally start with heater and fan off
  digitalWrite(HEATER, LOW);
  digitalWrite(FAN, LOW);

  // Print CSV header
  Serial.println("Timestamp_ms,PeriodError_ms,Res1_Ohm,Temp1_C,Res2_Ohm,Temp2_C,Res3_Ohm,Temp3_C");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL) {
    long jitter = currentMillis - (previousMillis + INTERVAL);
    previousMillis += INTERVAL;

    float resistances[NUM_SENSORS];
    float temperatures[NUM_SENSORS];

    // ---- read the three channels ----
    for (int i = 0; i < NUM_SENSORS; i++) {
      float avgADC = averageADC(SENSOR_PINS[i], NUM_SAMPLES);

      if (avgADC <= 10) {                        // input at ground: NTC shorted
        resistances[i] = -1.0;
        temperatures[i] = -999.9;
      } else if (avgADC >= ADC_MAX - 10) {       // input at supply: NTC open
        resistances[i] = -1.0;
        temperatures[i] = -999.9;
      } else {
        resistances[i] = calculateResistance(avgADC);                           // raw R, logged
        temperatures[i] = calculateTempFromResistance(K_CAL[i] * resistances[i]); // corrected before conversion
      }
    }

    // ---- calibration set: one record per cycle, bounded by NUM_CAL ----

    // To calibrate:
    // Let the station rest with the heater off for 30 min.
    // Check that no channel moves more than 0.02 K over two minutes.
    // Send cal and wait two minutes.
    // Copy the three numbers from the # k_i = line into K_CAL and re-flash.
    
    if (calCount >= 0 && calCount < NUM_CAL) {
      bool valid = true;
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (resistances[i] < 0) valid = false;
      }

      if (valid) {
        for (int i = 0; i < NUM_SENSORS; i++) {
          calR[i][calCount] = resistances[i];
        }
        calCount++;

        if (calCount == NUM_CAL) {               // set complete: compute and report once
          float mean[NUM_SENSORS];
          float all = 0;
          for (int i = 0; i < NUM_SENSORS; i++) {
            mean[i] = 0;
            for (int k = 0; k < NUM_CAL; k++) {
              mean[i] += calR[i][k];
            }
            mean[i] /= NUM_CAL;
            all += mean[i] / NUM_SENSORS;
          }

          Serial.print("# k_i =");
          for (int i = 0; i < NUM_SENSORS; i++) {
            Serial.print(" ");
            Serial.print(all / mean[i], 5);
          }
          Serial.println();

          calCount = -1;                         // stop collecting
        }
      }
    }

    // ---- print CSV record ----
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(jitter);
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(",");
      Serial.print(resistances[i], 2);
      Serial.print(",");
      Serial.print(temperatures[i], 2);
    }
    Serial.println();
  }

  // ---- serial commands: heater on | heater off | fan on | fan off | cal ----
  if (Serial.available() > 0) {
    String incomingMessage = Serial.readStringUntil('\n');
    incomingMessage.trim();

    if (incomingMessage == "heater on") {
      digitalWrite(HEATER, HIGH);
      Serial.println("# Heater is now turned ON");
    } else if (incomingMessage == "heater off") {
      digitalWrite(HEATER, LOW);
      Serial.println("# Heater is now turned OFF");
    } else if (incomingMessage == "fan on") {
      digitalWrite(FAN, HIGH);
      Serial.println("# Fan is now turned ON");
    } else if (incomingMessage == "fan off") {
      digitalWrite(FAN, LOW);
      Serial.println("# Fan is now turned OFF");
    } else if (incomingMessage == "cal") {
      calCount = 0;
      Serial.println("# calibration set started");
    }
  }
}

// Average multiple ADC samples for noise reduction
float averageADC(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (float)sum / samples;
}

// Calculate resistance from ADC value using voltage divider formula
float calculateResistance(float adcVal) {
  return SERIES_RESISTOR / (ADC_MAX / adcVal - 1.0);
}

// Calculate temperature from resistance using Beta formula
float calculateTempFromResistance(float resistance) {
  float steinhart;
  steinhart = log(resistance / NOMINAL_RESISTANCE);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;  // Convert Kelvin to Celsius
  return steinhart;
}