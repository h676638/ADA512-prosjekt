const int SENSOR_PINS[] = {A0, A1, A2};
const int NUM_SENSORS = 3;

const int FAN = 2;  // Heater resistor control pin
const int HEATER = 4;     // Fan control pin

const float SERIES_RESISTOR = 10000.0;              // Reference resistor value in ohms
const float NOMINAL_RESISTANCE = 10000.0;           // Nominal thermistor resistance at 25°C
const float NOMINAL_TEMPERATURE = 25.0;             // Nominal temperature in °C
const float BETA_COEFFICIENT = 3984.0;              // Beta coefficient for thermistor
const float ADC_MAX = 16383.0;                      // 14-bit ADC max value

const long INTERVAL = 1000;                         // Sampling interval in milliseconds
const int NUM_SAMPLES = 100;                         // Number of ADC samples to average

unsigned long previousMillis = 0;

float calibrationA0[NUM_SAMPLES];
float calibrationA1[NUM_SAMPLES];
float calibrationA2[NUM_SAMPLES];
float* calibrationMeasures[3] = {calibrationA0, calibrationA1, calibrationA2};
int j = 0;

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

  if (calibrationA0[NUM_SAMPLES] != 0.0) {
    float sumA0 = 0;
    float sumA1 = 0;
    float sumA2 = 0;
    for (int i = 0; i < NUM_SAMPLES;i++) {
      sumA0 += calibrationA0[i];
      sumA1 += calibrationA1[i];
      sumA2 += calibrationA2[i];
    }
    sumA0 /= NUM_SAMPLES;
    sumA1 /= NUM_SAMPLES;
    sumA2 /= NUM_SAMPLES;

    float sum = (sumA0 + sumA1 + sumA2)/3;
    Serial.println(sum-sumA0);
    Serial.println(sum-sumA1);
    Serial.println(sum-sumA2);
  }

  if (currentMillis - previousMillis >= INTERVAL) {
    long jitter = currentMillis - (previousMillis + INTERVAL);
    previousMillis += INTERVAL;

    float resistances[NUM_SENSORS];
    float temperatures[NUM_SENSORS];

    for (int i = 0; i < NUM_SENSORS; i++) {
      float avgADC = averageADC(SENSOR_PINS[i], NUM_SAMPLES);

      if (avgADC <= 10) {  // Near zero ADC reading, likely short or disconnected
        resistances[i] = -1.0;
        temperatures[i] = -999.9;
      } else if (avgADC >= ADC_MAX - 10) {  // Near max ADC reading, likely open circuit
        resistances[i] = -1.0;
        temperatures[i] = -999.9;
      } else {
        resistances[i] = calculateResistance(avgADC);
        temperatures[i] = calculateTempFromResistance(resistances[i]);

        if (i == 1) {
          temperatures[i] -= 1.05;  // Calibration offset in °C
        }
      }
      // Calibration
      calibrationMeasures[i][j] = avgADC;
      j++;
    }

    // Print CSV record
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

  // Serial command handling to turn on/off heeater or fan during runtime
  // Use commands: <fan on>, <fan off>, <heater on> and <heater off>
  if (Serial.available() > 0) {
    String incomingMessage = Serial.readStringUntil('\n');
    incomingMessage.trim();  // Remove any trailing newline or spaces

    if (incomingMessage == "heater on") {
      digitalWrite(HEATER, HIGH);
      Serial.println("Heater is now turned ON");
    } else if (incomingMessage == "heater off") {
      digitalWrite(HEATER, LOW);
      Serial.println("Heater is now turned OFF");
    } else if (incomingMessage == "fan on") {
      digitalWrite(FAN, HIGH);
      Serial.println("Fan is now turned ON");
    } else if (incomingMessage == "fan off") {
      digitalWrite(FAN, LOW);
      Serial.println("Fan is now turned OFF");
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