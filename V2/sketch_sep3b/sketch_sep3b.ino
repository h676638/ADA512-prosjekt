const int SENSOR_PINS[] = {A0, A1, A2};
const int NUM_SENSORS = 3;

const float SERIES_RESISTOR = 10000.0;
const float NOMINAL_RESISTANCE = 10000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float BETA_COEFFICIENT = 3950.0;
const float ADC_MAX = 16383.0; // 14-bit max

unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  Serial.begin(9600);
  analogReadResolution(14);
}


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    for (int i = 0; i < NUM_SENSORS; i++) {
      float rawVal = analogRead(SENSOR_PINS[i]);
      float tempC = calculateTemp(rawVal);
      
      // Apply calibration offset only to sensor A1 (index 1)
      if (i == 1) {
        tempC = tempC - 1.1;
      }
      
      Serial.print(tempC);
      if (i < NUM_SENSORS - 1) Serial.print(",");
      else Serial.println();
    }
  }
}

float calculateTemp(float adcVal) {
  if (adcVal <= 0) return -273.15;
  if (adcVal >= ADC_MAX) return 999.9;
  float resistance = SERIES_RESISTOR / (ADC_MAX / adcVal - 1.0);
  float steinhart = log(resistance / NOMINAL_RESISTANCE);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  return steinhart;
}

 
