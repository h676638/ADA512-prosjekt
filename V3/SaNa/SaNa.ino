const int SENSOR_PINS[] = {A0, A1, A2};
const int NUM_SENSORS = 3;

const float SERIES_RESISTOR = 10000.0;           // Reference resistor value in ohms
const float NOMINAL_RESISTANCE = 10000.0;        // Nominal thermistor resistance at 25°C
const float NOMINAL_TEMPERATURE = 25.0;          // Nominal temperature in °C
const float BETA_COEFFICIENT = 3984.0;            // Beta coefficient for thermistor
const float ADC_MAX = 16383.0;                    // 14-bit ADC max value

const long INTERVAL = 1000;                       // Sampling interval in milliseconds
const int NUM_SAMPLES = 100;                       // Number of ADC samples to average

unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  analogReadResolution(14);                        // Set ADC resolution to 14-bit
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to sample
  if (currentMillis - previousMillis >= INTERVAL) {
    // Calculate jitter (how late we are compared to ideal schedule)
    long jitter = currentMillis - (previousMillis + INTERVAL);

    // Update previousMillis by adding interval to reduce cumulative drift
    previousMillis += INTERVAL;

    Serial.print("Period error (ms): ");
    Serial.println(jitter);

    // Loop through each sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
      // Take multiple ADC samples and average to reduce noise
      float avgADC = averageADC(SENSOR_PINS[i], NUM_SAMPLES);

      // Calculate temperature from averaged ADC value
      float tempC = calculateTemp(avgADC);

      // Apply calibration offset only to sensor A1 (index 1) as example
      if (i == 1) {
        tempC = tempC - 1.1;  // Calibration offset in °C
      }

      // Print temperature values separated by commas
      Serial.print(tempC);
      if (i < NUM_SENSORS - 1) Serial.print(",");
      else Serial.println();
    }
  }
}

// Function to average multiple ADC samples for noise reduction
float averageADC(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (float)sum / samples;
}

// Function to calculate temperature from ADC value using Steinhart-Hart equation
float calculateTemp(float adcVal) {
  if (adcVal <= 0) return -273.15;               // Return absolute zero if invalid
  if (adcVal >= ADC_MAX) return 999.9;           // Return error if ADC max exceeded

  // Calculate resistance of thermistor using voltage divider formula
  float resistance = SERIES_RESISTOR / (ADC_MAX / adcVal - 1.0);

  // Calculate temperature using Beta formula
  float steinhart = log(resistance / NOMINAL_RESISTANCE);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;                            // Convert Kelvin to Celsius

  return steinhart;
}
