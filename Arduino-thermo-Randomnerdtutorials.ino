#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into digital pin 2 on the Arduino
#define TempSens1 1
#define TempSens2 2
#define TempSens3 3


// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to the temperature sensor library
R4Temperature sensors(&oneWire);

void setup(void) {
  // Start serial communication at 9600 baud
  Serial.begin(9600);

  // Start the library
  sensors.begin();

  Serial.println("Locating temperature sensors...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
}

void loop(void) {
  // Request a temperature reading from all devices
  sensors.requestTemperatures();

  // Read and print temperature for each sensor index (0, 1, and 2)
  Serial.print("Sensor 1 (Index 0): ");
  Serial.print(sensors.getTempCByIndex(0));
  Serial.println(" deg C");

  Serial.print("Sensor 2 (Index 1): ");
  Serial.print(sensors.getTempCByIndex(1));
  Serial.println(" deg C");

  Serial.print("Sensor 3 (Index 2): ");
  Serial.print(sensors.getTempCByIndex(2));
  Serial.println(" deg C");

  Serial.println("------------------");
  delay(1000); // Wait 1 second between readings
}