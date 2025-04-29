#include <ECE3.h> // Uncomment if the library is required

uint16_t sensorValues[8];

void setup() {
  ECE3_Init();
  Serial.begin(9600); // Set the data rate in bits per second for serial data transmission
  delay(2000);
}

void loop() {
  ECE3_read_IR(sensorValues);

  const unsigned int maxValues[8] = {1627, 1696, 1249, 1201, 1081, 1252, 1587, 1721};
  const unsigned int minValues[8] = {873, 804, 711, 664, 646, 642, 711, 779};
  double values[8];

  for (unsigned char i = 0; i < 8; i++) {

    int raw = sensorValues[i];
    if (raw < minValues[i]) raw = minValues[i];
    if (raw > maxValues[i]) raw = maxValues[i];

    values[i] = (raw - minValues[i]) * 1000.0 / maxValues[i] - minValues[i];
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // Tab to format the raw data into columns in the Serial monitor
  }

  double error = (-8 * values[0] - 4 * values[1] - 2 * values[2] - values[3] +
                  values[4] + 2 * values[5] + 4 * values[6] + 8 * values[7]) / 4.0;
  Serial.print("error: ");
  Serial.println(error);

  Serial.println();
  delay(50);
}
