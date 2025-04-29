// #include <ECE3.h>

uint16_t sensorValues[8];

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data
                      // transmission
  delay(2000);
}

void loop() {
  const unsigned int maxValues = {1627,	1696,	1249,	1201,	1081,	1252,	1587,	1721}
  const unsigned int minValues = {873, 804, 711, 664, 646, 642, 711, 779}
  // read raw sensor values
  ECE3_read_IR(sensorValues);
  double values[8];

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance
  for (unsigned char i = 0; i < 8; i++) {

    values[i] = (sensorValues[i] - min[i]) * 1000.0/max[i]
    Serial.print(values[i]);

    Serial.print(sensorValues[i]);
    Serial.print(
        '\t'); // tab to format the raw data into columns in the Serial monitor
  }

  double error = (-8*values[0]-4*values[1]-2*values[2]-values[3]+values[4]+2*values[5]+4*values[6]+8*values[7])/4;
  Serial.print("error: ", error)
}


Serial.println();

delay(500);
}
