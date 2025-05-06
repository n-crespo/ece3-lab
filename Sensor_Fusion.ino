#include <ECE3.h> // Uncomment if the library is required

uint16_t sensorValues[8];


// for moving the car:
const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;  // DIR_L
const int left_pwm_pin = 40;  // PWML (pin #)
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int LED_RF = 41;

void setup() {

  // for moving the car:
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  digitalWrite(right_dir_pin, HIGH); // set this to high for donut!
  digitalWrite(right_nslp_pin, HIGH);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);
  pinMode(LED_RF, OUTPUT);

  ECE3_Init();
  Serial.begin(9600); // Set the data rate in bits per second for serial data transmission
  delay(2000);
}

void loop() {
  ECE3_read_IR(sensorValues);

  const unsigned int maxValues[8] = {1627, 1696, 1249, 1201, 1081, 1252, 1587, 1721};
  const unsigned int minValues[8] = {873, 804, 711, 664, 646, 642, 711, 779};

  double values[8];

  // this goes for each sensor (8 sensors)
  for (unsigned char i = 0; i < 8; i++) {

    int raw = sensorValues[i];
    if (raw < minValues[i]) raw = minValues[i];
    if (raw > maxValues[i]) raw = maxValues[i];

    values[i] = (raw - minValues[i]) * 1000.0 / maxValues[i] - minValues[i];
    Serial.print(sensorValues[i]);
    Serial.print(' '); // Tab to format the raw data into columns in the Serial monitor
  }

  // this moves the car
  int leftSpd = 70;
  int rightSpd = 70;
  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);
  digitalWrite(LED_RF, HIGH);
  delay(250);
  digitalWrite(LED_RF, LOW);
  delay(250);


  // calculate the error based on all sensor values
  double error = (-8 * values[0] - 4 * values[1] - 2 * values[2] - values[3] +
      values[4] + 2 * values[5] + 4 * values[6] + 8 * values[7]) / 4.0;

  Serial.print("e ");
  Serial.print(error);

  Serial.println();
  delay(50);
}
