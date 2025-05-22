#include <ECE3.h> // comment if the library is not required

uint16_t sensorValues[8];

// for moving the car:
const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;  // DIR_L
const int left_pwm_pin = 40;  // PWML (pin #)

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

// PD Controller variables
float previous_error = 0.0;
// int reached_end = 0;

void setup() {
  // for moving the car:
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(right_dir_pin, LOW);   // set this to high for donut!
  digitalWrite(right_nslp_pin, HIGH); // sleep? high = no

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH); // sleep? high = no

  ECE3_Init();
  Serial.begin(9600); // Set the data rate in bits per second for serial data
                      // transmission
  delay(2000);
}

void loop() {
  ECE3_read_IR(sensorValues);

  const unsigned int maxValues[8] = {1627, 1696, 1249, 1201,
                                     1081, 1252, 1587, 1721};
  const unsigned int minValues[8] = {873, 804, 711, 664, 646, 642, 711, 779};

  float values[8];

  // normalize values raw --> in bounds
  for (unsigned char i = 0; i < 8; i++) {
    int raw = sensorValues[i];
    if (raw < minValues[i])
      raw = minValues[i];
    if (raw > maxValues[i])
      raw = maxValues[i];

    // prevent zero division (not possible but just in case)
    if (maxValues[i] == minValues[i]) {
      values[i] = 0; // just set it to zero in this case to avoid /0
    } else {
      values[i] =
          (float)(raw - minValues[i]) * 1000.0 / (maxValues[i] - minValues[i]);
    }
    // Serial.print(sensorValues[i]);
    // Serial.print( ' '); // Tab to format the raw data into columns in the
    // Serial monitor
  }

  // calculate the error based on all sensor values
  int bias = -1;
  float error =
      (-15 * values[0] - 14 * values[1] - 12 * values[2] - 8 * values[3] +
       8 * values[4] + 12 * values[5] + 14 * values[6] + 15 * values[7]) /
          8.0 +
      bias; // we could trade 4.0 for the sum of the weight? idk

  float derivative = (error - previous_error);
  // Serial.println(error);
  previous_error = error; // update for next iteration

  // this moves the car
  int base_speed = 30;
  float kp = 0.04;
  float kd = 0.15;
  // how much should derivative affect this thing? needs to be tuned

  float adjustment = (kp * error) + (kd * derivative);
  Serial.println(adjustment);

  float rightSpd_f = base_speed + adjustment;
  float leftSpd_f = base_speed - adjustment;

  if (rightSpd_f < 0) {
    digitalWrite(right_dir_pin, HIGH); // set this to high for donut!
  } else {
    digitalWrite(right_dir_pin, LOW); // set this to high for donut!
  }

  if (leftSpd_f < 0) {
    digitalWrite(left_dir_pin, HIGH); // set this to high for donut!
  } else {
    digitalWrite(left_dir_pin, LOW); // set this to high for donut!
  }

  analogWrite(left_pwm_pin, abs(leftSpd_f));
  analogWrite(right_pwm_pin, abs(rightSpd_f));

  // flashes the LED
  // digitalWrite(LED_RF, HIGH);
  // delay(250);
  // digitalWrite(LED_RF, LOW);
  // delay(250);

  // Serial.println();
}
