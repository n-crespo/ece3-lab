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
double previous_error = 0.0;
unsigned long previous_time = 0;

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
  previous_time = millis(); // Initialize previous_time
}

void loop() {
  ECE3_read_IR(sensorValues);

  const unsigned int maxValues[8] = {1627, 1696, 1249, 1201,
                                     1081, 1252, 1587, 1721};
  const unsigned int minValues[8] = {873, 804, 711, 664, 646, 642, 711, 779};

  double values[8];

  // this goes for each sensor (8 sensors)
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
          (double)(raw - minValues[i]) * 1000.0 / (maxValues[i] - minValues[i]);
    }
    // Serial.print(sensorValues[i]);
    // Serial.print( ' '); // Tab to format the raw data into columns in the
    // Serial monitor
  }

  // calculate the error based on all sensor values
  double error = (-8 * values[0] - 4 * values[1] - 2 * values[2] - values[3] +
                  values[4] + 2 * values[5] + 4 * values[6] + 8 * values[7]) /
                 4.0; // we could trade 4.0 for the sum of the weight? idk

  // Serial.print("e:");
  // Serial.print(error);

  /////////////// PD CONTROLLER ///////////////////
  // unsigned long current_time = millis();

  // // Time difference in seconds
  // double dt_seconds = (double)(current_time - previous_time) / 1000.0;

  // // prevent division by zero (dt is too small)
  // if (dt_seconds == 0) {
  //   dt_seconds = 1.0 / 50.0; // use a small dt instead of zero
  // }

  double derivative = (error - previous_error);
  // update for next iteration
  previous_error = error;
  // previous_time = current_time;

  // this moves the car
  int base_speed = 50;
  float kp = 0.03;
  float kd = 0.15;
  // how much should derivative affect this thing? needs to be tuned

  double adjustment = (kp * error) + (kd * derivative);

  float rightSpd_f = base_speed + adjustment;
  float leftSpd_f = base_speed - adjustment;

  // int leftSpd_pwm = constrain(leftSpd_f, 0, 255); // Constrain to 0-255
  // int rightSpd_pwm = constrain(rightSpd_f, 0, 255);

  // Serial.print(", l: ");
  // Serial.print(leftSpd_pwm);
  //
  // Serial.print(", r: ");
  // Serial.println(rightSpd_pwm);

  analogWrite(left_pwm_pin, leftSpd_f);
  analogWrite(right_pwm_pin, rightSpd_f);

  // flashes the LED
  // digitalWrite(LED_RF, HIGH);
  // delay(250);
  // digitalWrite(LED_RF, LOW);
  // delay(250);

  // Serial.println();
}
