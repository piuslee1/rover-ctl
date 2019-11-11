#include <Servo.h>
#define NUM_MTRS 6

const int MTR_FR = 8;
const int MTR_MR = 4;
const int MTR_BR = 5;

const int MTR_FL = 6;
const int MTR_ML = 2;
const int MTR_BL = 3;
const int MTRS[NUM_MTRS] = {MTR_FR, MTR_MR, MTR_BR, MTR_FL, MTR_ML, MTR_BL};
const Servo mtrs[NUM_MTRS];

const int MAX_SPEED = 255;
const int MIN_SPEED_CAL = 0; // for calibration
// Doesn't register until 104, 87
const int MIN_ANGLE = 40;
const int MAX_ANGLE = 150;
const int LOOP_DELAY = 20;
const float TIMEOUT_DUR = 0.5; // seconds
const int TIMEOUT_COUNTS = int(1000/float(LOOP_DELAY)) * TIMEOUT_DUR;
const bool DEBUG = true;
int timeout_cntr = 0;

void move(int * val) {
  int angle;
  for (int i=0; i<NUM_MTRS; i++) {
    val[i] = min(MAX_SPEED, max(val[i], -MAX_SPEED));
    angle = int(float(val[i])
                / 255. * (MAX_ANGLE-MIN_ANGLE)/2
                + (MAX_ANGLE-MIN_ANGLE)/2
                + MIN_ANGLE); 
    mtrs[i].write(angle);
    if (angle != 95) {
      Serial.println(angle);
    }
  }
}

void stop() {
  int mtr_vals[6];
  for (int i=0; i<NUM_MTRS; i++) {
    mtr_vals[i] = 0;
  }
  move(mtr_vals);
}

void setup() {
  for(int i = 0; i < NUM_MTRS; i++){
    mtrs[i] = Servo();
    mtrs[i].attach(MTRS[i]);
  }


}

void readSerial() {
  String input= Serial.readStringUntil('\n');
  int char_idx = 3;
  int pwm_idx = 0;
  int num_start = char_idx, num_end = char_idx, last_num_read;
  int mtr_vals[NUM_MTRS];
  while (char_idx < 255 && pwm_idx < NUM_MTRS && input[char_idx-1] != '\0')
  {
    if (partOfNum(input[char_idx]))
    {
      num_end++;
      /*Serial.println("reading digit");*/
    }
    else
    {
      last_num_read = input.substring(num_start, num_end).toInt();
      num_end++;
      num_start = num_end;
      mtr_vals[pwm_idx] = last_num_read;
      /*Serial.print(pins[pwm_idx]);*/
      /*Serial.print(",");*/
      /*Serial.println(last_num_read);*/
      pwm_idx++;
    }
    char_idx++;
  }
  move(mtr_vals);
}

void calibrate() {
  for (int i=0; i<2; i++) {
    for (int j=-MAX_SPEED; j<-MIN_SPEED_CAL; j+=10) {
      int mtr_vals[NUM_MTRS];
      for (int i=0; i<NUM_MTRS; i++) {
        mtr_vals[i] = j;
      }
      move(mtr_vals);
      delay(300);
    }
    for (int j=MIN_SPEED_CAL; j<MAX_SPEED; j+=10) {
      int mtr_vals[NUM_MTRS];
      for (int i=0; i<NUM_MTRS; i++) {
        mtr_vals[i] = j;
      }
      move(mtr_vals);
      delay(300);
    }
  }
  stop();
  delay(2000);
  stop();
  delay(2000);
}

bool serial_timeout() {
  if (!Serial.available()) {
    if ((timeout_cntr + 1) == TIMEOUT_COUNTS) {
      Serial.println("Timeout");
    }

    if ((timeout_cntr + 1) >= TIMEOUT_COUNTS) {
      stop();
    }
    timeout_cntr = min((timeout_cntr + 1), 2*TIMEOUT_COUNTS);
    return true;
  } else {
    timeout_cntr = 0;
    return false;
  }
}

void loop() {
  // put your main code here, to run repeatedly:


  delay(2000);

  int mtr_vals[NUM_MTRS];

  mtr_vals[0] = 0;
  mtr_vals[1] = 0;
  mtr_vals[2] = 0;
  mtr_vals[3] = 0;
  mtr_vals[4] = 0;
  mtr_vals[5] = 0;

  move(mtr_vals);

  delay(2000);

  mtr_vals[0] = 30;
  mtr_vals[1] = 30;
  mtr_vals[2] = 30;
  mtr_vals[3] = 30;
  mtr_vals[4] = 30;
  mtr_vals[5] = 30;

  move(mtr_vals);

  // if (!serial_timeout()) {
  //   readSerial();
  // }

  // /*calibrate();*/

  // delay(LOOP_DELAY);
}

bool partOfNum(char c) {
  return inrange(c, '0', '9') or
         c == '-' or 
         c == '.';
}

bool inrange(char c, char low, char h)
{
  return c >= low && c <= h;
}
