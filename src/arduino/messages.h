#include <Servo.h>

const uint32_t HASH_PRIME = 59359;

// Doesn't register until 104, 87
const int32_t MIN_ANGLE = 40;
const int32_t MAX_ANGLE = 150;
const int32_t MAX_SPEED_CHANGE = 2;

const int32_t UNCONNECTED = 11;
const int32_t MTR_FR = 8;
const int32_t MTR_MR = UNCONNECTED;
const int32_t MTR_BR = 3;

const int32_t MTR_FL = 4;
const int32_t MTR_ML = UNCONNECTED;
const int32_t MTR_BL = 6;

typedef struct {
  unsigned char pin;
  unsigned char max_speed;
  int32_t goal_speed;
  int32_t current_speed;
  Servo handle;
} Motor;

typedef struct {
  unsigned char num_speeds;
  unsigned char target_system; // ie drive, arm, drill
  uint32_t hash;
  int32_t * speeds;
} Message;

uint32_t hash_msg(Message * msg) {
  uint32_t sum = 0;
  for (int32_t i=0; i<msg->num_speeds; i++) {
    sum += msg->speeds[i] * HASH_PRIME;
  }
  return sum + msg->target_system * HASH_PRIME;
}

void init_motors(Motor * mtrs, int32_t num_mtrs) {
  for (int32_t i=0; i<num_mtrs; i++) {
    mtrs[i].handle = Servo();
    mtrs[i].handle.attach(mtrs[i].pin);
    mtrs[i].goal_speed = 0;
    mtrs[i].current_speed = 0;
  }
}

void set_goal_speeds(Motor * mtrs, int32_t num_mtrs, int32_t *val) {
  for (int32_t i=0; i<num_mtrs; i++) {
    mtrs[i].goal_speed = min(mtrs[i].max_speed,
                             max(val[i], -mtrs[i].max_speed));
  }
}

void update_system(Motor * mtrs, int32_t num_mtrs) {
  for (int32_t i=0; i<num_mtrs; i++) {
    int32_t difference = mtrs[i].goal_speed - mtrs[i].current_speed;
    int32_t actual_movement = min(MAX_SPEED_CHANGE, abs(difference));
    if(difference != 0){
      mtrs[i].current_speed += actual_movement * difference/abs(difference);
    }
    int32_t current_angle = int32_t(float(mtrs[i].current_speed)
                / 255. * (MAX_ANGLE-MIN_ANGLE)/2
                + (MAX_ANGLE-MIN_ANGLE)/2
                + MIN_ANGLE); 
    mtrs[i].handle.write(current_angle);
  }
}

void move(Motor * mtrs, int32_t num_mtrs, int32_t * val) {
  for (int32_t i=0; i<num_mtrs; i++) {
    val[i] = min(mtrs[i].max_speed, max(val[i], -mtrs[i].max_speed));
    int32_t current_angle = int32_t(float(val[i])
                / 255. * (MAX_ANGLE-MIN_ANGLE)/2
                + (MAX_ANGLE-MIN_ANGLE)/2
                + MIN_ANGLE); 
    mtrs[i].handle.write(current_angle);
  }
}

void stop(Motor * mtrs, int32_t num_mtrs) {
  int32_t mtr_vals[num_mtrs];
  for (int32_t i=0; i<num_mtrs; i++) {
    mtr_vals[i] = 0;
  }
  move(mtrs, num_mtrs, mtr_vals);
}

void calibrate(Motor * mtrs, int32_t num_mtrs) {
  for (int32_t i=0; i<2; i++) {
    for (int32_t j=-255; j<255; j+=10) {
      int32_t mtr_vals[num_mtrs];
      for (int32_t i=0; i<num_mtrs; i++) {
        mtr_vals[i] = j;
      }
      move(mtrs, num_mtrs, mtr_vals);
      delay(300);
    }
  }
  stop(mtrs, num_mtrs);
  delay(2000);
  stop(mtrs, num_mtrs);
  delay(2000);
}
