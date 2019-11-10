#include <Servo.h>

const uint32_t HASH_PRIME = 59359;

// Doesn't register until 104, 87
const int32_t MIN_ANGLE = 40;
const int32_t MAX_ANGLE = 150;
const int32_t MAX_SPEED_CHANGE = 2;

const int32_t UNCONNECTED = 11;
const unsigned char MTR_FR = 8;
const unsigned char MTR_MR = 4;
const unsigned char MTR_BR = 5;

const unsigned char MTR_FL = 6;
const unsigned char MTR_ML = 2;
const unsigned char MTR_BL = 3;


struct Motor {
  unsigned char pin;
  unsigned char max_speed;
  bool bidirectional;
  int32_t goal_speed;
  int32_t current_speed;
  Servo handle;

  Motor(unsigned char pin, unsigned char max_speed, bool bidirectional, int32_t goal_speed, int32_t current_speed, Servo handle){
    this->pin = pin;
    this->max_speed = max_speed;
    this->bidirectional = bidirectional;
    this->goal_speed = goal_speed;
    this->current_speed = current_speed;
    this->handle = handle;
  }
};

typedef struct {
  uint32_t num_speeds;
  uint32_t target_system; // ie drive, arm, drill
  // unsigned char num_speeds;
  // unsigned char target_system; // ie drive, arm, drill
  uint32_t hash;
  int32_t speeds[6];
} __attribute__((packed)) Message;

uint32_t hash_msg(Message * msg) {
  uint32_t sum = 0;
  for (int32_t i=0; i<msg->num_speeds; i++) {
    sum += msg->speeds[i] * HASH_PRIME;
  }
  return sum + msg->target_system * HASH_PRIME;
}

void init_motors(Motor * mtrs, int32_t num_mtrs) {
  for (int32_t i=0; i<num_mtrs; i++) {
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

void write_motor(Motor * motor, int speed){
  Serial.print("writing_motor: ");

  if(motor->bidirectional){
    int32_t current_angle = int32_t(float(abs(speed))
                / 255. * (MAX_ANGLE-MIN_ANGLE)); 
    motor->handle.write(current_angle);
    Serial.print("BI DIRECTIONAL  ");
    Serial.println(current_angle);

    
    if(speed > 0){
      digitalWrite(motor->pin + 1, 1);
    }
    else{
      digitalWrite(motor->pin + 1, 0);
    }
  }

  else{
    int32_t current_angle = int32_t(speed
                / 255. * (MAX_ANGLE-MIN_ANGLE)
                - (MAX_ANGLE-MIN_ANGLE)/2
                + MIN_ANGLE); 
    motor->handle.write(current_angle);
    Serial.println(current_angle);
  }
}


void update_system(Motor * mtrs, int32_t num_mtrs) {
  for (int32_t i=0; i<num_mtrs; i++) {
    int32_t difference = mtrs[i].goal_speed - mtrs[i].current_speed;
    int32_t actual_movement = min(MAX_SPEED_CHANGE, abs(difference));
    if(difference != 0){
      mtrs[i].current_speed += actual_movement * difference/abs(difference);
    }
    write_motor(&mtrs[i], mtrs[i].goal_speed);
  }
}

void move(Motor * mtrs, int32_t num_mtrs, int32_t * val) {
  
}

void stop(Motor * mtrs, int32_t num_mtrs) {
  int32_t mtr_vals[num_mtrs];
  for (int32_t i=0; i<num_mtrs; i++) {
    write_motor(&mtrs[i], 0);
  }
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
