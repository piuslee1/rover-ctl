#include "messages.h"
// 1 for msg header, 2 bytes per a motor, 2 for hash

int ARM_NUM = 2;
Motor ARM_SYSTEM[] = {
  {.pin = MTR_FR, .max_speed=40},
  {.pin = MTR_FL, .max_speed=40},
};

int DRILL_NUM = 2;
Motor DRILL_SYSTEM[] = {
  {.pin = MTR_FR, .max_speed=255},
  {.pin = MTR_FL, .max_speed=255},
};

int DRIVE_NUM = 6;
Motor DRIVE_SYSTEM[] = {
  {.pin = MTR_FR, .max_speed=80},
  {.pin = MTR_MR, .max_speed=80},
  {.pin = MTR_BR, .max_speed=80},
  {.pin = MTR_FL, .max_speed=80},
  {.pin = MTR_ML, .max_speed=80},
  {.pin = MTR_BL, .max_speed=80},
};

Motor * SYSTEMS[] = {DRIVE_SYSTEM, ARM_SYSTEM, DRILL_SYSTEM};
int SYSTEM_MTR_NUMS[] = {6,2,2};
int SYSTEM_NUM = 3;

// Misc
const int LOOP_DELAY = 20;
const float TIMEOUT_DUR = 0.5; // seconds
const int TIMEOUT_COUNTS = int(1000/float(LOOP_DELAY)) * TIMEOUT_DUR;
const bool DEBUG = true;

int timeout_cntr = 0;
const int MSG_LENGTH = sizeof(Message)+5*sizeof(Motor)+10;
char buf[MSG_LENGTH];

void setup() {
  init_motors(DRIVE_SYSTEM, DRIVE_NUM);
  init_motors(DRILL_SYSTEM, DRILL_NUM);
  init_motors(ARM_SYSTEM, ARM_NUM);

  Serial.begin(115200);
}

void readSerial() {
  Serial.readBytes(buf, MSG_LENGTH);
  Message *msg = (Message*) buf;
  // Verify message
  Serial.print("Hash: ");
  Serial.println(msg->hash);
  Serial.print("New Hash: ");
  Serial.println(hash_msg(msg));
  Serial.println(msg->speeds[0]);
  Serial.println(msg->speeds[1]);
  Serial.println(msg->speeds[2]);
  Serial.println(msg->speeds[3]);
  Serial.println(msg->speeds[4]);
  Serial.println(msg->speeds[5]);
  if (msg->hash == hash_msg(msg) &&
      msg->target_system < SYSTEM_NUM &&
      msg->num_speeds == SYSTEM_MTR_NUMS[msg->target_system]) {
    set_goal_speeds(SYSTEMS[msg->target_system],
                    SYSTEM_MTR_NUMS[msg->target_system],
                    msg->speeds);
    Serial.println(msg->speeds[0]);
    Serial.println(msg->speeds[1]);
    Serial.println(msg->speeds[2]);
    Serial.println(msg->speeds[3]);
    Serial.println(msg->speeds[4]);
    Serial.println(msg->speeds[5]);
    timeout_cntr = 0;
  }
}

bool serial_timeout() {
  if (!Serial.available()) {
    if ((timeout_cntr + 1) == TIMEOUT_COUNTS) {
      Serial.println("Timeout");
    }
    if ((timeout_cntr + 1) >= TIMEOUT_COUNTS) {
      for (int i=0; i<3; i++) {
        stop(SYSTEMS[i], SYSTEM_MTR_NUMS[i]);
      }
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

  if (!serial_timeout()) {
    readSerial();
  }

  /*calibrate();*/

  delay(LOOP_DELAY);
  for (int i=0; i<SYSTEM_NUM; i++) {
    update_system(SYSTEMS[i], SYSTEM_MTR_NUMS[i]);
  }
}
