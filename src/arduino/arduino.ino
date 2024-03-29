#include "messages.h"
// 1 for msg header, 2 bytes per a motor, 2 for hash

int ARM_NUM = 0;
Motor ARM_SYSTEM[] = {
};

int DRILL_NUM = 0;
Motor DRILL_SYSTEM[] = {
};

int DRIVE_NUM = 6;
Motor DRIVE_SYSTEM[] = {
  Motor(MTR_FR, 255, true, 0,0, Servo()),
  Motor(MTR_MR, 255, false, 0,0, Servo()),
  Motor(MTR_BR, 255, false, 0,0, Servo()),
  Motor(MTR_FL, 255, true, 0,0, Servo()),
  Motor(MTR_ML, 255, false, 0,0, Servo()),
  Motor(MTR_BL, 255, false, 0,0, Servo()),
};

Motor * SYSTEMS[] = {DRIVE_SYSTEM, ARM_SYSTEM, DRILL_SYSTEM};
int SYSTEM_MTR_NUMS[] = {6,0,0};
int SYSTEM_NUM = 3;

// Misc
const int LOOP_DELAY = 20;
const float TIMEOUT_DUR = 0.5; // seconds
const int TIMEOUT_COUNTS = int(1000/float(LOOP_DELAY)) * TIMEOUT_DUR;
const bool DEBUG = true;

int timeout_cntr = 0;
/* const int MSG_LENGTH = sizeof(Message)+5*sizeof(Motor)+10; */
const int START_SEQ = 'a';
const int MSG_LENGTH = sizeof(Message);
char buf[MSG_LENGTH];

void setup() {
  init_motors(DRIVE_SYSTEM, DRIVE_NUM);
  init_motors(DRILL_SYSTEM, DRILL_NUM);
  init_motors(ARM_SYSTEM, ARM_NUM);

  DRIVE_SYSTEM[0].handle.write(40);

  Serial.begin(115200);
}

void readSerial() {
  char validate[1];
  do {
    Serial.readBytes(validate, sizeof(char));
  } while(validate[0] != START_SEQ);

  Serial.readBytes(buf, MSG_LENGTH);
  Message *msg = (Message*) buf;
  // Verify message
  Serial.print("Hash: ");
  Serial.println(msg->hash);
  Serial.print("New Hash: ");
  Serial.println(hash_msg(msg));
  /* Serial.println(msg->speeds[0]); */
  /* Serial.println(msg->speeds[1]); */
  /* Serial.println(msg->speeds[2]); */
  /* Serial.println(msg->speeds[3]); */
  /* Serial.println(msg->speeds[4]); */
  /* Serial.println(msg->speeds[5]); */
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
    // if ((timeout_cntr + 1) >= TIMEOUT_COUNTS) {
    //   for (int i=0; i<3; i++) {
    //     stop(SYSTEMS[i], SYSTEM_MTR_NUMS[i]);
    //   }
    // }
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
