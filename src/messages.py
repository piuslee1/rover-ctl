import struct

HASH_PRIME = 59359
UINT_MAX = 2**32

class TARGET_SYSTEMS:
    ARM = 2
    DRILL = 2
    DRIVE = 6

class Message:
    # Target system is a number declared by this array
    # Motor * SYSTEMS[] = {DRIVE_SYSTEM, ARM_SYSTEM, DRILL_SYSTEM};
    def __init__(self, target_system, speeds):
        self.speeds = speeds
        self.target_system = target_system

    @property
    def hash(self):
        s = 0
        for i in self.speeds:
            s += (i * HASH_PRIME) % UINT_MAX
        return (s + self.target_system * HASH_PRIME) % UINT_MAX

    def serialize(self):
        return struct.pack("<III{}".format('i'*len(self.speeds)),
                           len(self.speeds),
                           self.target_system,
                           self.hash,
                           *self.speeds)

"""
# For reference from messages.h
typedef struct {
  unsigned char num_speeds;
  unsigned char target_system; // ie drive, arm, drill
  unsigned int hash;
  int * speeds;
} Message;

unsigned int hash_msg(Message * msg) {
  unsigned int sum = 0;
  for (int i=0; i<msg->num_speeds; i++) {
    sum += msg->speeds[i] * HASH_PRIME;
  }
  return sum + msg->target_system * HASH_PRIME;
}
"""
