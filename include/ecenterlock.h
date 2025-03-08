#ifndef ECENTERLOCK_H
#define ECENTERLOCK_H

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

class Ecenterlock {

public:
  enum State { 
    UNHOMED,
    ENGAGED_4WD, 
    DISENGAGED_2WD,
    ENGAGING, 
    DISENGAGING
  };

  static const u8 SET_TORQUE_SUCCESS = 0;
  static const u8 SET_TORQUE_OUT_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_TORQUE_CAN_ERROR = 2;

  static const u8 SET_VELOCITY_SUCCCESS = 0;
  static const u8 SET_VELOCITY_IN_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_VELOCITY_OUT_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_VELOCITY_CAN_ERROR = 2;

  static const u8 HOME_SUCCESS = 0;
  static const u8 HOME_CAN_ERROR = 1;
  static const u8 HOME_TIMEOUT_ERROR = 2;

  Ecenterlock(ODrive *odrive);

  u8 init();
  u8 home(u32 timeout_ms);
  u8 change_state(State new_state) { current_state = new_state; }

  u8 set_torque(float torque);
  u8 set_velocity(float velocity);
  u8 set_position(float position) { this->position = position; }
  u8 set_prev_position(float position) { prev_position = position; }
  u8 set_engage(bool engage) { this->engage = engage; }
  u8 set_disengage(bool disengage) { this->disengage = disengage; }
  u8 set_num_tries(bool tries) { num_tries = tries; }

  bool get_outbound_limit();
  State get_state() { return current_state; }
  bool get_engage() { return engage; }
  bool get_disengage() { return disengage; }
  float get_offset() { return pos_rel_offset; }
  float get_position() { return position; }
  float get_prev_position() { return prev_position; }
  u8 get_num_tries() { return num_tries; }


private:
  bool torque_mode;
  bool velocity_mode = true; 
  ODrive *odrive;
  State current_state; 
  float pos_rel_offset; 
  float prev_position; 
  float position; 
  bool engage; 
  bool disengage; 
  u8 num_tries; 

};

#endif
