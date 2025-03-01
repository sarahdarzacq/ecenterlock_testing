#ifndef ECENTERLOCK_H
#define ECENTERLOCK_H

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

class Ecenterlock {

public:
  enum State {
    IDLE, 
    ENGAGED_4WD, 
    DISENGAGED_2WD,
    HOMING, 
    CHECKING_WHEEL_SPEED, 
    ATTEMPTING_TO_ENGAGE, 
    STUCK, 
    BACKING_UP
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
  u8 home_ecenterlock(u32 timeout_ms);
  u8 change_state(State new_state) { current_state = new_state; }

  u8 set_torque(float torque);
  u8 set_velocity(float velocity);

  bool get_outbound_limit();
  State get_state() { return current_state; }

private:
  bool torque_mode;
  bool velocity_mode = true; 
  ODrive *odrive;
  State current_state; 
};

#endif
