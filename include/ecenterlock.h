#ifndef ECENTERLOCK_H
#define ECENTERLOCK_H

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

class Ecenterlock {

public:
  static const u8 SET_TORQUE_SUCCESS = 0;
  static const u8 SET_TORQUE_OUT_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_TORQUE_CAN_ERROR = 2;

  static const u8 HOME_SUCCESS = 0;
  static const u8 HOME_CAN_ERROR = 1;
  static const u8 HOME_TIMEOUT_ERROR = 2;

  Ecenterlock(ODrive *odrive);

  u8 init();
  u8 home_ecenterlock(u32 timeout_ms);

  u8 set_torque(float velocity);

  bool get_outbound_limit();

private:
  bool torque_mode = true;
  ODrive *odrive;
};

#endif
