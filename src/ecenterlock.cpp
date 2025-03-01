#include <ecenterlock.h>
#include <constants.h>
#include <macros.h>
#include <odrive.h>
#include <types.h>

/**
 * Constructor for the actuator
 * @param odrive Pointer to ODrive object
 */
Ecenterlock::Ecenterlock(ODrive *odrive) : odrive(odrive), current_state(IDLE) {}

/**
 * Initializes connection to physical ODrive
 * @return 0 if successful
 */
u8 Ecenterlock::init() { return 0; }

/**
 * Instructs ODrive to attempt encoder homing
 * @return 0 if successful
 */
u8 Ecenterlock::home_ecenterlock(u32 timeout_ms) {
  change_state(HOMING);

  //start odrive as a closed loop controller
  if (odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL) != 0) {
    return HOME_CAN_ERROR;
  }

  // if not already in outbound position --> go to outbound
  if (!get_outbound_limit()) {
    u32 start_time = millis(); 
    while(!get_outbound_limit()) {
      //set_torque(DESIRED_TORQUE); 
      if ((millis() - start_time) > timeout_ms) {
        return HOME_TIMEOUT_ERROR; 
      }
      delay(100);
    }
  } else {
    //[TO DO] perhaps move back a bit then in
  }

  set_torque(0); 
  odrive->set_absolute_position(0);
  change_state(DISENGAGED_2WD);
  return HOME_SUCCESS; 
}

u8 Ecenterlock::set_velocity(float velocity) {
  if (odrive->get_axis_state() == ODrive::AXIS_STATE_IDLE) {
    odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  if (odrive->set_controller_mode(ODrive::CONTROL_MODE_VELOCITY_CONTROL,
                                  ODrive::INPUT_MODE_VEL_RAMP) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  // if (get_inbound_limit() && velocity > 0) {
  //   odrive->set_input_vel(0, 0);
  //   return SET_VELOCITY_IN_LIMIT_SWITCH_ERROR;
  // }

  // if (get_outbound_limit() && velocity < 0) {
  //   odrive->set_input_vel(0, 0);
  //   return SET_VELOCITY_OUT_LIMIT_SWITCH_ERROR;
  // }

  velocity = CLAMP(velocity, -ODRIVE_VEL_LIMIT, ODRIVE_VEL_LIMIT);
  if (odrive->set_input_vel(velocity, 0) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  velocity_mode = true;

  return SET_VELOCITY_SUCCCESS;
}

u8 Ecenterlock::set_torque(float torque) {
  if (odrive->get_axis_state() == ODrive::AXIS_STATE_IDLE) {
    odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  if (odrive->set_controller_mode(ODrive::CONTROL_MODE_TORQUE_CONTROL, ODrive::INPUT_MODE_TORQUE_RAMP) != 0) {
    return SET_TORQUE_CAN_ERROR; 
  }

  // if(get_outbound_limit() && torque > 0) {
  //   odrive->set_input_torque(0); 
  //   return SET_TORQUE_OUT_LIMIT_SWITCH_ERROR; 
  // }

  torque = CLAMP(torque, -ODRIVE_TORQUE_LIMIT, ODRIVE_TORQUE_LIMIT); 
  if (odrive->set_input_torque(torque) != 0) {
    return SET_TORQUE_CAN_ERROR; 
  }

  torque_mode = true; 
  return SET_TORQUE_SUCCESS; 
}

bool Ecenterlock::get_outbound_limit() {
  return !digitalRead(ECENTERLOCK_SENSOR_PIN);
}