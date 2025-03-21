#include <Arduino.h>
#include <odrive.h>
#include <ecenterlock.h>
#include <cmath>
#include <FlexCAN_T4.h> 
#include <TimeLib.h>

int c = 0; 

// global objects
IntervalTimer timer;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
ODrive ecenterlock_odrive(&flexcan_bus, ECENTERLOCK_ODRIVE_NODE_ID);
Ecenterlock ecenterlock(&ecenterlock_odrive);

// global functions
time_t get_teensy3_time() { return Teensy3Clock.get(); }

// system variables 
constexpr bool wait_for_can = true;
constexpr bool wait_for_serial = true; 
u32 control_cycle_count = 0;
bool last_button_state[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};
bool ecenterlock_state_sensor = LOW; 

u8 cycles_to_wait_for_vel = 1; 

void can_parse(const CAN_message_t &msg) { 
  ecenterlock_odrive.parse_message(msg); 
}

inline void write_all_leds(u8 state) {
  digitalWrite(LED_1_PIN, state);
  digitalWrite(LED_2_PIN, state);
  digitalWrite(LED_3_PIN, state);
  digitalWrite(LED_4_PIN, state);
  digitalWrite(LED_5_PIN, state);
}

// Engaged/engaging when sensor is HIGH
void on_ecenterlock_sensor() {
  if (digitalRead(ECENTERLOCK_SENSOR_PIN) == LOW) {
    Serial.printf("Sensor LOW\n");
    ecenterlock_state_sensor = LOW; 
  } else {
    Serial.printf("Sensor HIGH\n");
    ecenterlock_state_sensor = HIGH; 
  }
  // TODO: Should this be done on an interrupt at all?? Lowkey could be fine in control loop
  
}

void on_ecenterlock_switch_engage() {
  if(ecenterlock.get_state() == Ecenterlock::DISENGAGED_2WD) {
    ecenterlock.set_engage(true); 
  }
}

//TODO: Implement this, need to allocate another pin for this :) 
void on_ecenterlock_switch_disengage() { 
  if(ecenterlock.get_state() == Ecenterlock::ENGAGED_4WD) {
    ecenterlock.set_disengage(true); 
  }
}

void button_shift_mode() {

  ecenterlock_odrive.request_nonstand_pos_rel(); 
  ecenterlock.set_prev_position(ecenterlock.get_position()); 

  // TODO: Check if should add of subtract offset 
  float ecenterlock_position = ecenterlock_odrive.get_pos_rel() - ecenterlock.get_offset(); 
  ecenterlock.set_position(ecenterlock_position); 

  // State Machine for ECenterlock
  switch(ecenterlock.get_state()) {
    case Ecenterlock::UNHOMED:
      Serial.printf("Ecenterlock State: Unhomed\n");
      noInterrupts(); 
      break; 
  
    case Ecenterlock::DISENGAGED_2WD: 
      //Serial.printf("Ecenterlock State: Disengaged\n");

      if (ecenterlock.get_engage()) {

        ///////////////////////////////////////////
        // TODO: Implement Pre-Engage Safety Checks! 
        ///////////////////////////////////////////
        /*
        Case 1: Car is Stopped 

        Case 2: Allowable difference in wheel speed is surpassed 

        Case 3: Car is moving normally 
        
        */

        ecenterlock.set_num_tries(1); 

        ecenterlock.set_engage(false); 
        ecenterlock.set_velocity(ECENTERLOCK_VELOCITY);
        cycles_to_wait_for_vel = 10; 
        // TODO: Change to State
        ecenterlock.change_state(Ecenterlock::ENGAGING);
      }
      break; 

    case Ecenterlock::ENGAGED_4WD: 
      //Serial.printf("Ecenterlock State: Engaged\n");
      if (ecenterlock.get_disengage()) {
        ecenterlock.set_disengage(false); 
        ecenterlock.set_velocity(-ECENTERLOCK_VELOCITY);
        ecenterlock.change_state(Ecenterlock::DISENGAGING);
      }
      break; 

    case Ecenterlock::ENGAGING:
      if (cycles_to_wait_for_vel == 0) {
        
        Serial.printf("Ecenterlock State: Engaging!, %f, %f\n", ecenterlock.get_position(), ecenterlock.get_prev_position()); 
        // TODO: Should this be checking position
        if (ecenterlock.get_position() == ecenterlock.get_prev_position()) {
          
          // Stopped b/c Engaged 
          // TODO: Do we want to have that clearance difference there just in case some slipping happens? 
          // TODO: Add sensor here? 
          if (ecenterlock.get_position() >= ECENTERLOCK_ENGAGED_POSITION - 0.05) {
            Serial.printf("Ecenterlock Engaged %f\n", ecenterlock.get_position()); 
            ecenterlock.set_velocity(0); 
            ecenterlock.change_state(Ecenterlock::ENGAGED_4WD); 
          } else { //Stopped in Edge Case 
            ecenterlock.set_velocity(-ECENTERLOCK_VELOCITY); 
            ecenterlock.change_state(Ecenterlock::DISENGAGING); 
            
            u8 tries_left = ecenterlock.get_num_tries() - 1; 
            if (tries_left) {
              ecenterlock.set_engage(true); 
              ecenterlock.set_num_tries(tries_left); 
            }    
          }
        }
       
      } else {
        cycles_to_wait_for_vel--; 
      }
       break;

    case Ecenterlock::DISENGAGING: 
      // Serial.printf("Ecenterlock State: Disengaging\n"); 
      // TODO: Idk if we need all these checks, a bit redundant 
      if (ecenterlock.get_position() == ecenterlock.get_prev_position() && ecenterlock.get_position() <= 0.05 && !digitalRead(ECENTERLOCK_SENSOR_PIN)) {
        Serial.printf("Ecenterlock Disengaged! %f n", ecenterlock.get_position()); 
        ecenterlock.set_velocity(0); 
        ecenterlock.change_state(Ecenterlock::DISENGAGED_2WD); 
      }
  }
     
  control_cycle_count++;
}

void setup() {
  //init LEDs 
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(LED_4_PIN, OUTPUT);
  pinMode(LED_5_PIN, OUTPUT);

  write_all_leds(HIGH); 
  delay(1000);
  write_all_leds(LOW); 

  pinMode(ECENTERLOCK_SENSOR_PIN, INPUT); 
  if (digitalRead(ECENTERLOCK_SENSOR_PIN) == LOW) {
    write_all_leds(HIGH); 
  }

  timer.priority(255);

  // init buttons
  for (size_t i = 0; i < sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]); i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }
  pinMode(ECENTERLOCK_SWITCH_ENGAGE, INPUT_PULLUP);

  // Setup RTC
  setSyncProvider(get_teensy3_time);
  bool rtc_set = timeStatus() == timeSet && year() > 2021;
  if (!rtc_set) {
    Serial.println("Warning: Failed to sync time with RTC");
  }

  // init wheel speed geartooth sensors
  pinMode(L_WHEEL_GEARTOOTH_SENSOR_PIN, INPUT);
  pinMode(R_WHEEL_GEARTOOTH_SENSOR_PIN, INPUT);

  // attach sensor interrupts
  attachInterrupt(ECENTERLOCK_SENSOR_PIN, on_ecenterlock_sensor, CHANGE);
  // attachInterrupt(ECENTERLOCK_SWITCH_ENGAGE , on_ecenterlock_switch_engage, RISING); 
  // attachInterrupt(ECENTERLOCK_SWITCH_DISENGAGE, on_ecenterlock_switch_disengage, RISING); 
  attachInterrupt(BUTTON_PINS[0] , on_ecenterlock_switch_engage, RISING); 
  attachInterrupt(BUTTON_PINS[1], on_ecenterlock_switch_disengage, RISING); 

  // init CAN bus
  flexcan_bus.begin();
  flexcan_bus.setBaudRate(FLEXCAN_BAUD_RATE);
  flexcan_bus.setMaxMB(FLEXCAN_MAX_MAILBOX);
  flexcan_bus.enableFIFO();
  flexcan_bus.enableFIFOInterrupt();
  flexcan_bus.onReceive(can_parse);

  // Wait for ODrive can connection if enabled
  if (wait_for_can) {
    u32 led_flash_time_ms = 100;
    while (ecenterlock_odrive.get_time_since_heartbeat_ms() > 100) {   
      write_all_leds(millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
      delay(100);
    }
  }
  write_all_leds(LOW);

  // Wait for serial if enabled
  if (wait_for_serial) {
    u32 led_flash_time_ms = 500;
    while (!Serial) {
      write_all_leds(millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
    }
  }
  write_all_leds(LOW);

  // init subsystems
  u8 ecenterlock_odrive_status_code = ecenterlock_odrive.init();
  if (ecenterlock_odrive_status_code != 0) {
    Serial.printf("Error: ECenterlock ODrive failed to initialize with error %d\n",
                  ecenterlock_odrive_status_code);
  }

  if (ecenterlock.home(5000) != 0) {
    Serial.printf("Error: Ecenterlock failed to home\n"); 
  }

  timer.begin(button_shift_mode, CONTROL_FUNCTION_INTERVAL_MS * 1e3);

}

void loop() {

}