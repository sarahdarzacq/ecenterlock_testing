#include <Arduino.h>
#include <odrive.h>
#include <ecenterlock.h>
#include <cmath>
#include <FlexCAN_T4.h> 
#include <TimeLib.h>

// global objects
IntervalTimer timer;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
ODrive ecenterlock_odrive(&flexcan_bus, ECENTERLOCK_ODRIVE_NODE_ID);
Ecenterlock ecenterlock(&ecenterlock_odrive);

// global functions
time_t get_teensy3_time() { return Teensy3Clock.get(); }

// system variables 
constexpr bool wait_for_can = true;
u32 control_cycle_count = 0;
bool last_button_state[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};

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

// reset position to 0 each time shift fork passes hall effect 
void on_ecenterlock_sensor() {
  // TODO: Should this be done on an interrupt at all?? Lowkey could be fine in control loop
  // Serial.printf("Ecenterlock is Engaged!!");
}

void button_shift_mode() {

  // reset button states
  bool button_pressed[5] = {false, false, false, false, false};
  for (size_t i = 0; i < 5; i++) {
    button_pressed[i] = !digitalRead(BUTTON_PINS[i]) && last_button_state[i];
  }
  for (size_t i = 0; i < 5; i++) {
    last_button_state[i] = digitalRead(BUTTON_PINS[i]);
  }

  // get the current from odrive
  ecenterlock_odrive.request_iq();

  u8 return_code; 
  
  // NOTE: can change this variable to try different shifting velocities
  float velocity = 6;

  // Button 0 --> Idle 
  if (button_pressed[0]) {
    write_all_leds(LOW);
    digitalWrite(LED_1_PIN, HIGH);
    ecenterlock_odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);

  // Button 1 --> Closed Control Loop State 
  } else if (button_pressed[1]) {
    write_all_leds(LOW); 
    digitalWrite(LED_2_PIN, HIGH);
    return_code = ecenterlock_odrive.set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
    Serial.printf("Send Message Returned: %d", return_code); 

  // Button 2 --> Moving into 4WD (Negative Velocity)
  } else if (button_pressed[2]) {
    write_all_leds(LOW); 
    digitalWrite(LED_3_PIN, HIGH);
    ecenterlock.set_velocity(-velocity);

  // Button 3 --> Stop (Velocity = 0)
  } else if (button_pressed[3]) {
    write_all_leds(LOW); 
    digitalWrite(LED_4_PIN, HIGH);
    ecenterlock.set_velocity(0.0);

  // Button 4 --> Moving into 2WD (Positive Velocity)
  } else if (button_pressed[4]) {
    write_all_leds(LOW); 
    digitalWrite(LED_5_PIN, HIGH);
    ecenterlock.set_velocity(velocity);

  }

  float current_iq_measured = ecenterlock_odrive.get_iq_measured(); 

  // [UNTESTED] when reach edge case, just shift out until hit back wall 
  if (current_iq_measured < -8.0 && digitalRead(ECENTERLOCK_SENSOR_PIN) == HIGH) {
    Serial.printf("Edge Case Position!");
    ecenterlock.set_velocity(-velocity);
  }

  // [UNTESTED] when hitting wall, stop shifting
  if (current_iq_measured > 8.0 || current_iq_measured < -8.0) {
    ecenterlock.set_velocity(0.0); 
    Serial.printf("Shifting stopped due to Current Jump: %f\n", current_iq_measured);

    // [UNTESTED] if going backwards and hit wall, reset position to 0
    if (current_iq_measured > 0) {
      ecenterlock_odrive.set_absolute_position(0.0); 
    }
  }

  // Sensor should only be low when centerlock is fully engaged 
  // Could be HIGH, but i dont think so
  // [UNTESTED] --> TODO: Make sure this is true and works
  if (digitalRead(ECENTERLOCK_SENSOR_PIN) == LOW) {
    Serial.printf("Engaged! (according to hall effect)\n");
    write_all_leds(HIGH); // yay!!

    // should print out consistent value each time
    float ecenterlock_pos = ecenterlock_odrive.get_pos_estimate(); 
    Serial.printf("Position: %f\n",ecenterlock_pos); 
  }

  // NOTE: can change rate of logging here, or fully comment it out to view other values 
  if (control_cycle_count % 10 == 0) {
    Serial.printf("%f\n", ecenterlock_odrive.get_iq_measured());
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
  pinMode(ECENTERLOCK_SWITCH, INPUT_PULLUP);

  // just to use with the test bench 
  constexpr u8 TESTING_ECENTERLOCK_ENGAGE_BUTTON = BUTTON_PINS[0];
  constexpr u8 TESTING_ECENTERLOCK_DISENGAGE_BUTTON = BUTTON_PINS[1]; 

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
  attachInterrupt(ECENTERLOCK_SENSOR_PIN, on_ecenterlock_sensor, FALLING);

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

  // init subsystems
  u8 ecenterlock_odrive_status_code = ecenterlock_odrive.init();
  if (ecenterlock_odrive_status_code != 0) {
    Serial.printf("Error: ECenterlock ODrive failed to initialize with error %d\n",
                  ecenterlock_odrive_status_code);
  }

  // this doesn't actually do anything --> [SHOULD IT?]
  u8 ecenterlock_status_code = ecenterlock.init();
  if (ecenterlock_status_code != 0) {
    Serial.printf("Error: ECenterlock Actuator failed to initialize with error %d\n",
                  ecenterlock_status_code);
  }

  // // homin sequence for ecenterlock
  // digitalWrite(LED_2_PIN, HIGH); 
  // ecenterlock_status_code = ecenterlock.home_ecenterlock(ECENTERLOCK_HOME_TIMEOUT_MS);
  // if (ecenterlock_status_code != 0) {
  //   Serial.printf("Error: ECenterlock Actuator failed to home with error %d\n", ecenterlock_status_code); 
  // } else {
  //   digitalWrite(LED_2_PIN, LOW);
  // }

  timer.begin(button_shift_mode, CONTROL_FUNCTION_INTERVAL_MS * 1e3);

}

void loop() {

}


/*
STATES: 
1. idle
2. 2_wheel_drive
3. 4_wheel_drive
4. 


Basic Code Outline: 

int engage_centerlock() {
  numTries = 5; 

  //setting numTries to the right value 
  if (fws < 0.5) {  //some small value 
    double wheelSpeedDiff = abs(fws - bws); 
    if (wheelSpeedDiff > 0.5)  {  //Number given by PT
      // [FAILED TO ENGAGE CENTERLOCK]
      numTries = 0; 
    } else {
      numTries = 1; 
    }
  }

  if (numTries > 0) {
     
    numTries--; 
  } 

}
 
*/