#include <Arduino.h>
#include <odrive.h>
#include <actuator.h>
#include <cmath>
#include <FlexCAN_T4.h> 
#include <TimeLib.h>

constexpr bool wait_for_can = true;

// global objects
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
ODrive ecenterlock_odrive(&flexcan_bus, ECENTERLOCK_ODRIVE_NODE_ID);
Actuator ecenterlock_actuator(&ecenterlock_odrive);

// global functions
time_t get_teensy3_time() { return Teensy3Clock.get(); }

void can_parse(const CAN_message_t &msg) { ecenterlock_odrive.parse_message(msg); }

inline void write_all_leds(u8 state) {
  digitalWrite(LED_1_PIN, state);
  digitalWrite(LED_2_PIN, state);
  digitalWrite(LED_3_PIN, state);
  digitalWrite(LED_4_PIN, state);
  digitalWrite(LED_5_PIN, state);
}

// reset position to 0 each time shift fork passes hall effect 
void on_ecenterlock_sensor() {
  ecenterlock_odrive.set_absolute_position(0.0);
}

void setup() {
  //init LEDs 
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(LED_4_PIN, OUTPUT);
  pinMode(LED_5_PIN, OUTPUT);

  // init buttons
  for (size_t i = 0; i < sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]); i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  // Setup RTC
  setSyncProvider(get_teensy3_time);

  bool rtc_set = timeStatus() == timeSet && year() > 2021;
  if (!rtc_set) {
    Serial.println("Warning: Failed to sync time with RTC");
  }

  // init wheel speed geartooth sensors
  pinMode(L_WHEEL_GEARTOOTH_SENSOR_PIN, INPUT);
  pinMode(R_WHEEL_GEARTOOTH_SENSOR_PIN, INPUT);

  // attack sensor interrupts
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
  u8 ecenterlock_actuator_status_code = ecenterlock_actuator.init();
  if (ecenterlock_actuator_status_code != 0) {
    Serial.printf("Error: ECenterlock Actuator failed to initialize with error %d\n",
                  ecenterlock_actuator_status_code);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
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