#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <macros.h>
#include <math.h>
#include <stddef.h>
#include <types.h>

#define dancing 13

// Units
constexpr float MINUTES_PER_HOUR = 60.0;
constexpr float SECONDS_PER_MINUTE = 60.0; // s / min
constexpr float MS_PER_SECOND = 1.0e3;     // ms / s
constexpr float US_PER_SECOND = 1.0e6;     // us / s
constexpr float SECONDS_PER_MS = 1.0e-3;   // s / ms
constexpr float SECONDS_PER_US = 1.0e-6;   // s / us

constexpr float MM_PER_INCH = 25.4;              // mm / inch
constexpr float INCHES_PER_MM = 1 / MM_PER_INCH; // inch / mm

constexpr float FEET_PER_MILE = 5280.0; // feet / mile
constexpr float INCH_PER_FEET = 12.0;   // inch / feet

// ODrive
constexpr u8 ODRIVE_NODE_ID = 0x3;                                           //[Update??]
constexpr float ODRIVE_VEL_LIMIT = 80.0;        // rot / s
constexpr float ODRIVE_CURRENT_SOFT_MAX = 30.0; // A

// Actuator
// NOTE: Pitch is distance / rotation
constexpr float ACTUATOR_PITCH_MM = 5.0;                      // mm / rot
constexpr float ACTUATOR_PITCH_CM = ACTUATOR_PITCH_MM / 10.0; // cm / rot

constexpr float ACTUATOR_ENGAGE_POS_ROT = 3.5;   // rot
constexpr float ACTUATOR_INBOUND_POS_ROT = 16.2; // rot
constexpr float ACTUATOR_ENGAGE_POS_CM =
    ACTUATOR_ENGAGE_POS_ROT * ACTUATOR_PITCH_CM; // cm
constexpr float ACTUATOR_INBOUND_POS_CM =
    ACTUATOR_INBOUND_POS_ROT * ACTUATOR_PITCH_CM; // cm
constexpr float ACTUATOR_HOME_VELOCITY = 4.0;     // rot / s
constexpr float ACTUATOR_HOME_TIMEOUT_MS = 4000;  // ms

constexpr float ACTUATOR_SLOW_INBOUND_REGION_ROT = 5.0;
constexpr float ACTUATOR_SLOW_INBOUND_VEL = 30.0;
constexpr float ACTUATOR_FAST_INBOUND_VEL = 60.0;

// Control Function
constexpr u32 CONTROL_FUNCTION_INTERVAL_MS = 10; // ms


// Teensy Pins
constexpr u8 LED_1_PIN = 7; // D0 on PCB
constexpr u8 LED_2_PIN = 8; // D1 on PCB
constexpr u8 LED_3_PIN = 9; // D2 on PCB
constexpr u8 LED_4_PIN = 28; // D3 on PCB
constexpr u8 LED_5_PIN = 29; // D4 on PCB

constexpr u8 LIMIT_SWITCH_IN_PIN = 12;
constexpr u8 LIMIT_SWITCH_OUT_PIN = 10;
constexpr u8 LIMIT_SWITCH_ENGAGE_PIN = 11;

constexpr u8 BUTTON_PINS[] = {2, 3, 4, 5, 6};

constexpr u8 ENGINE_SENSOR_PIN = 38; 
constexpr u8 GEARTOOTH_SENSOR_PIN = 37; 

constexpr u8 THROTTLE_SENSOR_PIN = 40;
constexpr u8 BRAKE_SENSOR_PIN = 39;

// [Add front wheel speed sensors and centerlock sensors]

// Flexcan
constexpr u32 FLEXCAN_BAUD_RATE = 250000;     //[UPDATE???]
constexpr u32 FLEXCAN_MAX_MAILBOX = 16;      //[UPDATE???]

// Logging
// bytes_per_cycle * cycle_freq * time_to_flush_sd * safety_factor
// 100 * 100 * 0.4 * 2 = 8000
constexpr size_t LOG_BUFFER_SIZE = 65536;

constexpr u8 PROTO_HEADER_MESSAGE_ID = 0x00;
constexpr u8 PROTO_CONTROL_FUNCTION_MESSAGE_ID = 0x01;

constexpr size_t MESSAGE_BUFFER_SIZE = 512;
constexpr size_t PROTO_DELIMITER_LENGTH = 5;

#endif
