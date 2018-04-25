/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * cnc.cpp - CNC Routines
 */

#include "cnc.h"

#include "../Marlin.h"
#include "../gcode/gcode.h"
#include "../gcode/queue.h" // for gcodeN
#include "../module/motion.h"
#include "../module/planner.h"
#include "../module/stepper.h"
#include "../module/printcounter.h"
#include "../lcd/ultralcd.h"
#include "../core/language.h"

cnc cncManager;

/**
 * Class and Instance Methods
 */
cnc::cnc() { }

/**
 * Initialize the cnc manager
 * The manager is implemented by periodic calls to manage()
 */
void cnc::init() {

  #if EARLY_WATCHDOG
    // Flag that the manager should be running
    if (inited) return;
    inited = true;
  #endif

  #if HAS_FAN0
    SET_OUTPUT(FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
  #endif

  #if HAS_FAN1
    SET_OUTPUT(FAN1_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN1_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
  #endif

  #if HAS_FAN2
    SET_OUTPUT(FAN2_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN2_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
  #endif

  // todo: HAL: fix abstraction
  #ifdef __AVR__
    // Use timer0 for temperature measurement
    // Interleave temperature interrupt with millies interrupt
    OCR0B = 128;
    SBI(TIMSK0, OCIE0B);
  #endif
}

// Derived from Temperature.manage_heater() to keep WDT from tripping
void cnc::manage() {

  #if EARLY_WATCHDOG
    // If thermal manager is still not running, make sure to at least reset the watchdog!
    if (!inited) {
      watchdog_reset();
      return;
    }
  #endif

  #if ENABLED(EMERGENCY_PARSER)
    if (killed_by_M112) kill(PSTR(MSG_KILLED));
  #endif

  #if ENABLED(USE_WATCHDOG)
    // Reset the watchdog.
    watchdog_reset();
  #endif

  #if HAS_AUTO_FAN
    if (ELAPSED(ms, next_auto_fan_check_ms)) { // only need to check fan state very infrequently
      checkExtruderAutoFans();
      next_auto_fan_check_ms = ms + 2500UL;
    }
  #endif
}

// Code modified from grbl - report.c
// Prints build info line
void cnc::report_build_info(
  #if NUM_SERIAL > 1
    const int8_t port/*= -1*/
  #endif
) {
  SERIAL_ECHOPGM_P(port, "[VER:");
  SERIAL_ECHOPGM_P(port, MSG_MARLIN);
  SERIAL_CHAR_P(port, ' ');
  SERIAL_ECHOPGM_P(port, SHORT_BUILD_VERSION);
  SERIAL_ECHOLNPGM_P(port, "]");
  SERIAL_EOL_P(port);
  SERIAL_ECHOPGM_P(port, "[OPT:VNMHT");
  #ifndef HOMING_INIT_LOCK
    SERIAL_CHAR_P(port, 'L');
  #endif
  SERIAL_ECHOPAIR_P(port, ",", BLOCK_BUFFER_SIZE-1);
  SERIAL_ECHOPAIR_P(port, ",", RX_BUFFER_SIZE);
  SERIAL_ECHOLNPGM_P(port, "]");
}

// Code from grbl - report.c
// Print current gcode parser mode state
// Output example [GC:G0 G54 G17 G21 G90 G94 M3 M9 T0 F0 S0]
void cnc::report_gcode_modes(
  #if NUM_SERIAL > 1
    const int8_t port/*= -1*/
  #endif
) {
  SERIAL_ECHOPGM_P(port, "[GC:G");
  #if ENABLED(G38_PROBE_TARGET)
    if (G38_move && !G38_2)
      SERIAL_ECHOPGM_P(port, "38");
    else if (G38_move && G38_2)
      SERIAL_ECHOPGM_P(port, "38.2");
    else
      SERIAL_ECHOPGM_P(port, "0");
  #else
      SERIAL_ECHOPGM_P(port, "0");
  #endif
  SERIAL_ECHOPAIR_P(port, " G", gcode.active_coordinate_system+55);
  SERIAL_ECHOPAIR_P(port, " G", gcode.workspace_plane+17);
  #if ENABLED(INCH_MODE_SUPPORT)
    SERIAL_ECHOPGM_P(port, " G2");
    SERIAL_CHAR_P(port, parser.linear_unit_factor == 1.0 ? '1' : '0');
  #else
    SERIAL_ECHOPGM_P(port, " G21");
  #endif
  if(relative_mode) SERIAL_ECHO_P(port, " G91");
  else SERIAL_ECHO_P(port, " G90");
  // G94 is not supported by marlin

  switch (gcode.spindle_mode) {
    case 0 : SERIAL_ECHOPGM_P(port, " M5"); break;
    case 1 : SERIAL_ECHOPGM_P(port, " M3"); break;
    case 2 : SERIAL_ECHOPGM_P(port, " M4"); break;
  }

  switch (gcode.coolant_mode) {
    case 0 : SERIAL_ECHOPGM_P(port, " M9"); break;
    case 1 : SERIAL_ECHOPGM_P(port, " M7"); break;
    case 2 : SERIAL_ECHOPGM_P(port, " M8"); break;
  }

  // M56
  
  SERIAL_ECHOPAIR_P(port, " T", active_extruder);
  SERIAL_ECHOPAIR_P(port, " F", MMS_TO_MMM(feedrate_mm_s));
  SERIAL_ECHOPAIR_P(port, " S", gcode.spindle_rpm);
  SERIAL_ECHOLNPGM_P(port, "]");
}

// Code from grbl - report.c
// Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram
// and the actual location of the CNC machine. Users may change the following function to their
// specific needs, but the desired real-time data report must be as short as possible. This is
// requires as it minimizes the computational overhead and allows grbl to keep running smoothly,
// especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
void cnc::report_realtime_status(
  #if NUM_SERIAL > 1
    const int8_t port/*= -1*/
  #endif
) {
  // Report current machine state and sub-states
  SERIAL_CHAR_P(port, '<');
  if (!Running) SERIAL_ECHOPGM_P(port, "Idle");
  #if HAS_RESUME_CONTINUE
    else if (Running && wait_for_user) {
            SERIAL_ECHOPGM_P(port, "Hold:1");  // Actively holding
            // SERIAL_CHAR_P(port, '0'); // Ready to resume
    }
  #endif
  else SERIAL_ECHOPGM_P(port, "Run");
/*
    case STATE_JOG: SERIAL_ECHOPGM_P(port, "Jog"); break;
    case STATE_HOMING: SERIAL_ECHOPGM_P(port, "Home"); break;
    case STATE_ALARM: SERIAL_ECHOPGM_P(port, "Alarm"); break;
    case STATE_CHECK_MODE: SERIAL_ECHOPGM_P(port, "Check"); break;
    case STATE_SLEEP: SERIAL_ECHOPGM_P(port, "Sleep"); break;
*/

  // Report machine position
  if (gcode.active_coordinate_system == -1) {
    SERIAL_ECHOPGM_P(port, "|MPos:");
    printFloat(stepper.get_axis_position_mm(X_AXIS), (parser.linear_unit_factor == 1.0 ? 3 : 4));
    SERIAL_CHAR_P(port, ',');
    printFloat(stepper.get_axis_position_mm(Y_AXIS), (parser.linear_unit_factor == 1.0 ? 3 : 4));
    SERIAL_CHAR_P(port, ',');
    printFloat(stepper.get_axis_position_mm(Z_AXIS), (parser.linear_unit_factor == 1.0 ? 3 : 4));
    SERIAL_CHAR_P(port, ',');
    printFloat(stepper.get_axis_position_mm(E_AXIS), (parser.linear_unit_factor == 1.0 ? 3 : 4));
  } else {
    SERIAL_ECHOPGM_P(port, "|WPos:");
    printFloat((stepper.get_axis_position_mm(X_AXIS) + WORKSPACE_OFFSET(X_AXIS)), (parser.linear_unit_factor == 1.0 ? 3 : 4));
    SERIAL_CHAR_P(port, ',');
    printFloat((stepper.get_axis_position_mm(Y_AXIS) + WORKSPACE_OFFSET(Y_AXIS)), (parser.linear_unit_factor == 1.0 ? 3 : 4));
    SERIAL_CHAR_P(port, ',');
    printFloat((stepper.get_axis_position_mm(Z_AXIS) + WORKSPACE_OFFSET(Z_AXIS)), (parser.linear_unit_factor == 1.0 ? 3 : 4));
    SERIAL_CHAR_P(port, ',');
    printFloat((stepper.get_axis_position_mm(E_AXIS) + WORKSPACE_OFFSET(E_AXIS)), (parser.linear_unit_factor == 1.0 ? 3 : 4));
  }

  // Returns planner and serial read buffer states.
  if (planner.block_buffer_head >= planner.block_buffer_tail)
    SERIAL_ECHOPAIR_P(port, "|Bf:", ((BLOCK_BUFFER_SIZE-1)-(planner.block_buffer_head-planner.block_buffer_tail)));
  else SERIAL_ECHOPAIR_P(port, "|Bf:", (planner.block_buffer_tail-planner.block_buffer_head-1));
  SERIAL_ECHOPAIR_P(port, ",", RX_BUFFER_SIZE);

  // Report current line number
  if (gcode_N > 0) SERIAL_ECHOPAIR_P(port, "|Ln:", gcode_N);

  // Report realtime feed speed
  SERIAL_ECHOPAIR_P(port, "|FS:", MMS_TO_MMM(gcode.st_get_realtime_rate));
  SERIAL_ECHOPAIR_P(port, ",", gcode.spindle_rpm);

  // REPORT_FIELD_PIN_STATE
  SERIAL_ECHOPGM_P(port, "|Pn:");
  #if ENABLED(Z_MIN_PROBE_ENDSTOP)
    if(READ(Z_MIN_PROBE_PIN)^Z_MIN_PROBE_ENDSTOP_INVERTING) SERIAL_CHAR_P(port, 'P');
  #endif
  #if HAS_X_MIN || HAS_X_MAX
    #if HAS_X_MIN
      if(READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) SERIAL_CHAR_P(port, 'X');
    #elif HAS_X_MAX
      if(READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) SERIAL_CHAR_P(port, 'X');
    #else
      if((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) || (READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)) SERIAL_CHAR_P(port, 'X');
    #endif
  #endif
  #if HAS_Y_MIN || HAS_Y_MAX
    #if HAS_Y_MIN
      if(READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING) SERIAL_CHAR_P(port, 'Y');
    #elif HAS_Y_MAX
      if(READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING) SERIAL_CHAR_P(port, 'Y');
    #else
      if((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING) || (READ(X_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)) SERIAL_CHAR_P(port, 'Y');
    #endif
  #endif
  #if HAS_Z_MIN || HAS_Z_MAX
    #if HAS_Z_MIN
      if(READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING) SERIAL_CHAR_P(port, 'Z');
    #elif HAS_Z_MAX
      if(READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING) SERIAL_CHAR_P(port, 'Z');
    #else
      if((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING) || (READ(X_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)) SERIAL_CHAR_P(port, 'Z');
    #endif
  #endif
  //if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_SAFETY_DOOR)) { SERIAL_CHAR_P(port, 'D'); } // Safety Door Pin
  //if (ctrl_pin_state,CONTROL_PIN_INDEX_RESET) { SERIAL_CHAR_P(port, 'R'); }         // Reset Button
  //if (ctrl_pin_state,CONTROL_PIN_INDEX_FEED_HOLD) { SERIAL_CHAR_P(port, 'H'); }     // Hold Button
  //if (ctrl_pin_state,CONTROL_PIN_INDEX_CYCLE_START) { SERIAL_CHAR_P(port, 'S'); }   // Start Button

  SERIAL_ECHOLNPGM_P(port, ">");
}

// Convert float to string by immediately converting to a long integer, which contains
// more digits than a float. Number of decimal places, which are tracked by a counter,
// may be set by the user. The integer is then efficiently converted to a string.
// NOTE: AVR '%' and '/' integer operations are very efficient. Bitshifting speed-up
// techniques are actually just slightly slower. Found this out the hard way.
void cnc::printFloat(
     #if NUM_SERIAL > 1
      const int8_t port,/*= -1*/
     #endif
     float n, uint8_t decimal_places)
{
  if (n < 0) {
    SERIAL_CHAR_P(port, '-');
    n = -n;
  }

  uint8_t decimals = decimal_places;
  while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // Add rounding factor. Ensures carryover through entire value.

  // Generate digits backwards and store in string.
  unsigned char buf[13];
  uint8_t i = 0;
  uint32_t a = (long)n;
  while(a > 0) {
    buf[i++] = (a % 10) + '0'; // Get digit
    a /= 10;
  }
  while (i < decimal_places) {
     buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
  }
  if (i == decimal_places) { // Fill in leading zero, if needed.
    buf[i++] = '0';
  }

  // Print the generated string.
  for (; i > 0; i--) {
    if (i == decimal_places) { SERIAL_CHAR_P(port, '.'); } // Insert decimal point in right place.
    SERIAL_CHAR_P(port, buf[i-1]);
  }
}

/**
 * Jog movement of X Y Z A axes
 * Set feed rate
 * Set spindle speed
 */
void cnc::jog() {
  float v = parser.codenum;
  float slp = parser.floatval('S');
  gcode.spindle_rpm=slp;

  switch(parser.command_letter) {
    case 'X':
      destination[X_AXIS] = (relative_mode) ? current_position[X_AXIS] + v
        : LOGICAL_TO_NATIVE(v, X_AXIS);
    break;
    case 'Y':
      destination[Y_AXIS] = (relative_mode) ? current_position[Y_AXIS] + v
        : LOGICAL_TO_NATIVE(v, Y_AXIS);
    break;
    case 'Z':
      destination[Z_AXIS] = (relative_mode) ? current_position[Z_AXIS] + v
        : LOGICAL_TO_NATIVE(v, Z_AXIS);
    break;
    case 'A':
      destination[E_AXIS] = (relative_mode) ? current_position[E_AXIS] + v
        : LOGICAL_TO_NATIVE(v, E_AXIS);
    break;
    case 'F':
      feedrate_mm_s = MMM_TO_MMS(v);
    break;
    case 'S':
        slp=v;
        gcode.spindle_rpm=slp;
    break;
  }

  if (IsRunning()) {
    for (uint8_t i=X_AXIS; i<=E_AXIS; i++) {
      if (parser.seen(axis_codes[i])) {
        v = parser.value_axis_units((AxisEnum)i);
        destination[i] = (relative_mode) ? current_position[i] + v
          : LOGICAL_TO_NATIVE(v, i);
      }
      else destination[i] = current_position[i];
  }

  if (parser.linearval('F') > 0.0)
    feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());

  if (slp > 0.0) {
    if (parser.seen('O')) {
      gcode.spindle_laser_power = parser.value_byte();
      WRITE(SPINDLE_LASER_ENABLE_PIN, SPINDLE_LASER_ENABLE_INVERT); // turn spindle on (active low)
      if (SPINDLE_LASER_PWM_INVERT) gcode.spindle_laser_power = 255 - gcode.spindle_laser_power;
      analogWrite(SPINDLE_LASER_PWM_PIN, gcode.spindle_laser_power);
      gcode.spindle_rpm = 0;
    }
    else {
      if (slp == 0) {
        WRITE(SPINDLE_LASER_ENABLE_PIN, !SPINDLE_LASER_ENABLE_INVERT);                                    // turn spindle off (active low)
        analogWrite(SPINDLE_LASER_PWM_PIN, SPINDLE_LASER_PWM_INVERT ? 255 : 0);                           // only write low byte
        gcode.dwell(SPINDLE_LASER_POWERDOWN_DELAY);
        #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
          print_job_timer.stop();
        #endif
      }
      else {
        #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
          print_job_timer.start();
        #endif
        int16_t ocr_val = (slp - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));  // convert RPM to PWM duty cycle
        NOMORE(ocr_val, 255);                                                                             // limit to max the Atmel PWM will support
        if (slp <= SPEED_POWER_MIN) {
          ocr_val = (SPEED_POWER_MIN - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));            // minimum setting
          gcode.spindle_rpm = SPEED_POWER_MIN;
        }
        else gcode.spindle_rpm = slp;

        if (slp >= SPEED_POWER_MAX) {
          ocr_val = (SPEED_POWER_MAX - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));            // limit to max RPM
          gcode.spindle_rpm = SPEED_POWER_MAX;
        }
        if (SPINDLE_LASER_PWM_INVERT) ocr_val = 255 - ocr_val;
        #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
          print_job_timer.start();
        #endif
        WRITE(SPINDLE_LASER_ENABLE_PIN, SPINDLE_LASER_ENABLE_INVERT);                                     // turn spindle on (active low)
        analogWrite(SPINDLE_LASER_PWM_PIN, ocr_val & 0xFF);                                               // only write low byte
        gcode.dwell(SPINDLE_LASER_POWERUP_DELAY);
      }
    }
  }

    prepare_move_to_destination();
  }
}
