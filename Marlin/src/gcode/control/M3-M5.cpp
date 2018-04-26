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

#include "../../inc/MarlinConfig.h"

#if ENABLED(SPINDLE_LASER_ENABLE)

#include "../gcode.h"
#include "../../module/stepper.h"

#if ENABLED(PRINTJOB_TIMER_AUTOSTART)
  #include "../../module/printcounter.h"
#endif

bool    GcodeSuite::spindle_on_off,
        GcodeSuite::spindle_rev;
float   GcodeSuite::spindle_rpm;

#if defined(CNC_MODE)
  // grbl compatibility
  float GcodeSuite::st_get_realtime_rate;

  uint8_t GcodeSuite::coolant_mode;
#endif

/**
 * M3: Spindle Clockwise
 * M4: Spindle Counter-clockwise
 *
 *  S0 turns off spindle.
 *
 *  If no speed PWM output is defined then M3/M4 just turns it on.
 *
 *  At least 12.8KHz (50Hz * 256) is needed for spindle PWM.
 *  Hardware PWM is required. ISRs are too slow.
 *
 * NOTE: WGM for timers 3, 4, and 5 must be either Mode 1 or Mode 5.
 *       No other settings give a PWM signal that goes from 0 to 5 volts.
 *
 *       The system automatically sets WGM to Mode 1, so no special
 *       initialization is needed.
 *
 *       WGM bits for timer 2 are automatically set by the system to
 *       Mode 1. This produces an acceptable 0 to 5 volt signal.
 *       No special initialization is needed.
 *
 * NOTE: A minimum PWM frequency of 50 Hz is needed. All prescaler
 *       factors for timers 2, 3, 4, and 5 are acceptable.
 *
 *  SPINDLE_FWD_PIN needs an external pullup or it may power on
 *  the spindle/laser during power-up or when connecting to the host
 *  (usually goes through a reset which sets all I/O pins to tri-state)
 *
 *  PWM duty cycle goes from 0 (off) to 255 (always on).
 */

// Wait for spindle to come up to speed
inline void delay_for_power_up() { gcode.dwell(SPINDLE_LASER_POWERUP_DELAY); }

// Wait for spindle to stop turning
inline void delay_for_power_down() { gcode.dwell(SPINDLE_LASER_POWERDOWN_DELAY); }

// Spindle Forward
void GcodeSuite::M3(bool use_delay) {
  spindle_rev=false;
  stepper.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle
  const bool rotation_dir = (spindle_rev == SPINDLE_INVERT_DIR);
  if (SPINDLE_STOP_ON_DIR_CHANGE) {
    spindle_on_off=false;
    Spindle_On_Off(use_delay); // turn spindle off
  }
  WRITE(SPINDLE_FWD_PIN, rotation_dir);
  #if SPINDLE_DIR_CHANGE
    WRITE(SPINDLE_REV_PIN, !rotation_dir);
  #endif
  spindle_on_off=true;
  Spindle_On_Off(use_delay); // turn spindle on
}

// Spindle Reverse
void GcodeSuite::M4(bool use_delay) {
  spindle_rev=true;
  stepper.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle
  const bool rotation_dir = (spindle_rev == SPINDLE_INVERT_DIR);
  #if SPINDLE_DIR_CHANGE
    if (SPINDLE_STOP_ON_DIR_CHANGE) {
      spindle_on_off=false;
      Spindle_On_Off(use_delay); // turn spindle off
    }
    WRITE(SPINDLE_FWD_PIN, !rotation_dir);
    WRITE(SPINDLE_REV_PIN, rotation_dir);
  #else
    WRITE(SPINDLE_FWD_PIN, rotation_dir);
  #endif
  spindle_on_off=true;
  Spindle_On_Off(use_delay); // turn spindle on
}

/**
 * M5 turn off spindle
 */
void GcodeSuite::M5(bool use_delay) {
  spindle_on_off=false;
  Spindle_On_Off(use_delay);
}

// Spindle on/off
void GcodeSuite::Spindle_On_Off(bool use_delay) {
  stepper.synchronize();
  if (spindle_on_off) {
    if (spindle_rev) M4(use_delay);
    else M3(use_delay);
  }
  else {
    WRITE(SPINDLE_FWD_PIN, !SPINDLE_INVERT_DIR);
    #if SPINDLE_DIR_CHANGE
      WRITE(SPINDLE_REV_PIN, !SPINDLE_INVERT_DIR);
    #endif
    if (use_delay) delay_for_power_down();
  }
  #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
    print_job_timer.stop();
  #endif
}

#if ENABLED(SPINDLE_LASER_PWM)
  void GcodeSuite::Spindle_Speed_Adjust(bool use_delay, bool speed_only) {
    /**
     * Our final value for ocr_val is an unsigned 8 bit value between 0 and 255 which usually means uint8_t.
     * Went to uint16_t because some of the uint8_t calculations would sometimes give 1000 0000 rather than 1111 1111.
     * Then needed to AND the uint16_t result with 0x00FF to make sure we only wrote the byte of interest.
     */
    if (!speed_only) gcode.spindle_rpm = parser.floatval('S');

    if (gcode.spindle_rpm == 0) {
      analogWrite(SPINDLE_LASER_PWM_PIN, SPINDLE_LASER_PWM_INVERT ? 255 : 0);                           // only write low byte
      if (use_delay) delay_for_power_down();
    }
    else {
      int16_t ocr_val = (gcode.spindle_rpm - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));          // convert RPM to PWM duty cycle
      NOMORE(ocr_val, 255);                                                                             // limit to max the Atmel PWM will support
      if (gcode.spindle_rpm <= SPEED_POWER_MIN) {
        ocr_val = (SPEED_POWER_MIN - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));            // minimum setting
        gcode.spindle_rpm = SPEED_POWER_MIN;
      }
      if (gcode.spindle_rpm >= SPEED_POWER_MAX) {
        ocr_val = (SPEED_POWER_MAX - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));            // limit to max RPM
        gcode.spindle_rpm = SPEED_POWER_MAX;
      }
      if (SPINDLE_LASER_PWM_INVERT) ocr_val = 255 - ocr_val;
      analogWrite(SPINDLE_LASER_PWM_PIN, ocr_val & 0xFF);                                               // only write low byte
      if (use_delay) delay_for_power_up();
    }
  }
#endif

#if ENABLED(SPINDLE_LASER_PWM)
  void GcodeSuite::Spindle_Fwd_Rev(bool use_delay) {
    if(gcode.spindle_rev) gcode.M4(use_delay);
    else gcode.M3(use_delay);
  }
#endif

#endif // SPINDLE_LASER_ENABLE
