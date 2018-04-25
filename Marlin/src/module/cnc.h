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
 * cnc.h - CNC Routines
 */

#ifndef CNC_H
#define CNC_H

#include "../inc/MarlinConfig.h"
#include "../gcode/gcode.h"
#include "../gcode/parser.h"
#include "motion.h"

#if defined(CNC_MODE)

class cnc {

  public:

    #if ENABLED(FAN_SOFT_PWM)
      static uint8_t soft_pwm_amount_fan[FAN_COUNT],
                     soft_pwm_count_fan[FAN_COUNT];
    #endif

  private:

    #if EARLY_WATCHDOG
      // If controller is running
      static bool inited;
    #endif

  public:
    #if ENABLED(ADC_KEYPAD)
      static uint32_t current_ADCKey_raw;
      static uint8_t ADCKey_count;
    #endif

    /**
     * Instance Methods
     */

    cnc();

    void init();

    /**
     * Call periodically to manage wdt
     */
    static void manage() _O2; // Added _O2 to work around a compiler error

    // grbl - Print current gcode parser mode state
    static void report_gcode_modes(
    #if NUM_SERIAL > 1
      , const int8_t port/*= -1*/
    #endif
    );

    // grbl - Print current gcode parser mode state
    static void report_build_info(
    #if NUM_SERIAL > 1
      const int8_t port/*= -1*/
    #endif
    );
    
    // grbl - Prints real-time data.
    static void report_realtime_status(
    #if NUM_SERIAL > 1
      const int8_t port/*= -1*/
    #endif
    );

    static void jog();

  private:

    static void printFloat(
     #if NUM_SERIAL > 1
      const int8_t port,/*= -1*/
     #endif
     float n, uint8_t decimal_places);
};

extern cnc cncManager;

#endif // CNC_MODE

#endif // CNC_H
