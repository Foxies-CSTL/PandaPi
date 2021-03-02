/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * PANDAPI STM32F1 pin assignments
 *  https://gitee.com/markyue/pandapi_wiki/raw/master/imges/doc/Schematic_PandaPiV2.8.pdf
 */

#if NOT_TARGET(__STM32F1__, STM32F1xx, STM32F0xx)
  #error "Oops! Select an STM32 board in your IDE."
#elif HOTENDS > 2 || E_STEPPERS > 2
  #error "Pandapi supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "PandaPi V28"
#endif

#define BOARD_WEBSITE_URL    "github.com/markniu/PandaPi"
#define DEFAULT_MACHINE_NAME "PANDAPI 3D"

// Release PB3/PB4 (TMC_SW Pins) from JTAG pins
#define DISABLE_JTAG

//#ifdef PANDA0 / PANDA1 /PANDA2 
/* U6 STM32 */
/* U8 PANDA */
/* U  KLIPPER */

//
// Servos
//
#define SERVO0_PIN                          PA11   // PWM0 SERVOS

//
// TMC StallGuard DIAG pins
//
#define X_DIAG_PIN                          PA3   // "X-STOP"
#define Y_DIAG_PIN                          PA4   // "Y-STOP"
#define Z_DIAG_PIN                          PA1  // "Z-STOP"
//#define E0_DIAG_PIN                       PC2  // E0DET
//#define E1_DIAG_PIN                       P  // E1DET

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN                      PA3  // X-MIN
  #else
    #define X_MAX_PIN                      PA3  // E0DET
  #endif
#else
  #define X_STOP_PIN                       PA3  // X-STOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN                      PA4  // Y-MIN
  #else
    #define Y_MAX_PIN                      PA4  // E0DET
  #endif
#else
  #define Y_STOP_PIN                       PA4  // Y-STOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN                      PA1  // Z-MIN
  #else
    #define Z_MAX_PIN                      PA1  // PWRDET
  #endif
#else
  #define Z_STOP_PIN                       PA1  // Y-STOP
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                  PC14  // "PROBE"
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
    #define FIL_RUNOUT_PIN                    PC2   // "E0-STOP"
#endif

//
// Steppers
//
#define X_STEP_PIN         PB8  //7 
#define X_DIR_PIN          PB6  //2 
#define X_ENABLE_PIN       PB9  //0 
//#ifndef X_CS_PIN
//  #define X_CS_PIN       P1_10
//#endif

#define Y_STEP_PIN         PB5  //12 
#define Y_DIR_PIN          PB4  //5 
#define Y_ENABLE_PIN       PB9  //0 
//#ifndef Y_CS_PIN
//  #define Y_CS_PIN       P1_10
//#endif

#define Z_STEP_PIN         PB3  //14 
#define Z_DIR_PIN          PA15 //13 
#define Z_ENABLE_PIN       PB9  //0 
//#ifndef Z_CS_PIN
//  #define Z_CS_PIN       P1_10
//#endif

#define E0_STEP_PIN        PB15 //10 
#define E0_DIR_PIN         PB14 //6 
#define E0_ENABLE_PIN      PB9  //0 
//#ifndef E0_CS_PIN
//  #define E0_CS_PIN       P1_10
//#endif

#define E1_STEP_PIN        PB2  //26
#define E1_DIR_PIN         PA7  //25
#define E1_ENABLE_PIN      PB9  //0
//#ifndef E1_CS_PIN
//  #define E1_CS_PIN       P1_10
//#endif

#define E2_STEP_PIN        PA6  //31
#define E2_DIR_PIN         PA5  //21
#define E2_ENABLE_PIN      PB9  //0
//#ifndef E2_CS_PIN
//  #define E2_CS_PIN       
//#endif

#if HAS_TRINAMIC
  #define TMC_BAUD_RATE                   19200
  #ifdef TMC_HARDWARE_SERIAL /*  TMC2209 */
    #define X_HARDWARE_SERIAL  MSerial1
  //  #define X2_HARDWARE_SERIAL MSerial1
    #define Y_HARDWARE_SERIAL  MSerial1
  //  #define Y2_HARDWARE_SERIAL MSerial1
    #define Z_HARDWARE_SERIAL  MSerial1
    // #define Z2_HARDWARE_SERIAL MSerial1
    #define E0_HARDWARE_SERIAL MSerial1
    #define E1_HARDWARE_SERIAL MSerial1
    #define E2_HARDWARE_SERIAL MSerial1
  #else
  //
  // Software serial
  //
    #define X_SERIAL_TX_PIN                  PB10
    #define X_SERIAL_RX_PIN                  PB11

    #define Y_SERIAL_TX_PIN                  PB10
    #define Y_SERIAL_RX_PIN                  PB11

    #define Z_SERIAL_TX_PIN                  PB10
    #define Z_SERIAL_RX_PIN                  PB11

    #define E0_SERIAL_TX_PIN                 PB10
    #define E0_SERIAL_RX_PIN                 PB11

    #define E1_SERIAL_TX_PIN                 PB10
    #define E1_SERIAL_RX_PIN                 PB11

    #define E2_SERIAL_TX_PIN                 PB10
    #define E2_SERIAL_RX_PIN                 PB11
  #endif
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN     PB0  // Analog Input "TH0"
#define TEMP_1_PIN     PB1  //
#define TEMP_2_PIN     PA2  //255
#define TEMP_3_PIN     DTH11

#define TEMP_BED_PIN     TEMP_1_PIN
#define TEMP_BOARD_PIN   TEMP_3_PIN  //255

#if HOTENDS == 1
  #if TEMP_SENSOR_PROBE
    #define TEMP_PROBE_PIN            TEMP_1_PIN
  #elif TEMP_SENSOR_CHAMBER
    #define TEMP_CHAMBER_PIN          TEMP_1_PIN
  #endif
#endif

//
// Heaters / Fans
//

#define HEATER_0_PIN     PB12 //255
#define HEATER_1_PIN     PA12 //25

#define HEATER_BED_PIN   PB13 //255

#define FAN_SOFT_PWM
#define FAN_PIN          PA8  //255
#define FAN1_PIN         PA9  //255
#define FAN2_PIN         PA10 //255
#define FAN3_PIN         PB7  //255

//#define FAN_PIN          FAN0_PIN  //255

//#ifndef MOSFET_D_PIN
//  #define MOSFET_D_PIN     -1
//#endif

// heat connector index
//#define HOTBED_CODE       0  
//#define HOTEND_0_CODE     1  
//#define HOTEND_1_CODE     2  

#ifdef MAX31856_PANDAPI
  #define HEATER_0_PIN     30
#else
/* U6 PI*/

#endif

//
// Misc. Functions
//
//
#define BEEPER_PIN       PA0
#define CASE_LIGHT_PIN 255

#ifdef MAX31856_PANDAPI
  #define MAX31856_CLK_PIN  29
  #define MAX31856_MISO_PIN 24
  #define MAX31856_MOSI_PIN 28
  #define MAX31856_CS_PIN   27
#endif

//
// Power Supply Control
//

//
// LED / NEOPixel
//

//
// SD Card
//
//


//
// LCD / Controller
//
#if HAS_WIRED_LCD
//#if HAS ULTRA_LCD
  // LCD Display output pins
  #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
    #define LCD_PINS_RS      28 
    #define LCD_PINS_ENABLE  29 
    #define LCD_PINS_D4      27  
  #endif

	#define SD_DETECT_PIN   255   
   // LCD Display input pins
  #if ENABLED(NEWPANEL)
    #define BEEPER_PIN		24 
    #define BTN_ENC 		30 
    #define BTN_EN1 		22 
    #define BTN_EN2 		23 
  #endif // NEWPANEL

#endif // ULTRA_LCD

// Augmentation for auto-assigning RAMPS plugs
//
/**
#if DISABLED(IS_RAMPS_EEB) && DISABLED(IS_RAMPS_EEF) && DISABLED(IS_RAMPS_EFB) && DISABLED(IS_RAMPS_EFF) && DISABLED(IS_RAMPS_SF) && !PIN_EXISTS(MOSFET_D)
  //#if HOTENDS > 1
  #if HAS_MULTI_HOTEND
    #if TEMP_SENSOR_BED
      #define IS_RAMPS_EEB
    #else
      #define IS_RAMPS_EEF
    #endif
  #elif TEMP_SENSOR_BED
    #define IS_RAMPS_EFB
  #else
    #define IS_RAMPS_EFF
  #endif
#endif
*/
