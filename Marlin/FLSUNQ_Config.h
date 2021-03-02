/*==========================================================================
*=================== FLSun QQS - DELTA Printer =============================
*================= With pins_FLSUN_HiSPEED.h BOARD =========================
*===========================================================================
*
* For a Delta printer start with one of the configuration files in the
* config/examples/delta/FLSUN/ directory and customize for your machine.
*
* TIPS:
* -For reduce binary size : https://thborges.github.io/blog/marlin/2019/01/07/reducing-marlin-binary-size.html
* 
* -For NeoPixel use library : https://github.com/Foxies-CSTL/Nano-NeoPixel-Lib/archive/master.zip,
* and active it into the part of platformio.ini "env:flsun_hispeed",
* and commented error in \src\HAL\STM32F1\inc\SanityCheck.h for pass error check.
* -For 2209 change TMC2208 by TMC2209 at the bottom file.
* Téléchargez Marlin pour tester avec le dernier code.bugfix-2.0.x
* Activez et re-flashez le firmware.DEBUG_LEVELING_FEATUREM114_DETAIL
* Connectez-vous à votre imprimante à partir de logiciels hôtes tels que Cura, Printrun ou Repetier Host.
* Envoyez et assurez-vous que vos configurations sont appliquées.M502M500
* Émettons la commande pour activer l’enregistrement maximum.M111 S247
* Effectuez un pour faire votre procédure d’homing standard.G28
* Faites un pour sonder le lit. Cela permettra également le nivellement du lit.G29
* Faites quelques-uns des mouvements qui ont révélé des problèmes avant. Prends des notes.
* Copiez la sortie de journal dans un . Fichier TXT et l’attacher à votre prochaine réponse.
*/


//==============PANDAPI===================env:stm32f103
#define FamPDPI  //

#ifdef FamPDPI
    #define PANDA_PI                     // envs = mega2560
    #define STOCK
    //#define QQSP
    //#define Q_UARTH
    //#define MONITOR_DRIVER_STATUS
    //#define TMC_HARDWARE_SERIAL

    //--------Spec---------------------//
    //#define PANDAPI  1    // PandaPi shield for raspberry Pi printer
    //#define CPU_32_BIT
    //#define MOTOR_SPEED_CALIBRATE 
    //#define DGUS_LCD_UI_PANDAPI
    //#define DGUS_LCD_UI_ORIGIN
    //#define ULTRA_LCD
    //#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
    //---adv----//
    #define SLOWDOWN
    //---------------------------------------------//
    //#define ALL_TMC8                   //(8) For 4xTMC220x_STANDALONE
    //#define Q_UART8                    //(U8/U9) 4xTMC2208
    //#define Q_UART9                    //(U9) 4xTMC2209
    //#define H_TFT
    //#define TFT_COLOR_UI               //(C) UI MARLIN 
    //#define TFT_CLASSIC_UI
    //#define MKS_ROBIN_TFT32
    //#define MKS_ROBIN_TFT_V1_1R
    //#define TFT_GENERIC
    //#define ESP_WIFI                   //(W) Module ESP8266/ESP12
    //#define INV_EXT                    // Uncommment to reverse direction.
    //#define BMG                        //(B) Uncomment to change Extruder step(417).
    //#define MICROSTEPS32
    //#define AUTO_BED_LEVELING_BILINEAR //(A)
    #define AUTO_BED_LEVELING_UBL          //(U)
    #define FILAMENT_RUNOUT_SENSOR     // NC LVGL
    #define ADVANCED_PAUSE_FEATURE     // NC LVGL
    //#define LCD_SET_PROGRESS_MANUALLY
    //#define TCHM
    //----2209--
    //#define StallGuard
    //#define XYZ_CURRENT_HOME 900
    //#define TMC_HOME_PHASE { -1, -1, -1 }//-1 for Homing phase
    //#define StallGuard2
    //#define Z_OFFSET        0.0
    /*----Mode Debug----------*/
    //#define MARLIN_DEV_MODE
    //#define POSTMORTEM_DEBUGGING
    //#define BAUD_RATE_GCODE               // to change BAUDRATES (M575)
    //#define PINS_DEBUGGING                // M43 - display pin status
    //#define DIRECT_PIN_CONTROL            // M42 - Set pin states
    //#define DEBUG_LEVELING_FEATURE        //debug "M111 S32" or "M111 S247"
    //#define TOUCH_UI_DEBUG
    //#define TMC_DEBUG                     //actif debug "M122 S1"
#endif
//----------------------------
//========= Hardware ==========//
/*-------Motherboard-----------*/
//#define QQSP                     // env = hispeedv1
//#define Q5                      // env = hispeed  
// In progress........... ;-)

//*------Drivers-(1 CHOICE)-----*/
//#define STOCK                        //(S) For 4xA4988(green or red color)

/* MODE STANDALONE XYZ+E for QQS & Q5 */
//#define ALL_TMC8                    //(8) For 4xTMC2208_STANDALONE
//#define ALL_TMC9                    //(9) For 4xTMC2209_STANDALONE

/* MODE UART XYZ+E for QQS & Q5 */
//#define Q_UART8                    //(U8) 4xTMC2208 Note: remove on your printer the module WIFI and wire your TMC.
//#define Q_UART9                    //(U9) 4xTMC2209 Note: remove on your printer the module WIFI and wire your TMC.

/* SPECIAL MODE UART XYZ+E for QQS-Pro */
//#define QQS_UARTH                  //(UH) Mode special 2209 wiring with one I/O pin (Remove module ESP12)

/*------- Choice Other driver for EXTRUDER-------//
* Options: 
* LV8729/A4988/TMC2208_STANDALONE/TMC2209_STANDALONE/TMC2208/TMC2209 
*/
//#define DRIVER_EXT A4988

/* QQS Stock have a clone TITAN EXtruder,
* also if you have another try this.
* (T=397)/(B=417)/(b=141)
*/
//#define INV_EXT                    //(T) Uncommment to reverse direction.

#ifdef BMG                        
  #define EXTRUDER_STEPS 417         //(B) BMG to change Extruder step(417).
#endif
#ifdef Mini  
  #define EXTRUDER_STEPS 141         //(b) BMG Mini to change Extruder step(141).
#endif

/*-------Driver TFT Color--(1 CHOICE)-----*/
  //#define MKS_TS35_V2_0
  //#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
  //#define FYSETC_MINI_12864_2_1
  //#define BTT_UI_SPI  // 
  //#define MKS_ROBIN_TFT32            //Mks_Robin_TFT_V2.0
/*-MKS Robin TFT24/28/32 ： ST7789V 320*240 MCU 16bit FSMC-*/
  //#define MKS_ROBIN_TFT35            //Mks_Robin_TFT
  //#define MKS_ROBIN_TFT_V1_1R
  //#define MKS_ROBIN_TFT24
  //#define TFT_GENERIC

/*--- Choice UI TFT ----*/
//#define TFT_CLASSIC_UI             //(F) UI STANDARD 
//#define TFT_COLOR_UI               //(C) UI MARLIN (too big with mode UART+UBL=ok with nanolib)
//#define TFT_LVGL_UI                //(I) UI MKS  => (Bug)

/*----  Modules -----*/
//#define ESP_WIFI                   //(W) Module ESP8266/ESP12
//#define ESP3D_WIFISUPPORT          //(W)

/*For LedStrip which need an external power source on Vcc pin.*/
//#define NEOPIXEL_LED               //(N) Use port GPIO Wifi module (wired to PC7)

//Many options for Modules:
#if ANY(TFT_COLOR_UI, TFT_CLASSIC_UI)
  #define POWER_LOSS_RECOVERY        // NC LVGL pb SD
  #define FILAMENT_RUNOUT_SENSOR     // NC LVGL
  #define ADVANCED_PAUSE_FEATURE     // NC LVGL
  #define LCD_SET_PROGRESS_MANUALLY
#endif
#define LIN_ADVANCE                //(L) Possible Bug with BabyStep.For TMC_UART prefer mode spreadCycle         
#define ARC_SUPPORT                //(R)

//============= End_Hardware ===============//

//Choice add menu: (OPT)
//#define DELTA_CALIBRATION_MENU     //For edit parameters Delta for CLASSIC and COLOR (NC LVGL)
#define PID_EDIT_MENU              //
#define PID_AUTOTUNE_MENU          //
#define LCD_INFO_MENU              // Informations printer.

#ifdef H_TFT
    //-------General options---------//
    #define HOST_ACTION_COMMANDS  //
    #define REPORT_FAN_CHANGE //
    #define M114_DETAIL //
    #define M115_GEOMETRY_REPORT  //
    //---------Options to support printing from onboard SD--------//
    //#define SDSUPPORT //actif
    //#define LONG_FILENAME_HOST_SUPPORT //actif
    //#define AUTO_REPORT_TEMPERATURES //actif
    #define AUTO_REPORT_SD_STATUS
    #define SDCARD_CONNECTION ONBOARD
    #define FILAMENT_RUNOUT_SENSOR 
    #define ADVANCED_PAUSE_FEATURE
    //-----------others-----------//
    #define FAN_KICKSTART_TIME 100
#endif

//  Type Calibration (CAL)
//#define AUTO_BED_LEVELING_BILINEAR //(A)
//#define AUTO_BED_LEVELING_UBL      //(U)

// ---Expe tools
//#define LEVEL_BED_CORNERS
//#define PROBE_OFFSET_WIZARD        // Bug because Delta no have #define HOMING_FEEDRATE_XY

// Option for Octoprint (OCTO)
//#define HOST_ACTION_COMMANDS       // Action Command Prompt support Message on Octoprint
//#define UTF_FILENAME_SUPPORT       // Bug at the reboot
//#define CANCEL_OBJECTS

//
//==================Part for Driver defintions=============//
// Options for Modules Hardware
#ifdef ESP_WIF
  #define BINARY_FILE_TRANSFER       // Bin transfert for host like ESP3D or others.
#endif
#ifdef NEOPIXEL_LED
  #define LED_CONTROL_MENU           // To control LedStrip.
#endif

//TFT Type For TFT_GENERIC
#if ENABLED(TFT_GENERIC)
  #define TFT_DRIVER AUTO //ST7789
  #define TFT_INTERFACE_FSMC
  //#define TFT_RES_480x320
  #define TFT_RES_320x240
#endif

//variables to calculate steps and current
#ifdef MICROSTEPS32
  #define XYZ_MICROSTEPS 32
  #define E_MICROSTEPS 32
#else
  #define XYZ_MICROSTEPS 16
  #define E_MICROSTEPS 16
#endif

#ifndef EXTRUDER_STEPS
  #define EXTRUDER_STEPS 397
#endif
#ifndef XYZJERK
  #define XYZJERK  10
#endif
#ifndef EJERK
  #define EJERK     5
#endif
#ifndef Z_OFFSET
  #define Z_OFFSET         -16.2
#endif
#ifndef XYZ_CURRENT
  #define XYZ_CURRENT       900
#endif
#ifndef XYZ_CURRENT_HOME
  #define XYZ_CURRENT_HOME  800
#endif  
 #ifndef E_CURRENT
  #define E_CURRENT         850
#endif

// Set for QQS(4xA4988) or Q5(3x2208+A4988) 
#ifdef STOCK
  #ifdef Q5
    #define DRIVER_AXES TMC2208_STANDALONE
  #else
    #define DRIVER_AXES A4988
  #endif  
  #ifndef DRIVER_EXT
    #define DRIVER_EXT A4988
  #endif
#endif

// Set for TMC2208_STANDALONE
#ifdef ALL_TMC8
    #define Q_TMC
    #define DRIVER_AXES TMC2208_STANDALONE
    #ifndef DRIVER_EXT
      #define DRIVER_EXT TMC2208_STANDALONE
    #endif
#endif
// Set for TMC2209_STANDALONE 
#ifdef ALL_TMC9
    #define Q_TMC
    #define DRIVER_AXES TMC2209_STANDALONE
    #ifndef DRIVER_EXT
      #define DRIVER_EXT TMC2209_STANDALONE
    #endif
#endif

// Software Serial UART for TMC2208
#ifdef Q_UART8
    #define Q_TMC
    #define DRIVER_AXES TMC2208
    #ifndef DRIVER_EXT
      #define DRIVER_EXT TMC2208
    #endif
#endif

// Software Serial UART for TMC2209
#ifdef Q_UART9
    #define Q_TMC
    #define STEALTHCHOP_E
    #define DRIVER_AXES TMC2209
    #ifndef DRIVER_EXT
      #define DRIVER_EXT TMC2209
    #endif
#endif

// Note:
// HardwareSerial with one pins for four drivers
// Compatible with TMC2209. Provides best performance.
// Requires SLAVE_ADDRESS definitions in Configuration_adv.h
// and proper jumper configuration. Uses I/O pins
// like PA10/PA9/PC7/PA8 only.
#ifdef Q_UARTH
    #define Q_TMC
    #define TMC_HARDWARE_SERIAL
    #define STEALTHCHOP_E
    #define DRIVER_AXES TMC2209
    #ifndef DRIVER_EXT
      #define DRIVER_EXT TMC2209
    #endif
#endif
