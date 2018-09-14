/**************************************************************
*
*   "iSpindel"
*   All rights reserverd by S.Lang <universam@web.de>
*
**************************************************************/

#ifndef _GLOBALS_H
#define _GLOBALS_H

//#pragma once
//#define USE_DMP false
#include <Arduino.h>
#include <Hash.h>

#include <Ticker.h>

#include <I2Cdev.h>
#include <MPU6050.h>
extern Ticker flasher;

// defines go here
#define FIRMWAREVERSION    "6.0.0 modified"
#define MY_BROKER "www.ferduino.com"
#define MY_PORT 1883

#define WM_DEBUG false

#ifdef NO_CONSOLE
#define CONSOLE(...) \
  do                 \
  {                  \
  } while (0)
#define CONSOLELN    CONSOLE
#define CONSOLEF     CONSOLE
#else
#define CONSOLE(...)           \
  do                           \
  {                            \
    Serial.print(__VA_ARGS__); \
  } while (0)
#define CONSOLELN(...)           \
  do                             \
  {                              \
    Serial.println(__VA_ARGS__); \
  } while (0)
#endif

#define PORTALTIMEOUT      300

#define ADCDIVISOR         191.8
#define ONE_WIRE_BUS       D7 // DS18B20 on ESP pin12
#define RESOLUTION         12 // 12bit resolution == 750ms update rate
#define OWinterval         (760 / (1 << (12 - RESOLUTION)))
#define CFGFILE            "/config.json"
#define TKIDSIZE           40

#define MEDIANROUNDSMAX    49
#define MEDIANROUNDSMIN    29
#define MEDIANAVRG         MEDIANROUNDSMIN
#define MEDIAN_MAX_SIZE    MEDIANROUNDSMAX

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT       1
// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS       0

#define WIFIENADDR        1
#define RTCVALIDFLAG      0xCAFEBABE

// sleep management
#define RTCSLEEPADDR      5
#define MAXSLEEPTIME      3600UL //TODO
#define EMERGENCYSLEEP    (my_sleeptime * 3 < MAXSLEEPTIME ? MAXSLEEPTIME : my_sleeptime * 3)
#define LOWBATT           3.3

#define UNINIT            0

#define MAX_MESSAGE_SIZE 300
#define MAX_TOPIC_ID_SIZE 50

extern int16_t ax, ay, az;
extern float   Volt, Temperatur, Tilt, Gravity;
extern int16_t my_aX, my_aY, my_aZ;

extern MPU6050_Base accelgyro;
extern bool saveConfig();
extern void formatSpiffs();

bool testAccel();
float scaleTemperature(float t);
String tempScaleLabel();
bool isNumeric(String str);
bool isNumericInt(String str);

#endif
