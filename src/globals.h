#pragma once
#include <Arduino.h>
#include "DFRobot_GNSS.h"
#include <ESP32Time.h>

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debuglnf(x, y) Serial.printf("%.*f\n", y, x)
#define debugf(x, y) Serial.printf(x, y)
#define debugg(x,y,z) Serial.printf(x,y,z)
#define debugh(w,x,y,z) Serial.printf(w,x,y,z)
#else
#define debug(x)
#define debugln(x)
#define debugf(x, y)
#define debuglnf(x, y)
#define debugg(x,y,z)
#define debugh(w,x,y,z)
#endif

namespace globals
{
 
}