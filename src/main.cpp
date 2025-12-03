#include "globals.h"
#include "i2cGps.h"
#include "canHelper.h"

uint16_t currentYear = 0;
uint8_t currentMonth = 0;
uint8_t currentDay = 0;
uint8_t currentHour = 0;
uint8_t currentMinute = 0;
uint8_t currentSecond = 0;
float currentLatitude = 0.0;
float currentLongitude = 0.0;
double currentAltitude = 0.0;
uint8_t currentNumSatUsed = 0;
double currentSpeedOverGround = 0.0;
double currentCourseOverGround = 0.0;
uint8_t currentGnssMode = 0;

unsigned long updateStartMillis;
unsigned long currentUpdateMillis;
const unsigned long updateIntervalMillis = 100;

void setup()
{
  Serial.begin(115200);
  i2cGps::setup();
  canHelper::setup();
}

void loop()
{
  currentUpdateMillis = millis();
  if (currentUpdateMillis - updateStartMillis >= updateIntervalMillis)
  {
    i2cGps::getGnssData(currentYear, currentMonth, currentDay, currentHour, currentMinute, currentSecond, currentLatitude, currentLongitude, currentAltitude, currentNumSatUsed, currentSpeedOverGround, currentCourseOverGround, currentGnssMode);
    canHelper::sendGnssData(currentYear, currentMonth, currentDay, currentHour, currentMinute, currentSecond, currentLatitude, currentLongitude, currentAltitude, currentNumSatUsed, currentSpeedOverGround, currentCourseOverGround, currentGnssMode);
    updateStartMillis = currentUpdateMillis;
  }
}
