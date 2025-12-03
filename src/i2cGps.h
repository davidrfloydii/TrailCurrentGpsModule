#pragma once
#include <Arduino.h>
#include "globals.h"
#include "DFRobot_GNSS.h"

DFRobot_GNSS_I2C gnss(&Wire, GNSS_DEVICE_ADDR);

namespace i2cGps
{
// Sets up the initialization of the GNSS receiver using I2C
    void setup()
    {
        while (!gnss.begin())
        {
            debugln("NO Deivces !");
        }
        gnss.enablePower(); // Enable gnss power
        /** Set GNSS to be used
         *   eGPS              use gps
         *   eBeiDou           use beidou
         *   eGPS_BeiDou       use gps + beidou
         *   eGLONASS          use glonass
         *   eGPS_GLONASS      use gps + glonass
         *   eBeiDou_GLONASS   use beidou +glonass
         *   eGPS_BeiDou_GLONASS use gps + beidou + glonass
         */
        gnss.setGnss(eGPS_BeiDou_GLONASS);
        // Turn off RGB to save power since the case will not be in a visible location.
        gnss.setRgbOff();
    }

    void getGnssData(uint16_t &currentYear,
                     uint8_t &currentMonth,
                     uint8_t &currentDay,
                     uint8_t &currentHour,
                     uint8_t &currentMinute,
                     uint8_t &currentSecond,
                     float &currentLatitude,
                     float &currentLongitude,
                     double &currentAltitude,
                     uint8_t &currentNumSatUsed,
                     double &currentSpeedOverGround,
                     double &currentCourseOverGround,
                     uint8_t &currentGnssMode)
    {
        sTim_t utc = gnss.getUTC();
        sTim_t date = gnss.getDate();
        sLonLat_t lat = gnss.getLat();
        sLonLat_t lon = gnss.getLon();
        char latDir = lat.latDirection;
        char lonDir = lon.lonDirection;
        double speedOverGround = gnss.getSog();
        double courseOverGround = gnss.getCog();
        double alt = gnss.getAlt();
        uint8_t numSatUsed = gnss.getNumSatUsed();
        uint8_t gnssMode = gnss.getGnssMode();

        currentYear = date.year;
        currentMonth = date.month;
        currentDay = date.date;
        currentHour = utc.hour;
        currentMinute = utc.minute;
        currentSecond = utc.second;
        if (((char)lat.latDirection) == 'N')
        {
            currentLatitude = lat.latitudeDegree;
        }
        else
        {
            currentLatitude = -lat.latitudeDegree;
        }
        if (((char)lon.lonDirection) == 'E')
        {
            currentLongitude = lon.lonitudeDegree;
        }
        else
        {
            currentLongitude = -lon.lonitudeDegree;
        }
        currentSpeedOverGround = speedOverGround;
        currentCourseOverGround = courseOverGround;
        currentNumSatUsed = numSatUsed;
        currentAltitude = alt;
        currentGnssMode = gnssMode;
        debugln("GNSS Data Retrieved");
        return;
    }
}   