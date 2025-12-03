# Project Overview
This project is a stand alone GNSS module that utlizes the DFRobot Gravity GNSS receiver. The data that comes in is then formatted and sent over the CAN bus for other modules to use.

Some example use cases are:
-  Digital Clocks
-  Navigation devices
-  Geospatial macros that control devices based on pinned locations


## Enums for GNSS Mode
 -- 1   eGPS              use gps
 -- 2   eBeiDou           use beidou
 -- 3   eGPS_BeiDou       use gps + beidou
 -- 4   eGLONASS          use glonass
 -- 5   eGPS_GLONASS      use gps + glonass
 -- 6   eBeiDou_GLONASS   use beidou +glonass
 -- 7   eGPS_BeiDou_GLONASS use gps + beidou + glonass