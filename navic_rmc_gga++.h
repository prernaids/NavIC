/*
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2022 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __navic_gn_rmc_gga_h
#define __navic_gn_rmc_gga_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <limits.h>

#define _NavIC_VERSION "1.0.3" // software version of this library
#define _NavIC_MPH_PER_KNOT 1.15077945
#define _NavIC_MPS_PER_KNOT 0.51444444
#define _NavIC_KMPH_PER_KNOT 1.852
#define _NavIC_MILES_PER_METER 0.00062137112
#define _NavIC_KM_PER_METER 0.001
#define _NavIC_FEET_PER_METER 3.2808399
#define _NavIC_MAX_FIELD_SIZE 15

struct RawDegrees
{
   uint16_t deg;
   uint32_t billionths;
   bool negative;

public:
   RawDegrees() : deg(0), billionths(0), negative(false)
   {
   }
};

struct NavIC_Location
{
   friend class navic_gn_rmc_gga;

public:
   bool isValid() const { return valid; }
   bool isUpdated() const { return updated; }
   uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   const RawDegrees &rawLat()
   {
      updated = false;
      return rawLatData;
   }
   const RawDegrees &rawLng()
   {
      updated = false;
      return rawLngData;
   }
   double lat();
   double lng();

   NavIC_Location() : valid(false), updated(false)
   {
   }

private:
   bool valid, updated;
   RawDegrees rawLatData, rawLngData, rawNewLatData, rawNewLngData;
   uint32_t lastCommitTime;
   void commit();
   void setLatitude(const char *term);
   void setLongitude(const char *term);
};

struct NavIC_date
{
   friend class navic_gn_rmc_gga;

public:
   bool isValid() const { return valid; }
   bool isUpdated() const { return updated; }
   uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }

   uint32_t value()
   {
      updated = false;
      return date;
   }
   uint16_t year();
   uint8_t month();
   uint8_t day();

   NavIC_date() : valid(false), updated(false), date(0)
   {
   }

private:
   bool valid, updated;
   uint32_t date, newDate;
   uint32_t lastCommitTime;
   void commit();
   void setDate(const char *term);
};

struct NavIC_time
{
   friend class navic_gn_rmc_gga;

public:
   bool isValid() const { return valid; }
   bool isUpdated() const { return updated; }
   uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }

   uint32_t value()
   {
      updated = false;
      return time;
   }
   uint8_t hour();
   uint8_t minute();
   uint8_t second();
   uint8_t centisecond();

   NavIC_time() : valid(false), updated(false), time(0)
   {
   }

private:
   bool valid, updated;
   uint32_t time, newTime;
   uint32_t lastCommitTime;
   void commit();
   void setTime(const char *term);
};

struct NavIC_decimal
{
   friend class navic_gn_rmc_gga;

public:
   bool isValid() const { return valid; }
   bool isUpdated() const { return updated; }
   uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   int32_t value()
   {
      updated = false;
      return val;
   }

   NavIC_decimal() : valid(false), updated(false), val(0)
   {
   }

private:
   bool valid, updated;
   uint32_t lastCommitTime;
   int32_t val, newval;
   void commit();
   void set(const char *term);
};

struct NavIC_integer
{
   friend class navic_gn_rmc_gga;

public:
   bool isValid() const { return valid; }
   bool isUpdated() const { return updated; }
   uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   uint32_t value()
   {
      updated = false;
      return val;
   }

   NavIC_integer() : valid(false), updated(false), val(0)
   {
   }

private:
   bool valid, updated;
   uint32_t lastCommitTime;
   uint32_t val, newval;
   void commit();
   void set(const char *term);
};

struct NavIC_speed : NavIC_decimal
{
   double knots() { return value() / 100.0; }
   double mph() { return _NavIC_MPH_PER_KNOT * value() / 100.0; }
   double mps() { return _NavIC_MPS_PER_KNOT * value() / 100.0; }
   double kmph() { return _NavIC_KMPH_PER_KNOT * value() / 100.0; }
};

struct NavIC_course : public NavIC_decimal
{
   double deg() { return value() / 100.0; }
};

struct NavIC_altitudes : NavIC_decimal
{
   double meters() { return value() / 100.0; }
   double miles() { return _NavIC_MILES_PER_METER * value() / 100.0; }
   double kilometers() { return _NavIC_KM_PER_METER * value() / 100.0; }
   double feet() { return _NavIC_FEET_PER_METER * value() / 100.0; }
};

struct NavIC_HDOP : NavIC_decimal
{
   double hdop() { return value() / 100.0; }
};

class navic_gn_rmc_gga;
class NavIC_CUSTOM
{
public:
   NavIC_CUSTOM(){};
   NavIC_CUSTOM(navic_gn_rmc_gga &gps, const char *sentenceName, int termNumber);
   void begin(navic_gn_rmc_gga &gps, const char *_sentenceName, int _termNumber);

   bool isUpdated() const { return updated; }
   bool isValid() const { return valid; }
   uint32_t age() const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   const char *value()
   {
      updated = false;
      return buffer;
   }

private:
   void commit();
   void set(const char *term);

   char stagingBuffer[_NavIC_MAX_FIELD_SIZE + 1];
   char buffer[_NavIC_MAX_FIELD_SIZE + 1];
   unsigned long lastCommitTime;
   bool valid, updated;
   const char *sentenceName;
   int termNumber;
   friend class navic_gn_rmc_gga;
   NavIC_CUSTOM *next;
};

class navic_gn_rmc_gga
{
public:
   navic_gn_rmc_gga();
   bool encode(char c); // process one character received from GPS
   navic_gn_rmc_gga &operator<<(char c)
   {
      encode(c);
      return *this;
   }

   NavIC_Location location;
   NavIC_date
       date;
   NavIC_time time;
   NavIC_speed speed;
   NavIC_course course;
   NavIC_altitudes altitude;
   NavIC_integer satellites;
   NavIC_HDOP hdop;

   static const char *libraryVersion() { return _NavIC_VERSION; }

   static double distanceBetween(double lat1, double long1, double lat2, double long2);
   static double courseTo(double lat1, double long1, double lat2, double long2);
   static const char *cardinal(double course);

   static int32_t parseDecimal(const char *term);
   static void parseDegrees(const char *term, RawDegrees &deg);

   uint32_t charsProcessed() const { return encodedCharCount; }
   uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
   uint32_t failedChecksum() const { return failedChecksumCount; }
   uint32_t passedChecksum() const { return passedChecksumCount; }

private:
   enum
   {
      GPS_SENTENCE_GPGGA,
      GPS_SENTENCE_GPRMC,
      GPS_SENTENCE_OTHER
   };

   // parsing state variables
   uint8_t parity;
   bool isChecksumTerm;
   char term[_NavIC_MAX_FIELD_SIZE];
   uint8_t curSentenceType;
   uint8_t curTermNumber;
   uint8_t curTermOffset;
   bool sentenceHasFix;

   // custom element support
   friend class NavIC_CUSTOM;
   NavIC_CUSTOM *customElts;
   NavIC_CUSTOM *customCandidates;
   void insertCustom(NavIC_CUSTOM *pElt, const char *sentenceName, int index);

   // statistics
   uint32_t encodedCharCount;
   uint32_t sentencesWithFixCount;
   uint32_t failedChecksumCount;
   uint32_t passedChecksumCount;

   // internal utilities
   int fromHex(char a);
   bool endOfTermHandler();
};

#endif // def(__navic_gn_rmc_gga_h)
