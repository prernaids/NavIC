/*
Tinynavic++ - a small navic library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
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

#include "navic_rmc_gga++.cpp"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>

// #define _GNRMCterm "GNRMC"
// #define _GNGGAterm "GNGGA"
#define _GNRMCterm "GNRMC"
#define _GNGGAterm "GNGGA"

navic_gn_rmc_gga::navic_gn_rmc_gga()
    : parity(0), isChecksumTerm(false), curSentenceType(NAVIC_SENTENCE_OTHER), curTermNumber(0), curTermOffset(0), sentenceHasFix(false), customElts(0), customCandidates(0), encodedCharCount(0), sentencesWithFixCount(0), failedChecksumCount(0), passedChecksumCount(0)
{
  term[0] = '\0';
}

//
// public methods
//

bool navic_gn_rmc_gga::encode(char c)
{
  ++encodedCharCount;

  switch (c)
  {
  case ',': // term terminators
    parity ^= (uint8_t)c;
  case '\r':
  case '\n':
  case '*':
  {
    bool isValidSentence = false;
    if (curTermOffset < sizeof(term))
    {
      term[curTermOffset] = 0;
      isValidSentence = endOfTermHandler();
    }
    ++curTermNumber;
    curTermOffset = 0;
    isChecksumTerm = c == '*';
    return isValidSentence;
  }
  break;

  case '$': // sentence begin
    curTermNumber = curTermOffset = 0;
    parity = 0;
    curSentenceType = NAVIC_SENTENCE_OTHER;
    isChecksumTerm = false;
    sentenceHasFix = false;
    return false;

  default: // ordinary characters
    if (curTermOffset < sizeof(term) - 1)
      term[curTermOffset++] = c;
    if (!isChecksumTerm)
      parity ^= c;
    return false;
  }

  return false;
}

//
// internal utilities
//
int navic_gn_rmc_gga::fromHex(char a)
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

// static
// Parse a (potentially negative) number with up to 2 decimal digits -xxxx.yy
int32_t navic_gn_rmc_gga::parseDecimal(const char *term)
{
  bool negative = *term == '-';
  if (negative)
    ++term;
  int32_t ret = 100 * (int32_t)atol(term);
  while (isdigit(*term))
    ++term;
  if (*term == '.' && isdigit(term[1]))
  {
    ret += 10 * (term[1] - '0');
    if (isdigit(term[2]))
      ret += term[2] - '0';
  }
  return negative ? -ret : ret;
}

// static
// Parse degrees in that funny NMEA format DDMM.MMMM
void navic_gn_rmc_gga::parseDegrees(const char *term, RawDegrees &deg)
{
  uint32_t leftOfDecimal = (uint32_t)atol(term);
  uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
  uint32_t multiplier = 10000000UL;
  uint32_t tenMillionthsOfMinutes = minutes * multiplier;

  deg.deg = (int16_t)(leftOfDecimal / 100);

  while (isdigit(*term))
    ++term;

  if (*term == '.')
    while (isdigit(*++term))
    {
      multiplier /= 10;
      tenMillionthsOfMinutes += (*term - '0') * multiplier;
    }

  deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
  deg.negative = false;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool navic_gn_rmc_gga::endOfTermHandler()
{
  // If it's the checksum term, and the checksum checks out, commit
  if (isChecksumTerm)
  {
    byte checksum = 16 * fromHex(term[0]) + fromHex(term[1]);
    if (checksum == parity)
    {
      passedChecksumCount++;
      if (sentenceHasFix)
        ++sentencesWithFixCount;

      switch (curSentenceType)
      {
      case NAVIC_SENTENCE_GNRMC:
        date.commit();
        time.commit();
        if (sentenceHasFix)
        {
          location.commit();
          speed.commit();
          course.commit();
        }
        break;
      case NAVIC_SENTENCE_GNGGA:
        time.commit();
        if (sentenceHasFix)
        {
          location.commit();
          altitude.commit();
        }
        satellites.commit();
        hdop.commit();
        break;
      }

      // Commit all custom listeners of this sentence type
      for (navic_rmc_gga
               Custom *p = customCandidates;
           p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0; p = p->next)
        p->commit();
      return true;
    }

    else
    {
      ++failedChecksumCount;
    }

    return false;
  }

  // the first term determines the sentence type
  if (curTermNumber == 0)
  {
    if (!strcmp(term, _GNRMCterm))
      curSentenceType = NavIC_SENTENCE_GNRMC;
    else if (!strcmp(term, _GNGGAterm))
      curSentenceType = NavIC_SENTENCE_GNGGA;
    else
      curSentenceType = NAVIC_SENTENCE_OTHER;

    // Any custom candidates of this sentence type?
    for (customCandidates = customElts; customCandidates != NULL && strcmp(customCandidates->sentenceName, term) < 0; customCandidates = customCandidates->next)
      ;
    if (customCandidates != NULL && strcmp(customCandidates->sentenceName, term) > 0)
      customCandidates = NULL;

    return false;
  }

  if (curSentenceType != NAVIC_SENTENCE_OTHER && term[0])
    switch (COMBINE(curSentenceType, curTermNumber))
    {
    case COMBINE(NAVIC_SENTENCE_GNRMC, 1): // Time in both sentences
    case COMBINE(NAVIC_SENTENCE_GNGGA, 1):
      time.setTime(term);
      break;
    case COMBINE(NAVIC_SENTENCE_GNRMC, 2): // GNRMC validity
      sentenceHasFix = term[0] == 'A';
      break;
    case COMBINE(NAVIC_SENTENCE_GNRMC, 3): // Latitude
    case COMBINE(NAVIC_SENTENCE_GNGGA, 2):
      location.setLatitude(term);
      break;
    case COMBINE(NAVIC_SENTENCE_GNRMC, 4): // N/S
    case COMBINE(NAVIC_SENTENCE_GNGGA, 3):
      location.rawNewLatData.negative = term[0] == 'S';
      break;
    case COMBINE(NAVIC_SENTENCE_GNRMC, 5): // Longitude
    case COMBINE(NAVIC_SENTENCE_GNGGA, 4):
      location.setLongitude(term);
      break;
    case COMBINE(NAVIC_SENTENCE_GNRMC, 6): // E/W
    case COMBINE(NAVIC_SENTENCE_GNGGA, 5):
      location.rawNewLngData.negative = term[0] == 'W';
      break;
    case COMBINE(NAVIC_SENTENCE_GNRMC, 7): // Speed (GNRMC)
      speed.set(term);
      break;
    case COMBINE(NAVIC_SENTENCE_GNRMC, 8): // Course (GNRMC)
      course.set(term);
      break;
    case COMBINE(NAVIC_SENTENCE_GNRMC, 9): // Date (GNRMC)
      date.setDate(term);
      break;
    case COMBINE(NAVIC_SENTENCE_GNGGA, 6): // Fix data (GNGGA)
      sentenceHasFix = term[0] > '0';
      break;
    case COMBINE(NAVIC_SENTENCE_GNGGA, 7): // Satellites used (GNGGA)
      satellites.set(term);
      break;
    case COMBINE(NAVIC_SENTENCE_GNGGA, 8): // HDOP
      hdop.set(term);
      break;
    case COMBINE(NAVIC_SENTENCE_GNGGA, 9): // Altitude (GNGGA)
      altitude.set(term);
      break;
    }

  // Set custom values as needed
  for (navic_rmc_gga
           Custom *p = customCandidates;
       p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0 && p->termNumber <= curTermNumber; p = p->next)
    if (p->termNumber == curTermNumber)
      p->set(term);

  return false;
}

/* static */
double navic_gn_rmc_gga::distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double navic_gn_rmc_gga::courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

const char *navic_gn_rmc_gga::cardinal(double course)
{
  static const char *directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

void navic_location::commit()
{
  rawLatData = rawNewLatData;
  rawLngData = rawNewLngData;
  lastCommitTime = millis();
  valid = updated = true;
}

void navic_location::setLatitude(const char *term)
{
  navic_rmc_gga
      Plus::parseDegrees(term, rawNewLatData);
}

void navic_location::setLongitude(const char *term)
{
  navic_rmc_gga
      Plus::parseDegrees(term, rawNewLngData);
}

double navic_location::lat()
{
  updated = false;
  double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
  return rawLatData.negative ? -ret : ret;
}

double navic_location::lng()
{
  updated = false;
  double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
  return rawLngData.negative ? -ret : ret;
}

void navic_date::commit()
{
  date = newDate;
  lastCommitTime = millis();
  valid = updated = true;
}

void navic_time::commit()
{
  time = newTime;
  lastCommitTime = millis();
  valid = updated = true;
}

void navic_time::setTime(const char *term)
{
  newTime = (uint32_t)navic_rmc_gga
      Plus::parseDecimal(term);
}

void navic_date::setDate(const char *term)
{
  newDate = atol(term);
}

uint16_t navic_date::year()
{
  updated = false;
  uint16_t year = date % 100;
  return year + 2000;
}

uint8_t navic_date::month()
{
  updated = false;
  return (date / 100) % 100;
}

uint8_t navic_date::day()
{
  updated = false;
  return date / 10000;
}

uint8_t navic_time::hour()
{
  updated = false;
  return time / 1000000;
}

uint8_t navic_time::minute()
{
  updated = false;
  return (time / 10000) % 100;
}

uint8_t navic_time::second()
{
  updated = false;
  return (time / 100) % 100;
}

uint8_t navic_time::centisecond()
{
  updated = false;
  return time % 100;
}

void navic_decimal::commit()
{
  val = newval;
  lastCommitTime = millis();
  valid = updated = true;
}

void navic_decimal::set(const char *term)
{
  newval = navic_rmc_gga
      Plus::parseDecimal(term);
}

void navic_integer::commit()
{
  val = newval;
  lastCommitTime = millis();
  valid = updated = true;
}

void navic_integer::set(const char *term)
{
  newval = atol(term);
}

navic_custom::navic_custom(navic_gn_rmc_gga &navic, const char *_sentenceName, int _termNumber)
{
  begin(navic, _sentenceName, _termNumber);
}

void navic_custom::begin(navic_gn_rmc_gga &navic, const char *_sentenceName, int _termNumber)
{
  lastCommitTime = 0;
  updated = valid = false;
  sentenceName = _sentenceName;
  termNumber = _termNumber;
  memset(stagingBuffer, '\0', sizeof(stagingBuffer));
  memset(buffer, '\0', sizeof(buffer));

  // Insert this item into the navic tree
  navic.insertCustom(this, _sentenceName, _termNumber);
}

void navic_custom::commit()
{
  strcpy(this->buffer, this->stagingBuffer);
  lastCommitTime = millis();
  valid = updated = true;
}

void navic_custom::set(const char *term)
{
  strncpy(this->stagingBuffer, term, sizeof(this->stagingBuffer));
}

void navic_gn_rmc_gga::insertCustom(navic_custom *pElt, const char *sentenceName, int termNumber)
{
  navic_rmc_gga
      Custom **ppelt;

  for (ppelt = &this->customElts; *ppelt != NULL; ppelt = &(*ppelt)->next)
  {
    int cmp = strcmp(sentenceName, (*ppelt)->sentenceName);
    if (cmp < 0 || (cmp == 0 && termNumber < (*ppelt)->termNumber))
      break;
  }

  pElt->next = *ppelt;
  *ppelt = pElt;
}
