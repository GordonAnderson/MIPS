/*
  softRTC 


  MIT License

  Copyright (c) 2024 Manzar-E-Hassin

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  Published at 15 March, 2025 @ 12:55 PM (UTC+6)

*/

#include "softRTC.h"
#include <Arduino.h>

// Error message macros
#define ERR_INVALID_DATE() Serial.println(F("Invalid date & time!"))

uint8_t softRTC::getWeekdays(uint8_t day, uint8_t month, uint16_t year) 
{
  const int y0 = year - (month < 3);
  const int x  = y0 + y0/4 - y0/100 + y0/400;
  const int m0 = month + 12*(month < 3) - 2;
  return (day + x + m0*31/12) % 7 + 1;
}

void softRTC::Convert_To_12h(uint8_t &hour, bool &isPM) 
{
  isPM = (hour >= 12);
  if(hour==0) hour=12; else if(hour>12) hour-=12;
}

void softRTC::Convert_To_24h(uint8_t &hour, bool isPM) 
{
  if(isPM && hour<12) hour+=12; else if(!isPM && hour==12) hour=0;
}

void softRTC::setCenturybit(uint16_t year) 
{
  startclk.century = (year>=2100);
}

void softRTC::manageYear(bool select, uint16_t &Full, uint8_t &Half, bool century) 
{
  if(select)
    Full = (20+century)*100 + Half;
  else
    Half = Full % 100;
}

void softRTC::write(uint8_t day, uint8_t month, uint16_t year, uint8_t hour, uint8_t minute, uint8_t second, bool isPM, bool is12H) 
{
  if(day==0 || day>31 || month==0 || month>12 || minute>59 || second>59 || (is12H==MODE_12H ? (hour==0 || hour>12) : (hour>23))) 
  {
      ERR_INVALID_DATE();
      clkset.sync_ = 0;
      return;
  }
  startclk.day   = day;
  startclk.month = month;
  setCenturybit(year);
  uint8_t sy;
  manageYear(rtc_short_year, year, sy, startclk.century);
  startclk.year   = sy;
  startclk.minute = minute;
  startclk.second = second;
  if(is12H)
    Convert_To_24h(hour, isPM);
  startclk.hour = hour;
  clkset.millis = millis();
  clkset.sync_  = 1;
  clkset.is12H  = is12H;
}

void softRTC::read(uint8_t &day, uint8_t &month, uint16_t &year, uint8_t &hour, uint8_t &minute, uint8_t &second, bool &isPM, bool &is12H, uint8_t &week)
{
  if(!clkset.sync_){return;}
  calcTime(year, month, day, hour, minute, second);
  if(clkset.is12H){ Convert_To_12h(hour, isPM); is12H = true; }
  else is12H = false;
  week = getWeekdays(day, month, year);
}

void softRTC::calcTime(uint16_t &year, uint8_t &month, uint8_t &day,uint8_t &hour, uint8_t &minute, uint8_t &second)
{
  //if(!clkset.sync_){return;}
  const uint8_t PROGMEM dpm[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  unsigned long secs = (millis()-clkset.millis)/1000UL;
  unsigned long ts = startclk.second+secs;
  second = ts % 60;
  ts = startclk.minute + ts/60;
  minute = ts % 60;
  ts = startclk.hour + ts/60;
  hour = ts % 24;
  unsigned long daysElapsed = ts/24;
  uint8_t sy = startclk.year;
  manageYear(rtc_full_year, year, sy, startclk.century);
  month = startclk.month;
  day = startclk.day;
  while(daysElapsed){
    uint8_t d = pgm_read_byte(&dpm[month-1]);
    if(month==2 && isLeapYear(year)) d = 29;
    uint8_t rem = d - day + 1;
    if(daysElapsed < rem){ day += daysElapsed; break; }
    daysElapsed -= rem;
    day = 1;
    if(++month > 12){ month = 1; year++; }
  }
  if(month==2 && day==29 && !isLeapYear(year)){ day = 1; month = 3; }
}

bool softRTC::isLeapYear(uint16_t year) 
{
  return ((year%4==0)&&(year%100!=0)) || (year%400==0);
}

// The optimized print() writes numbers and weekday names directly without using String.
void softRTC::printsw(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, bool is12H) 
{
  bool ampm;
  // Date
  Serial.print(day); Serial.print(F("-"));
  Serial.print(month); Serial.print(F("-"));
  Serial.print(year); Serial.print(F(" "));
  // Time
  if(is12H) 
  {
    Convert_To_12h(hour, ampm);
  }
  Serial.print(leadingZero(hour));
  Serial.print(F(":"));
  Serial.print(leadingZero(minute));
  Serial.print(F(":"));
  Serial.print(leadingZero(second));
  Serial.print(F(" "));
  if(is12H) { Serial.print(ampm ? F("PM") : F("AM")); }
  else      { Serial.print(F("24H")); }
  Serial.print(F(" "));
  hour = getWeekdays(day, month, year); //week storage
  Serial.println(weekdaysName(hour));
}

String softRTC::leadingZero(uint8_t value)
{
  if((value<10) or (!value)) { return String(F("0")) + String(value); }
  return String(value);
}

String softRTC::weekdaysName(uint8_t val)
{
    switch(val)
    {
      case 1: return F("Sun");
      case 2: return F("Mon");
      case 3: return F("Tue");
      case 4: return F("Wed");
      case 5: return F("Thu");
      case 6: return F("Fri");
      case 7: return F("Sat");
    };
}

void softRTC::print() 
{
  uint8_t m, d, h, min, s;
  uint16_t y;
  if(!clkset.sync_){ERR_INVALID_DATE();return;}
  calcTime(y, m, d, h, min, s);
  printsw(y, m, d, h, min, s, clkset.is12H);
}
