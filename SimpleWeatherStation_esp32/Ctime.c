//--------------------------------------------------------------
// 日時 <-> 1970/1/1 00:00:00からの秒数(UNIXの日付) 相互変換
//----------------------------------------------------------------
#include "Arduino.h"
#include "Ctime.h"
#include <time.h>

#define SECONDS_IN_DAY   86400   // 24 * 60 * 60
#define SECONDS_IN_HOUR  3600    // 60 * 60

//typedef unsigned char byte;
typedef bool boolean;
//#define true 1
//#define false 0

//----------------------------------------------------------------
// 西暦０年１月１日からの日数を返す
//----------------------------------------------------------------
unsigned long calc_0_days(int year, byte month, byte day) {
  unsigned long days;
  int daysinmonth_ruiseki[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  
  year--; // 当年は含まず 
  days = (unsigned long)year * 365;
  days += year / 4;  // 閏年の日数を足しこむ
  days -= year/100;  // 閏年で無い日数を差し引く
  days += year/400;  // 差し引きすぎた日数を足しこむ
  days += (unsigned long)daysinmonth_ruiseki[month-1];
  if( is_leapyear( year ) &&  3 <= month ) {
    day++;
  }
  days += (unsigned long)day;
  
  return days;
}

//----------------------------------------------------------------
// 西暦１９７０年１月１日からの日数を返す
//----------------------------------------------------------------
unsigned long calc_unix_days(int year, byte month, byte day) {
  unsigned long days;

  return calc_0_days(year, month, day) - calc_0_days(1970, 1, 1);
}

//----------------------------------------------------------------
// 西暦１９７０年１月１日　００：００：００　からの秒数を返す
//----------------------------------------------------------------
unsigned long calc_unix_seconds(int year, byte month, byte day, byte hour, byte minutes, byte second) {
  unsigned long days;
  unsigned long seconds;

  days = calc_unix_days(year, month, day);
  seconds = days * SECONDS_IN_DAY;
  seconds += (unsigned long)hour * SECONDS_IN_HOUR;
  seconds += (unsigned long)minutes * 60;
  seconds += (unsigned long)second;
  seconds += -9 * SECONDS_IN_HOUR; // JPN(GMT+9) 日本時間

  return seconds;
}

//----------------------------------------------------------------
// 閏年か否かを返す
//----------------------------------------------------------------
boolean is_leapyear( int year )
{
  if( (year % 400) == 0 || ((year % 4) == 0 && (year % 100) != 0)) {
    return true;
  } else {
    return false;
  }
}

//----------------------------------------------------------------
// Unixの日付を日時に変換　ループ処理しているので2msec程度かかる
//----------------------------------------------------------------
struct tm unix_time_to_date( unsigned long unix_datetime)
{
  struct tm ctm;
  memset( &ctm , 0x00, sizeof(struct tm));
  ctm.tm_year  = 1970;
  ctm.tm_mon = 1;
  ctm.tm_yday   = 1;
  ctm.tm_hour  = 0;
  ctm.tm_min   = 0;
  ctm.tm_sec   = 0;
  
  unsigned long yearindate;
  unsigned long seconds_year;
  unsigned long seconds_month;
  byte daysinmonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

  unix_datetime += 9 * SECONDS_IN_HOUR;
  while( unix_datetime > 0 ) {
    if( is_leapyear(ctm.tm_year) ) {
      yearindate = 366;
      daysinmonth[1] = 29;
    } else {
      yearindate = 365;
      daysinmonth[1] = 28;
    }
    seconds_year  = yearindate * SECONDS_IN_DAY;
    seconds_month = daysinmonth[ctm.tm_mon - 1] * SECONDS_IN_DAY;
    if( unix_datetime >= seconds_year ) {
      ctm.tm_year++;
      unix_datetime -= seconds_year;
    } else if( unix_datetime >= seconds_month ) {
      ctm.tm_mon++;
      unix_datetime -= seconds_month;
    } else if( unix_datetime >= SECONDS_IN_DAY ) {
      ctm.tm_yday++;
      unix_datetime -= SECONDS_IN_DAY;  
    } else if( unix_datetime >= SECONDS_IN_HOUR ) {
      ctm.tm_hour++;
      unix_datetime -= SECONDS_IN_HOUR; 
    } else if( unix_datetime >= 60 ) {
      ctm.tm_min++;
      unix_datetime -= 60;
    } else {
      ctm.tm_sec = (byte)unix_datetime;
      unix_datetime= 0;
    }
  }
  return ctm;
}
