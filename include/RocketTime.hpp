#ifndef ROCKET_TIME
#define ROCKET_TIME

#include <time.h>
#include <WiFi.h>

const char displayDateFormat[] = "%m/%d/%Y %H:%M:%S";

class RocketTime{
public:
  void setTime(tm *timeInfo);
  void getTime(const char *format, const int dateSize, char *rocketTime);
};
#endif