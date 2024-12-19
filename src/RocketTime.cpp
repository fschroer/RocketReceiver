#include <RocketTime.hpp>

void RocketTime::setTime(tm *timeInfo){
  struct timeval tv;

  tv.tv_sec = mktime(timeInfo);
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);
  setenv("TZ", "PST8PDT", 1); // Set timezone to PT
  tzset();
  char currentTime[40];
  strncpy(currentTime, "<br>Set time: ", 14);
  getTime(displayDateFormat, sizeof(currentTime) - 14, currentTime + 14);
}

void RocketTime::getTime(const char *format, const int dateSize, char *rocketTime){
  time_t now;
  struct tm timeInfo;
  time(&now);
  localtime_r(&now, &timeInfo);
  strftime(rocketTime, dateSize, format, &timeInfo);
}