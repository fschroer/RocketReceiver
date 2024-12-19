#ifndef ROCKET_GPS
#define ROCKET_GPS

#include <LoRa.h>
#include <RocketTime.hpp>
#include <TinyGPSPlus.h>
#include <RocketDefs.h>
#include <Adafruit_MMC56x3.h>

#define GPS_SENTENCE_TYPE_LEN 6
#ifdef BOARD_LEVEL_RR915_B_01
  #define GPS_TX_PIN 34
  #define GPS_RX_PIN 13
  #define I2C_SDA 27
  #define I2C_SCL 26
#endif
#ifdef BOARD_LEVEL_RR915_B_02
  #define GPS_TX_PIN 35
  #define GPS_RX_PIN 13
  #define I2C_SDA 26
  #define I2C_SCL 27
#endif

class RocketGPS{
public:
  void begin();
  void processGPSData();
  unsigned long getLastGPSFixTime();
  unsigned long getLastGPSMsgTime();
  float GetDistanceFromLatLon();
  Bearing GetBearingFromLatLon();
  float GetCompassBearingRadians();
  float GetCompassBearingDegrees();
  void GetCompassBearing(float *x, float *y);
  double getLatitude();
  char getLatitudeHemisphere();
  double getLongitude();
  char getLongitudeHemisphere();
  int getSatellites();
  void setRemoteGPSCoordinates(double latitude, double longitude);

private:
  double Deg2Rad(double deg);

  TinyGPSPlus gps;
  char gpsSentence[200] = {0};
  char gpsSentencePos = 0;
  struct tm timeInfo;
  double latitude = 0.0, longitude = 0.0;
  char latitudeHemisphere = 'N', longitudeHemisphere = 'W';
  double altitude = 0.0;
  bool gpsFix;
  unsigned long lastGPSMsgTime = millis();
  unsigned long lastGPSFixTime = millis();
  double remote_latitude_ = 0.0, remote_longitude_ = 0.0;
  Offset mag_offset;
  bool mag_begin_ok_ = false;
};
#endif