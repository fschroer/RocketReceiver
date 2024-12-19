#include <RocketGPS.hpp>
#include <math.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);

void RocketGPS::begin(){
  Serial.println("RocketGPS begin");
  gpsFix = false;
  setlocale(LC_NUMERIC, "");
  Serial1.begin(9600, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
    Serial.println("MMC5603 not detected");
  }
  else{
    mag_begin_ok_ = true;
    Serial.println("MMC5603 detected");
    mag_offset = mag.GetOffset();
    Serial.printf("X offset: %6.2f Y offset: %6.2f Z offset: %6.2f\t", mag_offset.x, mag_offset.y, mag_offset.z);
  }
}

void RocketGPS::processGPSData()
{
  if (Serial1.available()) {
    while (Serial1.available() > 0) {
      char gpsChar = Serial1.read();
//      if (gpsChar == '$') {
//        lastGPSMsgTime = millis();
//        gpsSentence[gpsSentencePos] = 0;
//        if (gpsSentencePos >= GPS_SENTENCE_TYPE_LEN && strncmp(gpsSentence, "$GPGGA", GPS_SENTENCE_TYPE_LEN) == 0 || strncmp(gpsSentence, "$GNGGA", GPS_SENTENCE_TYPE_LEN) == 0){
//        }
//        gpsSentencePos = 0;
//      }
//      gpsSentence[gpsSentencePos++] = gpsChar;
      gps.encode(gpsChar);
    }

    if (!gpsFix) {
      bool dateIsValid = false;
      bool timeIsValid = false;
      if (gps.date.isValid() && gps.date.year() > 2000) {
        printf("GPS Datestamp: %d/%d/%d %d:%d:%d %d %d %d %d\n"
          , gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()
          , gps.date.isUpdated(), gps.date.isValid(), gps.time.isUpdated(), gps.time.isValid());
        dateIsValid = true;
        timeInfo.tm_year = gps.date.year() - 1900;
        timeInfo.tm_mon = gps.date.month() - 1;
        timeInfo.tm_mday = gps.date.day();
      }
      if (gps.time.isValid() && gps.date.year() > 2000) {
        printf("GPS Datestamp: %d/%d/%d %d:%d:%d %d %d %d %d\n"
          , gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()
          , gps.date.isUpdated(), gps.date.isValid(), gps.time.isUpdated(), gps.time.isValid());
        timeIsValid = true;
        timeInfo.tm_hour = gps.time.hour();
        timeInfo.tm_min = gps.time.minute();
        timeInfo.tm_sec = gps.time.second();
      }
      if (dateIsValid && timeIsValid) {
        gpsFix = true;
        RocketTime rocketTime;
        rocketTime.setTime(&timeInfo);
      }
    }
    if (gps.location.isUpdated()) {
      latitude = abs(gps.location.lat());
      if (gps.location.lat() > 0)
        latitudeHemisphere = 'N';
      else
        latitudeHemisphere = 'S';
      longitude = abs(gps.location.lng());
      if (gps.location.lng() > 0)
        longitudeHemisphere = 'E';
      else
        longitudeHemisphere = 'W';
      lastGPSFixTime = millis();
      altitude = gps.altitude.meters();
      //Serial.printf("Altitude: %3.2f\n", altitude);
      //Serial.printf("# of satellites: %d\n", gps.satellites.value());
    }
  }
}

float RocketGPS::GetDistanceFromLatLon() {
  if (remote_latitude_ == 0 || remote_longitude_ == 0 || gps.location.lat() == 0 || gps.location.lng() == 0)
    return 0;
  int r = 6371000; // Radius of the earth in m
  float dLat = Deg2Rad(remote_latitude_ - gps.location.lat());
  float dLon = Deg2Rad(remote_longitude_ - gps.location.lng()); 
  float a = sin(dLat / 2) * sin(dLat / 2) +
    cos(Deg2Rad(gps.location.lat())) * cos(Deg2Rad(remote_latitude_)) * 
    sin(dLon / 2) * sin(dLon / 2)
    ;
  float c = 2 * atan2(sqrt(a), sqrt(1 - a)); 
  float d = r * c; // Distance in m
  return d;
}

double RocketGPS::Deg2Rad(double deg) {
  return deg * (PI / 180);
}

Bearing RocketGPS::GetBearingFromLatLon() {
  //Serial.printf("Receiver Latitude: %11.7f\r\n", gps.location.lat());
  //Serial.printf("Receiver Longitude: %11.7f\r\n", gps.location.lng());
  //Serial.printf("Rocket Latitude: %11.7f\r\n", remote_latitude_);
  //Serial.printf("Rocket Longitude: %11.7f\r\n", remote_longitude_);

  Bearing bearing;
  if (remote_latitude_ == 0 || remote_longitude_ == 0 || gps.location.lat() == 0 || gps.location.lng() == 0){
    bearing.x = 0;
    bearing.y = 0;
    return bearing;
  }
  double latitude1_rad = Deg2Rad(gps.location.lat());
  double latitude2_rad = Deg2Rad(remote_latitude_);
  double longitude1_rad = Deg2Rad(gps.location.lng());
  double longitude2_rad = Deg2Rad(remote_longitude_);
  double delta_longitude = (longitude2_rad - longitude1_rad);
  double x = cos(latitude2_rad) * sin(delta_longitude);
  double y = cos(latitude1_rad) * sin(latitude2_rad) - sin(latitude1_rad) * cos(latitude2_rad) * cos(delta_longitude);
  double normal = sqrt(x * x + y * y);
  float offset = GetCompassBearingRadians();
  bearing.x = (x * cos(offset) - y * sin(offset)) / normal;
  bearing.y = (y * cos(offset) + x * sin(offset)) / normal;
  //bearing.x = x / normal;
  //bearing.y = y / normal;
  bearing.compass_degrees = int(atan2(x, y) / PI * 180 + 360) % 360;
  bearing.relative_degrees = int((atan2(x, y) - GetCompassBearingRadians())  / PI * 180 + 360) % 360;

  return bearing;
}

float RocketGPS::GetCompassBearingRadians(){
  float x, y;
  GetCompassBearing(&x, &y);
  return atan2(y, x);
}

float RocketGPS::GetCompassBearingDegrees(){
  if (!mag_begin_ok_)
    return 0;
  float x, y;
  GetCompassBearing(&x, &y);
  float heading = (atan2(y, x) * 180) / PI;
  if (heading < 0)
    heading = 360 + heading;
  //Serial.printf("Heading: %6.2f\r\n", heading);
  // Display the results (magnetic vector values are in micro-Tesla (uT))
  /*Serial.print("X: ");
  Serial.print(x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(y);
  //Serial.print("  ");
  //Serial.print("Z: ");
  //Serial.print(event.magnetic.z);
  //Serial.print("  ");
  Serial.println("uT");*/

  // Read and display temperature
//  float temp_c = mag.readTemperature();
//  Serial.print("Temp: "); Serial.print(temp_c); Serial.println(" *C");
  return heading;
}

void RocketGPS::GetCompassBearing(float *x, float *y){
  const float x_offset = 27.5;
  const float y_offset = -6;
  //mag_offset = mag.GetOffset();
  //Serial.printf("X offset: %6.2f Y offset: %6.2f Z offset: %6.2f\t", mag_offset.x, mag_offset.y, mag_offset.z);
  mag.reset();
  sensors_event_t event;
  mag.getEvent(&event);
  //Serial.printf("X: %6.2f Y: %6.2f Z: %6.2f\t", event.magnetic.x, event.magnetic.y, event.magnetic.z);
  //Serial.printf("X: %6.2f Y: %6.2f Z: %6.2f\r\n", event.magnetic.x - mag_offset.x, event.magnetic.y - mag_offset.y, event.magnetic.z - mag_offset.z);
  *x = -(event.magnetic.y + y_offset);
  *y = event.magnetic.x + x_offset;
}

unsigned long RocketGPS::getLastGPSFixTime(){
  return lastGPSFixTime;
}

unsigned long RocketGPS::getLastGPSMsgTime(){
  return lastGPSMsgTime;
}

double RocketGPS::getLatitude(){
  return latitude;
}

char RocketGPS::getLatitudeHemisphere(){
  return latitudeHemisphere;
}

double RocketGPS::getLongitude(){
  return longitude;
}

char RocketGPS::getLongitudeHemisphere(){
  return longitudeHemisphere;
}

int RocketGPS::getSatellites(){
  return gps.satellites.value();
}

void RocketGPS::setRemoteGPSCoordinates(double latitude, double longitude){
  remote_latitude_ = latitude;
  remote_longitude_ = longitude;
}