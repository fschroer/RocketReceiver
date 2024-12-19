#ifndef EINK_DISPLAY
#define EINK_DISPLAY

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
//#include <Fonts/FreeSans9pt7b.h>
//#include <Fonts/Org_01.h>
#include "Bitmaps200x200.h" // 1.54" b/w
#include <RocketDefs.h>

#define BATT_LEVEL_PIN 35
#define RECEIVER_MIN_BATTERY_LEVEL 2000
#ifdef ROCKET_RECEIVER_1
  #define RECEIVER_MAX_BATTERY_LEVEL 2500
#endif
#ifdef ROCKET_RECEIVER_2
  #define RECEIVER_MAX_BATTERY_LEVEL 2400
#endif
#ifdef ROCKET_RECEIVER_3
  #define RECEIVER_MAX_BATTERY_LEVEL 2360
#endif
#define LOCATOR_MIN_BATTERY_LEVEL 3300
#define LOCATOR_MAX_BATTERY_LEVEL 4200
#ifdef BOARD_LEVEL_RR915_B_01
  #define CLK 21
  #define SDO -1
  #define SDI 32
  #define CS 17
  #define DC 33
  #define RST -1
  #define BUSY 22
#endif
#ifdef BOARD_LEVEL_RR915_B_02
  #define CLK 22
  #define SDO -1
  #define SDI 32
  #define CS 21
  #define DC 33
  #define RST -1
  #define BUSY 36
#endif

class EinkDisplay{
public:
  void Begin();
  void DisplayUnarmedStatus(int remote_satellites, int receiver_satellites, uint8_t lora_channel, PreLaunch_t pre_launch_data_, char* s_sample_time, char* restart_message);
  void DisplayArmedStatus(FlightStates flight_state, int sample_count, float agl, float distance, Bearing bearing, float compass_bearing
    , double remote_latitude, double remote_longitude, int remote_satellites, int receiver_satellites, char* s_sample_time, uint8_t lora_channel);
  void DisplayNoStatus(int receiver_satellites, uint8_t lora_channel);
  void DisplayRestartMessage(char* message);

private:
  void SetDisplayWindow();
  void DisplayCompass(Bearing bearing);
  void DisplayDeviceName(char* device_name);
  void DisplayVersion(char* restart_message);
  void DisplayChannel(uint8_t lora_channel);
  void DisplayDistanceToRocket(float distance);
  void DisplayReceiverGPSStatus(int receiver_satellites, uint8_t display_x, uint8_t display_y);
  void DisplayLocatorGPSStatus(int remote_satellites, uint8_t display_x, uint8_t display_y);
  void DisplayBatteryLevel(uint16_t battery_level, uint16_t min_battery_level, uint16_t max_battery_level, uint8_t battery_x, uint8_t battery_y);
  void DisplayFlightData(FlightStates flight_state, int sample_count, float agl, uint8_t lora_channel);
  void DisplayGPSCoordinates(double latitude, double longitude);
  void DisplayTimeStamp(char* s_sample_time);
  void DisplayStartupData(PreLaunch_t pre_launch_data);
};
#endif