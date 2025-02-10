#ifndef MAIN
#define MAIN

#include "stdint.h"
#include "time.h"
#include <Preferences.h>
#include <RocketDefs.h>
#include <Adafruit_MMC56x3.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include "BluetoothSerial.h"
#include <RocketConfig.hpp>

#ifdef ROCKET_RECEIVER_1
  #define DEVICE_NUMBER 1
#endif
#ifdef ROCKET_RECEIVER_2
  #define DEVICE_NUMBER 2
#endif
#ifdef ROCKET_RECEIVER_3
  #define DEVICE_NUMBER 3
#endif

#define BT_BUFFER_SIZE 10 + sizeof(RocketSettings)
#define FLASH_PAGE_SIZE 2048
#define FLIGHT_PROFILE_BUFFER_SIZE FLASH_PAGE_SIZE * 6
#define MESSAGE_SEQUENCE_SIZE 2
#define MESSAGE_FOOTER_SIZE 1

enum DisplayState{
  kUnarmed = 0,
  kArmed,
  kNoStatus,
  kRestart
};

FlightStats flight_stats_;
GPS_t gps_data_;
PreLaunch_t pre_launch_data_;
char gps_sentence_[256] = {'$','G','P','G','G','A'};
float groundLevel = 0.0;
double dLatitude, prevLatitude = 0.0;
double dLongitude, prevLongitude = 0.0;
tm sample_time_, prev_sample_time_;
char s_sample_time_[DATE_STRING_LENGTH] = {0};
bool tourActionsTaken = false;
unsigned long int clock_start_ = millis();
bool start_rocket_locator_ = false;
bool stop_rocket_locator_ = false;
uint16_t sample_count_ = 0;
DisplayState display_state_ = DisplayState::kNoStatus;
TaskHandle_t display_task_handle_;
UserCommand user_command_ = UserCommand::kNone;
char restart_message_[MAX_RESTART_MESSAGE_SIZE] = {0};
int last_message_time;
char config_data[sizeof(RocketSettings)];
int locator_archive_position_ = 0;
const char arm_cmd[] = {'R', 'u', 'n'};
const char disarm_cmd[] = {'S', 't', 'o', 'p'};
const char channel_cmd[] = {'C', 'H', 'N'};
const char config_cmd[] = {'C', 'F', 'G'};
const char flight_profile_metadata_cmd[] = {'F', 'P', 'M'};
const char flight_profile_data_cmd[] = {'F', 'P', 'D'};
const char test_cmd[] = {'T', 'S', 'T'};

void resetBTBuffer();
void dump_message_hex(char *message, int message_size, int header_size, int body_size, int body_line_break);
void decodePreLaunchMsg(char *loraMsg, int packetSize);
void decodeTelemetryMsg(char *loraMsg, int packetSize);
void updateLocatorGPSData(char *loraMsg, int packetSize);
void MakeDateTime(char *target, int date, int time);
void writeGpxTrackpoint();
void writeTour();
void writeTourLookAt();
int LoraFrequency();
void display_service(void* pvParameters);

#endif