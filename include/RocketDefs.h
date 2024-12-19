#ifndef ROCKET_DEFS
#define ROCKET_DEFS

#include "stdint.h"

//#define ROCKET_RECEIVER_1
//#define ROCKET_RECEIVER_2
#define ROCKET_RECEIVER_3
#define DEVICE_NAME "RocketReceiver"
#define SAMPLES_PER_SECOND 20
//#define AGL_ARRAY_SIZE 15 * 60 * SAMPLES_PER_SECOND
#define GPS_SENTENCE_TYPE_LEN 6
#define MSG_HDR_SIZE 3
#define FLIGHT_STATS_MSG_SIZE 81
#define GPS_SENTENCE_CHECKSUM_LEN 3
#define FEET_PER_METER 3.2808399
#define CENTURY 100
#define DATE_STRING_LENGTH 23
#define DELAY1_MICROS 100000
#define DELAY2_MICROS 500000
#define BOARD_LEVEL_RR915_B_01
//#define BOARD_LEVEL_RR915_B_02
#define ALTIMETER_SCALE 10
#define DEVICE_NAME_LENGTH 12
#define MAX_RESTART_MESSAGE_SIZE 255

enum DeviceState{
  kStandby = 0,
  kRunning,
  kConfig,
  kConfigSavePending,
  kTest
};

enum DeployMode : uint8_t
{
  kDroguePrimaryDrogueBackup = 0,
  kMainPrimaryMainBackup,
  kDroguePrimaryMainPrimary,
  kDrogueBackupMainBackup
};

enum __attribute__ ((packed)) FlightStates
{
  kWaitingLaunch = 0,
  kLaunched = 1,
  kBurnout = 2,
  kNoseover = 3,
  kDroguePrimaryDeployed = 4,
  kDrogueBackupDeployed = 5,
  kMainPrimaryDeployed = 6,
  kMainBackupDeployed = 7,
  kLanded = 8,
  kNoSignal
};

typedef struct{
  int16_t x;
  int16_t y;
  int16_t z;
}Accelerometer_t;

struct FlightStats {
  int launch_date;
  int launch_time;
  float max_altitude;
  int max_altitude_sample_count;
  float launch_detect_altitude;
  int launch_detect_sample_count;
  float burnout_altitude;
  int burnout_sample_count;
  float nose_over_altitude;
  int nose_over_sample_count;
  float drogue_primary_deploy_altitude;
  int drogue_primary_deploy_sample_count;
  float drogue_backup_deploy_altitude;
  int drogue_backup_deploy_sample_count;
  float main_primary_deploy_altitude;
  int main_primary_deploy_sample_count;
  float main_backup_deploy_altitude;
  int main_backup_deploy_sample_count;
  float landing_altitude;
  int landing_sample_count = 0;
  int reserved = 0; //reserved for future use, positioned for backward compatibility
  float g_range_scale = 0;
  uint16_t sample_count = 0;
  FlightStates flight_state = FlightStates::kWaitingLaunch;
  float agl_adjust = 0.0;
  uint16_t flight_data_array_index = 0;
  uint16_t test_data_sample_count = 0;
  float agl[SAMPLES_PER_SECOND] = {0.0};
  Accelerometer_t accelerometer[SAMPLES_PER_SECOND];
};

struct __attribute__ ((packed)) GPS_t {
 	char sentenceType[GPS_SENTENCE_TYPE_LEN];
  int dateStamp;
  int timeStamp;
  double latitude;
  double longitude;
  char qInd;
  uint8_t satellites;
  float hdop;
  float altitude;
  char checksum[GPS_SENTENCE_CHECKSUM_LEN];
};

struct __attribute__ ((packed)) RocketSettings {
  DeployMode deploy_mode = kDroguePrimaryDrogueBackup;
  uint16_t launch_detect_altitude = 30; // meters
  uint8_t drogue_primary_deploy_delay = 0; // tenths of a second
  uint8_t drogue_backup_deploy_delay = 20; // tenths of a second
  uint16_t main_primary_deploy_altitude = 130; // meters
  uint16_t main_backup_deploy_altitude = 100; // meters
  uint8_t deploy_signal_duration = 10; // tenths of a second
  char device_name[DEVICE_NAME_LENGTH + 1] = {0};
};

struct __attribute__ ((packed)) PreLaunch_t {
  uint8_t device_status; // Device status: locator state, altimeter, accelerometer, deployment channel 1, deployment channel 2
  uint16_t agl; // AGL
  Accelerometer_t accelerometer; // Raw accelerometer x, y, z values
  RocketSettings rocket_settings; // Locator settings
  uint16_t battery_voltage_mvolt; // Rocket locator battery level
};

struct Bearing{
  double x = 0.0;
  double y = 0.0;
  int compass_degrees = 0;
  int relative_degrees = 0;
};

#endif