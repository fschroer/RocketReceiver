#ifndef ROCKETCONFIG
#define ROCKETCONFIG

//#include <RocketFile.hpp>
#include <RocketDefs.h>
#include <LoRa.h>
#include <Preferences.h>
#include "stdint.h"
#include "string.h"
#include "time.h"
#include "math.h"


#define UART_LINE_MAX_LENGTH 255
#define USER_INPUT_MAX_LENGTH 15
#define DATE_STRING_LENGTH 23
#define ALTIMETER_STRING_LENGTH 7
#define ACCELEROMETER_STRING_LENGTH 9

#define MAX_LAUNCH_DETECT_ALTITUDE 50
#define MAX_DROGUE_PRIMARY_DEPLOY_DELAY 20
#define MAX_DROGUE_BACKUP_DEPLOY_DELAY 40
#define MAX_MAIN_PRIMARY_DEPLOY_ALTITUDE 400
#define MAX_MAIN_BACKUP_DEPLOY_ALTITUDE 400
#define MAX_DEPLOY_SIGNAL_DURATION 20

enum UserInteractionState
{
  kWaitingForCommand = 0,
  kConfigHome,
  kEditDeployMode,
  kEditLaunchDetectAltitude,
  kEditDroguePrimaryDeployDelay,
  kEditDrogueBackupDeployDelay,
  kEditMainPrimaryDeployAltitude,
  kEditMainBackupDeployAltitude,
  kEditDeploySignalDuration,
  kEditLoraChannel,
  kEditDeviceName,
  kDataHome,
  kTestHome,
  kTestDeploy1,
  kTestDeploy2,
  kDfuHome
};

enum UserCommand
{
  kNone = 0,
  kArm,
  kDisarm,
  kUpdateConfig,
  kTestDeployment1,
  kTestDeployment2,
  kCancelTestDeployment
};

class RocketConfig{
public:
  RocketConfig();
  void ProcessChar(PreLaunch_t pre_launch_data, UserCommand *user_command);
  void SetLoraChannel(int channel);
  void ResetDeviceState();
  void ResetUserInteractionState();
  uint8_t ReadLoraChannel();
  uint8_t LoraChannel();
  int LoraFrequency();

private:
  RocketSettings *rocket_settings_;
  FlightStats flight_stats_;
  DeviceState locator_device_state_;
  UserInteractionState user_interaction_state_ = kWaitingForCommand;
  Preferences preferences;

  char* user_input_ = new char[USER_INPUT_MAX_LENGTH + 1];
  const char* clear_screen_ = "\x1b[.J\r\0";
  const char* config_command_ = "config\0";
  const char* test_command_ = "test\0";
  const char* arm_command_ = "arm\0";
  const char* disarm_command_ = "disarm\0";
  const char* channel_command_ = "channel\0";
  const char* restart_command_ = "restart\0";
  const char* crlf_ = "\r\n\0";
  const char* cr_ = "\r\0";
  const char* bs_ = "\b \b\0";
  const char* config_menu_intro_ = "Rocket Locator Configuration\r\n\0";
  const char* config_save_text_ = "Saved Configuration\r\n\r\n\0";
  const char* cancel_text_ = "Cancelled\r\n\r\n\0";
  const char* deploy_mode_text_ = "1) Deploy Mode:\t\t\t\t\0";
  const char* drogue_primary_drogue_backup_text_ = "Drogue Primary, Drogue Backup\0";
  const char* main_primary_main_backup_text_ = "Main Primary, Main Backup    \0";
  const char* drogue_primary_main_primary_text_ = "Drogue Primary, Main Primary \0";
  const char* drogue_backup_main_backup_text_ = "Drogue Backup, Main Backup   \0";
  const char* launch_detect_altitude_text_ = "2) Launch Detect Altitude (m):\t\t\0";
  const char* drogue_primary_deploy_delay_text_ = "3) Drogue Primary Deploy Delay (s):\t\0";
  const char* drogue_backup_deploy_delay_text_ = "4) Drogue Backup Deploy Delay (s):\t\0";
  const char* main_primary_deploy_altitude_text_ = "5) Main Primary Deploy Altitude (m):\t\0";
  const char* main_backup_deploy_altitude_text_ = "6) Main Backup Deploy Altitude (m):\t\0";
  const char* deploy_signal_duration_text_ = "7) Deploy Signal Duration (s):\t\t\0";
  const char* device_name_text_ = "9) Device Name:\t\t\t\t\0";
  const char* num_edit_guidance_text_ = "[ = down, ] = up. Hit Enter to update, Esc to cancel.\r\n\0";
  const char* text_edit_guidance_text_ = "Type text. Hit Enter to update, Esc to cancel.\r\n\0";
  const char* deploy_mode_edit_text_ = "Edit Deploy Mode\r\n\0";
  const char* launch_detect_altitude_edit_text_ = "Edit Launch Detect Altitude (m):\r\n\0";
  const char* drogue_primary_deploy_delay_edit_text_ = "Edit Drogue Primary Deploy Delay (s):\r\n\0";
  const char* drogue_backup_deploy_delay_edit_text_ = "Edit Drogue Backup Deploy Delay (s):\r\n\0";
  const char* main_primary_deploy_altitude_edit_text_ = "Edit Main Primary Deploy Altitude (m):\r\n\0";
  const char* main_backup_deploy_altitude_edit_text_ = "Edit Main Backup Deploy Altitude (m):\r\n\0";
  const char* deploy_signal_duration_edit_text_ = "Edit Deploy Signal Duration (s):\r\n\0";

  const char* test_menu_intro_ = "Rocket Locator Test Menu\r\n\r\n\0";
  const char* test_deploy1_text_ = "1) Test Deployment Channel 1\r\n\0";
  const char* test_deploy2_text_ = "2) Test Deployment Channel 2\r\n\0";
  const char* test_exit_text_ = "Exiting Data Menu\r\n\r\n\0";
  const char* test_guidance_text_ = "\r\nSelect an option and deployment test will fire in 10 seconds\r\n";
  const char* test_complete_text_ = "Test complete, exiting test mode.\r\n\r\n\0";

  DeployMode deploy_mode_;
  int launch_detect_altitude_;
  int drogue_primary_deploy_delay_;
  int drogue_backup_deploy_delay_;
  int main_primary_deploy_altitude_;
  int main_backup_deploy_altitude_;
  int deploy_signal_duration_;
  int remote_lora_channel_;
  int test_deploy_count_;
  char device_name_[DEVICE_NAME_LENGTH + 1];

  uint8_t lora_channel_ = 0;

  void DisplayConfigSettingsMenu();
  const char* DeployModeString(DeployMode deploy_mode_value);
  void AdjustConfigNumericSetting(uint8_t uart_char, int *config_mode_setting, int max_setting_value, bool tenths);
  void AdjustConfigTextSetting(char uart_char, char *config_mode_setting);
  void DisplayTestMenu();
};

#endif /* ROCKETCONFIG */