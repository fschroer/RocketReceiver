#include <RocketConfig.hpp>

RocketConfig::RocketConfig(){
  //HAL_UART_Transmit(huart2_, (uint8_t*)&crlf_, 2, UART_TIMEOUT);
}

void RocketConfig::ProcessChar(PreLaunch_t pre_launch_data, UserCommand *user_command){
  static int char_pos = 0;
  char uart_char;
  while(Serial.available()){
    Serial.readBytes(&uart_char, 1);
    //Serial.print(uart_char);
    switch (user_interaction_state_){
    case UserInteractionState::kWaitingForCommand:
      if (uart_char > ' ' && uart_char <= '~' && char_pos < USER_INPUT_MAX_LENGTH){
        Serial.print(uart_char);
        user_input_[char_pos++] = uart_char;
      }
      else switch (uart_char){
      case 13: // Enter key
        Serial.println();
        while(Serial.available())
          Serial.read();
        user_input_[char_pos] = 0;
        if (!strcmp(user_input_, arm_command_)){
          *user_command = UserCommand::kArm;
        }
        else if (!strcmp(user_input_, disarm_command_)){
          *user_command = UserCommand::kDisarm;
        }
        else if (!strcmp(user_input_, restart_command_))
          ESP.restart();
        else if (user_input_[0] >= '0' && user_input_[0] <= '9' && (user_input_[1] == 0 || user_input_[1] >= '0' && user_input_[1] <= '9')){
          SetLoraChannel(atoi(user_input_));
        }
        else if ((pre_launch_data.device_status & 0xF0) >> 4 == DeviceState::kStandby){
          //Serial.println("Device State = Standby");
          //Serial.printf("char_pos = %d\r\n", char_pos);
          if (!strcmp(user_input_, config_command_)){
            locator_device_state_ = DeviceState::kConfig;
            user_interaction_state_ = UserInteractionState::kConfigHome;
            deploy_mode_ = pre_launch_data.rocket_settings.deploy_mode;
            launch_detect_altitude_ = pre_launch_data.rocket_settings.launch_detect_altitude;
            drogue_primary_deploy_delay_ = pre_launch_data.rocket_settings.drogue_primary_deploy_delay;
            drogue_backup_deploy_delay_ = pre_launch_data.rocket_settings.drogue_backup_deploy_delay;
            main_primary_deploy_altitude_ = pre_launch_data.rocket_settings.main_primary_deploy_altitude;
            main_backup_deploy_altitude_ = pre_launch_data.rocket_settings.main_backup_deploy_altitude;
            deploy_signal_duration_ = pre_launch_data.rocket_settings.deploy_signal_duration;
            for (int i = 0; i < DEVICE_NAME_LENGTH + 1; i++)
              device_name_[i] = pre_launch_data.rocket_settings.device_name[i];
            DisplayConfigSettingsMenu();
          }
          else if (!strcmp(user_input_, test_command_)){
            locator_device_state_ = DeviceState::kConfig;
            user_interaction_state_ = UserInteractionState::kTestHome;
            DisplayTestMenu();
          }
        }
        char_pos = 0;
        user_input_[0] = 0;
        break;
      case 8: // Backspace
        Serial.print(bs_);
        user_input_[--char_pos] = 0;
        break;
      }
      break;
    case UserInteractionState::kConfigHome:
      //Serial.print(" ");
      //Serial.print((uint8_t)uart_char);
      //Serial.print(" ");
      switch (uart_char){
      case 13: // Enter key
        pre_launch_data.rocket_settings.deploy_mode = deploy_mode_;
        pre_launch_data.rocket_settings.launch_detect_altitude = launch_detect_altitude_;
        pre_launch_data.rocket_settings.drogue_primary_deploy_delay = drogue_primary_deploy_delay_;
        pre_launch_data.rocket_settings.drogue_backup_deploy_delay = drogue_backup_deploy_delay_;
        pre_launch_data.rocket_settings.main_primary_deploy_altitude = main_primary_deploy_altitude_;
        pre_launch_data.rocket_settings.main_backup_deploy_altitude = main_backup_deploy_altitude_;
        pre_launch_data.rocket_settings.deploy_signal_duration = deploy_signal_duration_;
        for (int i = 0; i < DEVICE_NAME_LENGTH + 1; i++)
          pre_launch_data.rocket_settings.device_name[i] = device_name_[i];
        Serial.println("CFG");
        for (int i = 0; i < sizeof(pre_launch_data.rocket_settings); i++)
          Serial.printf("%02X ", ((uint8_t*)(&pre_launch_data.rocket_settings))[i]);
        LoRa.beginPacket();
        LoRa.print("CFG");
        LoRa.write((uint8_t*)&pre_launch_data.rocket_settings, sizeof(pre_launch_data.rocket_settings));
        LoRa.println();
        LoRa.endPacket();
        locator_device_state_ = DeviceState::kConfigSavePending;
        user_interaction_state_ = UserInteractionState::kWaitingForCommand;
        Serial.print(config_save_text_);
        break;
      case 27: // Esc key
        locator_device_state_ = DeviceState::kStandby;
        user_interaction_state_ = UserInteractionState::kWaitingForCommand;
        Serial.print(cancel_text_);
        break;
      case 49: // 1 = Edit deploy mode
        user_interaction_state_ = UserInteractionState::kEditDeployMode;
        Serial.println();
        //Serial.print(deploy_mode_edit_text_);
        //Serial.print(num_edit_guidance_text_);
        //Serial.print(DeployModeString(deploy_mode_));
        Serial.printf("%s%s%s", deploy_mode_edit_text_, num_edit_guidance_text_, DeployModeString(deploy_mode_));
        break;
      case 50: // 2 = Edit launch detect altitude
        user_interaction_state_ = UserInteractionState::kEditLaunchDetectAltitude;
        Serial.printf("%s%s%d", launch_detect_altitude_edit_text_, num_edit_guidance_text_, launch_detect_altitude_);
        break;
      case 51: // 3 = Edit drogue primary deploy delay
        user_interaction_state_ = UserInteractionState::kEditDroguePrimaryDeployDelay;
        Serial.printf("%s%s%2.1f", drogue_primary_deploy_delay_edit_text_, num_edit_guidance_text_, (float)drogue_primary_deploy_delay_ / 10);
        break;
      case 52: // 4 = Edit drogue backup deploy delay
        user_interaction_state_ = UserInteractionState::kEditDrogueBackupDeployDelay;
        Serial.printf("%s%s%2.1f", drogue_backup_deploy_delay_edit_text_, num_edit_guidance_text_, (float)drogue_backup_deploy_delay_ / 10);
        break;
      case 53: // 5 = Edit main primary deploy altitude
        user_interaction_state_ = UserInteractionState::kEditMainPrimaryDeployAltitude;
        Serial.printf("%s%s%d", main_primary_deploy_altitude_edit_text_, num_edit_guidance_text_, main_primary_deploy_altitude_);
        break;
      case 54: // 6 = Edit main backup deploy altitude
        user_interaction_state_ = UserInteractionState::kEditMainBackupDeployAltitude;
        Serial.printf("%s%s%d", main_backup_deploy_altitude_edit_text_, num_edit_guidance_text_, main_backup_deploy_altitude_);
        break;
      case 55: // 7 = Edit deploy signal duration
        user_interaction_state_ = UserInteractionState::kEditDeploySignalDuration;
        Serial.printf("%s%s%2.1f", deploy_signal_duration_edit_text_, num_edit_guidance_text_, (float)deploy_signal_duration_ / 10);
        break;
      case 57: // 9 = Edit device name
        user_interaction_state_ = UserInteractionState::kEditDeviceName;
        Serial.print(text_edit_guidance_text_);
        break;
      }
      break;
    case UserInteractionState::kEditDeployMode:
      switch (uart_char){
      case 13: // Enter key
        user_interaction_state_ = UserInteractionState::kConfigHome;
        DisplayConfigSettingsMenu();
        break;
      case 27: // Esc key
        user_interaction_state_ = UserInteractionState::kConfigHome;
        DisplayConfigSettingsMenu();
        break;
      case 91: // [ = decrease value
        switch (deploy_mode_){
        case DeployMode::kDroguePrimaryDrogueBackup:
          deploy_mode_ = DeployMode::kDrogueBackupMainBackup;
          break;
        case DeployMode::kMainPrimaryMainBackup:
          deploy_mode_ = DeployMode::kDroguePrimaryDrogueBackup;
          break;
        case DeployMode::kDroguePrimaryMainPrimary:
          deploy_mode_ = DeployMode::kMainPrimaryMainBackup;
          break;
        case DeployMode::kDrogueBackupMainBackup:
          deploy_mode_ = DeployMode::kDroguePrimaryMainPrimary;
          break;
        }
        Serial.print(cr_);
        Serial.print(DeployModeString(deploy_mode_));
        break;
      case 93: // [ = increase value
        switch (deploy_mode_){
        case DeployMode::kDroguePrimaryDrogueBackup:
            deploy_mode_ = DeployMode::kMainPrimaryMainBackup;
            break;
        case DeployMode::kMainPrimaryMainBackup:
            deploy_mode_ = DeployMode::kDroguePrimaryMainPrimary;
            break;
        case DeployMode::kDroguePrimaryMainPrimary:
            deploy_mode_ = DeployMode::kDrogueBackupMainBackup;
            break;
        case DeployMode::kDrogueBackupMainBackup:
            deploy_mode_ = DeployMode::kDroguePrimaryDrogueBackup;
            break;
        }
        Serial.print(cr_);
        Serial.print(DeployModeString(deploy_mode_));
        break;
      }
      break;
    case UserInteractionState::kEditLaunchDetectAltitude:
      AdjustConfigNumericSetting(uart_char, &launch_detect_altitude_, MAX_LAUNCH_DETECT_ALTITUDE, false);
      break;
    case UserInteractionState::kEditDroguePrimaryDeployDelay:
      AdjustConfigNumericSetting(uart_char, &drogue_primary_deploy_delay_, MAX_DROGUE_PRIMARY_DEPLOY_DELAY, true);
      break;
    case UserInteractionState::kEditDrogueBackupDeployDelay:
      AdjustConfigNumericSetting(uart_char, &drogue_backup_deploy_delay_, MAX_DROGUE_BACKUP_DEPLOY_DELAY, true);
      break;
    case UserInteractionState::kEditMainPrimaryDeployAltitude:
      AdjustConfigNumericSetting(uart_char, &main_primary_deploy_altitude_, MAX_MAIN_PRIMARY_DEPLOY_ALTITUDE, false);
      break;
    case UserInteractionState::kEditMainBackupDeployAltitude:
      AdjustConfigNumericSetting(uart_char, &main_backup_deploy_altitude_, MAX_MAIN_BACKUP_DEPLOY_ALTITUDE, false);
      break;
    case UserInteractionState::kEditDeploySignalDuration:
      AdjustConfigNumericSetting(uart_char, &deploy_signal_duration_, MAX_DEPLOY_SIGNAL_DURATION, true);
      break;
    case UserInteractionState::kEditDeviceName:
      AdjustConfigTextSetting(uart_char, device_name_);
      break;
    case UserInteractionState::kTestHome:
      if (uart_char == '1'){
        locator_device_state_ = DeviceState::kTest;
        user_interaction_state_ = UserInteractionState::kTestDeploy1;
        *user_command = UserCommand::kTestDeployment1;
      }
      else if (uart_char == '2'){
        locator_device_state_ = DeviceState::kTest;
        user_interaction_state_ = UserInteractionState::kTestDeploy2;
        *user_command = UserCommand::kTestDeployment2;
      }
      else if (uart_char == 27){ // Esc key
        locator_device_state_ = DeviceState::kStandby;
        user_interaction_state_ = UserInteractionState::kWaitingForCommand;
        Serial.print(cancel_text_);
      }
      break;
    case UserInteractionState::kTestDeploy1:
    case UserInteractionState::kTestDeploy2:
      if (uart_char == 27){ // Esc key
        locator_device_state_ = DeviceState::kStandby;
        user_interaction_state_ = UserInteractionState::kWaitingForCommand;
        *user_command = UserCommand::kCancelTestDeployment;
        Serial.print(cancel_text_);
      }
      break;
    }
    //Serial.println("End");
  }
}

void RocketConfig::DisplayConfigSettingsMenu(){
  Serial.printf("%s%s\r\n", clear_screen_, config_menu_intro_);
  Serial.printf("%s%s\r\n", deploy_mode_text_, DeployModeString(deploy_mode_));
  Serial.printf("%s%d\r\n", launch_detect_altitude_text_, launch_detect_altitude_);
  Serial.printf("%s%2.1f\r\n", drogue_primary_deploy_delay_text_, (float)drogue_primary_deploy_delay_ / 10);
  Serial.printf("%s%2.1f\r\n", drogue_backup_deploy_delay_text_, (float)drogue_backup_deploy_delay_ / 10);
  Serial.printf("%s%d\r\n", main_primary_deploy_altitude_text_, main_primary_deploy_altitude_);
  Serial.printf("%s%d\r\n", main_backup_deploy_altitude_text_, main_backup_deploy_altitude_);
  Serial.printf("%s%2.1f\r\n", deploy_signal_duration_text_, (float)deploy_signal_duration_ / 10);
  Serial.printf("%s%s\r\n", device_name_text_, device_name_);
  Serial.println();
}

void RocketConfig::DisplayTestMenu(){
  Serial.printf("%s%s", clear_screen_, test_menu_intro_);
  Serial.printf("%s", test_deploy1_text_);
  Serial.printf("%s", test_deploy2_text_);
  Serial.printf("%s", test_guidance_text_);
}

const char* RocketConfig::DeployModeString(DeployMode deploy_mode_value){
  switch (deploy_mode_value){
  case DeployMode::kDroguePrimaryDrogueBackup:
    return drogue_primary_drogue_backup_text_;
    break;
  case DeployMode::kMainPrimaryMainBackup:
    return main_primary_main_backup_text_;
    break;
  case DeployMode::kDroguePrimaryMainPrimary:
    return drogue_primary_main_primary_text_;
    break;
  case DeployMode::kDrogueBackupMainBackup:
    return drogue_backup_main_backup_text_;
    break;
  }
  return "\0";
}

void RocketConfig::AdjustConfigNumericSetting(uint8_t uart_char, int *config_mode_setting, int max_setting_value, bool tenths){
  switch (uart_char){
  case 13: // Enter key
    user_interaction_state_ = UserInteractionState::kConfigHome;
    DisplayConfigSettingsMenu();
    break;
  case 27: // Esc key
    user_interaction_state_ = UserInteractionState::kConfigHome;
    DisplayConfigSettingsMenu();
    break;
  case 91: // [ = decrease value
    if (*config_mode_setting > 0)
        (*config_mode_setting)--;
    break;
  case 93: // [ = increase value
    if (*config_mode_setting < max_setting_value)
        (*config_mode_setting)++;
    break;
  }
  if (uart_char == 91 || uart_char == 93){
    if (tenths)
      Serial.printf("%s%2.1f", cr_, (float)*config_mode_setting / 10);
    else
      Serial.printf("%s%d", cr_, *config_mode_setting);
  }
}

void RocketConfig::AdjustConfigTextSetting(char uart_char, char *config_mode_setting){
  static int char_pos = 0;
  if (uart_char == 13 || uart_char == 27){
    if (uart_char == 13){
      int i = 0;
      for (; i < char_pos; i++)
        config_mode_setting[i] = user_input_[i];
      for (; i < DEVICE_NAME_LENGTH + 1; i++)
        config_mode_setting[i] = 0;
    }
    char_pos = 0;
    for (int i = 0; i < DEVICE_NAME_LENGTH + 1; i++)
      user_input_[i] = 0;
    user_interaction_state_ = UserInteractionState::kConfigHome;
    DisplayConfigSettingsMenu();
  }
  else if (uart_char == 8){
    Serial.print(bs_);
    user_input_[--char_pos] = 0;
  }
  else if (uart_char >= ' ' && uart_char <= '~' && char_pos < DEVICE_NAME_LENGTH){
      Serial.print(uart_char);
      user_input_[char_pos++] = uart_char;
    }
}

void RocketConfig::SetLoraChannel(int channel) {
  if (channel >= 0 && channel < 64){
    lora_channel_ = channel;
    preferences.begin(DEVICE_NAME, false);
    preferences.putChar(DEVICE_NAME, lora_channel_);
    preferences.end();
    LoRa.end();
    if (!LoRa.begin(LoraFrequency())) {
      Serial.println("Starting LoRa failed!");
      //while (1);
    }
    Serial.println("Starting LoRa success!");
    Serial.printf("Setting channel = %d\r\n", lora_channel_);
  }
}

void RocketConfig::ResetDeviceState(){
  locator_device_state_ = DeviceState::kStandby;
}

void RocketConfig::ResetUserInteractionState(){
  user_interaction_state_ = UserInteractionState::kWaitingForCommand;
}

uint8_t RocketConfig::ReadLoraChannel(){
  preferences.begin(DEVICE_NAME, false);
  lora_channel_ = preferences.getChar(DEVICE_NAME, 0);
  Serial.printf("Getting channel = %d, frequency = %d\r\n", lora_channel_, LoraFrequency());
  preferences.end();
  return lora_channel_;
}

uint8_t RocketConfig::LoraChannel(){
  return lora_channel_;
}

int RocketConfig::LoraFrequency(){
  return 902300000 + lora_channel_ * 200000;
}