#include "EinkDisplay.hpp"

#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DRIVER_CLASS GxEPD2_154_D67 // GDEH0154D67 200x200, SSD1681, (HINK-E154A07-A1)

#define MAX_DISPLAY_BUFFER_SIZE 65536ul // e.g.
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS(/*CS=5*/ CS, /*DC=*/ DC, /*RST=*/ RST, /*BUSY=*/ BUSY)); // RocketReceiver915
#undef MAX_DISPLAY_BUFFER_SIZE
#undef MAX_HEIGHT

SPIClass hspi(HSPI);

const char* flight_state_text[] = {"Waiting for Launch\0", "Launched\0", "Burnout\0", "Noseover\0", "Drogue Primary\0"
  , "Drogue Backup\0", "Main Primary\0", "Main Backup\0", "Landed\0", "No Signal\0"};

void EinkDisplay::Begin(){
  const char* title = "Rocket Receiver";
  hspi.begin(CLK, SDO, SDI, CS); // remap hspi for EPD (swap pins)
  display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  display.init(115200); // default 10ms reset pulse, e.g. for bare panels with DESPI-C02
  display.setRotation(3);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.getTextBounds(title, 0, 0, &tbx, &tby, &tbw, &tbh);
    //Serial.printf("Text Bounds: %d %d %d %d", tbx, tby, tbw, tbh);
    uint16_t x = ((display.width() - tbw) / 2) - tbx;
    display.setCursor(x, 12);
    display.println(title);
    display.drawInvertedBitmap(68, 68, rocketFireMid, 64, 64, GxEPD_BLACK);
    display.setCursor(0, 196);
    display.println("2024 Frank Schroer");
  }
  while (display.nextPage());
  display.powerOff();
  delay(2000);
  //display.end();
}

void EinkDisplay::DisplayUnarmedStatus(int remote_satellites, int receiver_satellites, uint8_t lora_channel, PreLaunch_t pre_launch_data, char* s_sample_time, char* restart_message){
  SetDisplayWindow();
  display.firstPage();
  do{
    display.fillScreen(GxEPD_WHITE);
    DisplayReceiverGPSStatus(receiver_satellites, 140, 0);
    DisplayBatteryLevel(analogRead(BATT_LEVEL_PIN), RECEIVER_MIN_BATTERY_LEVEL, RECEIVER_MAX_BATTERY_LEVEL, 184, 0);
    DisplayLocatorGPSStatus(remote_satellites, 140, 28);
    DisplayBatteryLevel(pre_launch_data.battery_voltage_mvolt, LOCATOR_MIN_BATTERY_LEVEL, LOCATOR_MAX_BATTERY_LEVEL, 184, 28);
    DisplayDeviceName(pre_launch_data.rocket_settings.device_name);
    DisplayVersion(restart_message);
    DisplayChannel(lora_channel);
    DisplayStartupData(pre_launch_data);
    DisplayTimeStamp(s_sample_time);
  }
  while (display.nextPage());
}

void EinkDisplay::DisplayArmedStatus(FlightStates flight_state, int sample_count, float agl, float distance, Bearing bearing, float compass_bearing
  , double latitude, double longitude, int remote_satellites, int receiver_satellites, char* s_sample_time, uint8_t lora_channel){
  SetDisplayWindow();
  display.firstPage();
  do{
    display.fillScreen(GxEPD_WHITE);
    DisplayCompass(bearing);
    DisplayReceiverGPSStatus(receiver_satellites, 140, 0);
    DisplayBatteryLevel(analogRead(BATT_LEVEL_PIN), RECEIVER_MIN_BATTERY_LEVEL, RECEIVER_MAX_BATTERY_LEVEL, 184, 0);
    DisplayLocatorGPSStatus(remote_satellites, 140, 28);
    DisplayChannel(lora_channel);
    DisplayDistanceToRocket(distance);
    DisplayFlightData(flight_state, sample_count, agl, lora_channel);
    DisplayGPSCoordinates(latitude, longitude);
    DisplayTimeStamp(s_sample_time);
  }
  while (display.nextPage());
  //display.powerOff();
  //Serial.println("End update status");
}

void EinkDisplay::DisplayNoStatus(int receiver_satellites, uint8_t lora_channel){
  SetDisplayWindow();
  display.firstPage();
  do{
    display.fillScreen(GxEPD_WHITE);
    DisplayReceiverGPSStatus(receiver_satellites, 140, 0);
    DisplayBatteryLevel(analogRead(BATT_LEVEL_PIN), RECEIVER_MIN_BATTERY_LEVEL, RECEIVER_MAX_BATTERY_LEVEL, 184, 0);
    DisplayChannel(lora_channel);
    display.setCursor(0, 80);
    display.println("Waiting for");
    display.println("locator signal");
  }
  while (display.nextPage());
}

void EinkDisplay::DisplayRestartMessage(char* message){
  SetDisplayWindow();
  display.firstPage();
  do{
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(0, 12);
    int i = 0;
    while (message[i] != 0 && i < MAX_RESTART_MESSAGE_SIZE){
      if (message[i] != ' ')
        display.print(message[i]);
      else
        display.println();
      i++;
    }
  }
  while (display.nextPage());
}

void EinkDisplay::SetDisplayWindow(){
  uint16_t box_x = 0;
  uint16_t box_y = 0;
  uint16_t box_w = display.width();
  uint16_t box_h = display.height();
  display.setPartialWindow(box_x, box_y, box_w, box_h);
}

void EinkDisplay::DisplayCompass(Bearing bearing){
    uint8_t compass_radius = 32;
    uint8_t compass_x = 8;
    uint8_t compass_y = 0;
    uint8_t compass_center_x = compass_x + compass_radius + 1;
    uint8_t compass_center_y = compass_y + compass_radius + 1;
    display.drawCircle(compass_center_x, compass_center_y, compass_radius + 1, GxEPD_BLACK);
    display.drawLine(compass_center_x, compass_center_y, compass_center_x + (int16_t)(bearing.x * compass_radius)
      , compass_center_y - (int16_t)(bearing.y * compass_radius), GxEPD_BLACK);
    char degrees_text[4];
    sprintf(degrees_text, "%d", bearing.compass_degrees);
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.getTextBounds(degrees_text, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.fillRect(compass_center_x - tbw / 2, compass_center_y - tbh / 2 + 1, tbw, tbh, GxEPD_WHITE);
    display.setCursor(compass_center_x - tbw / 2 - 1, compass_center_y + tbh / 2);
    display.print(bearing.compass_degrees);
//    display.setCursor(compass_center_x - tbw / 2 - 1, compass_center_y + tbh / 2 + 12);
//    display.print((int)compass_bearing);
}

void EinkDisplay::DisplayDeviceName(char *device_name){
  display.setCursor(0, 30);
  display.printf("%s", device_name);
}

void EinkDisplay::DisplayVersion(char* restart_message){
  display.setCursor(0, 48);
  display.printf("%s", restart_message + 15);
}

void EinkDisplay::DisplayChannel(uint8_t lora_channel){
  display.setCursor(88, 12);
  display.printf("Ch%d", lora_channel);
}

void EinkDisplay::DisplayDistanceToRocket(float distance){
  display.setCursor(110, 68);
  display.printf("%7dm", (int)distance);
}

void EinkDisplay::DisplayReceiverGPSStatus(int receiver_satellites, uint8_t display_x, uint8_t display_y){
  display.drawInvertedBitmap(display_x, display_y, satellite_small, 24, 24, GxEPD_BLACK);
  display.setCursor(display_x + 24, display_y + 12);
  display.print(receiver_satellites);
}

void EinkDisplay::DisplayLocatorGPSStatus(int remote_satellites, uint8_t display_x, uint8_t display_y){
  display.drawInvertedBitmap(display_x, display_y, rocket_no_fire_small, 24, 24, GxEPD_BLACK);
  if (remote_satellites > 0){
    display.setCursor(display_x + 24, display_y + 12);
    display.print(remote_satellites);
  }
}

void EinkDisplay::DisplayBatteryLevel(uint16_t battery_level, uint16_t min_battery_level, uint16_t max_battery_level, uint8_t battery_x, uint8_t battery_y){
  //Serial.printf("Raw Battery Analog Level: %d\r\n", batteryLevel);
  int batteryIconLevel = ((float)battery_level - min_battery_level) / (max_battery_level - min_battery_level) * 15;
  //Serial.printf("min_battery_level: %d", min_battery_level);
  //Serial.printf("max_battery_level: %d", max_battery_level);
  //Serial.printf("battery_level: %d", battery_level);
  if (batteryIconLevel < 0)
    batteryIconLevel = 0;
  if (batteryIconLevel > 14)
    batteryIconLevel = 14;
  display.drawRect(battery_x, battery_y + 3, 12, 18, GxEPD_BLACK); // Outer battery outline
  display.drawRect(battery_x + 1, battery_y + 4, 10, 16, GxEPD_BLACK); // Inner battery outline
  display.drawRect(battery_x + 3, battery_y + 1, 6, 2, GxEPD_BLACK); // Battery top
  display.fillRect(battery_x + 2, battery_y + 19 - batteryIconLevel, 8, batteryIconLevel, GxEPD_BLACK); // Battery level
  //display.setCursor(154, 64);
  //display.printf("%d\r\n", batteryLevel);
}

void EinkDisplay::DisplayFlightData(FlightStates flight_state, int sample_count, float agl, uint8_t lora_channel){
    // Flight status and telemetry
  static float apogee = 0.0;
  if (agl > apogee)
    apogee = agl;
  display.setCursor(0, 88);
  display.printf("%s\r\n", flight_state_text[flight_state]);
  display.printf("AGL   : %9.1fm\r\n", agl);
  display.printf("Apogee: %9.1fm\r\n", apogee);
  display.printf("Flt Time: %7.1fs", (float)(sample_count - SAMPLES_PER_SECOND - 1) / SAMPLES_PER_SECOND);
}

void EinkDisplay::DisplayGPSCoordinates(double latitude, double longitude){
  // Rocket GPS coordinates
  display.setCursor(0, 160);
  display.printf("%11.7lf%c\r\n", abs(latitude), latitude >= 0 ? 'N' : 'S');
  display.printf("%11.7lf%c\r\n", abs(longitude), longitude >= 0 ? 'E' : 'W');
}

void EinkDisplay::DisplayTimeStamp(char* s_sample_time){
  // Time of last message
  display.setCursor(0, 196);
  display.printf("%.8s %.8s", s_sample_time + 2, s_sample_time + 11);
}

void EinkDisplay::DisplayStartupData(PreLaunch_t pre_launch_data){
  display.setCursor(0, 12);
  display.println("Unarmed");
  display.setCursor(0, 78);
  display.printf("Alt   %s%7.1fm\r\n", (pre_launch_data.device_status & 0x08) >> 3 ? "  OK" : "Fail", (float)pre_launch_data.agl / ALTIMETER_SCALE);
  display.printf("Acc   %s%7d\r\n", (pre_launch_data.device_status & 0x04) >> 2 ? "  OK" : "Fail", pre_launch_data.accelerometer.x);
  DeployMode deploy_mode = pre_launch_data.rocket_settings.deploy_mode;
  bool deploy1_armed = (pre_launch_data.device_status & 0x02) >> 1;
  bool deploy2_armed = pre_launch_data.device_status & 0x01;
  //Serial.printf("Deploy Mode: %d\r\n", pre_launch_data.rocket_settings.deploy_mode);
  switch (deploy_mode){
  case DeployMode::kDroguePrimaryDrogueBackup:
    display.printf("PDrogue %s%7.1fs\r\n", deploy1_armed ? "OK" : "NC", (float)pre_launch_data.rocket_settings.drogue_primary_deploy_delay / 10);
    display.printf("BDrogue %s%7.1fs\r\n", deploy2_armed ? "OK" : "NC", (float)pre_launch_data.rocket_settings.drogue_backup_deploy_delay / 10);
    break;
  case DeployMode::kMainPrimaryMainBackup:
    display.printf("PMain   %s%7dm\r\n", deploy1_armed ? "OK" : "NC", pre_launch_data.rocket_settings.main_primary_deploy_altitude);
    display.printf("BMain   %s%7dm\r\n", deploy2_armed ? "OK" : "NC", pre_launch_data.rocket_settings.main_backup_deploy_altitude);
    break;
  case DeployMode::kDroguePrimaryMainPrimary:
    display.printf("PDrogue %s%7.1fs\r\n", deploy1_armed ? "OK" : "NC", (float)pre_launch_data.rocket_settings.drogue_primary_deploy_delay / 10);
    display.printf("PMain   %s%7dm\r\n", deploy2_armed ? "OK" : "NC", pre_launch_data.rocket_settings.main_primary_deploy_altitude);
    break;
  case DeployMode::kDrogueBackupMainBackup:
    display.printf("BDrogue %s%7.1fs\r\n", deploy1_armed ? "OK" : "NC", (float)pre_launch_data.rocket_settings.drogue_backup_deploy_delay / 10);
    display.printf("BMain   %s%7dm\r\n", deploy2_armed ? "OK" : "NC", pre_launch_data.rocket_settings.main_backup_deploy_altitude);
    break;
  }
  display.setCursor(0, 160);
  display.printf("Launch Detect:%3dm\r\n", pre_launch_data.rocket_settings.launch_detect_altitude);
  display.printf("Deploy Time:%5.1fs\r\n", (float)pre_launch_data.rocket_settings.deploy_signal_duration / 10);
}