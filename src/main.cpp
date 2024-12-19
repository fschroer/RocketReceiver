#include "main.h"

#define SOFT_LED1_PIN 2 //2024
#define SOFT_LED2_PIN 5 //2024
#define ss 12 //2024
#define rst 14 //2024
#define dio0 16 //2024

//#define SOFT_LED1_PIN 12 //2022
//#define SOFT_LED2_PIN 13 //2022
//#define ss 14 //5 //2022
//#define rst 26 //10 //2022
//#define dio0 27 //13 //2022

BluetoothSerial SerialBT;
RocketGPS rocketGPS;
EinkDisplay eink_display_;
RocketConfig rocket_config_;

volatile bool delay1Interrupt = false;
hw_timer_t * delay1Timer = NULL;
portMUX_TYPE delay1TimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool delay2Interrupt = false;
hw_timer_t * delay2Timer = NULL;
portMUX_TYPE delay2TimerMux = portMUX_INITIALIZER_UNLOCKED;

char device_id_[sizeof(DEVICE_NAME) + sizeof(DEVICE_NUMBER) + 1];

void IRAM_ATTR onDelay1Timer()
{
  portENTER_CRITICAL_ISR(&delay1TimerMux);
  delay1Interrupt = true;
  portEXIT_CRITICAL_ISR(&delay1TimerMux);
}

void IRAM_ATTR onDelay2Timer()
{
  portENTER_CRITICAL_ISR(&delay2TimerMux);
  delay2Interrupt = true;
  portEXIT_CRITICAL_ISR(&delay2TimerMux);
}

void setup() {
  sprintf(device_id_, "%s%d", DEVICE_NAME, DEVICE_NUMBER);
  Serial.begin(115200);
  //while (!Serial);
  delay(1000);
  eink_display_.Begin();
  tourActionsTaken = false;
  pinMode(SOFT_LED1_PIN, OUTPUT);
  pinMode(SOFT_LED2_PIN, OUTPUT);
  digitalWrite(SOFT_LED1_PIN, HIGH);
  Serial.println("Starting Bluetooth");
  SerialBT.begin(device_id_); //Bluetooth device name
  Serial.println("LoRa Receiver");
  LoRa.setPins(ss, rst, dio0);
  //LoRa.setSyncWord(0xF0);
  rocket_config_.ReadLoraChannel();
  Serial.printf("LoRa Frequency: %d\r\n", rocket_config_.LoraFrequency());
  if (!LoRa.begin(rocket_config_.LoraFrequency())) {
    Serial.println("Starting LoRa failed!");
    //while (1);
  }
  Serial.println("Starting LoRa success!");
  rocketGPS.begin();
  delay1Timer = timerBegin(1, 80, true);
  timerAttachInterrupt(delay1Timer, &onDelay1Timer, true);
  timerAlarmWrite(delay1Timer, DELAY1_MICROS, true);
  timerAlarmEnable(delay1Timer);
  delay2Timer = timerBegin(2, 80, true);
  timerAttachInterrupt(delay2Timer, &onDelay2Timer, true);
  timerAlarmWrite(delay2Timer, DELAY2_MICROS, true);
  timerAlarmEnable(delay2Timer);
  digitalWrite(SOFT_LED1_PIN, LOW);
  xTaskCreatePinnedToCore(display_service, "FlightService", 4096, NULL, 1, &display_task_handle_, 0); //Dedicated core 0 thread for altimeter functions
  display_state_ = DisplayState::kUnarmed;
  Serial.println("Setup complete.");
}

void loop() {
  if (delay2Interrupt){
    digitalWrite(SOFT_LED1_PIN, HIGH);
    timerStop(delay2Timer);
    timerRestart(delay2Timer);
    portENTER_CRITICAL(&delay2TimerMux);
    delay2Interrupt = false;
    portEXIT_CRITICAL(&delay2TimerMux);
    //Serial.println("Call display update");
    switch (user_command_){ // Timed to avoid sending message when Rocket Locator is transmitting.
      case UserCommand::kArm: //Arm Rocket Locator
        //Serial.println("Run");
        LoRa.beginPacket();
        LoRa.println("Run");
        LoRa.endPacket();
        user_command_ = UserCommand::kNone;
        break;
      case UserCommand::kDisarm: //Disarm Rocket Locator
        //Serial.println("Stop");
        LoRa.beginPacket();
        LoRa.println("Stop");
        LoRa.endPacket();
        user_command_ = UserCommand::kNone;
        break;
      case UserCommand::kTestDeployment1: //Test deployment channel 1
        //Serial.println("TST1");
        LoRa.beginPacket();
        LoRa.print("TST");
        LoRa.write(UserInteractionState::kTestDeploy1);
        LoRa.println();
        LoRa.endPacket();
        user_command_ = UserCommand::kNone;
        break;
      case UserCommand::kTestDeployment2: //Test deployment channel 2
        //Serial.println("TST2");
        LoRa.beginPacket();
        LoRa.print("TST");
        LoRa.write(UserInteractionState::kTestDeploy2);
        LoRa.println();
        LoRa.endPacket();
        user_command_ = UserCommand::kNone;
        break;
      case UserCommand::kCancelTestDeployment: //Cancel test deployment
        //Serial.println("TST0");
        LoRa.beginPacket();
        LoRa.print("TST");
        LoRa.write(UserInteractionState::kWaitingForCommand);
        LoRa.println();
        LoRa.endPacket();
        user_command_ = UserCommand::kNone;
        break;
    }
    digitalWrite(SOFT_LED1_PIN, LOW);
  }
  //Serial.println("Checking for serial input");
  if (Serial.available()){
    //Serial.println("Serial input received");
    rocket_config_.ProcessChar(pre_launch_data_, &user_command_);
  }
  //Serial.println("Checking GPS");
  if (Serial1.available()){
    //Serial.println("GPS data received");
    rocketGPS.processGPSData();
  }
  int packetSize = LoRa.parsePacket();
  if (millis() - last_message_time > 2250){
      //Serial.printf("Rocket locator timeout at %d. Restarting LoRa.\r\n", millis() - last_message_time);
      LoRa.end();
      if (!LoRa.begin(rocket_config_.LoraFrequency()))
        Serial.println("Starting LoRa failed!");
      //else
        //Serial.printf("Restart on channel %d at %d succeeded\r\n", rocket_config_.LoraChannel(), millis() - last_message_time);
    last_message_time = millis();
  }
  if (packetSize) {
    //Serial.println("LoRa packet received");
    digitalWrite(SOFT_LED2_PIN, HIGH);
    timerStart(delay1Timer);
    timerStart(delay2Timer);
    last_message_time = millis();
    char loraMsg[packetSize];
    LoRa.readBytes(loraMsg, packetSize);
    int i = 0;
    while (i < packetSize)
      SerialBT.write(loraMsg[i++]);
    if (!strncmp(loraMsg, "PRE", MSG_HDR_SIZE)){
      display_state_ = DisplayState::kUnarmed;
      decodePreLaunchMsg(loraMsg, packetSize);
      clock_start_ = millis();
    }
    else if (!strncmp(loraMsg, "TLM", MSG_HDR_SIZE)){
      display_state_ = DisplayState::kArmed;
      decodeTelemetryMsg(loraMsg, packetSize);
      if (flight_stats_.flight_state == FlightStates::kWaitingLaunch){
        writeGpxTrackpoint();
      }
      else if (flight_stats_.flight_state <= FlightStates::kLanded){
        writeGpxTrackpoint();
      }
      if (flight_stats_.flight_state >= FlightStates::kNoseover && !tourActionsTaken){
        tourActionsTaken = true;
        writeTourLookAt();
      }
      clock_start_ = millis();
    }
    else if (!strncmp(loraMsg, "TST", MSG_HDR_SIZE)){
      Serial.printf("Countdown: %d\r\n", loraMsg[3]);
      if (loraMsg[3] == 0){
        Serial.println("Deployment test complete - exiting test mode.");
        rocket_config_.ResetDeviceState();
        rocket_config_.ResetUserInteractionState();
      }
    }
    else if (!strncmp(loraMsg, "FSM", MSG_HDR_SIZE)){
      int i = 0;
      memcpy(&flight_stats_, &loraMsg[MSG_HDR_SIZE], FLIGHT_STATS_MSG_SIZE);
      Serial.printf("Flight State          : %d\n", flight_stats_.flight_state);
      Serial.printf("AGL Adjust            : %3.2f\n", flight_stats_.agl_adjust);
      Serial.printf("Apogee                : %d\t%3.2f\n", flight_stats_.max_altitude_sample_count, flight_stats_.max_altitude);
      Serial.printf("Launch Detect         : %d\t%3.2f\n", flight_stats_.launch_detect_sample_count, flight_stats_.launch_detect_altitude);
      Serial.printf("Burnout               : %d\t%3.2f\n", flight_stats_.burnout_sample_count, flight_stats_.burnout_altitude);
      Serial.printf("Noseover              : %d\t%3.2f\n", flight_stats_.nose_over_sample_count, flight_stats_.nose_over_altitude);
      Serial.printf("Drogue Primary Deploy : %d\t%3.2f\n", flight_stats_.drogue_primary_deploy_sample_count, flight_stats_.drogue_primary_deploy_altitude);
      Serial.printf("Drogue Backup Deploy  : %d\t%3.2f\n", flight_stats_.drogue_backup_deploy_sample_count, flight_stats_.drogue_backup_deploy_altitude);
      Serial.printf("Main Primary Deploy   : %d\t%3.2f\n", flight_stats_.main_primary_deploy_sample_count, flight_stats_.main_primary_deploy_altitude);
      Serial.printf("Main Backup Deploy    : %d\t%3.2f\n", flight_stats_.main_backup_deploy_sample_count, flight_stats_.main_backup_deploy_altitude);
      Serial.printf("Landing               : %d\t%3.2f\n", flight_stats_.landing_sample_count, flight_stats_.landing_altitude);
      Serial.printf("Samples               : %d\n", flight_stats_.sample_count);
    }
    else if (!strncmp("Rocket Locator", loraMsg, 14)){
      int i = 0;
      for (; i < packetSize; i++)
        restart_message_[i] = loraMsg[i];
      restart_message_[i] = 0;
      display_state_ = DisplayState::kRestart;
    }
    else{
      Serial.print(loraMsg);
    }
    //Serial.println("Checked LoRa");
    LoRa.sleep();
    LoRa.idle();
  }
}

void decodePreLaunchMsg(char *loraMsg, int packetSize){
  //Populate pre launch data from Lora message
  updateLocatorGPSData(loraMsg, packetSize);
  memcpy(&pre_launch_data_, loraMsg + MSG_HDR_SIZE + sizeof(gps_data_) - sizeof(gps_data_.sentenceType), sizeof(pre_launch_data_));
  //pre_launch_data_.device_name[10] = 0;
/*  
  for (int i = 0; i < packetSize; i++){
    Serial.printf("%02X ", loraMsg[i]);
    if (i == 39 || i == packetSize - 1)
      Serial.println();
  }
  Serial.printf("Locator state: %d\r\n", pre_launch_data_.device_status >> 4);
  Serial.printf("altimeter_init_status: %d\r\n", (pre_launch_data_.device_status & 8) >> 3);
  Serial.printf("accelerometer_init_status_: %d\r\n", (pre_launch_data_.device_status & 4) >> 2);
  Serial.printf("deploy_sense1: %d\r\n", (pre_launch_data_.device_status & 2) >> 1);
  Serial.printf("deploy_sense2: %d\r\n", pre_launch_data_.device_status & 1);
  Serial.printf("agl: %6.1f\r\n", (float)pre_launch_data_.agl / ALTIMETER_SCALE);
  Serial.printf("accelerometer: %d %d %d\r\n", pre_launch_data_.accelerometer.x, pre_launch_data_.accelerometer.y, pre_launch_data_.accelerometer.z);
  Serial.printf("deploy_mode: %d\r\n", pre_launch_data_.rocket_settings.deploy_mode);
  Serial.printf("launch_detect_altitude: %d\r\n", pre_launch_data_.rocket_settings.launch_detect_altitude);
  Serial.printf("drogue_primary_deploy_delay: %d\r\n", pre_launch_data_.rocket_settings.drogue_primary_deploy_delay / 10);
  Serial.printf("drogue_backup_deploy_delay: %d\r\n", pre_launch_data_.rocket_settings.drogue_backup_deploy_delay / 10);
  Serial.printf("main_primary_deploy_altitude: %d\r\n", pre_launch_data_.rocket_settings.main_primary_deploy_altitude);
  Serial.printf("main_backup_deploy_altitude: %d\r\n", pre_launch_data_.rocket_settings.main_backup_deploy_altitude);
  Serial.printf("deploy_signal_duration: %d\r\n", pre_launch_data_.rocket_settings.deploy_signal_duration);
  Serial.printf("device_name: %s\r\n", pre_launch_data_.rocket_settings.device_name);
  Serial.printf("battery_voltage_mvolt: %d\r\n", pre_launch_data_.battery_voltage_mvolt);
  for (int i = 0; i < sizeof(pre_launch_data_); i++){
    Serial.printf("%02X ", ((char*)&pre_launch_data_)[i]);
  }
  Serial.println();*/
}

void decodeTelemetryMsg(char *loraMsg, int packetSize){
  //Populate telemetry data from Lora message
  updateLocatorGPSData(loraMsg, packetSize);
  flight_stats_.flight_state = *(FlightStates*)(loraMsg + MSG_HDR_SIZE + sizeof(gps_data_) - sizeof(gps_data_.sentenceType));
  //Serial.printf("flight_state: %d\r\n", flight_stats_.flight_state);
  sample_count_ = *(uint16_t*)(loraMsg + MSG_HDR_SIZE + sizeof(gps_data_) - sizeof(gps_data_.sentenceType) + sizeof(flight_stats_.flight_state));
  //Serial.printf("sample_count_: %d\r\n", sample_count_);
  //int i = 0;
  //float agl_value_f = (float)*(uint16_t*)(loraMsg + MSG_HDR_SIZE + sizeof(gps_data_) - sizeof(gps_data_.sentenceType)
  //  + sizeof(flight_stats_.flight_state) + sizeof(sample_count_) + i * sizeof(uint16_t)) / ALTIMETER_SCALE;
  //Serial.printf("agl_f: %6.1f\r\n", agl_value_f);
  //int agl_values = (flight_stats_.flight_state > kLaunched && flight_stats_.flight_state < kLanded) ? SAMPLES_PER_SECOND : 1;
  //Serial.printf("agl_values: %d\r\n", agl_values);
  for (int i = 0; i < ((flight_stats_.flight_state > kLaunched && flight_stats_.flight_state < kLanded) ? SAMPLES_PER_SECOND : 1); i++){
    flight_stats_.agl[i] = (float)*(uint16_t*)(loraMsg + MSG_HDR_SIZE + sizeof(gps_data_) - sizeof(gps_data_.sentenceType)
    + sizeof(flight_stats_.flight_state) + sizeof(sample_count_) + i * sizeof(uint16_t)) / ALTIMETER_SCALE;
    //Serial.printf("agl_f: %d - %6.1f\r\n", i, flight_stats_.agl[i]);
  }
  //for (int i = 0; i < SAMPLES_PER_SECOND; i++)
  //  Serial.printf("%6.2f ", flight_stats_.agl[i]);
}

void updateLocatorGPSData(char *loraMsg, int packetSize){
  //Recreate NMEA message for Rocket Locator
  /*for (int i = 0; i < packetSize; i++){
    Serial.printf("%02X ", loraMsg[i]);
    if (i == 2 || i == 39 || i == packetSize - 1)
      Serial.println();
  }*/
  memcpy((uint8_t*)&gps_data_ + sizeof(gps_data_.sentenceType), loraMsg + MSG_HDR_SIZE, sizeof(gps_data_) - sizeof(gps_data_.sentenceType));
  MakeDateTime(s_sample_time_, gps_data_.dateStamp, gps_data_.timeStamp);
  sprintf(gps_sentence_ + GPS_SENTENCE_TYPE_LEN, ",%06d,%.5lf,%c,%.5lf,%c,%c,%02d,%.2f,%.2f,M,0,M,,%c%c%c\r\n", gps_data_.timeStamp, abs(gps_data_.latitude),
    gps_data_.latitude >= 0 ? 'N' : 'S', abs(gps_data_.longitude), gps_data_.longitude >= 0 ? 'E' : 'W', gps_data_.qInd, gps_data_.satellites, gps_data_.hdop,
    gps_data_.altitude, gps_data_.checksum[0], gps_data_.checksum[1], gps_data_.checksum[2]);
  prevLatitude = dLatitude;
  prevLongitude = dLongitude;
  dLatitude = (int)gps_data_.latitude / 100 + (gps_data_.latitude - ((int)gps_data_.latitude / 100 * 100)) / 60;
  dLongitude = (int)gps_data_.longitude / 100 + (gps_data_.longitude - ((int)gps_data_.longitude / 100 * 100)) / 60;
  rocketGPS.setRemoteGPSCoordinates(dLatitude, dLongitude);
  //Serial.printf("GPS: %.6s %06d %06d %.5lf %.5lf %c %02d %.2f %.2f %.3s\n",
  //  gps_data_.sentenceType, gps_data_.dateStamp, gps_data_.timeStamp, gps_data_.latitude, gps_data_.longitude
  //  , gps_data_.qInd, gps_data_.satellites, gps_data_.hdop, gps_data_.altitude, gps_data_.checksum);
  /*int i = 0;
  while (i < strlen(gps_sentence_))
    SerialBT.write(gps_sentence_[i++]);*/
}
  
void writeGpxTrackpoint(){
  prev_sample_time_.tm_mday = sample_time_.tm_mday;
  prev_sample_time_.tm_mon = sample_time_.tm_mon;
  prev_sample_time_.tm_year = sample_time_.tm_year;
  prev_sample_time_.tm_hour = sample_time_.tm_hour;
  prev_sample_time_.tm_min = sample_time_.tm_min;
  prev_sample_time_.tm_sec = sample_time_.tm_sec;

  //Write GPX file trackpoint - high velocity
  if (flight_stats_.flight_state == FlightStates::kWaitingLaunch){
      Serial.printf("<trkpt lat=\"%.5lf\" lon=\"%.5lf\"><ele>%.2f</ele><time>%sZ</time></trkpt>\r\n",
        dLatitude, dLongitude, flight_stats_.agl[0] * FEET_PER_METER, s_sample_time_);
  }
  if (flight_stats_.flight_state > FlightStates::kWaitingLaunch){
    if(flight_stats_.flight_state < FlightStates::kDroguePrimaryDeployed){
      float latitudeDelta = dLatitude - prevLatitude;
      float longitudeDelta = dLongitude - prevLongitude;
      for (int i = 0; i < SAMPLES_PER_SECOND; i++){
        Serial.printf("<trkpt lat=\"%.5lf\" lon=\"%.5lf\"><ele>%.2f</ele><time>%s.%d%dZ</time></trkpt>\r\n",
          prevLatitude + latitudeDelta * (i + 1) / SAMPLES_PER_SECOND, prevLongitude + longitudeDelta * (i + 1) / SAMPLES_PER_SECOND,
          flight_stats_.agl[i] * FEET_PER_METER, s_sample_time_, int((float)i / SAMPLES_PER_SECOND * 10), i % 2 * 5);
      }
    }
    //Write GPX file trackpoint - low velocity
    else{
      Serial.printf("<trkpt lat=\"%.5lf\" lon=\"%.5lf\"><ele>%.2f</ele><time>%sZ</time></trkpt>\r\n",
        dLatitude, dLongitude, flight_stats_.agl[0] * FEET_PER_METER, s_sample_time_);
    }
  }
}

void MakeDateTime(char *target, int date, int time){
  setenv("TZ", "UTC0", 1); // Set timezone to UTC
  tzset();
  tm *local_time;
  int sample_time_length;
  sample_time_.tm_mday = date / 10000;
  sample_time_.tm_mon = (date - sample_time_.tm_mday * 10000) / 100 - 1;
  sample_time_.tm_year = CENTURY + date % 100;
  sample_time_.tm_hour = time / 10000;
  sample_time_.tm_min = (time - sample_time_.tm_hour * 10000) / 100;
  sample_time_.tm_sec = time - time / 100 * 100;
  time_t mtt;
  mtt = mktime(&sample_time_);
  setenv("TZ", "PST8PDT", 1); // Set timezone to PT
  tzset();
  local_time = localtime(&mtt);
  sample_time_length = strftime(target, DATE_STRING_LENGTH, "%Y-%m-%dT%H:%M:%S", local_time);
  unsetenv("TZ");
}

void writeTour(){
  float radius = 0.02;
  int startingAltitude = 600;
  int endingAltitude = gps_data_.altitude * FEET_PER_METER + 100;
  int deg = 0;
  int startingTilt = 40;
  int endingTilt = 0;
  int circumSteps = 24;
  int flyInSteps = 10;
  Serial.printf("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\" \
xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" \
xmlns:atom=\"http://www.w3.org/2005/Atom\">\n<gx:Tour>\n\t<name>Rocket Flight %s</name>\n\t<gx:Playlist>\n\
\t\t<gx:AnimatedUpdate>\n\t\t\t<Update>\n\t\t\t\t<targetHref></targetHref>\n\
\t\t\t\t<Change><Placemark targetId=\"Tri-Cities Rocketeers Launch Site\"><gx:balloonVisibility>1</gx:balloonVisibility></Placemark></Change>\n\
\t\t\t</Update>\n\t\t</gx:AnimatedUpdate>\n", s_sample_time_);
  for (; deg < 360; deg += 360 / circumSteps){
    Serial.printf("\t\t<gx:FlyTo>\n\t\t\t<gx:duration>1.0</gx:duration>\n\t\t\t<gx:flyToMode>smooth</gx:flyToMode>\n\t\t\t<Camera>\n\
\t\t\t\t<gx:horizFov>60</gx:horizFov>\n\
\t\t\t\t<longitude>%.5lf</longitude>\n\t\t\t\t<latitude>%.5lf</latitude>\n\
\t\t\t\t<altitude>%d</altitude>\n\t\t\t\t<heading>%d</heading>\n\
\t\t\t\t<tilt>%d</tilt>\n\t\t\t\t<gx:altitudeMode>absolute</gx:altitudeMode>\n\t\t\t</Camera>\n\t\t</gx:FlyTo>\n",
      dLongitude + radius * cos(deg * M_PI / 180),
      dLatitude + radius * sin(deg * M_PI / 180),
      startingAltitude,
      270 - deg,
      startingTilt);
  }
  for (int i = flyInSteps - 1; i >= 0; i--){
    Serial.printf("\t\t<gx:FlyTo>\n\t\t\t<gx:duration>1.0</gx:duration>\n\t\t\t<gx:flyToMode>smooth</gx:flyToMode>\n\t\t\t<Camera>\n\
\t\t\t\t<gx:horizFov>60</gx:horizFov>\n\
\t\t\t\t<longitude>%.5lf</longitude>\n\t\t\t\t<latitude>%.5lf</latitude>\n\
\t\t\t\t<altitude>%d</altitude>\n\t\t\t\t<heading>%d</heading>\n\
\t\t\t\t<tilt>%d</tilt>\n\t\t\t\t<gx:altitudeMode>absolute</gx:altitudeMode>\n\t\t\t</Camera>\n\t\t</gx:FlyTo>\n",
      dLongitude + radius * cos(deg * M_PI / 180) * i / flyInSteps,
      dLatitude + radius * sin(deg * M_PI / 180) * i / flyInSteps,
      endingAltitude + (startingAltitude - endingAltitude) * i / flyInSteps,
      270 - deg,
      endingTilt + (startingTilt - endingTilt) * i / flyInSteps);
  }
  Serial.print("\t\t<gx:AnimatedUpdate>\n\t\t\t<Update>\n\t\t\t\t<targetHref></targetHref>\n\
		\t\t\t\t<Change><Placemark targetId=\"Tri-Cities Rocketeers Launch Site\"><gx:balloonVisibility>0</gx:balloonVisibility></Placemark></Change>\n\
		\t\t\t</Update>\n\t\t</gx:AnimatedUpdate>\n\t</gx:Playlist>\n</gx:Tour>\n</kml>\n");
}

void writeTourLookAt(){
  int startingRange = 1200;
  int endingRange = 100;
  int deg = 360;
  int startingTilt = 80;
  int endingTilt = 20;
  int circumSteps = 24;
  int flyInSteps = 10;
  Serial.printf("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\" \
xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" \
xmlns:atom=\"http://www.w3.org/2005/Atom\">\n<gx:Tour>\n\t<name>Rocket Flight %04d-%02d-%02d %02d:%02d:%02d</name>\n\t<gx:Playlist>\n\
\t\t<gx:AnimatedUpdate>\n\t\t\t<Update>\n\t\t\t\t<targetHref></targetHref>\n\
\t\t\t\t<Change><Placemark targetId=\"Tri-Cities Rocketeers Launch Site\"><gx:balloonVisibility>1</gx:balloonVisibility></Placemark></Change>\n\
\t\t\t</Update>\n\t\t</gx:AnimatedUpdate>\n", s_sample_time_);
  for (; deg > 0; deg -= 360 / circumSteps){
    Serial.printf("\t\t<gx:FlyTo>\n\t\t\t<gx:duration>1.0</gx:duration>\n\t\t\t<gx:flyToMode>smooth</gx:flyToMode>\n\t\t\t<LookAt>\n\
\t\t\t\t<gx:horizFov>60</gx:horizFov>\n\
\t\t\t\t<longitude>%.5lf</longitude>\n\t\t\t\t<latitude>%.5lf</latitude>\n\
\t\t\t\t<altitude>%d</altitude>\n\t\t\t\t<range>%d</range>\n\t\t\t\t<heading>%d</heading>\n\
\t\t\t\t<tilt>%d</tilt>\n\t\t\t\t<gx:altitudeMode>absolute</gx:altitudeMode>\n\t\t\t</LookAt>\n\t\t</gx:FlyTo>\n",
      dLongitude,
      dLatitude,
      (int)(gps_data_.altitude * FEET_PER_METER / 2),
      startingRange,
      deg,
      startingTilt);
  }
  for (int i = flyInSteps - 1; i >= 0; i--){
    Serial.printf("\t\t<gx:FlyTo>\n\t\t\t<gx:duration>1.0</gx:duration>\n\t\t\t<gx:flyToMode>smooth</gx:flyToMode>\n\t\t\t<LookAt>\n\
\t\t\t\t<gx:horizFov>60</gx:horizFov>\n\
\t\t\t\t<longitude>%.5lf</longitude>\n\t\t\t\t<latitude>%.5lf</latitude>\n\
\t\t\t\t<altitude>%d</altitude>\n\t\t\t\t<range>%d</range>\n\t\t\t\t<heading>%d</heading>\n\
\t\t\t\t<tilt>%d</tilt>\n\t\t\t\t<gx:altitudeMode>absolute</gx:altitudeMode>\n\t\t\t</LookAt>\n\t\t</gx:FlyTo>\n",
      dLongitude,
      dLatitude,
      (int)(gps_data_.altitude * FEET_PER_METER / 2),
      endingRange + (startingRange - endingRange) * i / flyInSteps,
      deg,
      endingTilt + (startingTilt - endingTilt) * i / flyInSteps);
  }
  Serial.print("\t\t<gx:AnimatedUpdate>\n\t\t\t<Update>\n\t\t\t\t<targetHref></targetHref>\n\
		\t\t\t\t<Change><Placemark targetId=\"Tri-Cities Rocketeers Launch Site\"><gx:balloonVisibility>0</gx:balloonVisibility></Placemark></Change>\n\
		\t\t\t</Update>\n\t\t</gx:AnimatedUpdate>\n\t</gx:Playlist>\n</gx:Tour>\n</kml>\n");
}

void display_service(void* pvParameters) {
  //static int i = 0;
  //UBaseType_t uxHighWaterMark;
  while(1){
    if (delay1Interrupt){
      //Serial.println("Loop Interrupt");
      timerStop(delay1Timer);
      timerRestart(delay1Timer);
      portENTER_CRITICAL(&delay1TimerMux);
      delay1Interrupt = false;
      portEXIT_CRITICAL(&delay1TimerMux);
      digitalWrite(SOFT_LED2_PIN, LOW);
      //Serial.println("Call display update");
      //Serial.println(display_state_);
      switch (display_state_) {
        case DisplayState::kArmed:
          eink_display_.DisplayArmedStatus(flight_stats_.flight_state, sample_count_, flight_stats_.agl[0], rocketGPS.GetDistanceFromLatLon(), rocketGPS.GetBearingFromLatLon()
            , rocketGPS.GetCompassBearingDegrees(), dLatitude, dLongitude, gps_data_.satellites, rocketGPS.getSatellites(), s_sample_time_, rocket_config_.LoraChannel());
          break;
        case DisplayState::kUnarmed:
          eink_display_.DisplayUnarmedStatus(gps_data_.satellites, rocketGPS.getSatellites(), rocket_config_.LoraChannel(), pre_launch_data_, s_sample_time_, restart_message_);
          break;
        case DisplayState::kRestart:
        eink_display_.DisplayRestartMessage(restart_message_);
        break;
      }
    }
    //Serial.printf("clock_start_: %d, millis: %d\r\n", clock_start_, millis());
    if (millis() - clock_start_ > 5000){
      //Serial.println("Rocket locator timeout");
      //LoRa.end();
      //if (!LoRa.begin(rocket_config_.LoraFrequency())) {
      //  Serial.println("Starting LoRa failed!");
        //while (1);
      //}
      //Serial.println("Starting LoRa success!");
      //LoRa.sleep();
      //LoRa.idle();
      //eink_display_.DisplayNoStatus(rocketGPS.getSatellites(), rocket_config_.LoraChannel());
      switch (display_state_) {
        case DisplayState::kArmed:
          eink_display_.DisplayArmedStatus(FlightStates::kNoSignal, sample_count_, flight_stats_.agl[0], rocketGPS.GetDistanceFromLatLon(), rocketGPS.GetBearingFromLatLon()
            , rocketGPS.GetCompassBearingDegrees(), dLatitude, dLongitude, gps_data_.satellites, rocketGPS.getSatellites(), s_sample_time_, rocket_config_.LoraChannel());
          break;
        case DisplayState::kUnarmed:
          eink_display_.DisplayNoStatus(rocketGPS.getSatellites(), rocket_config_.LoraChannel());
          break;
      }
      clock_start_ = millis();
    }
    vTaskDelay(1);
/*    if (++i == SAMPLES_PER_SECOND){
      i = 0;
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.print("Task Stack High Water Mark: ");
      Serial.println(uxHighWaterMark);
    }*/
  }
}