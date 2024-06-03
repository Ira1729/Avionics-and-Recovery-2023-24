#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <BNO055_support.h>
#include <SerialFlash.h>
#include <Wire.h>
#include <SPI.h>
#include <CRC8.h>
#include <Preferences.h>
#include <BasicLinearAlgebra.h>

//Initialisations
CRC8 crc;

//Flight Stages---------------------------------------
//The mark variables are to ensure that the data logger core writes them to esp flash
bool liftoff = false;
bool liftoff_mark = false;
bool landed = false;
bool landed_mark = false;
bool apogee = false;
bool apogee_mark = false;
bool target = false;
bool target_mark = false;
bool reef = false;
bool reef_mark = false;
float maxalt = 0;
uint64_t maxalt_time = 0; 
int apogee_detect_timer = 2;

float landed_window[10];
int landed_count = 10;

float variance(float arr[], int len){
  float sumx = 0, sumx2 = 0;
  for (int i=0; i < len; i++){
    sumx = sumx + arr[i];
    sumx2 = sumx2 + (arr[i] * arr[i]);
  }
  return ((sumx2/len) - ((sumx/len)*(sumx/len)));
}

//ESP Misc
uint64_t fc_time = 0;
uint64_t now_time = esp_timer_get_time();
uint64_t prev_time = now_time;
uint8_t flight_count=0;
bool block_core = false;

Preferences fc_pref;
char key_liftoff[15];
char key_landed[15];
char key_apogee[15];
char key_target[15];
char key_reef[15];
char key_maxalt[15];
char key_maxalt_time[15];
char key_file_offset[15];
char key_press_initial[15];
char key_temp_initial[15];
char key_fc_time[15];


TaskHandle_t DataLogger;
TaskHandle_t FrontEnd;

//SPI Flash
#define FLASH_CS 33
#define FLASH_WP 2
#define FLASH_HOLD 15

SPIClass FC_SPI(HSPI);
#define FLASH_TIME 4
int flash_countdown = FLASH_TIME;
int file_offset = 0;
SerialFlashFile file;
char file_name[20];

void init_spi_flash(){
  pinMode(FLASH_HOLD,OUTPUT);
  pinMode(FLASH_WP,OUTPUT);
  digitalWrite(FLASH_HOLD,HIGH);
  digitalWrite(FLASH_WP,HIGH);

  while (!SerialFlash.begin(FC_SPI, FLASH_CS)) {
    Serial.println("Unable to access SPI Flash chip");
    delay(100);
  }
  delay(100);
  snprintf(file_name, sizeof(file_name), "flight_%d", flight_count);
  if (liftoff == false){
    for (int i = 0; i < 3; i++){
      if (SerialFlash.create(file_name, 2000000)) {
        Serial.println("File Created!");
        break;
      } else {
        Serial.println("File already exists!");
        delay(100);
      }
    }
  }
}

void write_to_flash_file(uint8_t arr[],int len){
  file = SerialFlash.open(file_name);
  if (file) {
    //CRC
    crc.restart();
    crc.add(arr, len);
    uint8_t crc_value = crc.calc();
    file.seek(file_offset);
    file.write(arr, len);
    file.write(&crc_value, 1);
    file_offset = file_offset + len + 1;
    file.close();
  } 
  else {
    Serial.println("Failed to open file for writing.");
  }
}

//GPS
#define GPS_TX 16
#define GPS_RX 17
SoftwareSerial FC_GPS(GPS_RX, GPS_TX);
TinyGPSPlus gps;
float Pos_lat, Pos_long;
float initial_gps_alt = 0, gps_alt;
uint8_t gps_init_count = 100;

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do { 
    //Encode data read from GPS while data is available on serial port
    while (FC_GPS.available())
      gps.encode(FC_GPS.read());
  //Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it 
  } while (millis() - start < ms);
}

void DegMinSec(float tot_val, uint8_t* degree, uint8_t* minutes, uint8_t* seconds){
  //Convert data in decimal degrees into degrees minutes seconds form
  float deg,min,sec;
  deg = (int)tot_val;
  min = (tot_val - deg) * 60;
  sec = (int)(60 * (min-(int)min));
  min = (int)min;
  *degree=(uint8_t)deg;
  *minutes=(uint8_t)min;
  *seconds=(uint8_t)sec; 
}

//BME280-----------------------------------------------------------------
Adafruit_BME280 bme;
float bme_disable = false;

float getAltitude(float pressure, float base_press, float base_temp) {
  return (float) (base_temp+273.15)/0.0065 * (1.0 - pow(pressure/base_press,0.1903));
}

//Pressure Moving avg
#define PRESS_WIN 10
float press_arr[PRESS_WIN];
int press_index = 0;
float press_avg = 0;
float press_initial = 0;

void press_win_update(){
  block_core = true;
  float val=bme.readPressure();
  if (val == -16180.55) ESP.restart();
  block_core = false;
  press_avg=press_avg+(val-press_arr[press_index])/PRESS_WIN;
  press_arr[press_index]=val;
  press_index=(press_index+1)%PRESS_WIN;
}

//Temp Moving avg
#define TEMP_WIN 20
float temp_arr[TEMP_WIN];
int temp_index = 0;
float temp_avg = 0;
float temp_initial = 0;

void temp_win_update(){
  block_core = true;
  float val=bme.readTemperature();//check if bme is disconnected
  if (isnanf(val)){
    for (int i=0;i<3;i++){
      val=bme.readTemperature();
      if (!isnanf(val)) break;
      if (i==2) ESP.restart();
    }
  }
  block_core = false;
  temp_avg=temp_avg+(val-temp_arr[temp_index])/TEMP_WIN;
  temp_arr[temp_index]=val;
  temp_index=(temp_index+1)%TEMP_WIN;
}

float bme_alt=0;
float filtered_alt=0;

void init_bme(){
  //Function to initialise bme in setup
  int countdown = 1;
  while (!bme.begin(0x76)){
    Serial.println("BME not found");
    delay(100);
    countdown--;
    if (countdown == 0){
      bme_disable = true;
      return;
    }
  }
  bme.setSampling(bme.MODE_NORMAL,bme.SAMPLING_X16,bme.SAMPLING_X16,bme.SAMPLING_NONE,bme.FILTER_X16,bme.STANDBY_MS_0_5);
  //Initialising moving avg for BME data
  for(int i=0;i<2*PRESS_WIN;i++) press_win_update();
  for(int i=0;i<2*TEMP_WIN;i++) temp_win_update();
}

//BNO055----------------------------------------------------
struct bno055_t fcBNO;
struct bno055_accel bno_accel;
struct bno055_mag bno_mag;
struct bno055_gyro bno_gyro;
float mag_x = 0, mag_y = 0, mag_z = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;
float acc_x = 0, acc_y = 0, acc_z = 0;
float ang_x = 0, ang_y =0, ang_z =0;
bool bno_disable = false;

#define CRCN_GYRO_X -0.109475962
#define CRCN_GYRO_Y -0.056584376
#define CRCN_GYRO_Z -0.03013

float simple_low_pass(float x, float y1, float coef){
  return (coef*x+y1)/(coef+1);
}

void bno_raw_update(){
  block_core = true;
  BNO_Init(&fcBNO);
  if (fcBNO.chip_id != 160) ESP.restart();//check if bno is disconnected
  block_core = false;
  bno055_read_mag_xyz(&bno_mag);
  bno055_read_gyro_xyz(&bno_gyro);
  bno055_read_accel_xyz(&bno_accel);

  mag_x = simple_low_pass(float(bno_mag.x)/16, mag_x, 8);
  mag_y = simple_low_pass(float(bno_mag.y)/16, mag_y, 8);
  mag_z = simple_low_pass(float(bno_mag.z)/16, mag_z, 8);
  
  gyro_x = simple_low_pass(float(bno_gyro.x)/16 - CRCN_GYRO_X, gyro_x, 31);
  gyro_y = simple_low_pass(float(bno_gyro.y)/16 - CRCN_GYRO_Y, gyro_y, 31);
  gyro_z = simple_low_pass(float(bno_gyro.z)/16 - CRCN_GYRO_Z, gyro_z, 31);

  acc_x = simple_low_pass(float(bno_accel.x)/100, acc_x, 8);
  acc_y = simple_low_pass(float(bno_accel.y)/100, acc_y, 8);
  acc_z = simple_low_pass(float(bno_accel.z)/100, acc_z, 8);
}

void init_bno(){
  BNO_Init(&fcBNO);
  int countdown = 1;
  while(fcBNO.chip_id!=160){
    BNO_Init(&fcBNO);
    Serial.println("BNO not found");
    delay(100);
    countdown--;
    if (countdown == 0) {
      bno_disable = true;
      return;
    }
  }
  bno055_set_operation_mode(OPERATION_MODE_CONFIG);
  delay(30);
  bno055_set_accel_range(ACCEL_RANGE_16G);
  delay(30);  
  bno055_set_operation_mode(OPERATION_MODE_AMG);

  for(int i=0; i<50; i++) bno_raw_update();
}

//LORA-----------------------------------------------------
#define LORA_M0 32 
#define LORA_M1 25 
#define LORA_TX 26 
#define LORA_RX 35 

SoftwareSerial SerialLORA(LORA_RX, LORA_TX);
uint8_t mymsg = 0x00;
#define T_TIME 5
uint8_t transmit_countdown = T_TIME;
void init_lora(){
  pinMode(LORA_M0,OUTPUT);
  pinMode(LORA_M1,OUTPUT);
  pinMode(LORA_RX,INPUT);
  pinMode(LORA_TX,OUTPUT);
  SerialLORA.begin(9600,SWSERIAL_8N1);
  digitalWrite(LORA_M0,HIGH);
  digitalWrite(LORA_M1,HIGH);
  delay(100);
  uint8_t config[]={0xC2,0x00,0x01,0x18,0x23,0x47};//The 2nd and 3rd byte is address
  SerialLORA.write(config,6);
  delay(100);
  digitalWrite(LORA_M0,LOW);
  digitalWrite(LORA_M1,LOW);
}

void lora_transmit(uint8_t arr[], uint8_t len){
  //Sends an array via lora, adds crc and start byte
  crc.restart();
  SerialLORA.write(0x01);//Start byte
  for (int i=0;i<len;i++){
    crc.add(arr[i]);
    SerialLORA.write(arr[i]);
  }
  SerialLORA.write(crc.calc());
}


//Channels--------------------------------------------
#define CH1 27
#define TEST_CH1 36
#define CH2 4
#define TEST_CH2 39
#define BATTERY 0
float ch1_volt = 0;
float ch2_volt = 0;
float bat_volt = 0;

void get_channel_data(){
  ch1_volt=analogRead(TEST_CH1)*13.3/(3.3*4096.0);
  ch2_volt=analogRead(TEST_CH2)*13.3/(3.3*4096.0);
  bat_volt=analogRead(BATTERY)*0.002466 ;
}

//Indicators------------------------------------------
#define LED1 5
#define LED2 19
#define LED3 18
#define BUZZER 23
int8_t indicator_reset = -1;
int8_t land_beep_count = 0;

//Kalman Filter--------------------------------------
#define APPROX_BURNOUT_ALT 1200
#define ACC_G 9.81

BLA::Matrix<3,1> kalman_x;
BLA::Matrix<3,1> kalman_x_inter;
BLA::Matrix<3,1> kalman_x_prev = {0, 0, 0};

BLA::Matrix <3,3> kalman_P;
BLA::Matrix <3,3> kalman_P_inter;
BLA::Matrix <3,3> kalman_P_prev = {5, 0, 0,
                                   0, 5, 0,
                                   0, 0, 5};

BLA::Matrix <3,3> kalman_A;
                            
BLA::Matrix <3,3> kalman_A_T;

BLA::Matrix <3,3> kalman_Q = {5, 0, 0,
                              0, 5, 0,
                              0, 0, 2};
BLA::Matrix <3,1> kalman_K;
BLA::Matrix <1,3> kalman_H;
BLA::Matrix <3,1> kalman_H_T;
BLA::Matrix <1,1> kalman_R;
BLA::Matrix <1,1> kalman_z;

void kalman_predict(float dt){
  kalman_A = {1, dt, dt*dt/2,
              0, 1, dt,
              0, 0, 1};
                            
  kalman_A_T = { 1, 0, 0,
                dt, 1, 0,
                dt*dt/2, dt, 1};
  kalman_x_inter = kalman_A * kalman_x_prev;
  kalman_P_inter = kalman_A * kalman_P_prev * kalman_A_T + kalman_Q;
}

void kalman_gain(int sensor){
  if (sensor == 1){
    //bme
    kalman_H = {1, 0, 0};
    kalman_H_T = {1, 0, 0};
    kalman_R = {2.062};
  }
  else if (sensor == 2){
    //gps
    kalman_H = {1, 0, 0};
    kalman_H_T = {1, 0, 0};
    kalman_R = {10};
  }
  if (sensor == 3){
    //bno
    kalman_H = {0, 0, 1};
    kalman_H_T = {0, 0, 1};
    kalman_R = {0.0029};
  }

  BLA::Matrix <1,1> intermediate = kalman_H * kalman_P_inter * kalman_H_T + kalman_R;
  kalman_K = kalman_P_inter * kalman_H_T * BLA::Inverse(intermediate);
}

void kalman_estimate_update(int sensor){
  if(sensor == 1){
    kalman_z = {bme_alt};
  }

  if(sensor == 2){
    kalman_z = {gps_alt - initial_gps_alt};
  }

  if(sensor == 3){
    if ((apogee == true) || bme_alt> APPROX_BURNOUT_ALT) kalman_z = {-ACC_G};
    else kalman_z = {acc_z - ACC_G};
  }

  kalman_x = kalman_x_inter + kalman_K * (kalman_z - (kalman_H * kalman_x_inter));
  kalman_P = kalman_P_inter - kalman_K * kalman_H * kalman_P_inter ;

  kalman_x_prev = kalman_x;
  kalman_P_prev = kalman_P;

  filtered_alt = kalman_x(0,0);
}

void loop1(void* pvparameters);
void loop2(void* pvparameters);

void setup() {
  Wire.begin();
  Serial.begin(9600);

  //Indicators
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(BUZZER,OUTPUT);
  ledcAttachPin(BUZZER,0);
  noTone(BUZZER);

  //Channels
  pinMode(CH1,OUTPUT);
  pinMode(TEST_CH1,INPUT);
  pinMode(CH2,OUTPUT);
  pinMode(TEST_CH2,INPUT);
  pinMode(BATTERY,INPUT);
  digitalWrite(CH1,LOW);
  digitalWrite(CH2,LOW);

  //BME
  init_bme();

  //BNO
  init_bno();

  //ESP Flash
  fc_pref.begin("Abhyuday FC", false);
  
  flight_count = fc_pref.getUChar("flt_cnt", 0);
  
  snprintf(key_landed, sizeof(key_landed), "landed%d", flight_count);
  landed = fc_pref.getBool(key_landed, false);

  if (landed == true) {
    if (flight_count == 3){
      Serial.println("Max flight count reached! Clear data");
      while (true);
    }
    else {
      flight_count += 1;
      fc_pref.putUChar("flt_cnt", flight_count);
    }
  }

  snprintf(key_landed, sizeof(key_landed), "landed%d", flight_count);
  landed = fc_pref.getBool(key_landed, false);
  landed_mark = landed;
  snprintf(key_liftoff, sizeof(key_liftoff), "liftoff%d", flight_count);
  liftoff = fc_pref.getBool(key_liftoff, false);
  liftoff_mark = liftoff;
  snprintf(key_apogee, sizeof(key_apogee), "apogee%d", flight_count);
  apogee =  fc_pref.getBool(key_apogee, false);
  apogee_mark = apogee;
  snprintf(key_target, sizeof(key_target), "target%d", flight_count);
  target = fc_pref.getBool(key_target, false);
  target_mark = target;
  snprintf(key_reef, sizeof(key_reef), "reef%d", flight_count);
  reef = fc_pref.getBool(key_reef, false);
  reef_mark = reef;
  snprintf(key_maxalt, sizeof(key_maxalt), "maxalt%d", flight_count);
  maxalt = fc_pref.getFloat(key_maxalt, 0);
  snprintf(key_maxalt_time, sizeof(key_maxalt_time), "maxalt_t%d", flight_count);
  maxalt_time = fc_pref.getULong64(key_maxalt_time, 0);
  snprintf(key_file_offset, sizeof(key_file_offset), "file_off%d", flight_count);
  file_offset = fc_pref.getInt(key_file_offset, 0);

  snprintf(key_press_initial, sizeof(key_press_initial), "p_init%d", flight_count);
  snprintf(key_temp_initial, sizeof(key_temp_initial), "t_init%d", flight_count);
  snprintf(key_fc_time, sizeof(key_fc_time), "fc_time%d", flight_count);

  if (liftoff == false){
    press_initial = press_avg;
    temp_initial = temp_avg;
  }
  else {
    press_initial = fc_pref.getFloat(key_press_initial, press_avg);
    temp_initial = fc_pref.getFloat(key_temp_initial, temp_avg);
    fc_time = fc_pref.getULong64(key_fc_time, 0) + 100000;
  }
  fc_pref.end();
  Serial.printf("Flight %d\n",flight_count);
  Serial.printf("Initial pressure: %f\n",press_initial);
  Serial.printf("Initial temp: %f\n",temp_initial);


  //LORA
  init_lora();
  
  FC_GPS.begin(9600);

  //SPI Flash
  while (SerialFlash.ready() == false) {
   Serial.println("Waiting");
   delay(1000);
  }
  init_spi_flash();


  //Start indication
  digitalWrite(LED3,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED1,HIGH);
  tone(BUZZER,131);
  delay(200);
  digitalWrite(LED3,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED1,LOW);
  noTone(BUZZER);
  mymsg = 0x02;
  xTaskCreatePinnedToCore(loop1, "Data Logger", 10000, NULL, 1, &DataLogger, 0);
  xTaskCreatePinnedToCore(loop2, "Front End", 10000, NULL, 1, &FrontEnd, 1);
}

void save_data(){
   //Compiles and saves data
   //Encode all data into two bytes and save
    uint8_t tr_temp = (temp_avg - 12) / 0.25;
    uint8_t tr_press_l = (int(press_avg - 50000)) & (0xff);
    uint8_t tr_press_h = ((int(press_avg - 50000)) >> 8) & (0xff);
    uint8_t tr_alt_l = (int(filtered_alt * 10 + 100)) & 0xff;
    uint8_t tr_alt_h = ((int(filtered_alt * 10 + 100) >> 8)) & 0xff;

    uint8_t tr_gyrox_l = (int((gyro_x + 2000) * 16)) & 0xff;
    uint8_t tr_gyrox_h = (int((gyro_x + 2000) * 16) >> 8) & 0xff;
    uint8_t tr_gyroy_l = (int((gyro_y + 2000) * 16)) & 0xff;
    uint8_t tr_gyroy_h = (int((gyro_y + 2000) * 16) >> 8) & 0xff;
    uint8_t tr_gyroz_l = (int((gyro_z + 2000) * 16)) & 0xff;
    uint8_t tr_gyroz_h = (int((gyro_z + 2000) * 16) >> 8) & 0xff;

    uint8_t tr_magx_l = (int((mag_x + 1300) * 16)) & 0xff;
    uint8_t tr_magx_h = (int((mag_x + 1300) * 16) >> 8) & 0xff;
    uint8_t tr_magy_l = (int((mag_y + 1300) * 16)) & 0xff;
    uint8_t tr_magy_h = (int((mag_y + 1300) * 16) >> 8) & 0xff;
    uint8_t tr_magz_l = (int((mag_z + 2500) * 16)) & 0xff;
    uint8_t tr_magz_h = (int((mag_z + 2500) * 16) >> 8) & 0xff;
    int time_100ms = int(fc_time / 100);
    uint8_t tr_time_l = (time_100ms & 0xff);
    uint8_t tr_time_m = ((time_100ms >> 8) & 0xff);
    uint8_t tr_time_h = ((time_100ms >> 16) & 0xff);

    uint8_t tr_accz_l = (int((acc_z + 160) * 100)) & 0xff;
    uint8_t tr_accz_h = ((int((acc_z + 160) * 100)) >> 8) & 0xff;
    uint8_t tr_accy_l = (int((acc_y + 160) * 100)) & 0xff;
    uint8_t tr_accy_h = ((int((acc_y + 160) * 100)) >> 8) & 0xff;
    uint8_t tr_accx_l = (int((acc_x + 160) * 100)) & 0xff;
    uint8_t tr_accx_h = ((int((acc_x + 160) * 100)) >> 8) & 0xff;

    uint8_t tr_bat = int((bat_volt/12) * 255);

    uint8_t byteArr[] = {tr_temp, tr_press_l, tr_press_h, tr_alt_l, tr_alt_h, tr_gyrox_l, tr_gyrox_h, tr_gyroy_l, tr_gyroy_h, tr_gyroz_l, tr_gyroz_h,
                         tr_magx_l, tr_magx_h, tr_magy_l, tr_magy_h, tr_magz_l, tr_magz_h, tr_time_l, tr_time_m, tr_time_h,
                          tr_accz_l, tr_accz_h, tr_accy_l, tr_accy_h, tr_accx_l, tr_accx_h, tr_bat};
    
    write_to_flash_file(byteArr, 27);
}

void data_collector(){
  smartDelay(50);
  now_time = esp_timer_get_time();
  fc_time = fc_time + now_time - prev_time;
  float dt = float(now_time - prev_time)/1000000;
  if (bme_disable == false){
    temp_win_update();
    press_win_update();
    bme_alt = getAltitude(press_avg, press_initial, temp_initial);
    kalman_predict(dt);
    kalman_gain(1);
    kalman_estimate_update(1);
  }
  if (bno_disable == false){
    bno_raw_update();
    ang_x = (ang_x + gyro_x * dt);
    if (ang_x > 360) ang_x -= 360;
    else if (ang_x < 0) ang_x +=360;
    ang_y = (ang_y + gyro_y * dt);
    if (ang_y > 360) ang_y -= 360;
    else if (ang_y < 0) ang_y +=360;
    ang_z = (ang_z + gyro_z * dt);
    if (ang_z > 360) ang_z -= 360;
    else if (ang_z < 0) ang_z +=360;
    kalman_predict(dt);
    kalman_gain(3);
    kalman_estimate_update(3);
  }

  Pos_lat = gps.location.lat();
  Pos_long = gps.location.lng();

  if (gps.altitude.isUpdated()){
    gps_alt = gps.altitude.value();
    if (gps_init_count>0){
      gps_init_count--;
      initial_gps_alt = simple_low_pass(gps_alt, initial_gps_alt, 5);
      if (gps_init_count == 0) Serial.printf("Initial gps alt %f\n",initial_gps_alt);
    }
    else{
      kalman_predict(dt);
      kalman_gain(2);
      kalman_estimate_update(2);
    }
  }
  if (liftoff == true){
    fc_pref.begin("Abhyuday FC");
    fc_pref.putULong64(key_fc_time,fc_time);
    flash_countdown--;
    if (flash_countdown == 0){
      save_data();
      fc_pref.putInt(key_file_offset, file_offset);
      flash_countdown = FLASH_TIME;
    }
    fc_pref.end();
  }

  if ((liftoff == true) && (filtered_alt > maxalt)) {
    maxalt = filtered_alt;
    maxalt_time = fc_time;
    fc_pref.begin("Abhyuday FC");
    fc_pref.putFloat(key_maxalt, maxalt);
    fc_pref.putULong64(key_maxalt_time, maxalt_time);
    fc_pref.end();
  }
  get_channel_data();

  //Writing the flags to esp flash
  if ((liftoff == true) && (liftoff_mark == false)){
    fc_pref.begin("Abhyuday FC");
    fc_pref.putBool(key_liftoff, true);
    fc_pref.putFloat(key_press_initial, press_avg);
    fc_pref.putFloat(key_temp_initial, temp_avg);
    fc_pref.end();
    liftoff_mark = true;
  }
  if ((target == true) && (target_mark == false)){
    fc_pref.begin("Abhyuday FC");
    fc_pref.putBool(key_target, true);
    fc_pref.end();
    target_mark = true;
  }
  if ((apogee == true) && (apogee_mark == false)){
    fc_pref.begin("Abhyuday FC");
    fc_pref.putBool(key_apogee, true);
    fc_pref.end();
    apogee_mark = true;
  }
  if ((reef == true) && (reef_mark == false)){
    fc_pref.begin("Abhyuday FC");
    fc_pref.putBool(key_reef, true);
    fc_pref.end();
    reef_mark = true;
  }
  if ((landed == true) && (landed_mark == false)){
    fc_pref.begin("Abhyuday FC");
    fc_pref.putBool(key_landed, true);
    fc_pref.end();
    landed_mark = true;
  }

  prev_time = now_time;
}

void loop1(void* pvparameters){
  for (;;) data_collector();
}

void transmit_data(){
   //Compiles and transmits telemetry data
  
   //Check if there is any msg that has not been sent
   if (mymsg != 0x00){
    lora_transmit(&mymsg,1);
    mymsg = 0x00;
   }
   //Encode all data into two bytes and transmit
    uint8_t msg_type = 255;
    uint8_t tr_alt_l = (int(filtered_alt * 10 + 100)) & 0xff;
    uint8_t tr_alt_h = ((int(filtered_alt * 10 + 100)) >> 8) & 0xff;
    uint8_t tr_temp = (temp_avg - 12)/0.25;
    uint8_t tr_pitch_l = ((int(ang_x * 16)) & 0xff);
    uint8_t tr_pitch_h = (((int(ang_x * 16)) >> 8) & 0xff);
    uint8_t tr_roll_l = ((int(ang_y * 16)) & 0xff);
    uint8_t tr_roll_h = (((int(ang_y * 16)) >> 8) & 0xff);
    uint8_t tr_yaw_l = ((int(ang_z * 16)) & 0xff);
    uint8_t tr_yaw_h = (((int(ang_z * 16)) >> 8) & 0xff);

    uint8_t tr_accz_l = ((int((acc_z + 30) * 100)) & 0xff);
    uint8_t tr_accz_h = (((int((acc_z + 30) * 100)) >> 8) & 0xff);

    uint8_t tr_lat_deg, tr_lat_min, tr_lat_sec;
    DegMinSec(Pos_lat, &tr_lat_deg, &tr_lat_min, &tr_lat_sec);
    uint8_t tr_long_deg, tr_long_min, tr_long_sec;
    DegMinSec(Pos_long, &tr_long_deg, &tr_long_min, &tr_long_sec);

    uint8_t tr_bat = (int)((bat_volt/12)*255);
    uint8_t tr_ch1 = (int)((ch1_volt/12)*255);
    uint8_t tr_ch2 = (int)((ch2_volt/12)*255);

    uint8_t time_hour = fc_time/3600000000;
    uint8_t time_min = fc_time/60000000 - int(time_hour) * 60;
    uint8_t time_sec = fc_time/1000000 - int(time_hour) * 3600 - int(time_min) * 60; 

    uint8_t byteArr[] = {msg_type, tr_temp, tr_alt_l, tr_alt_h, tr_pitch_l, tr_pitch_h, tr_roll_l, tr_roll_h, tr_yaw_l, tr_yaw_h, tr_accz_l, tr_accz_h,
                         tr_lat_deg, tr_lat_min, tr_lat_sec, tr_long_deg, tr_long_min, tr_long_sec, time_hour, time_min, time_sec, tr_bat, tr_ch1, tr_ch2};
    
    lora_transmit(byteArr,24);
}

void loop2(void* pvparameters) {
  for (;;){
    while (block_core) delay(100);

    //Comment these serial print finally, it is causing some 400ms lag
    //Serial.printf("BME Pressure %f Temp %f Alt %f\n", press_avg, temp_avg, bme_alt);
    //Serial.printf("BNO Acc %f %f %f\n", acc_x, acc_y, acc_z);
    //Serial.printf("BNO Gyro %f %f %f\n", gyro_x, gyro_y, gyro_z);
    //Serial.printf("BNO Mag %f %f %f\n", mag_x, mag_y, mag_z);
    //Serial.printf("BNO Angle %f %f %f\n", ang_x, ang_y, ang_z);
    //Serial.printf("GPS Lat %f Long %f Alt %f\n", Pos_lat, Pos_long, gps_alt);
    //Serial.printf("FC Time %d\n",fc_time);
    //Serial.printf("Filtered Alt %f\n\n", filtered_alt);

    //Flight Logic
    //The values i think is valid for the final flight is written in comments
    //at the side of the if conditions, might change the values in if conditions
    //for testing
    if (liftoff == false){
      //Check for liftoff
      if (((filtered_alt > 2) && (acc_z > 20)) || (filtered_alt > 5)){ // 5, 60 , 35
        liftoff = true;
        mymsg = 0x03;
        Serial.println("Liftoff detected");
        digitalWrite(LED1, HIGH);
        tone(BUZZER, 261);
        indicator_reset = 2;
        //Shifting the fc time to be zero at liftoff
        fc_time = 0;
        now_time = esp_timer_get_time();
        prev_time = now_time;
      }
    }
    else {
      if ((target == false) && (filtered_alt > 10)){ //3047
        //Target altitude of 10000ft
        target = true;
        mymsg = 0x04;
        Serial.println("Target reached");
        digitalWrite(LED2, HIGH);
        tone(BUZZER, 392);
        indicator_reset = 2;
      }
    
      if (apogee == false){
        //Apogee logic
        //After 5000ft (except for timer)
        //A decrease in altitude from maximum by at least 25m for 1 second(which is 2 loop executions)
        //Or a decrease in altitude by at least 100m
        //A check timer comparing with simulated time(has a positive offset to consider variations)
        //Simulated time till apogee is 23s
        if ((filtered_alt > 5) && ((maxalt - filtered_alt) > 1.5)) apogee_detect_timer--; // 1500,25
        else apogee_detect_timer = 2;

        if (((filtered_alt > 5) && ((maxalt - filtered_alt) > 3)) || (fc_time > 120000000)) apogee_detect_timer = 0; //1500, 100, 33000000

        if (apogee_detect_timer == 0){
          apogee = true;
          mymsg = 0x05;
          Serial.println("Apogee detected");
          digitalWrite(LED3, HIGH);
          tone(BUZZER, 523);
          digitalWrite(CH1, HIGH);
          indicator_reset = 2;
        }
      }
      else {
        //Reefing altitude check
        if (reef == false){
          if ((filtered_alt < 3) || (fc_time > 200000000)){ //300, 165000000
            reef = true;
            mymsg = 0x06;
            Serial.println("Reefing initialised");
            digitalWrite(LED2, HIGH);
            digitalWrite(LED1, HIGH);
            tone(BUZZER, 784);
            digitalWrite(CH2, HIGH);
            indicator_reset = 2;
          }
        }
        else {
          //landed check
          if (landed == false){
            //Take a window of 10 samples, find altitudes and find their std deviation
            //If it is less than 1m, landed.
            landed_count--;
            landed_window[landed_count] = filtered_alt;
            if (landed_count == 0){
              if (variance(landed_window, 10) < 1){
                landed = true;
                mymsg = 0x07;
                Serial.println("Landed");
                digitalWrite(LED1, HIGH);
                digitalWrite(LED2, HIGH);
                tone(BUZZER, 1046);
                indicator_reset = 2;
              }
              else landed_count = 10;
            }
          }
          if (landed == true){
            if (land_beep_count == 0) land_beep_count = 10;
            land_beep_count--;
            if (land_beep_count == 2){
              tone(BUZZER, 1046);
              indicator_reset = 2;
            }
          }
        }
      } 
    }
    if (indicator_reset > -1) indicator_reset--;
    if (indicator_reset == 0){
      noTone(BUZZER);
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,LOW);
      digitalWrite(CH1,LOW);
      digitalWrite(CH2,LOW);
    }
    transmit_data();
    delay(400);
  }
}

void loop(){}
