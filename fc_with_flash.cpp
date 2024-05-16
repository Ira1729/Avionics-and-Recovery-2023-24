#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <CRC8.h>
#include <Preferences.h>

//Initialisations
//SPI Flash
#define FLASH_CS 33
#define FLASH_WP 2
#define FLASH_HOLD 15

SPIClass FC_SPI(HSPI);
SPISettings FC_SPI_Set(1000000,MSBFIRST,SPI_MODE3);
int flash_addr=0x000010;
#define F_TIME 2
int flash_countdown=2;

bool myflashbegin();
void myflash_readID(uint8_t *buf);
void myflash_writebyte(uint32_t addr,uint8_t mybyte);
void myflash_write_enable();
bool myflash_isbusy();
uint8_t myflash_readbyte(uint32_t addr);
void save_data();

//ESP Flash
Preferences fc_pref;

//GPS
#define GPS_TX 16
#define GPS_RX 17

float Pos_lat,Pos_long,Pos_alt;
uint8_t Time_hour,Time_min,Time_sec;
SoftwareSerial GPS_SoftSerial(GPS_RX,GPS_TX); 
TinyGPSPlus gps;

void get_gps_data();
static void smartDelay(unsigned long ms);
void DegMinSec(float tot_val,uint8_t* degree, uint8_t* minutes, uint8_t* seconds);

//BNO
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float Wx;float Wy;float Wz;
float Pitch; float Roll; float Yaw;
float Acc_body_z;

void get_bno_data();

//BME
#define SEALEVELPRESSURE_HPA (1013.25)
#define WINDOW_SIZE 15
Adafruit_BME280 bme;
int INDEX = 0;
float VALUE = 0;float SUM = 0;
float READINGS[WINDOW_SIZE];float Altitude;
float Temp;
float initAlt=0;
int alt_init_count=WINDOW_SIZE*2;

void get_bme_data();

//LORA
#define LORA_M0 32 
#define LORA_M1 25 
#define LORA_TX 26 
#define LORA_RX 35 

SoftwareSerial SerialLORA(LORA_RX,LORA_TX);
uint8_t mymsg=0x00;
CRC8 crc;
#define T_TIME 5
uint8_t transmit_countdown=T_TIME;

void transmit_data();
void lora_transmit(uint8_t arr[],uint8_t len);

//Message IDs
//0x02 FC Started
//0x03 Liftoff
//0x04 Target Reached
//0x05 Apogee Detected
//0x06 Reefing
//0x07 Landed
//0x10 BME Error Restarting


//Indicators
#define LED1 5
#define LED2 19
#define LED3 18
#define LED4 2
#define BUZZER 23
int8_t indicator_reset=-1;

void print_data();


//Flight Stages
#define T_ALT 20
#define R_ALT 10
#define LIFTOFF_ALT 10
bool liftoff=false;
bool landed=false;
bool apogee=false;
bool target=false;
bool reef=false;
float maxalt=0; 

void check_flight_stage();

//Channels
#define CH1 27
#define TEST_CH1 36
#define CH2 4
#define TEST_CH2 39
#define BATTERY 0
int apo_timer=1;
int reef_timer=1;
float ch1_volt=0;
float ch2_volt=0;
float bat_volt=0;
#define CH_ON_TIME 100

void get_channel_data();
	

void setup() {
  //Indicators
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(LED4,OUTPUT);
  pinMode(BUZZER,OUTPUT);
  ledcAttachPin(BUZZER,0);
  Serial.begin(9600);

  //Channels
  pinMode(CH1,OUTPUT);
  pinMode(TEST_CH1,INPUT);
  pinMode(CH2,OUTPUT);
  pinMode(TEST_CH2,INPUT);
  pinMode(BATTERY,INPUT);
  digitalWrite(CH1,LOW);
  digitalWrite(CH2,LOW);

  //BME
  if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		delay(100);
    ESP.restart();
	}

  //ESP Flash
  fc_pref.begin("Abhyuday FC",false);
  liftoff=fc_pref.getBool("liftoff",false);
  landed=fc_pref.getBool("landed",false);
  apogee=fc_pref.getBool("apogee",false);
  target=fc_pref.getBool("target",false);
  reef=fc_pref.getBool("reef",false);
  maxalt=fc_pref.getFloat("maxalt",0);
  flash_addr=fc_pref.getInt("flashaddr",0x000010);
  if (liftoff==true){
    initAlt=fc_pref.getFloat("initAlt",0);
    alt_init_count=-1;
    for (int i=0;i<WINDOW_SIZE;i++) get_bme_data();
  }

  //BNO
  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  //LORA
  pinMode(LORA_M0,OUTPUT);
  pinMode(LORA_M1,OUTPUT);
  pinMode(LORA_RX,INPUT);
  pinMode(LORA_TX,OUTPUT);
  SerialLORA.begin(9600,SWSERIAL_8N1);
  digitalWrite(LORA_M0,HIGH);
  digitalWrite(LORA_M1,HIGH);
  delay(100);
  uint8_t config[]={0xC2,0x00,0x01,0x18,0x45,0x47};//The 2nd and 3rd byte is address
  SerialLORA.write(config,6);
  delay(100);
  digitalWrite(LORA_M0,LOW);
  digitalWrite(LORA_M1,LOW);
  delay(100);
  
  //GPS
  GPS_SoftSerial.begin(9600);

  //Flash SPI
  FC_SPI.begin();
  pinMode(FLASH_HOLD,OUTPUT);
  pinMode(FLASH_WP,OUTPUT);
  digitalWrite(FLASH_HOLD,HIGH);
  digitalWrite(FLASH_WP,HIGH);

  while(!myflashbegin()){
    Serial.println("Didnt get spi flash");
    delay(1000);
  }

  //Start indication
  digitalWrite(LED4,HIGH);
  digitalWrite(LED3,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED1,HIGH);
  tone(BUZZER,131);
  delay(200);
  digitalWrite(LED4,LOW);
  digitalWrite(LED3,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED1,LOW);
  noTone(BUZZER);
  mymsg=0x02;

  Serial.println(myflash_readbyte(0));
  Serial.println(myflash_readbyte(0x10));
  myflash_writebyte(0,1);
}

void loop() {
  smartDelay(200);
  
  get_gps_data();
  get_bme_data();
  get_bno_data();
  get_channel_data();

  //Initialising the initial altitude
  if (alt_init_count>=0) alt_init_count--;
  if (alt_init_count==1){
    initAlt=Altitude;
    fc_pref.putFloat("initAlt",initAlt);
  }
  if (alt_init_count==-1) check_flight_stage();

  print_data();

  transmit_countdown--;
  if (transmit_countdown==0){
    transmit_data();
    transmit_countdown=T_TIME;
  }

  flash_countdown--;
  if ((flash_countdown==0)){
    save_data();
    fc_pref.putInt("flashaddr",flash_addr);
    flash_countdown=F_TIME;
  }
  
  if ((apogee==true) && (apo_timer>-CH_ON_TIME)){
    apo_timer--;
    if (apo_timer==0) digitalWrite(CH1,HIGH);
    if (apo_timer==-CH_ON_TIME) digitalWrite(CH1,LOW);
  }
  if ((reef==true) && (reef_timer>-CH_ON_TIME)){
    reef_timer--;
    if (reef_timer==0) digitalWrite(CH2,HIGH);
    if (reef_timer==-CH_ON_TIME) digitalWrite(CH2,LOW);
  }

  if (indicator_reset>0){
    indicator_reset--;
    if (indicator_reset==0){
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,LOW);
      digitalWrite(LED4,LOW);
      noTone(BUZZER);
    }
  }
}

//Function Definitions

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do { 
    //Encode data read from GPS while data is available on serial port
    while (GPS_SoftSerial.available())
      gps.encode(GPS_SoftSerial.read());
  //Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it 
  } while (millis() - start < ms);
}

void get_bme_data(){
  //moving average filter for altitude
  VALUE = bme.readAltitude(SEALEVELPRESSURE_HPA);
  if (isnanf(VALUE)){
    for (int i=0;i<5;i++){
      VALUE = bme.readAltitude(SEALEVELPRESSURE_HPA);
      if (!isnanf(VALUE)) break;
      if (i==4) {
        mymsg=0x10;
        transmit_data();
        ESP.restart();
      }
    }
  }
  SUM = SUM - READINGS[INDEX];          
  READINGS[INDEX] = VALUE;           
  SUM = SUM + VALUE;                 
  INDEX = (INDEX+1) % WINDOW_SIZE; 

  Altitude = (SUM / WINDOW_SIZE);  
  Temp = bme.readTemperature();
}

void get_bno_data(){
  // Absolute orientation
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Pitch = (euler.y());
  Roll=(euler.z());
  Yaw = euler.x();
  // Angular velocity 
  imu::Vector<3> angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Wz=angular_vel.z();

  imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Acc_body_z=linacc.z();
}

void get_gps_data(){
  //If invalid returns a value indicating that
  Pos_lat = gps.location.lat();	/* Get latitude data */
  Pos_long = gps.location.lng();	/* Get longtitude data */
  Pos_alt = gps.altitude.meters();
  Time_hour = gps.time.hour();
  Time_min = gps.time.minute();
  Time_sec = gps.time.second();
}

void get_channel_data(){
  ch1_volt=analogRead(TEST_CH1)*13.3/(3.3*4096.0);
  ch2_volt=analogRead(TEST_CH2)*13.3/(3.3*4096.0);
  bat_volt=analogRead(BATTERY)*13.3/(3.3*4096.0);
}

void print_data(){
  Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%f,%f\n",initAlt,Altitude,Pitch,Roll,Yaw,Wz,Acc_body_z,Pos_lat,Pos_long,Pos_alt,Time_hour,Time_min,Time_sec,Temp,bat_volt,ch1_volt,ch2_volt);
}

void transmit_data(){
   //Compiles and transmits telemetry data
  
   //Check if there is any msg that has not been sent
   if (mymsg!=0x00){
    lora_transmit(&mymsg,1);
    mymsg=0x00;
   }
   //Encode all data into two bytes and transmit
    uint8_t msg_type=255;
    uint8_t tr_alt_l=((int)(Altitude*10+100)&0xff);
    uint8_t tr_alt_h=(((int)(Altitude*10+100)>>8)&0xff);
    uint8_t tr_temp = (Temp-12)/0.25;
    uint8_t tr_pitch_l=(((int)((Pitch+180)*16))&0xff);
    uint8_t tr_pitch_h=((((int)((Pitch+180)*16)>>8))&0xff);
    uint8_t tr_roll_l=(((int)((Roll+180)*16))&0xff);
    uint8_t tr_roll_h=((((int)((Roll+180)*16+180))>>8)&0xff);
    uint8_t tr_yaw_l=(((int)(Yaw*16))&0xff);
    uint8_t tr_yaw_h=((((int)(Yaw*16)>>8))&0xff);

    uint8_t tr_accz_l=(((int)((Acc_body_z+30)*100))&0xff);
    uint8_t tr_accz_h=((((int)((Acc_body_z+30)*100))>>8)&0xff);

    uint8_t tr_lat_deg,tr_lat_min,tr_lat_sec;
    DegMinSec(Pos_lat,&tr_lat_deg,&tr_lat_min,&tr_lat_sec);
    uint8_t tr_long_deg,tr_long_min,tr_long_sec;
    DegMinSec(Pos_long,&tr_long_deg,&tr_long_min,&tr_long_sec);

    uint8_t tr_bat = (int)((bat_volt/12)*255);
    uint8_t tr_ch1 = (int)((ch1_volt/12)*255);
    uint8_t tr_ch2 = (int)((ch2_volt/12)*255);

    uint8_t byteArr[] = {msg_type,tr_temp, tr_alt_l,tr_alt_h,tr_pitch_l,tr_pitch_h,tr_roll_l,tr_roll_h,tr_yaw_l,tr_yaw_h,tr_accz_l,tr_accz_h,tr_lat_deg,tr_lat_min,tr_lat_sec,tr_long_deg,tr_long_min,tr_long_sec,Time_hour,Time_min,Time_sec,tr_bat,tr_ch1,tr_ch2};
    
    lora_transmit(byteArr,24);
}

void DegMinSec(float tot_val,uint8_t* degree, uint8_t* minutes, uint8_t* seconds){
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

void lora_transmit(uint8_t arr[],uint8_t len){
  //Sends an array via lora, adds crc and start byte
  crc.restart();
  SerialLORA.write(0x01);//Start byte
  for (int i=0;i<len;i++){
    crc.add(arr[i]);
    SerialLORA.write(arr[i]);
  }
  SerialLORA.write(crc.getCRC());
}

void check_flight_stage(){
  //To check the different stages of flight according to altitude, also triggers indicators for testing
  float checkalt=Altitude-initAlt;
  if (liftoff==false){
      if (checkalt> LIFTOFF_ALT){
        liftoff=true;
        fc_pref.putBool("liftoff",liftoff);
        digitalWrite(LED3,HIGH);
        Serial.println("liftoff");
        tone(BUZZER,130);
        maxalt=checkalt;
        indicator_reset=10;
        mymsg=0x03;
      } 
  }
  else{
    if (landed==false){
      if (checkalt>maxalt){
        maxalt=checkalt;
        fc_pref.putFloat("maxalt",maxalt);
      }
      if ((target==false)&&(checkalt>T_ALT)){
        target=true;
        fc_pref.putBool("target",target);
        digitalWrite(LED2,HIGH);
        Serial.println("Target");
        tone(BUZZER,147);
        indicator_reset=10;
        mymsg=0x04;
      }
      else if ((apogee==false)&&(maxalt-checkalt>3)){
        apogee=true;
        fc_pref.putBool("apogee",apogee);
        digitalWrite(LED1,HIGH);
        Serial.println("apogee");
        tone(BUZZER,165);
        indicator_reset=10;
        mymsg=0x05;
        }
      else if ((reef==false)&&(apogee==true)&&(checkalt<R_ALT)){
        reef=true;
        fc_pref.putBool("reef",reef);
        digitalWrite(LED1,HIGH);
        digitalWrite(LED2,HIGH);
        Serial.println("reefing");
        tone(BUZZER,174);
        indicator_reset=10;
        mymsg=0x06;
      }
      else if ((landed==false)&&(checkalt)<0.5){
        landed=true;
        fc_pref.putBool("landed",landed);
        digitalWrite(LED2,HIGH);
        digitalWrite(LED3,HIGH);
        Serial.println("landed");
        tone(BUZZER,196);
        indicator_reset=10;
        mymsg=0x07;
      }
    }
  }
}

//Flash Functions
bool myflashbegin(){
  uint8_t id[5];
  uint8_t f;
  uint32_t size;
  FC_SPI.begin();
  pinMode(FLASH_CS,OUTPUT);
  digitalWrite(FLASH_CS,HIGH);

  myflash_readID(id);
  if ((id[0]==0 && id[1]==0 && id[2]==0) || (id[0]==255 && id[1]==255 && id[2]==255)) {
		return false;
	}
	f = 0;
  Serial.printf("SPI Flash %d %d %d\n",id[0],id[1],id[2]);
  return true;
}

void myflash_readID(uint8_t *buf){
  while (myflash_isbusy()) {
		delayMicroseconds(10);
	}
  FC_SPI.beginTransaction(FC_SPI_Set);
  delayMicroseconds(10);
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x9F);
	buf[0] = FC_SPI.transfer(0); // manufacturer ID
	buf[1] = FC_SPI.transfer(0); // memory type
	buf[2] = FC_SPI.transfer(0); // capacity
	digitalWrite(FLASH_CS,HIGH);
	FC_SPI.endTransaction();
} 

bool myflash_isbusy(){
  uint8_t status_reg;
  FC_SPI.beginTransaction(FC_SPI_Set);
  delayMicroseconds(1);
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x05);
  status_reg = FC_SPI.transfer(0);
  digitalWrite(FLASH_CS,HIGH);
  FC_SPI.endTransaction();
  return status_reg & 1;
}

void myflash_writebyte(uint32_t addr,uint8_t mybyte){
  myflash_write_enable();
  delayMicroseconds(10);
  while (myflash_isbusy()) {
		delayMicroseconds(10);
	}
  FC_SPI.beginTransaction(FC_SPI_Set);  
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x02);
  //24bit address
  FC_SPI.transfer((uint8_t)((addr >> 16) & 255));
  FC_SPI.transfer((uint8_t)((addr >> 8) & 255));
  FC_SPI.transfer((uint8_t)((addr) & 255));
  //Data
  FC_SPI.transfer(mybyte);
  digitalWrite(FLASH_CS,HIGH);
  FC_SPI.endTransaction();
}

void myflash_write_enable(){
  while (myflash_isbusy()) {
		delayMicroseconds(10);
	}
  FC_SPI.beginTransaction(FC_SPI_Set);
  delayMicroseconds(1);
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x06);
  digitalWrite(FLASH_CS,HIGH);
  FC_SPI.endTransaction();
}

uint8_t myflash_readbyte(uint32_t addr){
  while (myflash_isbusy()) {
		delayMicroseconds(10);
	}
  FC_SPI.beginTransaction(FC_SPI_Set);

  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x03);
  //24bit address
  FC_SPI.transfer((uint8_t)((addr >> 16) & 255));
  FC_SPI.transfer((uint8_t)((addr >> 8) & 255));
  FC_SPI.transfer((uint8_t)((addr) & 255));
  //Data
  uint8_t mybyte = FC_SPI.transfer(0);
  digitalWrite(FLASH_CS,HIGH);
  FC_SPI.endTransaction();
  return mybyte;
}

void myflash_write_data(uint8_t arr[],uint8_t len){
  //Writes data to flash, adds crc and start byte
  crc.restart();
  myflash_writebyte(flash_addr++,0x01);//Start byte
  delayMicroseconds(2);
  for (int i=0;i<len;i++){
    crc.add(arr[i]);
    myflash_writebyte(flash_addr++,arr[i]);
    delayMicroseconds(2);
  }
  myflash_writebyte(flash_addr++,crc.getCRC());
  delayMicroseconds(2);
}

void save_data(){
   //Compiles and saves data
  
   //Encode all data into two bytes and save
    uint8_t tr_alt_l=((int)(Altitude*10+100)&0xff);
    uint8_t tr_alt_h=(((int)(Altitude*10+100)>>8)&0xff);
    uint8_t tr_temp = (Temp-12)/0.25;
    uint8_t tr_pitch_l=(((int)((Pitch+180)*16))&0xff);
    uint8_t tr_pitch_h=((((int)((Pitch+180)*16)>>8))&0xff);
    uint8_t tr_roll_l=(((int)((Roll+180)*16))&0xff);
    uint8_t tr_roll_h=((((int)((Roll+180)*16+180))>>8)&0xff);
    uint8_t tr_yaw_l=(((int)(Yaw*16))&0xff);
    uint8_t tr_yaw_h=((((int)(Yaw*16)>>8))&0xff);

    uint8_t tr_accz_l=(((int)((Acc_body_z+30)*100))&0xff);
    uint8_t tr_accz_h=((((int)((Acc_body_z+30)*100))>>8)&0xff);

    uint8_t tr_lat_deg,tr_lat_min,tr_lat_sec;
    DegMinSec(Pos_lat,&tr_lat_deg,&tr_lat_min,&tr_lat_sec);
    uint8_t tr_long_deg,tr_long_min,tr_long_sec;
    DegMinSec(Pos_long,&tr_long_deg,&tr_long_min,&tr_long_sec);

    uint8_t tr_bat = (int)((bat_volt/12)*255);
    uint8_t tr_ch1 = (int)((ch1_volt/12)*255);
    uint8_t tr_ch2 = (int)((ch2_volt/12)*255);

    uint8_t byteArr[] = {tr_temp, tr_alt_l,tr_alt_h,tr_pitch_l,tr_pitch_h,tr_roll_l,tr_roll_h,tr_yaw_l,tr_yaw_h,tr_accz_l,tr_accz_h,tr_lat_deg,tr_lat_min,tr_lat_sec,tr_long_deg,tr_long_min,tr_long_sec,Time_hour,Time_min,Time_sec,tr_bat,tr_ch1,tr_ch2};
    
    myflash_write_data(byteArr,23);
}
