//Rough FC Code
//Need to include spi flash code

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


//PIN Defs
#define GPS_TX 16
#define GPS_RX 17

#define LORA_M0 32 
#define LORA_M1 25 //rst
#define LORA_TX 26 //nss
#define LORA_RX 35 //d100

#define LED1 5
#define LED2 19
#define LED3 18
#define LED4 2
#define BUZZER 23

//Rest all are default i2c and spi

//Function templates
static void smartDelay(unsigned long ms);
void get_bme_data();
void get_bno_data();
void get_gps_data();
void print_data();
void DegMinSec(float tot_val,uint8_t* degree, uint8_t* minutes, uint8_t* seconds);
void transmit_data();
void lora_transmit(uint8_t arr[],uint8_t len);
void check_flight_stage();

//stage flags
#define T_ALT 5
#define R_ALT 2
#define LIFTOFF_ALT 3
bool liftoff=false;
bool landed=false;
bool apogee=false;
bool target=false;
bool reef=false;
float maxalt=0;
int8_t indicator_reset=-1; 

// Gps 
float Pos_lat,Pos_long,Pos_alt;
uint8_t Time_hour,Time_min,Time_sec;
SoftwareSerial GPS_SoftSerial(GPS_RX,GPS_TX); 
TinyGPSPlus gps;	

//For bme	
#define SEALEVELPRESSURE_HPA (1013.25)
#define WINDOW_SIZE 15
Adafruit_BME280 bme;
int INDEX = 0;
float VALUE = 0;float SUM = 0;
float READINGS[WINDOW_SIZE];float Altitude;
float Temp;
float initAlt=0;
int alt_init_count=WINDOW_SIZE*2;


// For bno 
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float Wx;float Wy;float Wz;
float Pitch; float Roll; float Yaw;
float Acc_body_z;

//For SPIFlash
SPIClass FC_SPI(HSPI);
SPISettings FC_SPI_Set(10000000,MSBFIRST,SPI_MODE0);

//For LORA
SoftwareSerial SerialLORA(LORA_RX,LORA_TX);
uint8_t mymsg=0x00;
CRC8 crc;
#define T_TIME 5
uint8_t transmit_countdown=T_TIME;

void setup() {
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(LED4,OUTPUT);
  pinMode(BUZZER,OUTPUT);
  ledcAttachPin(BUZZER,0);

  Serial.begin(9600);

  GPS_SoftSerial.begin(9600);
  FC_SPI.begin();

	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  //Setting up LORA for transmission
  pinMode(LORA_M0,OUTPUT);
  pinMode(LORA_M1,OUTPUT);
  pinMode(LORA_RX,INPUT);
  pinMode(LORA_TX,OUTPUT);
  SerialLORA.begin(9600,SWSERIAL_8N1);
  digitalWrite(LORA_M0,HIGH);
  digitalWrite(LORA_M1,HIGH);
  delay(1000);
  uint8_t config[]={0xC2,0x00,0x01,0x18,0x45,0x44};//The 2nd and 3rd byte is address
  SerialLORA.write(config,6);
  delay(1000);
  digitalWrite(LORA_M0,LOW);
  digitalWrite(LORA_M1,LOW);
  delay(1000);
  

  //Start indication
  digitalWrite(LED4,HIGH);
  tone(BUZZER,1000);
  delay(500);
  digitalWrite(LED4,LOW);
  noTone(BUZZER);
  mymsg=0x02;
}

void loop() {
  smartDelay(200);
  
  get_gps_data();
  get_bme_data();
  get_bno_data();

  //Initialising the initial altitude
  if (alt_init_count>=0) alt_init_count--;
  if (alt_init_count==1){
    initAlt=Altitude;
  }
  if (alt_init_count==-1) check_flight_stage();

  print_data();
  transmit_countdown--;
  if (transmit_countdown==0){
    transmit_data();
    transmit_countdown=T_TIME;
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

void print_data(){
  Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f\n",initAlt,Altitude,Pitch,Roll,Yaw,Wz,Acc_body_z,Pos_lat,Pos_long,Pos_alt,Time_hour,Time_min,Time_sec,Temp);
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


    //radio.transmit("Hello");
    uint8_t byteArr[] = {msg_type,tr_temp, tr_alt_l,tr_alt_h,tr_pitch_l,tr_pitch_h,tr_roll_l,tr_roll_h,tr_yaw_l,tr_yaw_h,tr_accz_l,tr_accz_h,tr_lat_deg,tr_lat_min,tr_lat_sec,tr_long_deg,tr_long_min,tr_long_sec,Time_hour,Time_min,Time_sec};
    
    lora_transmit(byteArr,21);
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
      if (checkalt>maxalt) maxalt=checkalt;
      if ((target==false)&&(checkalt>T_ALT)){
        target=true;
        digitalWrite(LED2,HIGH);
        Serial.println("Target");
        tone(BUZZER,147);
        indicator_reset=10;
        mymsg=0x04;
      }
      else if ((apogee==false)&&(maxalt-checkalt>3)){
        apogee=true;
        digitalWrite(LED1,HIGH);
        Serial.println("apogee");
        tone(BUZZER,165);
        indicator_reset=10;
        mymsg=0x05;
        }
      else if ((reef==false)&&(apogee==true)&&(checkalt<R_ALT)){
        reef=true;
        digitalWrite(LED1,HIGH);
        digitalWrite(LED2,HIGH);
        Serial.println("reefing");
        tone(BUZZER,174);
        indicator_reset=10;
        mymsg=0x06;
      }
      else if ((landed==false)&&(checkalt)<0.5){
        landed=true;
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
