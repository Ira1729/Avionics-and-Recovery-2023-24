#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <RadioLib.h>

//PIN Defs
#define GPS_TX 16
#define GPS_RX 17
#define LORA_CS 26
#define LORA_DIO 35
#define LORA_RST 25
#define LORA_BUSY 32
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

//Moving average variables for bme	
#define SEALEVELPRESSURE_HPA (1013.25)
#define WINDOW_SIZE 15
Adafruit_BME280 bme;
int INDEX = 0;
float VALUE = 0;float SUM = 0;
float READINGS[WINDOW_SIZE];float Altitude;
float Temp;
float initAlt=0;


// For bno 
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float Wx;float Wy;float Wz;
float Pitch; float Roll; float Yaw;
float Acc_body_z;

//For LORA
SPIClass FC_SPI(HSPI);
SPISettings FC_SPI_Set(10000000,MSBFIRST,SPI_MODE0);
SX1262 radio = new Module(LORA_CS, LORA_DIO, LORA_RST, LORA_BUSY,FC_SPI,FC_SPI_Set);
#define LORA_FREQ 865.0F
#define LORA_BW 31.0F
#define LORA_SF 11U
#define LORA_CR 7U
#define LORA_SW 18U
#define LORA_POW 22
bool sending=false;

// flag to indicate that a packet was sent or received
volatile bool receivedFlag = false;

// Keeps count of how many packets transmitted.
int count = 0;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}


void setup() {
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(LED4,OUTPUT);
  pinMode(BUZZER,OUTPUT);

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

  int state = radio.begin(LORA_FREQ,LORA_BW,LORA_SF,LORA_CR,LORA_SW,LORA_POW);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("LORA success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    //while (true);
  } 

  delay(1000);
  bno.setExtCrystalUse(true);

  //Setting the initial altitude
  for (int i=0;i<WINDOW_SIZE;i++) get_bme_data();
  initAlt=Altitude;

  //Initial Beep
  digitalWrite(LED4,HIGH);
  tone(BUZZER,1000);
  delay(1000);
  digitalWrite(LED4,LOW);
  noTone(BUZZER);
}

void loop() {
  smartDelay(100);
  
  get_gps_data();
  get_bme_data();
  get_bno_data(); 
  
  print_data();
  //transmit_data();

  if (liftoff==false){
    if (Altitude> LIFTOFF_ALT){
      liftoff=true;
      digitalWrite(LED3,HIGH);
      Serial.println("liftoff");
      tone(BUZZER,130);
      maxalt=Altitude;
      indicator_reset=10;
    } 
  }
  else{
    if (landed==false){
      if (Altitude>maxalt) maxalt=Altitude;
      if ((target==false)&&(Altitude>T_ALT)){
        target=true;
        digitalWrite(LED2,HIGH);
        Serial.println("Target");
        tone(BUZZER,147);
        indicator_reset=10;
      }
      else if ((apogee==false)&&(maxalt-Altitude>3)){
        apogee=true;
        digitalWrite(LED1,HIGH);
        Serial.println("apogee");
        tone(BUZZER,165);
        indicator_reset=10;
      }
      else if ((reef==false)&&(apogee==true)&&(Altitude<R_ALT)){
        reef=true;
        digitalWrite(LED1,HIGH);
        digitalWrite(LED2,HIGH);
        Serial.println("reefing");
        tone(BUZZER,174);
        indicator_reset=10;
      }
      else if ((landed==false)&&(Altitude)<0.5){
        landed=true;
        digitalWrite(LED2,HIGH);
        digitalWrite(LED3,HIGH);
        Serial.println("landed");
        tone(BUZZER,196);
        indicator_reset=10;
      }
    }
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


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())	/* Encode data read from GPS while data is available on serial port */
      gps.encode(GPS_SoftSerial.read());
/* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}

void get_bme_data(){
  //moving average filter for altitude
  VALUE = bme.readAltitude(SEALEVELPRESSURE_HPA);
  SUM = SUM - READINGS[INDEX];          
  READINGS[INDEX] = VALUE;           
  SUM = SUM + VALUE;                 
  INDEX = (INDEX+1) % WINDOW_SIZE; 

  //Altitude
  Altitude = (SUM / WINDOW_SIZE) -initAlt;  

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
  Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d\n",Altitude,Pitch,Roll,Yaw,Wz,Acc_body_z,Pos_lat,Pos_long,Pos_alt,Time_hour,Time_min,Time_sec);
}

void transmit_data(){
   //Encode all data into two bytes and transmit
    uint8_t tr_alt_l=(int(Altitude+100)&0xff);
    uint8_t tr_alt_h=((int(Altitude+100)>>8)&0xff);

    uint8_t tr_pitch_l=(int(Pitch*16)&0xff);
    uint8_t tr_pitch_h=((int(Pitch*16)>>8)&0xff);
    uint8_t tr_roll_l=(int(Roll*16)&0xff);
    uint8_t tr_roll_h=((int(Roll*16)>>8)&0xff);
    uint8_t tr_yaw_l=(int(Yaw*16)&0xff);
    uint8_t tr_yaw_h=((int(Yaw*16)>>8)&0xff);

    uint8_t tr_accz_l=(int(Acc_body_z*100)&0xff);
    uint8_t tr_accz_h=((int(Acc_body_z*100)>>8)&0xff);

    uint8_t tr_lat_deg,tr_lat_min,tr_lat_sec;
    DegMinSec(Pos_lat,&tr_lat_deg,&tr_lat_min,&tr_lat_sec);
    uint8_t tr_long_deg,tr_long_min,tr_long_sec;
    DegMinSec(Pos_long,&tr_long_deg,&tr_long_min,&tr_long_sec);


    //radio.transmit("Hello");
    uint8_t byteArr[] = {tr_alt_l, tr_alt_h, tr_pitch_l,tr_pitch_h,tr_roll_l,tr_roll_h,tr_yaw_l,tr_yaw_h,tr_accz_l,tr_accz_h,tr_lat_deg,tr_lat_min,tr_lat_sec,tr_long_deg,tr_long_min,tr_long_sec,Time_hour,Time_min,Time_sec};
    
    if (digitalRead(LORA_DIO)) sending=false;

    if (sending==true) Serial.println("Busy transmission");
    else{
    int state = radio.startTransmit(byteArr, 19);
    sending=true;
  digitalWrite(LED1,HIGH);
  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success in transmission!"));
    Serial.println(radio.getDataRate());
    digitalWrite(LED1,LOW); 
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("timeout!"));
  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }}

}

void DegMinSec(float tot_val,uint8_t* degree, uint8_t* minutes, uint8_t* seconds)		/* Convert data in decimal degrees into degrees minutes seconds form */
{  
  float deg,min,sec;
  deg = (int)tot_val;
  min = tot_val - deg;
  sec = 60 * min;
  min = (int) (sec);
  sec = sec - min;
  sec = (int) (60*sec);
  *degree=(uint8_t)deg;
  *minutes=(uint8_t)min;
  *seconds=(uint8_t)sec; 
}