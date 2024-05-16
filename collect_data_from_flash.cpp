//This collects data from flash and can also erase and reset the FC
#include <Arduino.h>
#include <SPI.h>
#include <CRC8.h>
#include <Preferences.h>

#define FLASH_CS 33
#define FLASH_WP 2
#define FLASH_HOLD 15

SPIClass FC_SPI(HSPI);
SPISettings FC_SPI_Set(1000000,MSBFIRST,SPI_MODE3);
int myaddr=0x000010;
#define MAXEMPTY 1000
int empty_countdown=MAXEMPTY;
#define ARRLEN 24

CRC8 crc;
bool start_received=false;
uint8_t exp_len=0;
bool valid_msg=false;
uint8_t byte_arr[ARRLEN];

Preferences fc_pref;


bool myflashbegin();
void myflash_readID(uint8_t *buf);
void myflash_writebyte(uint32_t addr,uint8_t mybyte);
void myflash_eraseall();
uint8_t myflash_readbyte(uint32_t addr);
void myflash_readsr();
void myflash_write_enable();
bool myflash_isbusy();
void myflash_sector_erase(uint32_t addr);


void setup(){
  Serial.begin(9600);
  fc_pref.begin("Abhyuday FC");
  pinMode(FLASH_HOLD,OUTPUT);
  pinMode(FLASH_WP,OUTPUT);
  
  digitalWrite(FLASH_HOLD,HIGH);
  digitalWrite(FLASH_WP,HIGH);

  while(!myflashbegin()){
    Serial.println("Didnt get..");
    delay(1000);
  }
  delay(1000);
  Serial.println(myflash_readbyte(0));
}

void loop(){
  uint8_t mybyte=myflash_readbyte(myaddr++);
  //Serial.println(mybyte);
  if (mybyte!=0x01){
    empty_countdown--;
    if (empty_countdown==0){
      Serial.println("Over");

      //Careful-----
      //The below code erases the memory of flash, comment it when you dont want to clear
      myflash_eraseall();
      while (myflash_isbusy()) {
		    delay(1000);
        Serial.println("Waiting..");
	    }
      fc_pref.clear();
      Serial.println("Erase Done");
      //Erasing code ends here
      while(1);
    }
  }
  else empty_countdown=MAXEMPTY;

  if (1){
    if (start_received==false && mybyte == 0x01) {
      //Check if we received the start byte 0x01
      start_received=true;
      exp_len=ARRLEN-1;
      crc.restart();
    }
    else if (start_received==true){
      if (exp_len==0){
        //The last byte is crc check value, check if it matches
        if (mybyte==crc.getCRC()) valid_msg=true;
        else Serial.println("CRC Error");
        start_received=false;
      }
      else crc.add(mybyte);

      if (valid_msg==false){
        byte_arr[ARRLEN-exp_len]=mybyte;
        exp_len--;
      }
    }
  }
  if (valid_msg==true){
    float temp=byte_arr[1]*0.25+12;
    float alt= ((((int)byte_arr[3])<<8)+(int)byte_arr[2]-100)/10.0;
    float pitch= ((((int)byte_arr[5])<<8)+(int)byte_arr[4])/16.0 -180;
    float roll= ((((int)byte_arr[7])<<8)+(int)byte_arr[6])/16.0 -180;
    float yaw= ((((int)byte_arr[9])<<8)+(int)byte_arr[8])/16.0;
    float accz= ((((int)byte_arr[11])<<8)+(int)byte_arr[10])/100.0-30;
    uint8_t lat_deg=byte_arr[12];
    uint8_t lat_min=byte_arr[13];
    uint8_t lat_sec=byte_arr[14];
    uint8_t long_deg=byte_arr[15];
    uint8_t long_min=byte_arr[16];
    uint8_t long_sec=byte_arr[17];
    uint8_t time_hour=byte_arr[18];
    uint8_t time_min=byte_arr[19];
    uint8_t time_sec=byte_arr[20];
    float bat_volt=(byte_arr[21]/255.0)*12;
    float ch1_volt=(byte_arr[22]/255.0)*12;
    float ch2_volt=(byte_arr[23]/255.0)*12;

    Serial.printf("%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f\n",temp,alt,pitch,roll,yaw,accz,lat_deg,lat_min,lat_sec,long_deg,long_min,long_sec,time_hour,time_min,time_sec,bat_volt,ch1_volt,ch2_volt);
    valid_msg=false;
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

void myflash_eraseall(){
  myflash_write_enable();
  delayMicroseconds(1);

  while (myflash_isbusy()) {
		delayMicroseconds(10);
	}
  FC_SPI.beginTransaction(FC_SPI_Set);
  delayMicroseconds(10);
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0xC7); //Erase
  digitalWrite(FLASH_CS,HIGH);
  FC_SPI.endTransaction();
}

void myflash_sector_erase(uint32_t addr){
  while (myflash_isbusy()) {
		delayMicroseconds(10);
	}
  FC_SPI.beginTransaction(FC_SPI_Set);

  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x20);
  //24bit address of sector
  FC_SPI.transfer((uint8_t)((addr >> 16) & 255));
  FC_SPI.transfer((uint8_t)((addr >> 8) & 255));
  FC_SPI.transfer((uint8_t)((addr) & 255));
  
  digitalWrite(FLASH_CS,HIGH);
  FC_SPI.endTransaction();
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

void myflash_readsr(){
  Serial.println("Reading Status Registers");
  FC_SPI.beginTransaction(FC_SPI_Set);
  delayMicroseconds(10);
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x05);
  Serial.println(FC_SPI.transfer(0));
  digitalWrite(FLASH_CS,HIGH);

  /*
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x35);
  delayMicroseconds(1);
  Serial.println(FC_SPI.transfer(0));
  digitalWrite(FLASH_CS,HIGH);
  delayMicroseconds(1);

  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x15);
  delayMicroseconds(1);
  Serial.println(FC_SPI.transfer(0));
  digitalWrite(FLASH_CS,HIGH);
  */
  FC_SPI.endTransaction();
}

void myflash_write_enable(){
  while (myflash_isbusy()) {
		delayMicroseconds(10);
	}
  FC_SPI.beginTransaction(FC_SPI_Set);
  delayMicroseconds(10);
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x06);
  digitalWrite(FLASH_CS,HIGH);
  FC_SPI.endTransaction();
}

void myflash_reset(){
  while (myflash_isbusy()) {
		delayMicroseconds(10);
	}
  FC_SPI.beginTransaction(FC_SPI_Set);
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x66);
  digitalWrite(FLASH_CS,HIGH);
  digitalWrite(FLASH_CS,LOW);
  FC_SPI.transfer(0x99);
  digitalWrite(FLASH_CS,HIGH);
  FC_SPI.endTransaction();
}
