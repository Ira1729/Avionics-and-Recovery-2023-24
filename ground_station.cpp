//Receiver Code test
#include <Arduino.h>
#include <CRC8.h>

#define M0 23
#define M1 22
#define AUX 21
#define TX 19
#define RX 18

#define ARRLEN 24

HardwareSerial SerialAT(1);
CRC8 crc;
bool start_received=false;
uint8_t msg_type=0;
uint8_t exp_len=0;
bool valid_msg=false;
uint8_t byte_arr[ARRLEN];


void setup() {  
  Serial.begin(9600);
  pinMode(M0,OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(AUX,INPUT);
  digitalWrite(M0,HIGH);
  digitalWrite(M1,HIGH);
  pinMode(2,OUTPUT);

  SerialAT.begin(9600,SERIAL_8N1,RX,TX);
  delay(1000);
  uint8_t config[]={0xC2,0x00,0x01,0x18,0x45,0x44};//The 2nd and 3rd byte is address
  SerialAT.write(config,6);
  delay(1000);
  digitalWrite(M0,LOW);
  digitalWrite(M1,LOW);
  delay(1000);
  Serial.println("Initialsing Done");
}


void loop() {  
  if (SerialAT.available()){
    uint8_t mybyte=SerialAT.read();
    if (start_received==false && mybyte == 0x01) {
      //Check if we received the start byte 0x01
      start_received=true;
      msg_type=0;
      crc.restart();
    }
    else if (start_received==true){
      if (msg_type!=0 && exp_len==0){
        //The last byte is crc check value, check if it matches
        if (mybyte==crc.getCRC()) valid_msg=true;
        else Serial.println("CRC Error");
        start_received=false;
      }
      else crc.add(mybyte);

      if (msg_type==0){
        //The byte after start byte is the type of msg, if it is 255,it is data, else it is a message
        if (mybyte==255) {
          msg_type=255;
          exp_len=ARRLEN-1;
        }
        else{
          msg_type=mybyte;
        }
      }
      else if (msg_type==255 && valid_msg==false){
        byte_arr[ARRLEN-exp_len]=mybyte;
        exp_len--;
      }
    }
  }
  if (valid_msg==true && msg_type==255){
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
  else if (valid_msg==true){
    switch (msg_type)
    {
    case 0x02:
      Serial.println("FC Started");
      break;
    case 0x03:
      Serial.println("Liftoff detected");
      break;
    case 0x04:
      Serial.println("Target reached");
      break;
    case 0x05:
      Serial.println("Apogee Detected");
      break;
    case 0x06:
      Serial.println("Reefing initialised");
      break;
    case 0x07:
      Serial.println("Landed");
      break;
    case 0x10:
      Serial.println("BME Error Restarting");
      break;
    default:
      Serial.println("Random msg?");
      break;
    }
    msg_type=0x00;
    valid_msg=false;
  } 
}
