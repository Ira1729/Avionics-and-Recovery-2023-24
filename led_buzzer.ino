#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define Lpin1 9
#define Lpin2 12
#define Lpin3 11
#define Lpin4 10
#define buzzer 7
#define WINDOW_SIZE 15
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP280 bmp;

float alt;
int INDEX = 0;
float VALUE = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float Altitude;
float PrevAltitude;
int status=0;
int timer=0;
int ptimer=0;
void setup() {
	Serial.begin(9600);
  pinMode(Lpin1 , OUTPUT);
  pinMode(Lpin2 , OUTPUT);
  pinMode(Lpin3 , OUTPUT);
  pinMode(Lpin4 , OUTPUT);
  pinMode(buzzer , OUTPUT);

	if (!bmp.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
}
void loop() {
  if (status==0){
    for (int i=0;i<WINDOW_SIZE;i++){
      alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);      
      READINGS[i] = alt;           
      SUM = SUM + alt;                 
      INDEX = (INDEX+1) % WINDOW_SIZE;
    }
    PrevAltitude = SUM / WINDOW_SIZE; 
    digitalWrite(Lpin1,HIGH);
    digitalWrite(Lpin2,HIGH);
    digitalWrite(Lpin3,HIGH);
    delay(1000);
    digitalWrite(Lpin1,LOW);
    digitalWrite(Lpin2,LOW);
    digitalWrite(Lpin3,LOW);
    status=1; 
  }
  alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  SUM = SUM - READINGS[INDEX];       
  VALUE = alt;     
  READINGS[INDEX] = VALUE;           
  SUM = SUM + VALUE;                 
  INDEX = (INDEX+1) % WINDOW_SIZE; 

  Altitude = SUM / WINDOW_SIZE;      

  if (ptimer==0){
  Serial.print(VALUE);
  Serial.print(",");
  Serial.println(Altitude);


	Serial.print("Temperature = ");
	Serial.print(bmp.readTemperature());
	Serial.println("*C");

	Serial.print("Pressure = ");
	Serial.print(bmp.readPressure() / 100.0F);
	Serial.println("hPa\n");
   ptimer=100;
  }

 if((Altitude>PrevAltitude+1)&&(status==1)){
    digitalWrite(Lpin1,HIGH);
    tone(buzzer,3000,1000);
    timer=90;
    status = 2;
 }
 else if (status==2){
  if (timer==0){
    digitalWrite(Lpin1,LOW);
    noTone(buzzer);
    status=3;
  }
  timer--;
 }
 else if ((status==3)&&(Altitude>=3048)){
  digitalWrite(Lpin2,HIGH);
  tone(buzzer,1000,1000);
  timer=45;
  status = 4;
 }
 else if ((status == 4)){
  if (timer==0){
    digitalWrite(Lpin2,LOW);
    noTone(buzzer);
    status=5;
  }
  timer--;
 }
 else if (((status==3)||(status==5))&&(Altitude<PrevAltitude-1)){
  digitalWrite(Lpin3,HIGH);
  tone(buzzer,1000,1000);
  timer=90;
  status = 6;
 }
 else if ((status == 6)){
  if (timer==0){
    digitalWrite(Lpin3,LOW);
    noTone(buzzer);
    status=7;
  }
  timer--;
 }
 else if ((status==7)&&(Altitude<304.8)){
  digitalWrite(Lpin4,HIGH);
  tone(buzzer,1000,1000);
  timer=90;
  status = 8;
 }
 else if ((status == 8)){
  if (timer==0){
    digitalWrite(Lpin4,LOW);
    noTone(buzzer);
    status=9;
  }
  timer--;
 }
 else if ((status==9)&&((Altitude-PrevAltitude<1)||(-Altitude+PrevAltitude<1))){
  tone(buzzer,500,1000);
 }
	delay(10);
  ptimer--;
}