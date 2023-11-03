//SDA pins are set to 21
//SCL pins are set to 22

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Gps 
volatile float minutes, seconds;
volatile int degree, secs, mins; 
SoftwareSerial GPS_SoftSerial(16, 17); //16 and 17 are GPS pins
int lat_degree;int lat_mins;int lat_secs;
int long_degree;int long_mins;int long_secs;

//Moving average variables for bme
TinyGPSPlus gps;		
#define SEALEVELPRESSURE_HPA (1013.25)
#define WINDOW_SIZE 15
Adafruit_BME280 bme;float alt;int INDEX = 0;
float VALUE = 0;float SUM = 0;
float READINGS[WINDOW_SIZE];float Altitude;
float temp;


// For bno 
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float wx;float wy;float wz;
float pitch; float roll;
float acc_body_z;

//String generation and setup
String s_alt=""; String s_temp=""; String s_accz=""; String s_wz=""; String s_pitch="";String s_roll="";  
String s_lat_degree=""; String s_lat_mins=""; String s_lat_secs=""; 
String s_long_degree=""; String s_long_mins=""; String s_long_secs="";


void setup() {
	Serial.begin(9600);
  GPS_SoftSerial.begin(9600);

	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    
  bno.setExtCrystalUse(true);

}
void loop() 
{ smartDelay(1000);
  String output="";
  unsigned long start;
  double lat_val, lng_val, alt_m_val;
  bool loc_valid, alt_valid, time_valid;
  lat_val = gps.location.lat();	/* Get latitude data */
  loc_valid = gps.location.isValid();	/* Check if valid location data is available */
  lng_val = gps.location.lng();	/* Get longtitude data */

  //moving average filter
  alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  SUM = SUM - READINGS[INDEX];       
  VALUE = alt;     
  READINGS[INDEX] = VALUE;           
  SUM = SUM + VALUE;                 
  INDEX = (INDEX+1) % WINDOW_SIZE; 

  //Altitude
  Altitude = SUM / WINDOW_SIZE;   
  s_alt = String(Altitude);

  // Absolute orientation
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = abs((euler.y()));
  roll=abs((euler.z()-180));
  if(pitch > 180){
    pitch = 360-pitch;
  }
  s_pitch = String(pitch);
  if(roll > 180){
    roll = 360-roll;
  }
  s_roll = String(roll);
  // Angular velocity 
  imu::Vector<3> angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  wz=angular_vel.z();
  s_wz = String(wz);

  imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  acc_body_z=linacc.z();
  s_accz=String(acc_body_z);

  //Latitude data
  DegMinSec(lat_val);
  lat_degree=degree;
  lat_mins=mins;
  lat_secs=secs;

  s_lat_degree=String(lat_degree);
  s_lat_mins=String(lat_mins);
  s_lat_secs=String(lat_secs);


  //Longitudnal data
  DegMinSec(lng_val);	
  long_degree=degree;
  long_mins=mins;
  long_secs=secs;

  s_long_degree=String(long_degree);
  s_long_mins=String(long_mins);
  s_long_secs=String(long_secs);

  temp = bme.readTemperature();
  s_temp = String(temp);
  output = s_alt + "," + s_pitch + "," + s_roll + "," + s_wz + "," + s_accz + "," + s_lat_degree + "," + s_lat_mins + "," + s_lat_secs + "," + s_long_degree + "," + s_long_mins + "," + s_long_secs + "," + s_temp;

  Serial.println(output);
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

void DegMinSec(double tot_val)		/* Convert data in decimal degrees into degrees minutes seconds form */
{  
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;  
}