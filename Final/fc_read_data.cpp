//To read the spi flash and esp flash
//Upload this code and then run the python script to save all the data in pc

#include <Arduino.h>
#include <SerialFlash.h>
#include <CRC.h>
#include <Preferences.h>

CRC8 crc;
Preferences fc_pref;
uint8_t flight_count;
float maxalt; 
char key_maxalt[15];
uint64_t maxalt_time;
char key_maxalt_time[15];
char key_file_offset[15];
float press_initial;
char key_press_initial[15];
float temp_initial;
char key_temp_initial[15];

#define FLASH_CS 33
#define FLASH_WP 2
#define FLASH_HOLD 15
SPIClass FC_SPI(HSPI);
SerialFlashFile file;
char file_name[20];
int file_size;
#define DATA_BLOCK_SIZE 27
uint8_t data[DATA_BLOCK_SIZE+1];


uint8_t flight_index;
bool read_data = false;
int data_count;

float temp_avg, press_avg, filtered_alt;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
uint64_t time_100ms;
float acc_z, acc_y, acc_x;
float bat_volt;

int crc_counts = 0;


void setup() {
  Serial.begin(9600);
  fc_pref.begin("Abhyuday FC");
  flight_count = fc_pref.getUChar("flt_cnt", 0);

  pinMode(FLASH_HOLD,OUTPUT);
  pinMode(FLASH_WP,OUTPUT);
  digitalWrite(FLASH_HOLD,HIGH);
  digitalWrite(FLASH_WP,LOW);

  while (!SerialFlash.begin(FC_SPI, FLASH_CS)) {
    Serial.println("Unable to access SPI Flash chip");
    delay(100);
  }

  //The below line is to trigger the start of data transfer
  while(true){
    if (Serial.available()){
      if (Serial.read() == 'A') break;
    }
  }
  Serial.printf("No. of flights: %d\n\n", flight_count);
  delay(500);
  flight_index = 0;
}

void loop() {
  if (flight_index < flight_count){
    if (read_data == false){
      //Read first the details from ESP flash
      snprintf(key_file_offset, sizeof(key_file_offset), "file_off%d", flight_index);
      file_size = fc_pref.getInt(key_file_offset, 0);
      snprintf(key_maxalt, sizeof(key_maxalt), "maxalt%d", flight_index);
      maxalt = fc_pref.getFloat(key_maxalt, 0);
      snprintf(key_maxalt_time, sizeof(key_maxalt_time), "maxalt_t%d", flight_index);
      maxalt_time = fc_pref.getULong64(key_maxalt_time, 0);
      snprintf(key_press_initial, sizeof(key_press_initial), "p_init%d", flight_index);
      press_initial = fc_pref.getFloat(key_press_initial, 0);
      snprintf(key_temp_initial, sizeof(key_temp_initial), "t_init%d", flight_index);
      temp_initial = fc_pref.getFloat(key_temp_initial, 0);
      Serial.printf("Flight %d\n",flight_index);
      Serial.printf("%f,%f,%f,%lld\n\n",press_initial,temp_initial,maxalt,maxalt_time);
      
      snprintf(file_name, sizeof(file_name), "flight_%d", flight_index);
      file = SerialFlash.open(file_name);
      if (!file) {
        Serial.println("Failed to open file for reading.");
        flight_index++;
      }
      else{
        read_data = true;
        data_count = 0;
      }
      delay(100);
    }
    else {
      //Read each data array
      // Read the data block and its CRC
      file.read(data, DATA_BLOCK_SIZE + 1);
      data_count += DATA_BLOCK_SIZE + 1;
      crc.restart();
      crc.add(data, DATA_BLOCK_SIZE);
      if (crc.calc() == data[DATA_BLOCK_SIZE]){
        //CRC Check satisfied
        //Now decode all these data
        temp_avg = data[0] * 0.25 + 12;
        press_avg = ((data[1] | (data[2] << 8)) + 50000);
        filtered_alt = (((data[3] | (data[4] << 8)) - 100) / 10.0);

        gyro_x = ((data[5] | (data[6] << 8)) / 16.0) - 2000;
        gyro_y = ((data[7] | (data[8] << 8)) / 16.0) - 2000;
        gyro_z = ((data[9] | (data[10] << 8)) / 16.0) - 2000;

        mag_x = ((data[11] | (data[12] << 8)) / 16.0) - 1300;
        mag_y = ((data[13] | (data[14] << 8)) / 16.0) - 1300;
        mag_z = ((data[15] | (data[16] << 8)) / 16.0) - 2500;

        time_100ms = (data[17] | (data[18] << 8) | (data[19] << 16));

        acc_z = ((data[20] | (data[21] << 8)) / 100.0) - 160;
        acc_y = ((data[22] | (data[23] << 8)) / 100.0) - 160;
        acc_x = ((data[24] | (data[25] << 8)) / 100.0) - 160;

        bat_volt = data[26] * 12.0 / 255.0;

        Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lld\n",temp_avg, press_avg, filtered_alt, gyro_x, gyro_y, gyro_z, 
                      mag_x, mag_y, mag_z, acc_x, acc_y, acc_z, bat_volt, time_100ms * 100);
      }
      else crc_counts++;
      

      if (data_count >= file_size){
          file.close();
        read_data = false;
        flight_index++;
        Serial.println();
      }
    }
  }

  delay(50);
}

