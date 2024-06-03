//Clears the esp flash and spi flash

#include <Arduino.h>
#include <SerialFlash.h>
#include <Preferences.h>

#define FLASH_CS 33
#define FLASH_WP 2
#define FLASH_HOLD 15
SPIClass FC_SPI(HSPI);

Preferences fc_pref;

void setup() {
  Serial.begin(9600);

  pinMode(FLASH_HOLD,OUTPUT);
  pinMode(FLASH_WP,OUTPUT);
  digitalWrite(FLASH_HOLD,HIGH);
  digitalWrite(FLASH_WP,HIGH);

  while (!SerialFlash.begin(FC_SPI, FLASH_CS)) {
    Serial.println("Unable to access SPI Flash chip");
    delay(100);
  }
  Serial.println("Erasing SPI Flash");
  SerialFlash.eraseAll();
  while (SerialFlash.ready() == false) {
   Serial.println("Waiting");
   delay(1000);
  }
  Serial.println("SPI Flash Erased");

  fc_pref.begin("Abhyuday FC");
  fc_pref.clear();
  delay(1000);
  fc_pref.putUChar("flt_cnt", 0);
  char mykey[15];
  for (int i=0; i<4;i++){
    snprintf(mykey, sizeof(mykey), "landed%d", i);
    fc_pref.putBool(mykey, false);
    snprintf(mykey, sizeof(mykey), "liftoff%d", i);
    fc_pref.putBool(mykey, false);
    snprintf(mykey, sizeof(mykey), "apogee%d", i);
    fc_pref.putBool(mykey, false);
    snprintf(mykey, sizeof(mykey), "target%d", i);
    fc_pref.putBool(mykey, false);
    snprintf(mykey, sizeof(mykey), "reef%d", i);
    fc_pref.putBool(mykey, false);
    snprintf(mykey, sizeof(mykey), "maxalt%d", i);
    fc_pref.putFloat(mykey, 0);
    snprintf(mykey, sizeof(mykey), "maxalt_t%d", i);
    fc_pref.putULong64(mykey, 0);
    snprintf(mykey, sizeof(mykey), "file_off%d", i);
    fc_pref.putInt(mykey, 0);
    snprintf(mykey, sizeof(mykey), "p_init%d", i);
    fc_pref.putFloat(mykey, 0);
    snprintf(mykey, sizeof(mykey), "t_init%d", i);
    fc_pref.putFloat(mykey, 0);
    snprintf(mykey, sizeof(mykey), "fc_time%d", i);
    fc_pref.putULong64(mykey, 0);
  }
  Serial.println("Done");
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

