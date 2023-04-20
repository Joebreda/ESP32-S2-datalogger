/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <string.h>
//#include <WiFi.h>
//#include "AdafruitIO_WiFi.h"
#include <Adafruit_BME280.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_LTR329_LTR303.h"
#include "RTClib.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

//#define PRINT 0
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

RTC_PCF8523 rtc;
Adafruit_LTR329 ltr = Adafruit_LTR329();
Adafruit_TMP117  tmp117;
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char timestamp[30];
char row[50];
char filenametimestamp[30];
RTC_DATA_ATTR int file_created;
RTC_DATA_ATTR int wake_cycle_counter;
RTC_DATA_ATTR char filename[99]; // Preserves the string.
RTC_DATA_ATTR char batch_rows[500]; // Preserves the string until it is written.
char string_write[16];
int ltr_valid = 1;
int tmp_valid = 1;
int bme_valid = 1;
int sd_valid = 1;

void writeFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_WRITE);
    #ifdef PRINT
      Serial.printf("Writing file: %s\n", path);
      if(!file){
          Serial.println("Failed to open file for writing");
          return;
      }
    #endif
    bool written = file.print(message);
    #ifdef PRINT
      if(written){
          Serial.println("File written");
      } else {
          Serial.println("Write failed");
      }
    #endif
    file.close();
}
void appendFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_APPEND);
    #ifdef PRINT
      Serial.printf("Appending to file: %s\n", path);  
    #endif
    if(!file){
        #ifdef PRINT
          Serial.println("Failed to open file for appending");
        #endif
        return;
    }
    bool written = file.print(message);
    #ifdef PRINT
      if(written){
        Serial.println("Message appended");
      } else {
          Serial.println("Append failed");
      }
    #endif
    file.close();
}


void SoSBlink(char* error) {
  while (1) {
    #ifdef PRINT
      Serial.println(error);
      Serial.flush();
    #endif
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void setup() {
  #ifdef PRINT 
    Serial.begin(115200);
    Serial.flush();
    delay(100);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("ESP Board MAC Address:  ");
    //Serial.println(WiFi.macAddress());
  #endif

  #ifdef PRINT
    int count = 1;
    while(count) {
      delay(1000);
      Serial.println("stalling...");
      count = count - 1;    
    }
  #endif  

  // Initialize SD.
  if(!SD.begin(10)){
      // Serial.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
      // Serial.println("No SD card attached");
      return;
  }

  
  // Flags to handle if peripherals didn't init.
  ltr_valid = 1;
  tmp_valid = 1;
  bme_valid = 1;
  sd_valid = 1;
  // Initialize Peripherals.
  if ( ! rtc.begin() ) {
    //SoSBlink("Couldn't find RTC!");
  }
  if ( ! Wire.begin() ) {
    //SoSBlink("Couldn't find Wire!");
  }
  if ( ! ltr.begin() ) {
    //SoSBlink("Couldn't find LTR sensor!");
    ltr_valid = 0;
  }
  if ( !tmp117.begin() ) {
    //SoSBlink("Failed to find TMP117 chip!");
    tmp_valid = 0;
  }
  if ( !bme.begin() ) {
    //SoSBlink("Failed to find BME280 Builtin Sensor!");
    bme_valid = 0;
  }
  
  // Start RTC.
  if (! rtc.initialized() || rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  if (!file_created) {
    // undo previous configuration and start a 1Hz timer to send interrupts.
    rtc.deconfigureAllTimers(); 
    //rtc.enableSecondTimer();
    rtc.enableCountdownTimer(PCF8523_FrequencySecond, 5); 
    rtc.start();
  }
  
  // Calibrate Light Sensor.
  if (ltr_valid) {
    ltr.setGain(LTR3XX_GAIN_2);
    ltr.setIntegrationTime(LTR3XX_INTEGTIME_100);
    ltr.setMeasurementRate(LTR3XX_MEASRATE_200);
  }

  // Wake up code.
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF); // Sets to hibernate mode rather than deep sleep.
  //esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  

  gpio_pullup_en(GPIO_NUM_9); // configure as GPIO pull up.
  rtc_gpio_pullup_en(GPIO_NUM_9); // ?
  rtc_gpio_pulldown_dis(GPIO_NUM_9); // ?
  //rtc_gpio_hold_en(GPIO_NUM_9); // holds current position while sleeping.
  rtc_gpio_init(GPIO_NUM_9); // configure for analog signal?
  rtc_gpio_set_direction(GPIO_NUM_9, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_set_direction_in_sleep(GPIO_NUM_9, RTC_GPIO_MODE_INPUT_ONLY);

  //esp_sleep_enable_timer_wakeup(10e6);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_9, 0);

  // Set all other GPIO to low to save power and prevent environmental disturbance.
  gpio_pulldown_en(GPIO_NUM_18);
  gpio_pulldown_en(GPIO_NUM_17);
  gpio_pulldown_en(GPIO_NUM_16);
  gpio_pulldown_en(GPIO_NUM_15);
  gpio_pulldown_en(GPIO_NUM_14);
  gpio_pulldown_en(GPIO_NUM_8);
  gpio_pulldown_en(GPIO_NUM_36);
  gpio_pulldown_en(GPIO_NUM_35);
  gpio_pulldown_en(GPIO_NUM_37);
  gpio_pulldown_en(GPIO_NUM_38);
  gpio_pulldown_en(GPIO_NUM_39);

  gpio_pulldown_en(GPIO_NUM_13);
  gpio_pulldown_en(GPIO_NUM_12);
  gpio_pulldown_en(GPIO_NUM_11);
  gpio_pulldown_en(GPIO_NUM_10);
  gpio_pulldown_en(GPIO_NUM_6);
  gpio_pulldown_en(GPIO_NUM_5);

  gpio_pullup_dis(GPIO_NUM_18);
  gpio_pullup_dis(GPIO_NUM_17);
  gpio_pullup_dis(GPIO_NUM_16);
  gpio_pullup_dis(GPIO_NUM_15);
  gpio_pullup_dis(GPIO_NUM_14);
  gpio_pullup_dis(GPIO_NUM_8);
  gpio_pullup_dis(GPIO_NUM_36);
  gpio_pullup_dis(GPIO_NUM_35);
  gpio_pullup_dis(GPIO_NUM_37);
  gpio_pullup_dis(GPIO_NUM_38);
  gpio_pullup_dis(GPIO_NUM_39);

  gpio_pullup_dis(GPIO_NUM_13);
  gpio_pullup_dis(GPIO_NUM_12);
  gpio_pullup_dis(GPIO_NUM_11);
  gpio_pullup_dis(GPIO_NUM_10);
  gpio_pullup_dis(GPIO_NUM_6);
  gpio_pullup_dis(GPIO_NUM_5);
  
  //gpio_pulldown_en(GPIO_NUM_4);
  //gpio_pulldown_en(GPIO_NUM_3);

  // Initialize a file to write to and set deep sleep configurations only once.
  DateTime now = rtc.now();
  if (!file_created) {
    sprintf(filenametimestamp, "%02d-%02d-%02d-%02d-%02d", now.month(), now.day(), now.hour(), now.minute(), now.second());
    strcpy(filename, "/");
    strcat (filename, filenametimestamp);
    strcat (filename, "_HVACdata.txt");
    writeFile(SD, filename, "timestamp,temp,pressure,humidity,visible_plus_ir,ir\n");
    file_created = 1;
    wake_cycle_counter = 10;
  }
}

void loop() {
  wake_cycle_counter = wake_cycle_counter - 1;
  //io.run();
  bool valid;
  uint16_t visible_plus_ir, infrared;
  sensors_event_t temp_event, pressure_event, humidity_event;

  DateTime now = rtc.now();
  

  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  if (ltr_valid) {
    if (ltr.newDataAvailable()) {
      valid = ltr.readBothChannels(visible_plus_ir, infrared);
    }
  }

  sensors_event_t tmp117_temp; // create an empty event to be filled
  if (tmp_valid) {
    tmp117.getEvent(&tmp117_temp); //fill the empty event object with the current measurements
  }

  // feed->save(bme.temperature);


  // Store a few rows in RAM before writting once every 5 ticks.  

  // Populates a row with all data from this cycle.
  sprintf(row, "%02d-%02d-%02d-%02d-%02d,%.2f,%.2f,%.2f,%d,%d\n", now.month(), now.day(), now.hour(), now.minute(), now.second(), tmp117_temp.temperature, pressure_event.pressure, humidity_event.relative_humidity, visible_plus_ir, infrared);
  strcat(batch_rows, row);
  // Every 10th wake cycle, append all 10 rows and reset buffer and wake counter.
  if (!wake_cycle_counter) {
    appendFile(SD, filename, batch_rows);
    strcpy(batch_rows, "");
    wake_cycle_counter = 10;
  }


  /*
  // Write data to SD card.
  sprintf(timestamp, "%02hhu:%02hhu:%02hhu", now.hour(), now.minute(), now.second());
  appendFile(SD, filename, timestamp);
  appendFile(SD, filename, ",");
  appendFile(SD, filename, dtostrf(tmp117_temp.temperature, 6, 2, string_write));
  appendFile(SD, filename, ",");
  appendFile(SD, filename, dtostrf(pressure_event.pressure, 6, 2, string_write));
  appendFile(SD, filename, ",");
  appendFile(SD, filename, dtostrf(humidity_event.relative_humidity, 6, 2, string_write));
  appendFile(SD, filename, ",");
  appendFile(SD, filename, itoa(visible_plus_ir, string_write, 10));
  appendFile(SD, filename, ",");
  appendFile(SD, filename, itoa(infrared, string_write, 10));
  appendFile(SD, filename, "\n");
  */


  #ifdef PRINT
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    // RTC Timestamp.
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    // Light Sensor Data.
    if (valid) {
      Serial.print("CH0 Visible + IR: ");
      Serial.print(visible_plus_ir);
      Serial.print("\t\tCH1 Infrared: ");
      Serial.println(infrared);
    }
    // TMP117 data.
    Serial.print("Temperature  "); Serial.print(tmp117_temp.temperature);Serial.println(" degrees C");
    Serial.println("");
    // BME280 Data.
    //Serial.print(F("Temperature = "));
    //Serial.print(temp_event.temperature);
    //Serial.println(" *C");
    Serial.print(F("Humidity = "));
    Serial.print(humidity_event.relative_humidity);
    Serial.println(" %");
    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");
    // Triggering Sleep Now.
    Serial.println("Going to sleep.");
    Serial.flush();
  #endif


  //delay(10000);
  esp_deep_sleep_start();  
}