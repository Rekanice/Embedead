#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define S_TO_uS_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

// Save Data on RTC Memories
RTC_DATA_ATTR int wakeCount = 0;

// Method to print the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup(){

  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  //Increment wake number and print it every reboot
  ++wakeCount;
  Serial.println("WakeUp number: " + String(wakeCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  // First, configure the wake up source. Set our ESP32 to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * S_TO_uS_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  // Decide what all peripherals to shut down/keep on
  /*
  By default, ESP32 will automatically power down the peripherals not needed by the wakeup source, 
  but if you want to be a poweruser, this is for you. 
  
  Read in detail at the API docs
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html#power-down-of-rtc-peripherals-and-memories
  
  Left the line below commented, as an example of how to configure peripherals.
  It turns off all RTC peripherals in deep sleep.
  */
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  // Serial.println("Configured all RTC Peripherals to be powered down in sleep");


  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  // esp goes to sleep
  esp_deep_sleep_start();

  Serial.println("This will never be printed");
}

void loop(){
  //This is not going to be called
}
