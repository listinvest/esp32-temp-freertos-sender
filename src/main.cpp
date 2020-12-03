#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <config.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <config.h>

SemaphoreHandle_t wifisetup;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
float temp;
bool tempReady = false;
float bat;
bool batReady = false;
//ESP-NOW receiver MAC address
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//Local wifi SSID
// constexpr char WIFI_SSID[] = "";


void wifi_setup(void *pvParameters);
void send_data(void *pvParameters);

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

typedef struct struct_message {
    int id;
    float temp=0;
    float bat;
    bool error;
} struct_message;

struct_message myData;

void go_sleep(int time, char* message){
    esp_sleep_enable_timer_wakeup(time*uS_TO_S_FACTOR);
    Serial.println(message);
    esp_deep_sleep_start();
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println(millis());
}

void setup_sensor(void * some)
{
  Serial.println(millis());
  sensors.begin();
  vTaskDelay(100/portTICK_PERIOD_MS);
  if (!sensors.getAddress(insideThermometer, 0)){ 
    myData.error=true;
    tempReady=true;
    vTaskDelete(NULL);
  }
  sensors.setResolution(insideThermometer, 11);
  sensors.requestTemperatures();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  temp = sensors.getTempC(insideThermometer);
  tempReady = true;
  Serial.printf("Temp data ready: %d \n",millis());
  digitalWrite(SW_PIN,LOW);
  vTaskDelete(NULL);
}


void battery_read(void * some)
{   
     vTaskDelay(500/portTICK_PERIOD_MS);
    //read battery voltage per %
    long sum = 0;                  // sum of samples taken
    float voltage = 0.0;           // calculated voltage
    float output = 0.0;            //output value
    const float battery_max = 3.3; //maximum voltage of battery
    const float battery_min = 3.1; //minimum voltage of battery before shutdown

    float R1 = 10.0; // resistance of R1 (10)
    float R2 = 100.0;  // resistance of R2 (100)

    for (int i = 0; i < 500; i++)
    {
        sum += adc1_get_voltage(ADC1_CHANNEL_5);
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
    voltage = sum / (float)500;
    voltage = (voltage * 3.3) / 4096.0; //for internal 1.1v reference
    // use if added divider circuit
    voltage = voltage / (R2/(R1+R2));
    //round value by two precision
    voltage = roundf(voltage * 100) / 100;
    bat=voltage;
    Serial.printf("Voltage: %f \ttime: %d\n",bat,millis());
    batReady = true;
    vTaskDelete(NULL);
}

void watch_dog (void * some){
  while (millis()<5000){
    taskYIELD();
  }
  digitalWrite(SW_PIN, LOW);
    go_sleep(SLEEP_INTERVAL,"Something wrong");
    vTaskDelete(NULL);
}

void get_readings(void * some){
  pinMode(SW_PIN, OUTPUT);
  digitalWrite(SW_PIN, HIGH);
  xTaskCreate(battery_read,"battery Read",2048,NULL,1,NULL);
  xTaskCreate(setup_sensor,"sensor settup",2048,NULL,4000,NULL);
  vTaskDelete(NULL);
}

void send_data(void *pvParameters){
  xSemaphoreTake(wifisetup, portMAX_DELAY);
  while(1){
  if ( tempReady & batReady ){
  myData.id=ID;
  myData.temp=temp;
  myData.bat=bat;
  Serial.printf("\nTemp: %f\nBat: %f\nError: %d\n",myData.temp,myData.bat,myData.error);

  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  Serial.println(millis());
  xSemaphoreGive(wifisetup);
  go_sleep(SLEEP_INTERVAL,"Data Sent");
  vTaskDelete(NULL);
  }
  vTaskDelay(10);
  }
}

esp_now_peer_info_t peerInfo;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

  // put your setup code here, to run once:
    
  btStop();
// put your setup code here, to run once:
Serial.begin(115200);
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_11db);
  Serial.println(millis());
  wifisetup = xSemaphoreCreateMutex();
  xTaskCreate(watch_dog,"whatchDog",1000,NULL,1,NULL);
  xTaskCreate(get_readings,"getReadings",1000,NULL,1,NULL);
  vTaskDelay(10/portTICK_PERIOD_MS);
  xTaskCreate(wifi_setup, "espnow init", 4096, NULL, 1, NULL);
  xTaskCreate(send_data,"sendData",2000,NULL,1,NULL);
}

void loop() {
  taskYIELD(); 
}

void wifi_setup(void *pvParameters){
  xSemaphoreTake(wifisetup, portMAX_DELAY);
  WiFi.mode(WIFI_STA);
  Serial.println("wifi:");
  Serial.println(millis());
  int32_t channel = getWiFiChannel(WIFI_SSID);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

   // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  } else {
    Serial.println("Initializing ESP-NOW");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

    // Register peer
  peerInfo.encrypt = false;
  
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
  xSemaphoreGive(wifisetup);
  vTaskDelete(NULL);
}