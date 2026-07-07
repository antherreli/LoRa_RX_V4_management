/* Giovanni Antherreli
 * 30/10/2025
 * Function:
 * 1. Recebe dados de diferentes dispositivos
 * 2. Gerencia recebimento por meio dos endereços de cada nó sensor e via para o thing speak  
 * 3. armazena dados em cartão SD para backup
 * Referências:
 * timer WDT: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/timer.html
 * NTP Client-Server: Get Date and Time: https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
 *
 *
 *
 * */

//===================================Endereços LoRa=========================//
#define ADDR_00 0x0A //sensor 01 
#define ADDR_01 0x0B //sensor 02
#define ADDR_02 0x0C // sensor 03


//==================================Biliotecas=============================//
#include "LoRaWan_APP.h"
#include "Arduino.h"
//#include <WiFi.h>  // Conexão WiFi
//#include <WiFiClient.h>
//#include <WebServer.h>
#include <ThingSpeak.h>  // Comunicação com ThingSpeak
#include "sd_read_write.h"
#include "LoRaConfig.h"  //arquivo com configurações do radio LoRa
#include "time.h"

#include "esp_system.h"
#include "rom/ets_sys.h"

#if CONFIG_IDF_TARGET_ESP32  // ESP32/PICO-D4
#include "esp32/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C2
#include "esp32c2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C6
#include "esp32c6/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32H2
#include "esp32h2/rom/rtc.h"
#else
#error Target CONFIG_IDF_TARGET is not supported
#endif

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiClient.h>
  #include <ESP8266WebServer.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
#elif defined(TARGET_RP2040) || defined(TARGET_RP2350) || defined(PICO_RP2040) || defined(PICO_RP2350)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WiFiServer.h>
  #include <WebServer.h>
#endif

#include <ElegantOTA.h>

#include "esp_task_wdt.h"

#define WDT_TIMEOUT 5000  // tempo em ms (5s)
esp_task_wdt_config_t config = {
  .timeout_ms = WDT_TIMEOUT,
  .idle_core_mask = 0,   // não monitorar idle tasks
  .trigger_panic = true  // resetar sistema ao travar
};

//==========================Configurações de WiFi========================//
//const char* ssid = "GIO";
//const char* password = "";
//const char* password = "fak15far47";
const char* ssid = "TP-Link_2536";
const char* password = "Lena1123581321@";

//WebServer server(80);


#if defined(ESP8266)
  ESP8266WebServer server(80);
#elif defined(ESP32)
  WebServer server(80);
#elif defined(TARGET_RP2040) || defined(TARGET_RP2350) || defined(PICO_RP2040) || defined(PICO_RP2350)
  WebServer server(80);
#endif


//============================Configurações do ThingSpeak================//
WiFiClient client;

unsigned long channelID_0A = 3099475;
const char* writeAPIKey_0A = "D1LFNP76QNT6907Z";

unsigned long channelID_0B = 2988928;
const char* writeAPIKey_0B = "6L03G9YI3V07CBM0";

unsigned long channelID_0C = 2989011;
const char* writeAPIKey_0C = "8XG6E66W8S7K7OM6";

//================================configuração timer==================//
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */

const int wdtTimeout = 5000;  //tempo em ms para ativar watchdog
hw_timer_t* timer = NULL;



//========================periféricos de comunicação ================//
SPIClass sd_spi(HSPI);




//===========================state machine===================================//
#define WAIT 0
#define WiFi_Recnonnect 1
#define RX_MANAGEMENT 2
#define SD_CARD 3



//==============================variaveis globais=========================//
volatile char state = WAIT;        //variável responsável por armazenar o estado atual da máquina de estados
char txpacket[BUFFER_SIZE];        //pacote para possível transmissão
uint8_t rxpacket[BUFFER_SIZE];        //pacote de recepção lora
static RadioEvents_t RadioEvents;  //estrutura para callback do radio lora

volatile bool lora_idle = true;  //variável auxiliar para tratamento da recepção Lora
char counter = 0;                //variável auxiliar para reconexão WiFi
char counterTime = 0;            //variável auxiliar para atulização de data/hora

const long gmtOffset_sec = -3;        //define o gmt (-3 para o caso do Brazil)
const int daylightOffset_sec = 3600;  //duração de uma hora em segundos

struct tm tmstruct;  //estrutura para armazenar dados de data e hora
signed int receivedRSSI = 0;

//================================protótipo das funções axiliares=========//
void rxManagement();                  //gerencia o recebimento dos pacotes
//void ARDUINO_ISR_ATTR resetModule();  // Função de calback chamada após o disparo do WDT
void print_wakeup_reason();           //imprime o motivo pelo qual o sistema foi reiniciado
void print_reset_reason(int reason);
void sdInit(void);                  // inicializa cartão SD
void createFile(const char* path);  //cria uma arquivo txt caso ele não exista







void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  print_reset_reason(rtc_get_reset_reason(0));
  print_reset_reason(rtc_get_reset_reason(1));

  
  //nincializa cartão SD
  sdInit();
  createFile("/sensor_01.txt");
  createFile("/sensor_02.txt");
  createFile("/sensor_03.txt");



  // Conecta ao WiFi
  /*WiFi.disconnect(true); 
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && counter < 20) {
    delay(500);
    Serial.print(".");
    counter++;
    //esp_task_wdt_reset();  //reset timer (feed watchdog)
  }
  counter = 0;*/

  // Conecta Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(1000);
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  Serial.println(WiFi.localIP());
 
  configTime(3600 * gmtOffset_sec, daylightOffset_sec, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");

  esp_task_wdt_init(&config);
  esp_task_wdt_add(NULL);
  esp_task_wdt_reset();

  //unsigned long startAttemptTime = millis();

  /*while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 8000) {
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Libera CPU para tasks do WiFi
    Serial.print(".");
  }*/

  //timer = timerBegin(1000000);                     //timer 1Mhz resolution
  //timerAttachInterrupt(timer, &resetModule);       //attach callback
  //timerAlarm(timer, wdtTimeout * 1000, false, 0);  //set time in us
  //esp_task_wdt_reset();                            //reset timer (feed watchdog)
  //esp_task_wdt_reset();  //reset timer (feed watchdog)
  
  if (WiFi.status() == WL_CONNECTED) {
    tmstruct.tm_year = 0;
    getLocalTime(&tmstruct);
    Serial.println("\nWiFi conectado!");
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  Serial.printf("\nWifi status: %d", WiFi.status());
  Serial.println();

  esp_task_wdt_reset();  //reset timer (feed watchdog)


  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  // Inicializa ThingSpeak
  ThingSpeak.begin(client);
  server.begin();
  ElegantOTA.begin(&server);  // Start ElegantOTA
  
  //Serial.println("HTTP server started");
  Serial.println("OTA...again!");
}



void loop() {

  switch (state) {

    case WiFi_Recnonnect:
      esp_task_wdt_reset();  //reset timer (feed watchdog)
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED && counter < 7) {
        delay(500);
        Serial.print(".");
        counter++;
      }
      counter = 0;
      Serial.printf("\nWifi status: %d\n", WiFi.status());


      state = RX_MANAGEMENT;

      break;

    case RX_MANAGEMENT:
      esp_task_wdt_reset();  //reset timer (feed watchdog)
      rxManagement();
      state = WAIT;
      break;

    case SD_CARD:
      esp_task_wdt_reset();  //reset timer (feed watchdog)
      //grava dados
      state = WAIT;
      break;

    case WAIT:

      esp_task_wdt_reset();  //reset timer (feed watchdog)
      //atualiza data hora
      if (counterTime == 255 && WiFi.status() == WL_CONNECTED) {
        getLocalTime(&tmstruct);
        counterTime = 0;
      }




      if (lora_idle) {
        lora_idle = false;
        Serial.println("into RX mode");
        Radio.Rx(0);
      }
      Radio.IrqProcess();

      break;

    default:
      break;
  }

  server.handleClient();
  ElegantOTA.loop();
  //esp_task_wdt_reset(); //reset timer (feed watchdog)
}

void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  esp_task_wdt_reset();  //reset timer (feed watchdog)
  if (payload[0] != 0xF5) return;



  memcpy(rxpacket, payload, size);
  //rxpacket[size]='\0';
  Radio.Sleep();
  receivedRSSI = rssi;


  lora_idle = true;
  //Serial.printf("\r\n rssi: %d , length: %d\r\n",rssi,size);
  if (WiFi.status() != WL_CONNECTED) state = WiFi_Recnonnect;
  else state = RX_MANAGEMENT;
}

void rxManagement() {
  esp_task_wdt_reset();  //reset timer (feed watchdog)
  int x = 0;
  bool envioOK = false;
  //printf("Leitura: ");
  //for (int i = 0; i < 13; i++) {


  Serial.println("Frame Rx:");
  for(uint i = 0; i<14; i++) 
  {
    if (rxpacket[i] < 0x10) Serial.print("0");
    Serial.print(rxpacket[i],HEX);
    Serial.print(" ");
  }
  Serial.println();

  uint16_t buff = ((uint16_t)(rxpacket[2] << 8 ) & 0xFF00) | rxpacket[3];
  float humidity = (float)buff / 10; 

  buff = ((uint16_t)(rxpacket[4] << 8 ) & 0xFF00) | rxpacket[5];
  float temperature = (float)buff / 10;
  temperature  = (buff & 0x1000) ? (6553.4 - temperature) * -1 : temperature;
  
  uint16_t conductivity = ((uint16_t)(rxpacket[6] << 8) & 0xFF00) | rxpacket[7];
  Serial.println("===========Sensor 01================");
  Serial.println("Humidity: \t" + String(humidity));
  Serial.println("Temperature: \t" + String(temperature));
  Serial.println("Conductivity: \t" + String(conductivity));

  //====================================================================
  uint16_t buff_ = ((uint16_t)(rxpacket[8] << 8 ) & 0xFF00) | rxpacket[9];
  float humidity_ = (float)buff_ / 10; 

  buff_ = ((uint16_t)(rxpacket[10] << 8 ) & 0xFF00) | rxpacket[11];
  float temperature_ = (float)buff_ / 10;
  temperature_  = (buff_ & 0x1000) ? (6553.4 - temperature_) * -1 : temperature_;
  uint16_t conductivity_ = ((uint16_t)(rxpacket[12] << 8) & 0xFF00) | rxpacket[13];
  Serial.println("===========Sensor 02================");
  Serial.println("Humidity: \t" + String(humidity_));
  Serial.println("Temperature: \t" + String(temperature_));
  Serial.println("Conductivity: \t" + String(conductivity_));

  ThingSpeak.setField(1, humidity);
  ThingSpeak.setField(2, temperature);
  ThingSpeak.setField(3, conductivity);
  ThingSpeak.setField(4, receivedRSSI);
  ThingSpeak.setField(5, humidity_);
  ThingSpeak.setField(6, temperature_);
  ThingSpeak.setField(7, conductivity_);

  if (WiFi.status() == WL_CONNECTED) getLocalTime(&tmstruct);
  char timeYear[5];
  char timeDay[3];
  strftime(timeYear, 5, "%Y", &tmstruct);
  strftime(timeDay, 3, "%d", &tmstruct);
  String month = String((tmstruct.tm_mon) + 1);
  String hour = (String(tmstruct.tm_hour) + ":" + String(tmstruct.tm_min) + ":" + String(tmstruct.tm_sec));

  String dataMessage;
  dataMessage = String(timeDay) + "-" + month + "-" + String(timeYear) + "," + hour + "," 
              + String(temperature) + "," + String(humidity) + "," + String(conductivity) + "," 
              + String(temperature_) + "," + String(humidity_) + "," + String(conductivity_) + ","
              + String(receivedRSSI) + "\r\n";

  //Serial.print("Saving data: ");
  //Serial.println(dataMessage);

  switch (rxpacket[1]) {

    case ADDR_00:


      x = ThingSpeak.writeFields(channelID_0A, writeAPIKey_0A);
      appendFile(SD, "/sensor_01.txt", dataMessage.c_str());
      break;

    case ADDR_01:

      x = ThingSpeak.writeFields(channelID_0B, writeAPIKey_0B);
      appendFile(SD, "/sensor_02.txt", dataMessage.c_str());
      break;

    case ADDR_02:

      x = ThingSpeak.writeFields(channelID_0C, writeAPIKey_0C);
      appendFile(SD, "/sensor_03.txt", dataMessage.c_str());
      break;

    default:
      break;
  }

  /*envioOK = (x == 200);

          if (envioOK) {
                        Serial.println("Dados enviados com sucesso!");
                      } 
                      else Serial.println("Erro ao enviar: " + String(x));*/
}

/*void ARDUINO_ISR_ATTR resetModule() {
  ets_printf("reboot\n");
  //esp_restart();
  ESP.restart();
  //esp_restart_noos();
}*/

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void print_reset_reason(int reason) {
  switch (reason) {
    case 1: Serial.println("POWERON_RESET"); break;           /**<1,  Vbat power on reset*/
    case 3: Serial.println("SW_RESET"); break;                /**<3,  Software reset digital core*/
    case 4: Serial.println("OWDT_RESET"); break;              /**<4,  Legacy watch dog reset digital core*/
    case 5: Serial.println("DEEPSLEEP_RESET"); break;         /**<5,  Deep Sleep reset digital core*/
    case 6: Serial.println("SDIO_RESET"); break;              /**<6,  Reset by SLC module, reset digital core*/
    case 7: Serial.println("TG0WDT_SYS_RESET"); break;        /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8: Serial.println("TG1WDT_SYS_RESET"); break;        /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9: Serial.println("RTCWDT_SYS_RESET"); break;        /**<9,  RTC Watch dog Reset digital core*/
    case 10: Serial.println("INTRUSION_RESET"); break;        /**<10, Instrusion tested to reset CPU*/
    case 11: Serial.println("TGWDT_CPU_RESET"); break;        /**<11, Time Group reset CPU*/
    case 12: Serial.println("SW_CPU_RESET"); break;           /**<12, Software reset CPU*/
    case 13: Serial.println("RTCWDT_CPU_RESET"); break;       /**<13, RTC Watch dog Reset CPU*/
    case 14: Serial.println("EXT_CPU_RESET"); break;          /**<14, for APP CPU, reseted by PRO CPU*/
    case 15: Serial.println("RTCWDT_BROWN_OUT_RESET"); break; /**<15, Reset when the vdd voltage is not stable*/
    case 16: Serial.println("RTCWDT_RTC_RESET"); break;       /**<16, RTC Watch dog reset digital core and rtc module*/
    default: Serial.println("NO_MEAN");
  }
}

void sdInit(void) {
  sd_spi.begin(SCK, MISO, MOSI, CS);

  if (!SD.begin(CS, sd_spi)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void createFile(const char* path) {
  File file = SD.open(path);
  if (!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, path, "date, time, Temperature, Humidity, Condutivity,Temperature_2, Humidity_2, Condutivity_2,RSSI \r\n");
  } else {
    Serial.println("File already exists");
  }
  file.close();
}
