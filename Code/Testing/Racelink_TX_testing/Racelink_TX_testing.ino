// Racelink TX experimental sketch

#include <Arduino.h>
#include <RadioLib.h>
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

//================ SPI + LR1121 macros ======================
#define LR_SCK   6
#define LR_MOSI  4
#define LR_MISO  5
#define LR_NSS   7
#define LR_BUSY  3
#define LR_NRST  2
#define LR_IRQ   1

//======================= GPS macros ============================
#define GPS_RX_PIN  19  
#define GPS_TX_PIN  18  
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

//================= WS2812 ARGB LED macros ======================
#define NUM_LEDS 1
#define DATA_PIN 8
CRGB leds[NUM_LEDS];

//================= SPI, Radiolib config ==========================
static SPISettings SLOW(500000, MSBFIRST, SPI_MODE0);
static Module mod(LR_NSS, LR_IRQ, LR_NRST, LR_BUSY, SPI, SLOW);
static LR1121 radio(&mod);
static const uint8_t XR1_RFSW[8] = { 15, 0, 12, 8, 8, 6, 0, 5 };

int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false;        //TX interrupt flag
volatile bool telemReady = false;
uint32_t txCount = 0;
static int lastSec = -1;
static uint32_t secStart = 0;

#pragma pack(1)                               //packed struct, no padding
struct RadioConfig {
    float frequency;
    float bandwidth;
    uint8_t spreadingFactor;
    int8_t txPower;
};

struct telemetryPkt{
  uint8_t agt;
  int8_t lambda;
  uint8_t sebesseg;
  uint16_t feszultseg;
  uint16_t egt;
  uint8_t fokozat;
  uint8_t olajny;
  uint8_t olajh;
  uint8_t uzemanyagny;
  int8_t gyorsulas;
  uint8_t vizho;              
  uint8_t aps;
  uint8_t feknyomas;
  bool upshift;
  bool downshift;
  uint8_t GPSSpeed;
  uint8_t GPSSats;
  uint8_t GPSHDOP;
  int32_t GPSLat;
  int32_t GPSLng;
  uint8_t GPSHour;
  uint8_t GPSMinute;
  uint8_t GPSSecond;
  uint16_t GPSMillisecond;
  int8_t LoRaRssi;
  uint8_t LoRaSnr;
  uint8_t LoRaPktRate;
};

#pragma pack()

RadioConfig cfg;
telemetryPkt telem;
volatile bool configReceived = false;

uint8_t telemBuf[sizeof(telemetryPkt)];

/*
     ___ _____ ___  ___   ___ ___  ___ _____ ___  ___ 
    | _ \_   _/ _ \/ __| | _ \ _ \/ _ \_   _/ _ \/ __|
    |   / | || (_) \__ \ |  _/   / (_) || || (_) \__ \
    |_|_\ |_| \___/|___/ |_| |_|_\\___/ |_| \___/|___/
*/
void configReceiveTask(void *pvParameters);
void telemetryTransmitTask(void *pvParameters);
void uartReceiveTask(void *pvParameters);
void LEDTask(void *pvParameters);
void GPSTask(void *pvParameters);

/*
     ___ _   _ _  _  ___ _____ ___ ___  _  _   ___ ___  ___ _____ ___  ___ 
    | __| | | | \| |/ __|_   _|_ _/ _ \| \| | | _ \ _ \/ _ \_   _/ _ \/ __|
    | _|| |_| | .` | (__  | |  | | (_) | .` | |  _/   / (_) || || (_) \__ \
    |_|  \___/|_|\_|\___| |_| |___\___/|_|\_| |_| |_|_\\___/ |_| \___/|___/                                                            
*/
static void lr1121_send_rfsw(const uint8_t cfg[8]);
void xr1_apply_rfsw();
void lr1121_default_setup(uint32_t st);
void lr1121_setup(uint32_t st);
void lr1121_receive_setup(void);
void GPSInfo(void);
void setTxFlag(void);

/*
     ___ ___ _____ _   _ ___ 
    / __| __|_   _| | | | _ \
    \__ \ _|  | | | |_| |  _/
    |___/___| |_|  \___/|_|  
                         
*/
void setup() {
  Serial.begin(115200);
  delay(100);
  SPI.begin(LR_SCK, LR_MISO, LR_MOSI, LR_NSS);
  delay(100);
  GPSSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  radio.XTAL = true;
  int st = radio.begin();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("[ERROR] Init failed: ");
    Serial.println(st);
    while (1);
  }
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
  radio.setRegulatorDCDC();                   //DC reg mode for higher tx power
  st = radio.setModem(RADIOLIB_MODEM_LORA);   //LoRa modulation
  xr1_apply_rfsw();                           //rfswitch config  
  lr1121_default_setup(st);                   //radio Setup function call

  radio.setPacketSentAction(setTxFlag);       //tx flag irq interrupt flag

  xTaskCreate(configReceiveTask, "configReceive", 4096, NULL, 1, NULL);
  xTaskCreate(telemetryTransmitTask, "TelemetryTx", 3072, NULL, 1, NULL);
  xTaskCreate(uartReceiveTask, "UartRx", 3072, NULL, 1, NULL);
  xTaskCreate(LEDTask, "LEDTask", 2048, NULL, 2, NULL);
  xTaskCreate(GPSTask, "GPSTask", 3072, NULL, 2, NULL );
  Serial.println("[TX] Tasks created, scheduler running");

  Serial.print(F("entering loop"));
}
/*
       _    ___   ___  ___ 
      | |  / _ \ / _ \| _ \
      | |_| (_) | (_) |  _/
      |____\___/ \___/|_|  
*/
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/*
     ___ _____ ___  ___   _____ _   ___ _  _____ 
    | _ \_   _/ _ \/ __| |_   _/_\ / __| |/ / __|
    |   / | || (_) \__ \   | |/ _ \\__ \ ' <\__ \
    |_|_\ |_| \___/|___/   |_/_/ \_\___/_|\_\___/
                                                  
*/
void configReceiveTask(void *pvParameters)
{
  lr1121_default_setup(0);
    int st = radio.startReceive();  // ADD THIS - put radio into RX mode!
  Serial.println("[TX] RX mode started\n");
  while(1) {
    uint8_t buffer[sizeof(RadioConfig)];
    int state = radio.receive(buffer, sizeof(RadioConfig), 1000);
    if(state == RADIOLIB_ERR_NONE) {
      // Reconstruct struct from buffer using memcpy
      memcpy(&cfg, buffer, sizeof(RadioConfig));
      configReceived = true;
      
      Serial.printf("[TX] Config received: %.1f MHz, BW%.1f, SF%u, +%d dBm\n",
                    cfg.frequency, cfg.bandwidth, cfg.spreadingFactor, cfg.txPower);
        lr1121_setup(0);
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(NULL);     
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void telemetryTransmitTask(void *pvParameters)
{
  while(!configReceived) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  vTaskDelay(pdMS_TO_TICKS(500));

  transmissionState = radio.startTransmit(telemBuf, sizeof(telemetryPkt));
  Serial.println("[TX] First transmission started");

  while(1){
    if(transmittedFlag) {
      transmittedFlag = false;
      Serial.println("[TX] Flag received");

      if (transmissionState == RADIOLIB_ERR_NONE) {
        Serial.println(F("transmission finished!"));
      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);
      }
      
      radio.finishTransmit();
      vTaskDelay(pdMS_TO_TICKS(1));
      
    transmissionState = radio.startTransmit(telemBuf, sizeof(telemetryPkt));
      Serial.printf("[TX] Next transmission started \n");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void uartReceiveTask(void *pvParameters)
{
  while(1) {
  telem.agt = random(0,100);
  telem.lambda = random(-20, 20);
  telem.sebesseg = random(0, 200);
  telem.feszultseg = random(10000, 13000);
  telem.egt = random(20, 900);
  telem.fokozat = random(0, 6);
  telem.olajny = random(0,120);
  telem.olajh = random(20,150);
  telem.uzemanyagny = random(0, 100);
  telem.gyorsulas = random(-30, 30);
  telem.vizho = random(20, 120);
  telem.aps = random(0, 100);
  telem.feknyomas = random(0, 100);
  telem.upshift = random(0,1);
  telem.downshift = random(0,1);
  telem.LoRaRssi = 0;
  telem.LoRaSnr = 0;
  telem.LoRaPktRate = 0;
  memcpy(telemBuf, &telem, sizeof(telemetryPkt));
  telemReady = true;
  vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void GPSTask(void *pvParameters)
{
  while(1) {
    // Read all available GPS data
    while (GPSSerial.available() > 0) {
      char c = GPSSerial.read();
      if (gps.encode(c)) {
        GPSInfo();
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println("No GPS data received - check wiring!");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void LEDTask(void *pvParameters)
{
  while(1){
    if (configReceived){
      leds[0] = CRGB::Blue;
      FastLED.show();
    }else if(!configReceived){
      leds[0] = CRGB::Blue;
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(500));
      // Now turn the LED off, then pause
      leds[0] = CRGB::Black;
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/*
     ___ _   _ _  _  ___ _____ ___ ___  _  _ ___ 
    | __| | | | \| |/ __|_   _|_ _/ _ \| \| / __|
    | _|| |_| | .` | (__  | |  | | (_) | .` \__ \
    |_|  \___/|_|\_|\___| |_| |___\___/|_|\_|___/

*/
//-------------------------RF SWITCH-------------------------------
static void lr1121_send_rfsw(const uint8_t cfg[8]) {
  while (digitalRead(LR_BUSY)) {}
  digitalWrite(LR_NSS, LOW);
  SPI.transfer(0x01); SPI.transfer(0x12);            // SYSTEM_SET_DIO_AS_RF_SWITCH
  for (int i = 0; i < 8; i++) SPI.transfer(cfg[i]);
  digitalWrite(LR_NSS, HIGH);
  while (digitalRead(LR_BUSY)) {}
}
void xr1_apply_rfsw() {
  lr1121_send_rfsw(XR1_RFSW);
  Serial.println("RF Switch applied");
}

//-------------------------DEFAULT SETUP----------------------------
void lr1121_default_setup(uint32_t st)
{
  st |= radio.setFrequency(868.5);
  st |= radio.setBandwidth(250);
  st |= radio.setSpreadingFactor(7);
  st |= radio.setCodingRate(6);
  st |= radio.setSyncWord(RADIOLIB_LR11X0_LORA_SYNC_WORD_PUBLIC);
  st |= radio.setPreambleLength(8);
  st |= radio.setCRC(true);
  st |= radio.setOutputPower(22);
  radio.explicitHeader();
  radio.invertIQ(false);
  Serial.println("default setup set");
}
//-------------------------CONFIG SETUP----------------------------
void lr1121_setup(uint32_t st)
{
  st |= radio.setFrequency(cfg.frequency);
  st |= radio.setBandwidth(cfg.bandwidth, (cfg.frequency < 1000) ? false : true);
  st |= radio.setSpreadingFactor(cfg.spreadingFactor);
  st |= radio.setCodingRate(6);
  st |= radio.setSyncWord(RADIOLIB_LR11X0_LORA_SYNC_WORD_PUBLIC);
  st |= radio.setPreambleLength(8);
  st |= radio.setCRC(true);
  st |= radio.setOutputPower(cfg.txPower);
  radio.explicitHeader();
  radio.invertIQ(false);
}

void GPSInfo(void)
{
  if (gps.location.isValid()) {
      telem.GPSLat = gps.location.lat() * 1000000L;
      telem.GPSLng = gps.location.lng() * 1000000L;
          Serial.printf("Location: %.6f, %.6f\n", 
                  gps.location.lat(), 
                  gps.location.lng());
  }

  if (gps.speed.isValid())     telem.GPSSpeed = gps.speed.kmph();
  if (gps.satellites.isValid()){
    telem.GPSSats = gps.satellites.value();
    Serial.printf("Sats: %d \n", gps.satellites.value());
    }
  if (gps.hdop.isValid())      telem.GPSHDOP = (gps.hdop.hdop()*10);

  if (gps.time.isValid()) {
    int sec = gps.time.second();

    // Detect new GPS second boundary
    if (sec != lastSec) {
        lastSec = sec;
        secStart = millis();   // latch MCU time
    }

    telem.GPSHour   = gps.time.hour();
    telem.GPSMinute = gps.time.minute();
    telem.GPSSecond = sec;

    // Calculate ms since the start of this GPS second
    telem.GPSMillisecond = (millis() - secStart) % 1000;
  }
}


//----------------------SET TRANSMIT FLAG----------------------------
void setTxFlag(void) {
  transmittedFlag = true;
}
