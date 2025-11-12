// Racelink TX experimental sketch

#include <RadioLib.h>
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// SPI + LR1121 Defines
#define LR_SCK   6
#define LR_MOSI  4
#define LR_MISO  5
#define LR_NSS   7
#define LR_BUSY  3
#define LR_NRST  2
#define LR_IRQ   1

// WS2812 ARGB LED defines
#define NUM_LEDS 1
#define DATA_PIN 8
CRGB leds[NUM_LEDS];

static SPISettings SLOW(500000, MSBFIRST, SPI_MODE0);
static Module mod(LR_NSS, LR_IRQ, LR_NRST, LR_BUSY, SPI, SLOW);
static LR1121 radio(&mod);
static const uint8_t XR1_RFSW[8] = { 15, 0, 12, 8, 8, 6, 0, 5 };

int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false;
uint32_t txCount = 0;

#pragma pack(1)
struct RadioConfig {
    float frequency;
    float bandwidth;
    uint8_t spreadingFactor;
    int8_t txPower;
};
#pragma pack()
RadioConfig cfg;
volatile bool configReceived = false;

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

  radio.XTAL = true;
  int st = radio.begin();
  
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("[ERROR] Init failed: ");
    Serial.println(st);
    while (1);
  }

  radio.setRegulatorDCDC();                   //DC reg mode for higher tx power
  st = radio.setModem(RADIOLIB_MODEM_LORA);   //LoRa modulation
  xr1_apply_rfsw();                           //rfswitch config  
  lr1121_default_setup(st);                   //radio Setup function call
  lr1121_receive_setup();                     //receiving radio params blocking
  lr1121_setup(st);                           //setting up radio with received params
  radio.setPacketSentAction(setTxFlag);       //tx flag irq interrupt flag

  Serial.println(F("[TX] Starting first transmission..."));
  String str = "Hello World! #" + String(txCount++);
  transmissionState = radio.startTransmit(str);  // Start first TX here!

  Serial.print(F("entering loop"));
}
/*
       _    ___   ___  ___ 
      | |  / _ \ / _ \| _ \
      | |_| (_) | (_) |  _/
      |____\___/ \___/|_|  
*/
void loop() {
  if(transmittedFlag){
  transmittedFlag = false;

  if (transmissionState == RADIOLIB_ERR_NONE) {
    // packet was successfully sent
    Serial.println(F("transmission finished!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(transmissionState);
  }
  radio.finishTransmit();
  vTaskDelay(pdMS_TO_TICKS(1));
  String str = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
  transmissionState = radio.startTransmit(str);
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

//----------------------RECEIVE CONFIG SETUP----------------------------
void lr1121_receive_setup(void) {
  
  while(1) {
    uint8_t buffer[sizeof(RadioConfig)];
    int state = radio.receive(buffer, sizeof(RadioConfig), 1000);
    Serial.println("TXradio receive set");
    if(state == RADIOLIB_ERR_NONE) {
      // Reconstruct struct from buffer using memcpy
      memcpy(&cfg, buffer, sizeof(RadioConfig));
      configReceived = true;
      
      Serial.printf("[TX] Config received: %.1f MHz, BW%.1f, SF%u, +%d dBm\n",
                    cfg.frequency, cfg.bandwidth, cfg.spreadingFactor, cfg.txPower);
      return;
    }
  }
}

//----------------------SET TRANSMIT FLAG----------------------------
void setTxFlag(void) {
  transmittedFlag = true;
}
/*
     ___ _____ ___  ___   _____ _   ___ _  _____ 
    | _ \_   _/ _ \/ __| |_   _/_\ / __| |/ / __|
    |   / | || (_) \__ \   | |/ _ \\__ \ ' <\__ \
    |_|_\ |_| \___/|___/   |_/_/ \_\___/_|\_\___/
                                                  
*/



