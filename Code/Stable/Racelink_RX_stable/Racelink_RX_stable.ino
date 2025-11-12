// Racelink TX experimental sketch

#include <RadioLib.h>
#include <FastLED.h>

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

volatile bool receivedFlag = false;

#pragma pack(1)
struct RadioConfig {
    float frequency;
    float bandwidth;
    uint8_t spreadingFactor;
    int8_t txPower;
};
#pragma pack()
RadioConfig cfg;

//-------------------------SETUP--------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
  SPI.begin(LR_SCK, LR_MISO, LR_MOSI, LR_NSS);
  delay(300);
  radio.XTAL = true;
  int st = radio.begin();
  
  if (st == RADIOLIB_ERR_NONE) {
    Serial.println("[SUCCESS] Radio initialized\n");
  } else {
    Serial.print("[ERROR] Init failed, code: ");
    Serial.println(st);
    while (true) { delay(10); }
  }
  radio.setPacketReceivedAction(setFlag);
  radio.setRegulatorDCDC();
  st = radio.setModem(RADIOLIB_MODEM_LORA);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("  Error setting modem: ");
    Serial.println(st);
  }
  xr1_apply_rfsw();
  lr1121_default_setup(st);
  lr1121_get_setup();
  lr1121_send_setup();
  lr1121_setup(st);



  Serial.print(F("[LR1121] Starting to listen ... "));
  st = radio.startReceive();
  if (st == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(st);
    while (true) { delay(10); }
  }
  Serial.print(F("entering loop"));
}

//-------------------------LOOP--------------------------------
void loop() {
  // check if the flag is set
  if(receivedFlag) {
    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    int state = radio.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int numBytes = radio.getPacketLength();
      int state = radio.readData(byteArr, numBytes);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[LR1110] Received packet!"));

      // print data of the packet
      Serial.print(F("[LR1110] Data:\t\t"));
      Serial.println(str);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[LR1110] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[LR1110] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);

    }
  }
}
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
//----------------------GET CONFIG SETUP----------------------------
void lr1121_get_setup(void)
{
  Serial.println("Waiting for radio parameters");
  uint16_t buf[4] = {8680,250,6,22};
/*
  for (uint8_t i = 0; i < 4; i++) {
    while(!Serial.available()){
      delay(1);
    }
    uint8_t highByte = Serial.read();

    while(!Serial.available()){
      delay(1);
    }
    uint8_t lowByte = Serial.read();

    buf[i] = lowByte | (highByte << 8);
  }
*/
  cfg.frequency = (float)buf[0]/10;
  switch (buf[1])
  {
    case 812: cfg.bandwidth = 812.5; break;
    case 406: cfg.bandwidth = 406.25; break;
    case 203: cfg.bandwidth = 203.125; break;
    case 500: cfg.bandwidth = 500.0; break;
    case 250: cfg.bandwidth = 250.0; break;
    case 125: cfg.bandwidth = 125.0; break;
    case 62:  cfg.bandwidth = 62.5; break;
    default:
        (cfg.frequency < 1000) ? cfg.bandwidth = 500.0 : cfg.bandwidth = 812.5;
        break;
  }
  cfg.spreadingFactor = buf[2];
  cfg.txPower = buf[3];

  Serial.printf("[RX] Config loaded: %.1f MHz, BW%.1f, SF%u, +%d dBm\n", 
          cfg.frequency, cfg.bandwidth, cfg.spreadingFactor, cfg.txPower);
}

//----------------------SEND SETUP TO TX ----------------------------
void lr1121_send_setup(void) {
    // Pack struct into binary buffer and send 5 times
    uint8_t buffer[sizeof(RadioConfig)];
    memcpy(buffer, &cfg, sizeof(RadioConfig));
    
    Serial.printf("[RX] Sending config (size: %d bytes)\n", sizeof(RadioConfig));
    
    for(int i = 0; i < 5; i++) {
        radio.transmit(buffer, sizeof(RadioConfig));
        delay(200);
    }
    
    Serial.println("[RX] Config transmitted");
}

//----------------------SET RECEIVE FLAG----------------------------
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

