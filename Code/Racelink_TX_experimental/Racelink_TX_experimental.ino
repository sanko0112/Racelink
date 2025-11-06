/*
 * LR1121 RF Switch Configuration - ELRS Method
 * Uses SetDioAsRfSwitch command to configure GPIO control
 */

#include <RadioLib.h>

#define LR_SCK   6
#define LR_MOSI  4
#define LR_MISO  5
#define LR_NSS   7
#define LR_BUSY  3
#define LR_NRST  2
#define LR_IRQ   1

static SPISettings SLOW(500000, MSBFIRST, SPI_MODE0);
static Module mod(LR_NSS, LR_IRQ, LR_NRST, LR_BUSY, SPI, SLOW);
static LR1121 radio(&mod);
static const uint8_t XR1_RFSW[8] = { 15, 0, 12, 8, 8, 6, 0, 5 };

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(LR_SCK, LR_MISO, LR_MOSI, LR_NSS);
  delay(300);

  radio.XTAL = true;
  int st = radio.begin();
  
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("[ERROR] Init failed: ");
    Serial.println(st);
    while (1);
  }
  
  Serial.println("[INIT] Radio initialized\n");

  // Configure LoRa
  radio.setRegulatorDCDC();
  st = radio.setModem(RADIOLIB_MODEM_LORA);
  st |= radio.setFrequency(868.5);
  st |= radio.setBandwidth(500);
  st |= radio.setSpreadingFactor(7);
  st |= radio.setCodingRate(6);
  st |= radio.setSyncWord(RADIOLIB_LR11X0_LORA_SYNC_WORD_PUBLIC);
  st |= radio.setPreambleLength(8);
  st |= radio.setCRC(true);
  st |= radio.setOutputPower(22);
  radio.explicitHeader();
  radio.invertIQ(false);
  xr1_apply_rfsw();
  if (st) {
    Serial.print("[ERROR] Config failed: ");
    Serial.println(st);
    while (1);
  }
}

uint32_t txCount = 0;

void loop() {
  uint8_t data = 0xAA;
  int state = radio.transmit(&data, 1);
  
  if (state == RADIOLIB_ERR_NONE) {
    txCount++;
    Serial.printf("[TX #%lu] 0xAA @ +22 dBm with RF Switch\n", txCount);
  } else {
    Serial.print("[TX ERROR] ");
    Serial.println(state);
  }
  
  delay(1000);
}

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
