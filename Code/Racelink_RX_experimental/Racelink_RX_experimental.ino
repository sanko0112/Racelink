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
uint32_t packetCount = 0;


void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(LR_SCK, LR_MISO, LR_MOSI, LR_NSS);
  delay(300);
  radio.XTAL = true;  // Your board has XTAL, not TCXO
  radio.setRegulatorDCDC();
  int st = radio.begin();
  
  if (st == RADIOLIB_ERR_NONE) {
    Serial.println("[SUCCESS] Radio initialized\n");
  } else {
    Serial.print("[ERROR] Init failed, code: ");
    Serial.println(st);
    while (true) { delay(10); }
  }

  st = radio.setModem(RADIOLIB_MODEM_LORA);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("  Error setting modem: ");
    Serial.println(st);
  }

  st = radio.setFrequency(868.5);
  st = radio.setBandwidth(500);
  st = radio.setSpreadingFactor(7);
  st = radio.setCodingRate(6);
  st = radio.setSyncWord(RADIOLIB_LR11X0_LORA_SYNC_WORD_PUBLIC);
  st = radio.setPreambleLength(8);
  st = radio.setCRC(true);
  st = radio.setOutputPower(22);
  xr1_apply_rfsw();
  radio.explicitHeader();
}

void loop() {
  uint8_t data;
  // Receive with 2 second timeout
  int state = radio.receive(&data, 1, 2000);

  if (state == RADIOLIB_ERR_NONE) {
    packetCount++;
    
    // IMPORTANT: Read frequency error IMMEDIATELY after receiving
    // This is when it's most accurate
    float freqError = radio.getFrequencyError();
    float rssi = radio.getRSSI();
    float snr = radio.getSNR();
    
    // Print received packet
    Serial.printf("║ Packet #%d received\n", packetCount);
    
    Serial.printf("  Data: 0x%02X\n", data);
    Serial.printf("  RSSI: %.1f dBm\n", rssi);
    Serial.printf("  SNR:  %.2f dB\n", snr);
    Serial.printf("  Freq Error: %.1f Hz\n", freqError);
    
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // Timeout - just wait for next packet
    Serial.print(".");
    
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[ERROR] CRC mismatch - packet corrupted");
    
  } else {
    Serial.print("[ERROR] RX error: ");
    Serial.println(state);
  }
  
  delay(100);
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
