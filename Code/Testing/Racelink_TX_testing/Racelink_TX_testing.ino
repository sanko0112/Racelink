/*
 * LR1121 TX with BLE Control via nRF Connect - IMPROVED
 * - Better BLE message chunking (handles MTU properly)
 * - Messages now arrive complete in nRF Connect
 * - Change radio settings via BLE (send 1-6)
 * - Monitor TX status over BLE
 */

#include <RadioLib.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// LR1121 Pin Definitions
#define LR_SCK   6
#define LR_MOSI  4
#define LR_MISO  5
#define LR_NSS   7
#define LR_BUSY  3
#define LR_NRST  2
#define LR_IRQ   1

// BLE UUIDs
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // Nordic UART Service
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Radio setup
static SPISettings SLOW(500000, MSBFIRST, SPI_MODE0);
static Module mod(LR_NSS, LR_IRQ, LR_NRST, LR_BUSY, SPI, SLOW);
static LR1121 radio(&mod);
static const uint8_t XR1_RFSW[8] = { 15, 0, 12, 8, 8, 6, 0, 5 };

// BLE variables
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t currentSetting = 4; // Default setting
int st = 0;

// TX variables
uint32_t txCount = 0;
uint8_t txData = 0x00; // Will increment with each packet

// BLE Server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// Forward declarations
void lr1121_setup(uint8_t setting);
void xr1_apply_rfsw();
const char* getSettingDescription(uint8_t setting);
void sendBLEMessage(const char* message);

// BLE Characteristic callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValueStr = pCharacteristic->getValue();

      if (rxValueStr.length() > 0) {
        char cmd = rxValueStr[0];
        
        // Check if it's a setting command (1-6)
        if (cmd >= '1' && cmd <= '6') {
          currentSetting = cmd - '0';
          Serial.printf("BLE: Received setting %d\n", currentSetting);
          
          // Apply new settings
          lr1121_setup(currentSetting);
          xr1_apply_rfsw();
          
          // Send confirmation back
          char response[100];
          snprintf(response, sizeof(response), 
                   "Setting %d applied: %s\n", 
                   currentSetting, 
                   getSettingDescription(currentSetting));
          
          sendBLEMessage(response);
          
          // Reset TX counter for new test
          txCount = 0;
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== LR1121 TX with BLE Control (IMPROVED) ===\n");

  // Initialize BLE
  initBLE();
  
  // Initialize Radio
  SPI.begin(LR_SCK, LR_MISO, LR_MOSI, LR_NSS);
  delay(300);

  radio.XTAL = true;
  radio.setRegulatorDCDC();
  st = radio.begin();
  
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("[ERROR] Radio init failed: ");
    Serial.println(st);
    sendBLEMessage("Radio init failed!");
    while (1);
  }
  
  Serial.println("[INIT] Radio initialized");
  sendBLEMessage("Radio initialized\n");

  // Configure LoRa with default setting
  st = radio.setModem(RADIOLIB_MODEM_LORA);
  lr1121_setup(currentSetting);
  xr1_apply_rfsw();
  
  if (st) {
    Serial.print("[ERROR] Config failed: ");
    Serial.println(st);
    sendBLEMessage("Config failed!");
    while (1);
  }
  
  // Send initial status
  char msg[100];
  snprintf(msg, sizeof(msg), "TX Ready - Setting %d: %s\n", 
           currentSetting, getSettingDescription(currentSetting));
  sendBLEMessage(msg);
}

void loop() {
  // Handle BLE disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("BLE: Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE: Device connected");
    
    // Send current status
    char msg[100];
    snprintf(msg, sizeof(msg), "Connected! Setting %d active\n", currentSetting);
    sendBLEMessage(msg);
  }

  // Transmit packet
  txData = (txCount & 0xFF); // Use count as data for packet identification
  int state = radio.transmit(&txData, 1);
  
  if (state == RADIOLIB_ERR_NONE) {
    txCount++;
    
    // Print to Serial
    Serial.printf("[TX #%lu] Data: 0x%02X, Setting: %d\n", 
                  txCount, txData, currentSetting);
    
    // Send status via BLE
    if (deviceConnected) {
      char bleMsg[64];
      snprintf(bleMsg, sizeof(bleMsg), 
               "TX #%lu: 0x%02X [S%d]\n", 
               txCount, txData, currentSetting);
      sendBLEMessage(bleMsg);
    }
    
  } else {
    Serial.printf("[TX ERROR] %d\n", state);
    
    if (deviceConnected) {
      char bleMsg[32];
      snprintf(bleMsg, sizeof(bleMsg), "TX Error: %d\n", state);
      sendBLEMessage(bleMsg);
    }
  }
  
  delay(1000); // 1 second between transmissions
}

void initBLE() {
  BLEDevice::init("LR1121_TX");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
                                         );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  
  pService->start();
  pServer->getAdvertising()->start();
  
  Serial.println("BLE: Waiting for connections...");
  Serial.println("     Name: LR1121_TX");
}

/*
 * IMPROVED sendBLEMessage - properly chunks messages for BLE MTU
 * 
 * Standard BLE has MTU of 20 bytes + 3 bytes overhead = 17 bytes data per packet
 * However, to be safer and ensure message completion, we use 16 bytes per chunk
 * and add a delimiter (newline) to help apps reassemble messages properly
 */
void sendBLEMessage(const char* message) {
  if (!deviceConnected || !pTxCharacteristic) {
    return;
  }

  size_t len = strlen(message);
  if (len == 0) return;

  // Standard BLE MTU is 20 bytes (23 with L2CAP header)
  // Safe chunk size is 16 bytes to account for overhead
  const size_t CHUNK_SIZE = 16;
  
  // If message fits in one chunk, send it directly
  if (len <= CHUNK_SIZE) {
    pTxCharacteristic->setValue((uint8_t*)message, len);
    pTxCharacteristic->notify();
    delay(20); // Allow time for notification to be sent
    return;
  }

  // For longer messages, send in chunks
  size_t offset = 0;
  
  while (offset < len) {
    size_t remaining = len - offset;
    size_t chunkLen = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
    
    // Try to avoid cutting in the middle of a line if possible
    if (chunkLen == CHUNK_SIZE && offset + chunkLen < len) {
      // Look backward from the end of this chunk for a newline or space
      for (int i = CHUNK_SIZE - 1; i > CHUNK_SIZE / 2; i--) {
        if (message[offset + i] == '\n' || message[offset + i] == ' ') {
          if (message[offset + i] == '\n') {
            chunkLen = i + 1;  // Include the newline
          } else {
            chunkLen = i;      // Stop before the space
          }
          break;
        }
      }
    }
    
    // Send this chunk
    pTxCharacteristic->setValue((uint8_t*)(message + offset), chunkLen);
    pTxCharacteristic->notify();
    
    offset += chunkLen;
    
    // Add delay between chunks to allow buffer processing
    if (offset < len) {
      delay(25); // 25ms between chunks gives BLE stack time to process
    }
  }
  
  // Send final delimiter to signal end of message
  if (message[len - 1] != '\n') {
    delay(25);
    const uint8_t delimiter[] = {'\n'};
    pTxCharacteristic->setValue((uint8_t*)delimiter, 1);
    pTxCharacteristic->notify();
  }
}

const char* getSettingDescription(uint8_t setting) {
  switch (setting) {
    case 1: return "2.45GHz BW812.5 SF5";
    case 2: return "2.45GHz BW812.5 SF7";
    case 3: return "2.45GHz BW406.25 SF7";
    case 4: return "868MHz BW500 SF6";
    case 5: return "868MHz BW500 SF8";
    case 6: return "868MHz BW250 SF8";
    default: return "Unknown";
  }
}

void lr1121_setup(uint8_t setting) {
  st = 0;
  switch (setting) {
    case 1: 
      st |= radio.setFrequency(2450.0);
      st |= radio.setBandwidth(812.5, true);
      st |= radio.setSpreadingFactor(5);
      st |= radio.setOutputPower(13);
      break;
    case 2: 
      st |= radio.setFrequency(2450.0);
      st |= radio.setBandwidth(812.5, true);
      st |= radio.setSpreadingFactor(7);
      st |= radio.setOutputPower(13);
      break;
    case 3: 
      st |= radio.setFrequency(2450.0);
      st |= radio.setBandwidth(406.25);
      st |= radio.setSpreadingFactor(7);
      st |= radio.setOutputPower(13);
      break;
    case 4: 
      st |= radio.setFrequency(868.5);
      st |= radio.setBandwidth(500.0);
      st |= radio.setSpreadingFactor(6);
      st |= radio.setOutputPower(22);
      break;
    case 5: 
      st |= radio.setFrequency(868.5);
      st |= radio.setBandwidth(500.0);
      st |= radio.setSpreadingFactor(8);
      st |= radio.setOutputPower(22);
      break;
    case 6: 
      st |= radio.setFrequency(868.5);
      st |= radio.setBandwidth(250.0);
      st |= radio.setSpreadingFactor(8);
      st |= radio.setOutputPower(22);
      break;                      
  }
  st |= radio.setCodingRate(6);
  st |= radio.setSyncWord(RADIOLIB_LR11X0_LORA_SYNC_WORD_PUBLIC);
  st |= radio.setPreambleLength(8);
  st |= radio.setCRC(true);
  radio.explicitHeader();
  radio.invertIQ(false);
  
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("Setting %d config error: %d\n", setting, st);
  }
}

static void lr1121_send_rfsw(const uint8_t cfg[8]) {
  while (digitalRead(LR_BUSY)) {}
  digitalWrite(LR_NSS, LOW);
  SPI.transfer(0x01); 
  SPI.transfer(0x12); // SYSTEM_SET_DIO_AS_RF_SWITCH
  for (int i = 0; i < 8; i++) SPI.transfer(cfg[i]);
  digitalWrite(LR_NSS, HIGH);
  while (digitalRead(LR_BUSY)) {}
}

void xr1_apply_rfsw() {
  lr1121_send_rfsw(XR1_RFSW);
}
