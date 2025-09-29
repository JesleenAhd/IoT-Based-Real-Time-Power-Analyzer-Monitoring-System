// Combined code: Power Analyzer data to Cloud using SIM7600 modem with Watchdog Timer
// This code reads both float and int64 power analyzer data via Modbus RTU and sends it to cloud

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <esp_task_wdt.h>
#include <FS.h>
#include <SPIFFS.h>


// Watchdog configuration
#define WDT_TIMEOUT_SECONDS 150  // 30 seconds timeout for power analyzer operations
#define WDT_CHECK_INTERVAL 10000  // Reset every 15 seconds

// Modbus RTU pins for power analyzer communication
#define RX_PIN 16
#define TX_PIN 17
#define DE_RE_PIN 4
#define TX_LED 18
#define RX_LED 19
#define DERE_LED 5

// SIM7600 modem pins
#define SerialMon Serial
#define MODEM_RX 25
#define MODEM_TX 26
#define SerialAT Serial2



// === Buffer for Failed Transmission ===
String failedDataBuffer = "";
bool hasBufferedData = false;

// Auto-restart after 5 minutes (300000 ms)
const unsigned long AUTO_RESTART_INTERVAL = 300000;
unsigned long bootTime = 0;

// Hardware serial for Modbus RTU communication with power analyzer
HardwareSerial RS485(1);  // Using Serial1 for Modbus (Serial2 used by SIM7600)

// SIM7600 configuration
//const char apn[] = "mobitel3g";
const char apn[] = "internet";
const char user[] = "";
const char pass[] = "";

// Cloud server configuration
const char server[] = "esp32-data-receiver-31735516534.asia-south1.run.app";
const int httpPort = 80;
const int httpsPort = 443;

// SIM7600 failure tracking
static int simFailCount = 0;
const int SIM_FAIL_THRESHOLD = 5;  // Reset after 3 consecutive failures
unsigned long lastSimReset = 0;
const unsigned long SIM_RESET_COOLDOWN = 500000;  // 5 minutes between resets

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

// Watchdog management variables
unsigned long lastWdtReset = 0;
bool wdtInitialized = false;

// Updated slave configuration
#define NUM_SLAVES 10
const uint8_t SLAVE_IDS[NUM_SLAVES] = {1,2,6,7,8,9,10,11,12,13};

// Enhanced Modbus configuration with separate function codes and data types
#define FUNCTION_CODE_03 0x03  // Read Holding Registers
#define FUNCTION_CODE_04 0x04  // Read Input Registers

// Function code mapping for slaves
const uint8_t SLAVE_FUNCTION_CODES[NUM_SLAVES] = {
  FUNCTION_CODE_03,  // Slave 1
  FUNCTION_CODE_03,  // Slave 2
  FUNCTION_CODE_04,  // Slave 6
  FUNCTION_CODE_04,  // Slave 7
  FUNCTION_CODE_04,  // Slave 8
  FUNCTION_CODE_04,  // Slave 9
  FUNCTION_CODE_04,  // Slave 10
  FUNCTION_CODE_04,  // Slave 11
  FUNCTION_CODE_04,  // Slave 12
  FUNCTION_CODE_04   // Slave 13
};

// Data type enumeration
enum DataType {
  DATA_FLOAT,           // For slaves 1-5
  DATA_INT64,           // For slaves 1-5 (energy values)
  DATA_32BIT_LITTLE,    // For slaves 6-7
  DATA_32BIT_BIG        // For slaves 8-13
};

// Register arrays for each slave ID
const uint16_t modbusRegisters_1[] = {               // Schneider Electric EasyLogic™ PM2200,PM2100,  EM1000H
  0x0BD3, 0x0BD5, 0x0BD7, 0x0BDB, 0x0BCB, 0x0BCD, 0x0BCF, 0x0BD1,
  0x0BB7, 0x0BB9, 0x0BBB, 0x0BC1, 0x0BED, 0x0BEF, 0x0BF1, 0x0BFD,
  0x0BFF, 0x0C01, 0x0BF5, 0x0BF7, 0x0BF9, 0x0BF3, 0x0C03, 0x0BFB,
  0x0C05, 0x0C07, 0x0C09, 0x0C0B, 0x0C25, 0x0C8F, 0x0CAB, 0x0C9F,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0BBD
};

const uint16_t modbusRegisters_2[] = {               // Schneider Electric EasyLogic™ PM2200,PM2100,  EM1000H
  0x0BD3, 0x0BD5, 0x0BD7, 0x0BDB, 0x0BCB, 0x0BCD, 0x0BCF, 0x0BD1,
  0x0BB7, 0x0BB9, 0x0BBB, 0x0BC1, 0x0BED, 0x0BEF, 0x0BF1, 0x0BFD,
  0x0BFF, 0x0C01, 0x0BF5, 0x0BF7, 0x0BF9, 0x0BF3, 0x0C03, 0x0BFB,
  0x0C05, 0x0C07, 0x0C09, 0x0C0B, 0x0C25, 0x0C8F, 0x0CAB, 0x0C9F,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0BBD
};

const uint16_t modbusRegisters_6[] = {                 // SELEC MFM384 Power Analyzer
  0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E,
  0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E,
  0x20, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C, 0x3E,
  0x40, 0x42, 0x44, 0x46, 0x48, 0x50, 0x52, 0x54,
  0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60, 0x62, 0x64,
  0x66, 0x68, 0x6A, 0x6C, 0x6E, 0x70, 0x72, 0x74,
  0x76, 0x78, 0x7A
};

const uint16_t modbusRegisters_7[] = {                   // SELEC MFM384 Power Analyzer
  0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E,
  0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E,
  0x20, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C, 0x3E,
  0x40, 0x42, 0x44, 0x46, 0x48, 0x50, 0x52, 0x54,
  0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60, 0x62, 0x64,
  0x66, 0x68, 0x6A, 0x6C, 0x6E, 0x70, 0x72, 0x74,
  0x76, 0x78, 0x7A
};

const uint16_t modbusRegisters_8[] = {             // Circutor CVM-C4 Power Analyzer
  0x06, 0x08, 0x0A, 0x00, 0x0C, 0x0E, 0x10, 0x00,
  0x12, 0x14, 0x16, 0x00, 0x18, 0x1A, 0x1C, 0x28,
  0x2A, 0x2C, 0x20, 0x22, 0x24, 0x1E, 0x2E, 0x26,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x3C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x40, 0x00,
  0x00, 0x00, 0x00
};

const uint16_t modbusRegisters_9[] = {             // Circutor CVM-C4 Power Analyzer
  0x06, 0x08, 0x0A, 0x00, 0x0C, 0x0E, 0x10, 0x00,
  0x12, 0x14, 0x16, 0x00, 0x18, 0x1A, 0x1C, 0x28,
  0x2A, 0x2C, 0x20, 0x22, 0x24, 0x1E, 0x2E, 0x26,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x3C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x40, 0x00,
  0x00, 0x00, 0x00
};

const uint16_t modbusRegisters_10[] = {            // Circutor CVM-C4 Power Analyzer
  0x06, 0x08, 0x0A, 0x00, 0x0C, 0x0E, 0x10, 0x00,
  0x12, 0x14, 0x16, 0x00, 0x18, 0x1A, 0x1C, 0x28,
  0x2A, 0x2C, 0x20, 0x22, 0x24, 0x1E, 0x2E, 0x26,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x3C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x40, 0x00,
  0x00, 0x00, 0x00
};

const uint16_t modbusRegisters_11[] = {            // Circutor CVM-C4 Power Analyzer
  0x06, 0x08, 0x0A, 0x00, 0x0C, 0x0E, 0x10, 0x00,
  0x12, 0x14, 0x16, 0x00, 0x18, 0x1A, 0x1C, 0x28,
  0x2A, 0x2C, 0x20, 0x22, 0x24, 0x1E, 0x2E, 0x26,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x3C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x40, 0x00,
  0x00, 0x00, 0x00
};

const uint16_t modbusRegisters_12[] = {            // Circutor CVM-C4 Power Analyzer
  0x06, 0x08, 0x0A, 0x00, 0x0C, 0x0E, 0x10, 0x00,
  0x12, 0x14, 0x16, 0x00, 0x18, 0x1A, 0x1C, 0x28,
  0x2A, 0x2C, 0x20, 0x22, 0x24, 0x1E, 0x2E, 0x26,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x3C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x40, 0x00,
  0x00, 0x00, 0x00
};

const uint16_t modbusRegisters_13[] = {            // Circutor CVM-C4 Power Analyzer
  0x06, 0x08, 0x0A, 0x00, 0x0C, 0x0E, 0x10, 0x00,
  0x12, 0x14, 0x16, 0x00, 0x18, 0x1A, 0x1C, 0x28,
  0x2A, 0x2C, 0x20, 0x22, 0x24, 0x1E, 0x2E, 0x26,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x3C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x40, 0x00,
  0x00, 0x00, 0x00
};

// Modbus int64 registers for energy values
uint16_t modbusInt64Registers[] = {
  0x0C83, 0x0C93, 0x0CA3
};


// Parameter names for int64 readings (energy values)
const char *int64ParamNames[] = {
  "Energy 3203", "Energy 3219", "Energy 3235"
};

// Array of pointers to register arrays
const uint16_t* modbusRegisters[NUM_SLAVES] = {
  modbusRegisters_1, modbusRegisters_2, 
  modbusRegisters_6, modbusRegisters_7, modbusRegisters_8, modbusRegisters_9, modbusRegisters_10,
   modbusRegisters_11, modbusRegisters_12, modbusRegisters_13
};

// Register count for each slave
const uint8_t registerCounts[NUM_SLAVES] = {
  sizeof(modbusRegisters_1) / sizeof(uint16_t),
  sizeof(modbusRegisters_2) / sizeof(uint16_t),
  sizeof(modbusRegisters_6) / sizeof(uint16_t),
  sizeof(modbusRegisters_7) / sizeof(uint16_t),
  sizeof(modbusRegisters_8) / sizeof(uint16_t),
  sizeof(modbusRegisters_9) / sizeof(uint16_t),
  sizeof(modbusRegisters_10) / sizeof(uint16_t),
  sizeof(modbusRegisters_11) / sizeof(uint16_t),
  sizeof(modbusRegisters_12) / sizeof(uint16_t),
  sizeof(modbusRegisters_13) / sizeof(uint16_t)
};

//const int numFloatParams = sizeof(modbusFloatRegisters) / sizeof(modbusFloatRegisters[0]);
const int numInt64Params = sizeof(modbusInt64Registers) / sizeof(modbusInt64Registers[0]);

// Timing variables
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 45000; // Send data every 15 seconds (increased for more data)

void initializeWatchdog() {
  // Configure watchdog timer with new API
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,  // Convert to milliseconds
    .idle_core_mask = 0,                       // Don't monitor idle tasks
    .trigger_panic = true                      // Auto-restart on timeout
  };
  
  esp_err_t result = esp_task_wdt_init(&wdt_config);
  
  if (result == ESP_OK) {
    // Add the current task (main loop) to watchdog monitoring
    result = esp_task_wdt_add(NULL);
    
    if (result == ESP_OK) {
      wdtInitialized = true;
      lastWdtReset = millis();
      SerialMon.println("Watchdog initialized: 30s timeout, auto-restart enabled");
    } else {
      SerialMon.println("Failed to add task to watchdog!");
    }
  } else {
    SerialMon.println("Failed to initialize watchdog timer!");
  }
}

void feedWatchdog() {
  if (wdtInitialized) {
    unsigned long currentTime = millis();
    
    // Feed more frequently during critical operations
    if (currentTime - lastWdtReset >= WDT_CHECK_INTERVAL) {
      esp_task_wdt_reset();
      lastWdtReset = currentTime;
      // Reduce verbose logging to prevent serial buffer issues
      if (currentTime % 30000 == 0) { // Only log every 30 seconds
        SerialMon.println("⏰ Watchdog fed");
      }
    }
  }
}

void setup() {
  SerialMon.begin(115200);
  delay(10);

  if (!SPIFFS.begin(true)) {
    SerialMon.println("❌ SPIFFS Mount Failed");
  } else {
    SerialMon.println("✓ SPIFFS mounted successfully");
  }

  bootTime = millis();  // Record time at boot

  // Initialize Modbus RTU communication pins
  RS485.begin(9600, SERIAL_8E1, RX_PIN, TX_PIN);
  pinMode(DE_RE_PIN, OUTPUT);
  pinMode(TX_LED, OUTPUT);
  pinMode(RX_LED, OUTPUT);
  pinMode(DERE_LED, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);

  SerialMon.println("Power Analyzer to Cloud Bridge with Watchdog Initializing...");

  // Initialize Watchdog Timer FIRST
  initializeWatchdog();
  feedWatchdog();

  // Initialize SIM7600 modem
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  feedWatchdog();

  SerialMon.println("Initializing SIM7600 modem...");
  modem.restart();
  feedWatchdog();
  
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  feedWatchdog();

  // Wait for network registration
  SerialMon.print("Waiting for network registration...");
  unsigned long networkTimeout = millis() + 180000; // 3 minutes
  while (!modem.waitForNetwork(10000L) && millis() < networkTimeout) {
    feedWatchdog(); // Keep feeding during long network wait
    SerialMon.print(".");
  }
  
  if (!modem.isNetworkConnected()) {
    SerialMon.println(" fail - Network connection timeout!");
    // Don't halt - let watchdog restart the system
    while (true) {
      delay(1000);
      // Don't feed watchdog - let it restart
    }
  }
  SerialMon.println(" success");
  feedWatchdog();

  // Check signal quality
  int csq = modem.getSignalQuality();
  SerialMon.print("Signal quality: ");
  SerialMon.println(csq);
  feedWatchdog();

  // Connect to GPRS
  SerialMon.print("Connecting to GPRS...");
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(" fail");
    // Don't halt - let watchdog restart the system
    while (true) {
      delay(1000);
      // Don't feed watchdog - let it restart
    }
  }
  SerialMon.println(" success");
  feedWatchdog();

  // Check if we got an IP address
  String localIP = modem.getLocalIP();
  SerialMon.print("Local IP: ");
  SerialMon.println(localIP);
  feedWatchdog();

  SerialMon.println("Setup complete - Ready to read power analyzer and send to cloud");
  SerialMon.println("🐕 Watchdog active - system will auto-restart if frozen for >30s");
}


// Function to reset SIM7600 modem
bool resetSIM7600() {
  // Check cooldown period to prevent excessive resets
  if (millis() - lastSimReset < SIM_RESET_COOLDOWN) {
    SerialMon.println("⏳ SIM7600 reset on cooldown, skipping...");
    return false;
  }
  
  SerialMon.println("🔄 Resetting SIM7600 modem...");
  feedWatchdog(); // Keep watchdog fed during reset
  
  // Method 1: Software reset via AT command
  modem.sendAT("+CFUN=1,1");  // Full functionality reset with SIM reset
  if (modem.waitResponse(5000) == 1) {
    SerialMon.println("✓ SIM7600 software reset initiated");
  } else {
    SerialMon.println("⚠ AT+CFUN reset command failed, trying restart()");
    modem.restart();  // Fallback to TinyGSM restart
  }
  
  feedWatchdog();
  delay(15000);  // Wait for SIM7600 to fully reboot
  feedWatchdog();
  
  // Re-initialize modem connection
  SerialMon.println("🔄 Re-initializing SIM7600...");
  
  // Get modem info to verify it's responding
  String modemInfo = modem.getModemInfo();
  if (modemInfo.length() < 5) {
    SerialMon.println("❌ Modem not responding after reset");
    return false;
  }
  SerialMon.print("✓ Modem responding: ");
  SerialMon.println(modemInfo);
  feedWatchdog();
  
  // Wait for network registration
  SerialMon.print("🔄 Waiting for network...");
  unsigned long networkTimeout = millis() + 120000; // 2 minutes timeout
  while (!modem.waitForNetwork(10000L) && millis() < networkTimeout) {
    feedWatchdog();
    SerialMon.print(".");
  }
  
  if (!modem.isNetworkConnected()) {
    SerialMon.println(" ❌ Network registration failed after reset");
    return false;
  }
  SerialMon.println(" ✓ Network registered");
  feedWatchdog();
  
  // Reconnect to GPRS
  SerialMon.print("🔄 Reconnecting GPRS...");
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(" ❌ GPRS reconnection failed");
    return false;
  }
  SerialMon.println(" ✓ GPRS reconnected");
  feedWatchdog();
  
  // Verify we have an IP
  String localIP = modem.getLocalIP();
  SerialMon.print("✓ New IP: ");
  SerialMon.println(localIP);
  
  lastSimReset = millis();
  simFailCount = 0;  // Reset failure counter
  SerialMon.println("✅ SIM7600 reset completed successfully");
  
  return true;
}

// Updated loop section for reading all slaves with different data types
void readAllSlaveData(DynamicJsonDocument& doc) {

  for (int s = 0; s < NUM_SLAVES; s++) {
    feedWatchdog(); // ✓ Keep existing
   
    uint8_t slaveId = SLAVE_IDS[s];
    if (slaveId == 0) continue;
    String slaveKey = "slave_" + String(slaveId);
    JsonObject slaveData = doc.createNestedObject(slaveKey);
   
    SerialMon.print("Reading from Slave ID: ");
    SerialMon.print(slaveId);
    SerialMon.print(" (Function Code: 0x");
    SerialMon.print(SLAVE_FUNCTION_CODES[s], HEX);
    SerialMon.println(")");
   
    if (slaveId == 8) {
      JsonObject dataObj = slaveData.createNestedObject("float_values");
      JsonObject longData = slaveData.createNestedObject("float_values");

      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
          0,  1,  2,  4,  5,  6,
          8,  9, 10, 12, 13, 14, 15,
          16, 17, 18, 19, 20, 21, 22, 23,
          24, 25, 26, 27, 28,  30, 45, 53
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "V12", "V23", "V31",
        "I1", "I2", "I3", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency",  "Net_kVAh",
        "Total_KWh_Imp", 
        "Total_KVArh_Imp"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);

      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 3 == 0) feedWatchdog();

        float value = readModbus32BitBigEndian(slaveId, modbusRegisters_8[keyFloatParams[i]], FUNCTION_CODE_04);
        if (!isnan(value)) {
          dataObj[keyFloatParamNames[i]] = value; // Store the value in the JSON object
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2); // Print value to serial monitor with 2 decimal places
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]); // Indicate failure
        }
        // Removed redundant assignment and print for 'paramName' as it was incorrect for float values
        delay(50); // Short delay
      }

      SerialMon.println("Reading long Net_kVAh parameter...");
      feedWatchdog();
      
      long value = readModbusLong(slaveId, 0x072, FUNCTION_CODE_04);
      if (value != 0) {
        longData["Net_kVAh"] = value;
        SerialMon.print("Net_kVAh: ");
        SerialMon.println(value);
      } else {
        SerialMon.println("Failed to read long: Net_kVAh");
      }

    } else if (slaveId == 1) {
      JsonObject floatData = slaveData.createNestedObject("float_values");
      JsonObject int64Data = slaveData.createNestedObject("int64_values");
     
      SerialMon.println("Reading float parameters...");

      int keyFloatParams[] = {
        0,  1,  2,  3,  4,  5,  6,  7,
        8,  9,  10, 11, 12, 13, 14, 15, 
        16, 17, 18, 19, 20, 21, 22, 23, 
        24, 25, 26, 27, 28, 58
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "Avg_V_LN", "V12", "V23", "V31", "Avg_V_LL",
        "I1", "I2", "I3", "Avg_I", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency", "Neutral_I"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);
      
      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 5 == 0) feedWatchdog(); // ✓ Feed more frequently
       
        float value = readModbusFloat(slaveId, modbusRegisters_1[keyFloatParams[i]], FUNCTION_CODE_03);
        if (!isnan(value)) {
          floatData[keyFloatParamNames[i]] = value;
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2);
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]);
        }
        delay(50); // ✓ Reduced delay from 100ms to 50ms
      }
     
      feedWatchdog(); // ✓ Added between float and int64 reading
     
      SerialMon.println("Reading int64 energy parameters...");
      for (int i = 0; i < numInt64Params; i++) {
        if (i % 3 == 0) feedWatchdog(); // ✓ Feed every 3 readings instead of 5
       
        int64_t value = readModbusInt64(slaveId, modbusInt64Registers[i]);
        if (value != 0 || i == 0) {
          int64Data[int64ParamNames[i]] = (long long)value;
          SerialMon.print(int64ParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println((long long)value);
        } else {
          SerialMon.print("Failed to read int64: ");
          SerialMon.println(int64ParamNames[i]);
        }
        delay(50); // ✓ Reduced delay from 150ms to 50ms
      }
     
    } else if (slaveId == 2) {
      JsonObject floatData = slaveData.createNestedObject("float_values");
      JsonObject int64Data = slaveData.createNestedObject("int64_values");
     
      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
        0,  1,  2,  3,  4,  5,  6,  7,
        8,  9,  10, 11, 12, 13, 14, 15, 
        16, 17, 18, 19, 20, 21, 22, 23, 
        24, 25, 26, 27, 28, 58
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "Avg_V_LN", "V12", "V23", "V31", "Avg_V_LL",
        "I1", "I2", "I3", "Avg_I", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency", "Neutral_I"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);
      
      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 5 == 0) feedWatchdog(); // ✓ Feed more frequently
       
        float value = readModbusFloat(slaveId, modbusRegisters_2[keyFloatParams[i]], FUNCTION_CODE_03);
        if (!isnan(value)) {
          floatData[keyFloatParamNames[i]] = value;
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2);
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]);
        }
        delay(50); // ✓ Reduced delay from 100ms to 50ms
      }
     
      feedWatchdog(); // ✓ Added between float and int64 reading
     
      SerialMon.println("Reading int64 energy parameters...");
      for (int i = 0; i < numInt64Params; i++) {
        if (i % 3 == 0) feedWatchdog(); // ✓ Feed every 3 readings instead of 5
       
        int64_t value = readModbusInt64(slaveId, modbusInt64Registers[i]);
        if (value != 0 || i == 0) {
          int64Data[int64ParamNames[i]] = (long long)value;
          SerialMon.print(int64ParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println((long long)value);
        } else {
          SerialMon.print("Failed to read int64: ");
          SerialMon.println(int64ParamNames[i]);
        }
        delay(50); // ✓ Reduced delay from 150ms to 50ms
      }
     
    } else if (slaveId == 6) {
      JsonObject dataObj = slaveData.createNestedObject("float_values");

      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
          0,  1,  2,  3,  4,  5,  6,  7,
          8,  9, 10, 11, 12, 13, 14, 15,
          16, 17, 18, 19, 20, 21, 22, 23,
          24, 25, 26, 27, 28, 30, 45, 53,
          58
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "Avg_V_LN", "V12", "V23", "V31", "Avg_V_LL",
        "I1", "I2", "I3", "Avg_I", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency",  "Net_kVAh", "Total_KWh_Imp", "Total_KVArh_Imp",
        "Neutral_I"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);

      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 3 == 0) feedWatchdog();

        float value = readModbus32BitLittleEndian(slaveId, modbusRegisters_6[keyFloatParams[i]], FUNCTION_CODE_04);
        if (!isnan(value)) {
          dataObj[keyFloatParamNames[i]] = value; // Store the value in the JSON object
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2); // Print value to serial monitor with 2 decimal places
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]); // Indicate failure
        }
        // Removed redundant assignment and print for 'paramName' as it was incorrect for float values
        delay(50); // Short delay
      }

    } else if (slaveId == 7) {
      JsonObject dataObj = slaveData.createNestedObject("float_values");

      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
          0,  1,  2,  3,  4,  5,  6,  7,
          8,  9, 10, 11, 12, 13, 14, 15,
          16, 17, 18, 19, 20, 21, 22, 23,
          24, 25, 26, 27, 28, 30, 45, 53,
          58
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "Avg_V_LN", "V12", "V23", "V31", "Avg_V_LL",
        "I1", "I2", "I3", "Avg_I", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency",  "Net_kVAh", "Total_KWh_Imp", "Total_KVArh_Imp",
        "Neutral_I"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);

      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 3 == 0) feedWatchdog();

        float value = readModbus32BitLittleEndian(slaveId, modbusRegisters_7[keyFloatParams[i]], FUNCTION_CODE_04);
        if (!isnan(value)) {
          dataObj[keyFloatParamNames[i]] = value; // Store the value in the JSON object
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2); // Print value to serial monitor with 2 decimal places
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]); // Indicate failure
        }
        // Removed redundant assignment and print for 'paramName' as it was incorrect for float values
        delay(50); // Short delay
      }

    } else if (slaveId == 9) {
      JsonObject dataObj = slaveData.createNestedObject("float_values");

      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
          0,  1,  2,  4,  5,  6,
          8,  9, 10, 12, 13, 14, 15,
          16, 17, 18, 19, 20, 21, 22, 23,
          24, 25, 26, 27, 28,  30, 45, 53
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "V12", "V23", "V31",
        "I1", "I2", "I3", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency",  "Net_kVAh",
        "Total_KWh_Imp", 
        "Total_KVArh_Imp"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);

      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 3 == 0) feedWatchdog();


        float value = readModbus32BitBigEndian(slaveId, modbusRegisters_9[keyFloatParams[i]], FUNCTION_CODE_04);
        if (!isnan(value)) {
          dataObj[keyFloatParamNames[i]] = value; // Store the value in the JSON object
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2); // Print value to serial monitor with 2 decimal places
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]); // Indicate failure
        }
        // Removed redundant assignment and print for 'paramName' as it was incorrect for float values
        delay(50); // Short delay
      }

    } else if (slaveId == 10) {
      JsonObject dataObj = slaveData.createNestedObject("float_values");

      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
          0,  1,  2,  4,  5,  6,
          8,  9, 10, 12, 13, 14, 15,
          16, 17, 18, 19, 20, 21, 22, 23,
          24, 25, 26, 27, 28,  30, 45, 53
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "V12", "V23", "V31",
        "I1", "I2", "I3", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency",  "Net_kVAh",
        "Total_KWh_Imp", 
        "Total_KVArh_Imp"
      };
            
      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);

      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 3 == 0) feedWatchdog();

        float value = readModbus32BitBigEndian(slaveId, modbusRegisters_10[keyFloatParams[i]], FUNCTION_CODE_04);
        if (!isnan(value)) {
          dataObj[keyFloatParamNames[i]] = value; // Store the value in the JSON object
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2); // Print value to serial monitor with 2 decimal places
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]); // Indicate failure
        }
        // Removed redundant assignment and print for 'paramName' as it was incorrect for float values
        delay(50); // Short delay
      }
    } else if (slaveId == 11) {
      JsonObject dataObj = slaveData.createNestedObject("float_values");

      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
          0,  1,  2,  4,  5,  6,
          8,  9, 10, 12, 13, 14, 15,
          16, 17, 18, 19, 20, 21, 22, 23,
          24, 25, 26, 27, 28,  30, 45, 53
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "V12", "V23", "V31",
        "I1", "I2", "I3", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency",  "Net_kVAh",
        "Total_KWh_Imp", 
        "Total_KVArh_Imp"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);

      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 3 == 0) feedWatchdog();

        float value = readModbus32BitBigEndian(slaveId, modbusRegisters_11[keyFloatParams[i]], FUNCTION_CODE_04);
        if (!isnan(value)) {
          dataObj[keyFloatParamNames[i]] = value; // Store the value in the JSON object
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2); // Print value to serial monitor with 2 decimal places
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]); // Indicate failure
        }
        // Removed redundant assignment and print for 'paramName' as it was incorrect for float values
        delay(50); // Short delay
      }

    }else if (slaveId == 12) {
      JsonObject dataObj = slaveData.createNestedObject("float_values");

      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
          0,  1,  2,  4,  5,  6,
          8,  9, 10, 12, 13, 14, 15,
          16, 17, 18, 19, 20, 21, 22, 23,
          24, 25, 26, 27, 28,  30, 45, 53
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "V12", "V23", "V31",
        "I1", "I2", "I3", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency",  "Net_kVAh",
        "Total_KWh_Imp", 
        "Total_KVArh_Imp"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);

      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 3 == 0) feedWatchdog();

        float value = readModbus32BitBigEndian(slaveId, modbusRegisters_12[keyFloatParams[i]], FUNCTION_CODE_04);
        if (!isnan(value)) {
          dataObj[keyFloatParamNames[i]] = value; // Store the value in the JSON object
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2); // Print value to serial monitor with 2 decimal places
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]); // Indicate failure
        }
        // Removed redundant assignment and print for 'paramName' as it was incorrect for float values
        delay(50); // Short delay
      }

    } else if (slaveId == 13) {
      JsonObject dataObj = slaveData.createNestedObject("float_values");

      SerialMon.println("Reading float parameters...");
      int keyFloatParams[] = {
          0,  1,  2,  4,  5,  6,
          8,  9, 10, 12, 13, 14, 15,
          16, 17, 18, 19, 20, 21, 22, 23,
          24, 25, 26, 27, 28,  30, 45, 53
      };

      const char* keyFloatParamNames[] = {
        "V1N", "V2N", "V3N", "V12", "V23", "V31",
        "I1", "I2", "I3", "kW1", "kW2", "kW3", "KVA1",
        "KVA2", "KVA3", "kVAr1", "kVAr2", "kVAr3", "Total_KW", "Total_KVA", "Total_KVAr",
        "PF1", "PF2", "PF3", "Avg_PF", "Frequency",  "Net_kVAh",
        "Total_KWh_Imp", 
        "Total_KVArh_Imp"
      };

      int numKeyFloatParams = sizeof(keyFloatParams) / sizeof(keyFloatParams[0]);

      for (int i = 0; i < numKeyFloatParams; i++) {
        if (i % 3 == 0) feedWatchdog();

        float value = readModbus32BitBigEndian(slaveId, modbusRegisters_13[keyFloatParams[i]], FUNCTION_CODE_04);
        if (!isnan(value)) {
          dataObj[keyFloatParamNames[i]] = value; // Store the value in the JSON object
          SerialMon.print(keyFloatParamNames[i]);
          SerialMon.print(": ");
          SerialMon.println(value, 2); // Print value to serial monitor with 2 decimal places
        } else {
          SerialMon.print("Failed to read float: ");
          SerialMon.println(keyFloatParamNames[i]); // Indicate failure
        }
        // Removed redundant assignment and print for 'paramName' as it was incorrect for float values
        delay(50); // Short delay
      }

    }
    
    feedWatchdog(); // ✓ Feed after each slave
    SerialMon.println("-------------------------------------------------");
  }
}


// Main loop function
void loop() {
  feedWatchdog();

  // Time-based auto restart
  if (millis() - bootTime >= AUTO_RESTART_INTERVAL) {
    SerialMon.println("⏳ Time-based auto restart triggered (2 minutes elapsed)");
    ESP.restart();
  }

  if (millis() - lastUpdate >= updateInterval) {
    SerialMon.println("=== Reading Multi-Slave Power Analyzer Data ===");
    feedWatchdog();

    // === Step 1: Build fresh data ===
    DynamicJsonDocument doc(12288);
    doc["device_id"] = "PowerAnalyzer_002";
    doc["timestamp"] = millis();
    doc["total_slaves"] = NUM_SLAVES;

    JsonObject systemInfo = doc.createNestedObject("system_info");
    systemInfo["free_heap"] = ESP.getFreeHeap();
    systemInfo["uptime_ms"] = millis();

    feedWatchdog();
    readAllSlaveData(doc);
    feedWatchdog();

    String jsonData;
    size_t jsonSize = serializeJson(doc, jsonData);

    SerialMon.println("=== Sending New Data to Cloud ===");
    SerialMon.print("JSON Size: ");
    SerialMon.println(jsonSize);

    feedWatchdog();

    // === Step 2: Try to send once ===
    if (sendToCloudRunWithReset(jsonData)) {
      SerialMon.println("✅ New data sent successfully");
    } else {
      SerialMon.println("✗ Failed to send new data. Buffering...");

      // === Step 3: Limit buffer to 3 files ===
      int fileCount = 0;
      String oldestFile = "";
      unsigned long oldestTime = ULONG_MAX;

      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while (file) {
        if (!file.isDirectory()) {
          fileCount++;
          String name = file.name();
          if (name.startsWith("/fail_")) {
            unsigned long fileTime = name.substring(6, name.indexOf(".json")).toInt();
            if (fileTime < oldestTime) {
              oldestTime = fileTime;
              oldestFile = name;
            }
          }
        }
        file = root.openNextFile();
      }

      if (fileCount >= 3 && SPIFFS.exists(oldestFile)) {
        SerialMon.print("🗑 Removing oldest buffer: ");
        SerialMon.println(oldestFile);
        SPIFFS.remove(oldestFile);
      }

      String filename = "/fail_" + String(millis()) + ".json";
      File newFile = SPIFFS.open(filename, FILE_WRITE);
      if (newFile) {
        newFile.print(jsonData);
        newFile.close();
        SerialMon.print("📝 Buffered to SPIFFS: ");
        SerialMon.println(filename);
      } else {
        SerialMon.println("❌ Failed to save buffer");
      }
    }

    feedWatchdog();

    // === Step 4: Update timing
    lastUpdate = millis();
    SerialMon.println("Next cycle in " + String(updateInterval / 1000) + " seconds");
    SerialMon.println("=====================================");
  }

  feedWatchdog();
  delay(1000);
}



// Optional: Add a function to handle communication errors
void handleCommunicationError(uint8_t slaveId, const char* errorType) {
  SerialMon.print("Communication Error - Slave ");
  SerialMon.print(slaveId);
  SerialMon.print(": ");
  SerialMon.println(errorType);
  
  // Add any error recovery logic here
  // For example: reset communication, increment error counter, etc.
}

// Optional: Add system status monitoring
void printSystemStatus() {
  SerialMon.println("=== System Status ===");
  SerialMon.print("Free Heap: ");
  SerialMon.print(ESP.getFreeHeap());
  SerialMon.println(" bytes");
  SerialMon.print("Uptime: ");
  SerialMon.print(millis() / 1000);
  SerialMon.println(" seconds");
  SerialMon.print("Active Slaves: ");
  SerialMon.println(NUM_SLAVES);
  SerialMon.println("=====================");
}

// Enhanced function to read float value with specific function code
float readModbusFloat(uint8_t slaveId, uint16_t reg, uint8_t functionCode) {
  byte request[] = {slaveId, functionCode, highByte(reg), lowByte(reg), 0x00, 0x02, 0x00, 0x00};
  sendModbusRequest(request, 6);

  byte response[9];
  if (!readModbusResponse(response, 9, functionCode)) return NAN; // ✓ Fixed: Added functionCode parameter

  // Convert response bytes to float
  byte floatBytes[4] = { response[6], response[5], response[4], response[3] };
  float result;
  memcpy(&result, floatBytes, sizeof(result));
  return result;
}

// Function to read int64 value from Modbus RTU power analyzer
int64_t readModbusInt64(uint8_t slaveId, uint16_t reg) {
  byte request[] = {slaveId, FUNCTION_CODE_03, highByte(reg), lowByte(reg), 0x00, 0x04, 0x00, 0x00};
  sendModbusRequest(request, 6);

  byte response[13];
  if (!readModbusResponse(response, 13, FUNCTION_CODE_03)) return 0; // ✓ Fixed: Added FUNCTION_CODE_03 parameter

  // Convert response bytes to int64
  byte intBytes[8] = {
    response[10], response[9], response[8], response[7],
    response[6],  response[5], response[4], response[3]
  };
  int64_t result;
  memcpy(&result, intBytes, sizeof(result));
  return result;
}


// Enhanced function to read float value with specific function code
float readModbus32BitLittleEndian(uint8_t slaveId, uint16_t reg, uint8_t functionCode) {
  // Construct the Modbus request for reading 2 registers (4 bytes)
  // Slave ID, Function Code (0x04 for Read Input Registers), Starting Address (high byte, low byte), Number of Registers (0x00, 0x02), CRC (placeholder)
  byte request[] = {slaveId, FUNCTION_CODE_04, highByte(reg), lowByte(reg), 0x00, 0x02, 0x00, 0x00};
  sendModbusRequest(request, 6); // Send the request, length is 6 bytes before CRC

  byte response[9]; // Expected response length: Slave ID (1) + Function Code (1) + Byte Count (1) + Data (4) + CRC (2) = 9 bytes
  // Read the Modbus response, checking for expected length and function code
  if (!readModbusResponse(response, 9, FUNCTION_CODE_04)) return NAN; // Return Not-A-Number on failure

  // Convert response bytes to float (32-bit Little Endian format)
  // The order { response[4], response[3], response[6], response[5] } suggests a specific
  // little-endian word swap (e.g., AB CD -> CD AB) and then byte swap within words (e.g., CD AB -> DC BA)
  // This specific order is preserved as per your original code.
  byte floatBytes[4] = { response[4], response[3], response[6], response[5] };
  float result;
  memcpy(&result, floatBytes, sizeof(result)); // Copy the bytes into a float variable
  return result; // Return the converted float value
}


// Enhanced function to read float value with specific function code
float readModbus32BitBigEndian(uint8_t slaveId, uint16_t reg, uint8_t functionCode) {
  byte request[] = {slaveId, FUNCTION_CODE_04, highByte(reg), lowByte(reg), 0x00, 0x02, 0x00, 0x00};
  sendModbusRequest(request, 6); // Send the request, length is 6 bytes before CRC

  byte response[9]; // Expected response length: Slave ID (1) + Function Code (1) + Byte Count (1) + Data (4) + CRC (2) = 9 bytes
  if (!readModbusResponse(response, 9, FUNCTION_CODE_04)) return NAN; 

  byte floatBytes[4] = { response[6], response[5], response[4], response[3] };
  float result;
  memcpy(&result, floatBytes, sizeof(result)); // Copy the bytes into a float variable
  return result; // Return the converted float value
}

// NEW FUNCTION: Add this new function to read long (32-bit) values
long readModbusLong(uint8_t slaveId, uint16_t reg, uint8_t functionCode) {
  byte request[] = {slaveId, functionCode, highByte(reg), lowByte(reg), 0x00, 0x02, 0x00, 0x00};
  sendModbusRequest(request, 6);

  byte response[9]; // Expected response length: Slave ID (1) + Function Code (1) + Byte Count (1) + Data (4) + CRC (2) = 9 bytes
  if (!readModbusResponse(response, 9, functionCode)) return 0; // Return 0 on failure

  // Convert response bytes to long (32-bit Big Endian format)
  // Assuming big-endian format: most significant byte first
  long result = ((long)response[3] << 24) | ((long)response[4] << 16) | ((long)response[5] << 8) | (long)response[6];
  return result;
}

// Unified function to send Modbus request
void sendModbusRequest(byte *request, int len) {
  uint16_t crc = calculateCRC(request, len);
  request[len] = lowByte(crc);
  request[len + 1] = highByte(crc);

  // Set RS485 to transmit mode
  digitalWrite(DE_RE_PIN, HIGH);
  digitalWrite(DERE_LED, HIGH);
  digitalWrite(TX_LED, HIGH);
  delay(5);
  
  // Send request
  RS485.write(request, len + 2);
  RS485.flush();
  delay(2);
  
  // Set RS485 to receive mode
  digitalWrite(DE_RE_PIN, LOW);
  digitalWrite(DERE_LED, LOW);
  digitalWrite(TX_LED, LOW);
}

// Unified function to read Modbus response
bool readModbusResponse(byte *buffer, int expectedLen, uint8_t expectedFunctionCode) {
  int index = 0;
  unsigned long timeout = millis() + 1000;
  
  digitalWrite(RX_LED, HIGH);
  
  while (millis() < timeout && index < expectedLen) {
    if (RS485.available()) {
      buffer[index++] = RS485.read();
    }
  }
  
  digitalWrite(RX_LED, LOW);

  // Validate response
  if (index < expectedLen) {
    SerialMon.println("❌ Error: Incomplete Modbus response!");
    return false;
  }
  
  if (buffer[1] != expectedFunctionCode) {
    SerialMon.print("❌ Error: Invalid Modbus response! Expected FC: 0x");
    SerialMon.print(expectedFunctionCode, HEX);
    SerialMon.print(", Got FC: 0x");
    SerialMon.println(buffer[1], HEX);
    return false;
  }

  return true;
}

// Generic data reading function based on slave ID and data type
JsonVariant readSlaveData(uint8_t slaveId, uint16_t reg, DataType dataType) {
  JsonVariant result;
  
  switch (dataType) {
    case DATA_FLOAT:
      if (slaveId == 1 || slaveId == 2) {
        float value = readModbusFloat(slaveId, reg, FUNCTION_CODE_03);
        if (!isnan(value)) {
          result.set(value); // Use set() here
        }
      }
      break;
    case DATA_INT64:
      if (slaveId == 1 || slaveId == 2) {
        int64_t value = readModbusInt64(slaveId, reg);
        if (value != 0) {
          result.set((long long)value); // Use set() here
        }
      }
      break;
    case DATA_32BIT_LITTLE:
      if (slaveId == 6 || slaveId == 7) {
        // Corrected: Pass FUNCTION_CODE_04 as the third argument
        float value = readModbus32BitLittleEndian(slaveId, reg, FUNCTION_CODE_04);
        if (!isnan(value)) { // Check if the read was successful
          result.set(value); // Corrected: Set the JsonVariant with the float value directly
        }
      }
      break;
    case DATA_32BIT_BIG:
      if (slaveId >= 8 && slaveId <= 13) {
        // Corrected: Pass FUNCTION_CODE_04 as the third argument
        float value = readModbus32BitLittleEndian(slaveId, reg, FUNCTION_CODE_04);
        if (!isnan(value)) { // Check if the read was successful
          result.set(value); // Corrected: Set the JsonVariant with the float value directly
        }
      }
      break;      

  }
  return result;
}



// CRC calculation for Modbus RTU
uint16_t calculateCRC(byte *buffer, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= buffer[pos];
    for (int i = 0; i < 8; i++) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
    }
  }
  return crc;
}

// Ensure modem connection is stable
bool ensureConnection() {
  feedWatchdog(); // Feed at start of connection check
  
  // Check if GPRS is still connected
  if (!modem.isGprsConnected()) {
    SerialMon.println("GPRS disconnected, reconnecting...");
    feedWatchdog();
    if (!modem.gprsConnect(apn, user, pass)) {
      SerialMon.println("Failed to reconnect GPRS");
      return false;
    }
    SerialMon.println("GPRS reconnected");
    delay(5000); // Wait for connection to stabilize
    feedWatchdog();
  }
  
  // Check network registration
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected, waiting for reconnection...");
    feedWatchdog();
    if (!modem.waitForNetwork(60000L)) {
      SerialMon.println("Failed to reconnect to network");
      return false;
    }
    SerialMon.println("Network reconnected");
    feedWatchdog();
  }
  
  return true;
}

// Send data using SIM7600 SSL commands
bool sendDataUsingSIM7600SSL(String jsonData) {
  SerialMon.println("Trying SIM7600 SSL commands...");
  feedWatchdog();
  
  // Step 1: Stop any existing SSL service
  modem.sendAT("+CCHSTOP");
  modem.waitResponse(3000);
  delay(1000);
  feedWatchdog();
  
  // Step 2: Start SSL service
  modem.sendAT("+CCHSTART");
  if (modem.waitResponse(10000) != 1) {
    SerialMon.println("Failed to start SSL service");
    return false;
  }
  SerialMon.println("✓ SSL service started");
  feedWatchdog();
  
  // Step 3: Configure SSL context (don't verify server certificates)
  modem.sendAT("+CSSLCFG=\"authmode\",0,0");  // 0 = no authentication
  if (modem.waitResponse(3000) != 1) {
    SerialMon.println("Failed to set SSL auth mode");
    modem.sendAT("+CCHSTOP");
    modem.waitResponse();
    return false;
  }
  SerialMon.println("✓ SSL auth mode set");
  feedWatchdog();
  
  // Step 4: Configure SSL version
  modem.sendAT("+CSSLCFG=\"sslversion\",0,4");  // 4 = TLS 1.2
  modem.waitResponse(3000);
  SerialMon.println("✓ SSL version set");
  feedWatchdog();
  
  // Step 5: Open SSL connection
  String openCmd = "+CCHOPEN=0,\"" + String(server) + "\"," + String(httpsPort) + ",2";  // 2 = SSL
  modem.sendAT(openCmd);
  if (modem.waitResponse(15000) != 1) {
    SerialMon.println("Failed to open SSL connection");
    modem.sendAT("+CCHSTOP");
    modem.waitResponse();
    return false;
  }
  SerialMon.println("✓ SSL connection opened");
  feedWatchdog();
  
  // Step 6: Prepare HTTP request
  String httpRequest = "POST / HTTP/1.1\r\n";
  httpRequest += "Host: " + String(server) + "\r\n";
  httpRequest += "Content-Type: application/json\r\n";
  httpRequest += "Content-Length: " + String(jsonData.length()) + "\r\n";
  httpRequest += "Connection: close\r\n";
  httpRequest += "User-Agent: ESP32-SIM7600\r\n";
  httpRequest += "\r\n";
  httpRequest += jsonData;
  
  // Step 7: Send data
  String sendCmd = "+CCHSEND=0," + String(httpRequest.length());
  modem.sendAT(sendCmd);
  if (modem.waitResponse(3000, ">") != 1) {
    SerialMon.println("Failed to prepare data send");
    modem.sendAT("+CCHCLOSE=0");
    modem.waitResponse();
    modem.sendAT("+CCHSTOP");
    modem.waitResponse();
    return false;
  }
  
  feedWatchdog(); // Feed before sending large data
  
  // Send the actual data
  SerialAT.print(httpRequest);
  if (modem.waitResponse(10000) != 1) {
    SerialMon.println("Failed to send data");
    modem.sendAT("+CCHCLOSE=0");
    modem.waitResponse();
    modem.sendAT("+CCHSTOP");
    modem.waitResponse();
    return false;
  }
  SerialMon.println("✓ HTTPS request sent");
  feedWatchdog();
  
  // Step 8: Wait and receive response
  delay(3000);
  modem.sendAT("+CCHRECV=0,1000");
  String response = "";
  if (modem.waitResponse(10000, response) == 1) {
    SerialMon.println("=== HTTPS RESPONSE ===");
    SerialMon.println(response);
    SerialMon.println("=== END ===");
  }
  feedWatchdog();
  
  // Step 9: Close connection
  modem.sendAT("+CCHCLOSE=0");
  modem.waitResponse(3000);
  modem.sendAT("+CCHSTOP");
  modem.waitResponse(3000);
  
  // Check for success
  if (response.indexOf("200 OK") >= 0) {
    return true;
  }
  
  return false;
}

// Send data using built-in HTTP commands
bool sendDataUsingHTTPCommands(String jsonData) {
  SerialMon.println("Trying built-in HTTP commands...");
  feedWatchdog();
  
  // Terminate any existing HTTP service
  modem.sendAT("+HTTPTERM");
  modem.waitResponse(2000);
  delay(1000);
  feedWatchdog();
  
  // Initialize HTTP service
  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000) != 1) {
    SerialMon.println("HTTP Init failed");
    return false;
  }
  feedWatchdog();
  
  // Set parameters
  modem.sendAT("+HTTPPARA=\"CID\",1");
  modem.waitResponse(2000);
  
  // Try HTTPS URL
  String url = "https://" + String(server) + "/";
  modem.sendAT("+HTTPPARA=\"URL\",\"" + url + "\"");
  if (modem.waitResponse(3000) != 1) {
    SerialMon.println("Failed to set URL");
    modem.sendAT("+HTTPTERM");
    modem.waitResponse();
    return false;
  }
  feedWatchdog();
  
  // Set content type
  modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
  modem.waitResponse(2000);
  
  // Set up POST data
  modem.sendAT("+HTTPDATA=" + String(jsonData.length()) + ",10000");
  if (modem.waitResponse(2000, "DOWNLOAD") != 1) {
    SerialMon.println("Failed to start data input");
    modem.sendAT("+HTTPTERM");
    modem.waitResponse();
    return false;
  }
  
  feedWatchdog(); // Feed before sending data
  
  // Send the JSON data
  SerialAT.print(jsonData);
  if (modem.waitResponse(5000) != 1) {
    SerialMon.println("Failed to send data");
    modem.sendAT("+HTTPTERM");
    modem.waitResponse();
    return false;
  }
  
  SerialMon.println("✓ Data uploaded");
  feedWatchdog();
  
  // Execute POST request
  modem.sendAT("+HTTPACTION=1");  // 1 = POST
  if (modem.waitResponse(30000) != 1) {
    SerialMon.println("POST request failed");
    modem.sendAT("+HTTPTERM");
    modem.waitResponse();
    return false;
  }
  
  SerialMon.println("✓ POST request sent");
  feedWatchdog();
  
  // Wait for response
  delay(5000);
  
  // Read response
  modem.sendAT("+HTTPREAD");
  String response = "";
  if (modem.waitResponse(15000, response) == 1) {
    SerialMon.println("=== HTTP RESPONSE ===");
    SerialMon.println(response);
    SerialMon.println("=== END ===");
  }
  
  // Terminate HTTP service
  modem.sendAT("+HTTPTERM");
  modem.waitResponse();
  
  // Check for success
  if (response.indexOf("200") >= 0 || response.indexOf("OK") >= 0) {
    return true;
  }
  
  return false;
}

// Main function to send power analyzer data to cloud
// Modified sendToCloudRun function with integrated reset logic
bool sendToCloudRunWithReset(String jsonData) {
  if (!ensureConnection()) {
    simFailCount++;
    SerialMon.print("❌ Connection check failed, count = ");
    SerialMon.println(simFailCount);
    
    if (simFailCount >= SIM_FAIL_THRESHOLD) {
      SerialMon.println("🚨 Connection failures exceeded threshold, resetting SIM7600...");
      if (resetSIM7600()) {
        SerialMon.println("✅ SIM7600 reset successful, retrying send...");
        // One more attempt after reset
        return sendToCloudRunWithReset(jsonData);
      } else {
        SerialMon.println("❌ SIM7600 reset failed");
        return false;
      }
    }
    return false;
  }
  
  feedWatchdog();
  
  bool sendSuccess = false;
  
  // Method 1: Try SIM7600 specific SSL commands
  if (sendDataUsingSIM7600SSL(jsonData)) {
    sendSuccess = true;
  }
  // Method 2: Try built-in HTTP commands 
  else if (sendDataUsingHTTPCommands(jsonData)) {
    sendSuccess = true;
  }
  // Method 3: Fallback to direct HTTP connection
  else if (client.connect(server, httpPort)) {
    SerialMon.println("✓ Connected to Cloud Run server (HTTP)");
    feedWatchdog();
    
    // Send HTTP POST request
    client.print("POST / HTTP/1.1\r\n");
    client.print("Host: ");
    client.print(server);
    client.print("\r\n");
    client.print("Content-Type: application/json\r\n");
    client.print("Content-Length: ");
    client.print(jsonData.length());
    client.print("\r\n");
    client.print("Connection: close\r\n");
    client.print("User-Agent: ESP32-SIM7600\r\n");
    client.print("\r\n");
    client.print(jsonData);
    
    SerialMon.println("HTTP request sent, waiting for response...");
    feedWatchdog();
    
    // Wait for response with timeout
    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 15000) {
        SerialMon.println("Response timeout!");
        client.stop();
        sendSuccess = false;
        break;
      }
      if ((millis() - timeout) % 2000 == 0) {
        feedWatchdog();
      }
      delay(100);
    }
    
    if (sendSuccess != false) { // If we didn't timeout
      feedWatchdog();
      
      // Read response
      String response = "";
      while (client.available()) {
        response += client.readString();
      }
      
      SerialMon.println("=== HTTP RESPONSE ===");
      SerialMon.println(response);
      SerialMon.println("=== END ===");
      
      client.stop();
      feedWatchdog();
      
      // Check for success responses
      if (response.indexOf("200 OK") >= 0) {
        sendSuccess = true;
      } else if (response.indexOf("302") >= 0) {
        SerialMon.println("Got redirect - server accessible but requires HTTPS");
        sendSuccess = false;
      }
    }
  } else {
    SerialMon.println("✗ Failed to connect to Cloud Run server");
    sendSuccess = false;
    
    // Additional debugging
    SerialMon.print("Network connected: ");
    SerialMon.println(modem.isNetworkConnected() ? "Yes" : "No");
    SerialMon.print("GPRS connected: ");
    SerialMon.println(modem.isGprsConnected() ? "Yes" : "No");
    SerialMon.print("Local IP: ");
    SerialMon.println(modem.getLocalIP());
    SerialMon.print("Signal quality: ");
    SerialMon.println(modem.getSignalQuality());
    
    feedWatchdog();
  }
  
  // Handle success/failure and reset logic
  if (sendSuccess) {
    simFailCount = 0;  // ✅ Success, reset fail counter
    SerialMon.println("✅ SIM7600 data sent successfully");
    return true;
  } else {
    simFailCount++;
    SerialMon.print("❌ SIM7600 send failed, count = ");
    SerialMon.println(simFailCount);
    
    if (simFailCount >= SIM_FAIL_THRESHOLD) {
      SerialMon.println("🚨 Send failures exceeded threshold, resetting SIM7600...");
      if (resetSIM7600()) {
        SerialMon.println("✅ SIM7600 reset successful, retrying send...");
        feedWatchdog();
        // One more attempt after reset
        return sendToCloudRunWithReset(jsonData);
      } else {
        SerialMon.println("❌ SIM7600 reset failed");
        return false;
      }
    }
    return false;
  }
}