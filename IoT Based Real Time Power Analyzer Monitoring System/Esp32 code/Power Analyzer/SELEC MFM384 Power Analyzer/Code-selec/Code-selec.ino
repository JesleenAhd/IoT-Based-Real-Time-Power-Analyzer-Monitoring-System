#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define RX_PIN 16
#define TX_PIN 17
#define DE_RE_PIN 4
#define TX_LED 18
#define RX_LED 19
#define DERE_LED 5

HardwareSerial RS485(2);  // Serial2

const char* ssid = "HUAWEI Y7a";
const char* password = "20011125";

WiFiServer tcpServer(502);
WiFiClient client;

#define NUM_SLAVES 2
const uint8_t SLAVE_IDS[NUM_SLAVES] = {2,3 };
#define FUNCTION_CODE 0x04  // Read Input Registers

int16_t modbusRegisters[] = {
  0x06, 0x08, 0x0A, 0x00, 0x0C, 0x0E, 0x10, 0x00,
  0x12, 0x14, 0x16, 0x00, 0x18, 0x1A, 0x1C, 0x28,
  0x2A, 0x2C, 0x20, 0x22, 0x24, 0x1E, 0x2E, 0x26,
  0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3E, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x3C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x40, 0x00,
  0x00, 0x00, 0x00
};

const char *paramNames[] = {
  "Voltage V1N", "Voltage V2N", "Voltage V3N", "Avg Voltage LN",
  "Voltage V12", "Voltage V23", "Voltage V31", "Avg Voltage LL",
  "Current I1", "Current I2", "Current I3", "Avg Current",
  "kW1", "kW2", "kW3", "KVA1",
  "KVA2", "KVA3", "kVAr1", "kVAr2",
  "kVAr3", "Total KW", "Total KVA", "Total KVAr",
  "PF1", "PF2", "PF3", "Average PF", "Frequency",
  "Total net kWh", "Total net kVAh", "Total net kVArh",
  "kW Max DMD", "kW Min DMD", "kVAr Max DMD", "kVAr Min DMD", "KVA Max DMD",
  "Aux Interrupt", "Run hour", "kWh1 (Imp)", "kWh2 (Imp)", "kWh3 (Imp)",
  "kWh1 (Exp)", "kWh2 (Exp)", "kWh3 (Exp)", "Total kWh (Imp)", "Total kWh (Exp)",
  "kVArh1 (Imp)", "kVArh2 (Imp)", "kVArh3 (Imp)", "kVArh1 (Exp)", "kVArh2 (Exp)",
  "kVArh3 (Exp)", "Total kVArh (Imp)", "Total kVArh (Exp)",
  "kVAh1", "kVAh2", "kVAh3", "Neutral Current"
};

const int numParams = sizeof(modbusRegisters) / sizeof(modbusRegisters[0]);
uint16_t holdingRegs[NUM_SLAVES][118];  // Separate buffer for each slave

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 1000;

void setup() {
  Serial.begin(115200);
  RS485.begin(9600, SERIAL_8E1, RX_PIN, TX_PIN);

  pinMode(DE_RE_PIN, OUTPUT);
  pinMode(TX_LED, OUTPUT);
  pinMode(RX_LED, OUTPUT);
  pinMode(DERE_LED, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);

  Serial.println("Modbus RTU to TCP Bridge Initializing...");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  tcpServer.begin();
  tcpServer.setNoDelay(true);
}

void loop() {
  if (millis() - lastUpdate >= updateInterval) {
    for (int s = 0; s < NUM_SLAVES; s++) {
      for (int i = 0; i < numParams; i++) {
        float value = readModbusFloat(SLAVE_IDS[s], modbusRegisters[i]);
        Serial.print("Slave ");
        Serial.print(SLAVE_IDS[s]);
        Serial.print(" - ");
        Serial.print(paramNames[i]);
        Serial.print(": ");
        Serial.println(value, 2);
        floatToRegisters(value, i * 2, s);
      }
      Serial.println("-------------------------------------------------");
    }
    lastUpdate = millis();
  }

  handleTCPClient();
}

void floatToRegisters(float val, int startIndex, int slaveIndex) {
  byte bytes[4];
  memcpy(bytes, &val, 4);
  holdingRegs[slaveIndex][startIndex]     = (bytes[1] << 8) | bytes[0];
  holdingRegs[slaveIndex][startIndex + 1] = (bytes[3] << 8) | bytes[2];
}

float readModbusFloat(uint8_t slaveId, uint16_t reg) {
  byte request[] = {slaveId, FUNCTION_CODE, highByte(reg), lowByte(reg), 0x00, 0x02, 0x00, 0x00};
  uint16_t crc = calculateCRC(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  digitalWrite(DE_RE_PIN, HIGH);
  digitalWrite(DERE_LED, HIGH);
  digitalWrite(TX_LED, HIGH);
  delay(5);
  RS485.write(request, sizeof(request));
  RS485.flush();
  delay(2);
  digitalWrite(DE_RE_PIN, LOW);
  digitalWrite(DERE_LED, LOW);
  digitalWrite(TX_LED, LOW);

  byte response[9];
  int index = 0;
  unsigned long timeout = millis() + 1000;
  while (millis() < timeout && index < 9) {
    if (RS485.available()) {
      response[index++] = RS485.read();
    }
  }

  if (index < 9 || response[1] != FUNCTION_CODE) {
    Serial.println("❌ Error: Invalid response!");
    return NAN;
  }

  digitalWrite(RX_LED, HIGH);
  delay(10);
  digitalWrite(RX_LED, LOW);

  byte floatBytes[4] = { response[4], response[3], response[6], response[5] };
  float result;
  memcpy(&result, floatBytes, sizeof(result));
  return result;
}

void handleTCPClient() {
  if (tcpServer.hasClient()) {
    if (client) client.stop();
    client = tcpServer.available();
    Serial.println("Modbus TCP client connected");
  }

  if (client && client.connected() && client.available() >= 12) {
    byte request[12];
    int bytesRead = 0;
    while (client.available() && bytesRead < 12) {
      request[bytesRead++] = client.read();
    }

    if (bytesRead < 12) {
      Serial.println("❌ Incomplete TCP request received");
      return;
    }

    Serial.print("Received Request: ");
    for (int i = 0; i < 12; i++) {
      Serial.printf("%02X ", request[i]);
    }
    Serial.println();

    uint8_t unitID = request[6];  // Use Unit ID to select slave
    int slaveIndex = -1;
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (SLAVE_IDS[i] == unitID) {
        slaveIndex = i;
        break;
      }
    }

    if (slaveIndex == -1) {
      Serial.println("❌ Invalid Slave ID in TCP request");
      return;
    }

    uint16_t startAddr = (request[8] << 8) | request[9];
    uint16_t quantity  = (request[10] << 8) | request[11];

    if (startAddr >= 118 || (startAddr + quantity) > 118) {
      Serial.println("❌ Invalid Register Request");
      return;
    }

    byte response[9 + quantity * 2];
    memcpy(response, request, 2);  // Transaction ID
    response[2] = 0x00;
    response[3] = 0x00;
    response[4] = 0x00;
    response[5] = 3 + quantity * 2;
    response[6] = unitID;
    response[7] = FUNCTION_CODE;
    response[8] = quantity * 2;

    for (int i = 0; i < quantity; i++) {
      uint16_t val = holdingRegs[slaveIndex][startAddr + i];
      response[9 + i * 2] = val >> 8;
      response[9 + i * 2 + 1] = val & 0xFF;
    }

    client.write(response, 9 + quantity * 2);
    Serial.println("✅ Response Sent");
  }
}


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
