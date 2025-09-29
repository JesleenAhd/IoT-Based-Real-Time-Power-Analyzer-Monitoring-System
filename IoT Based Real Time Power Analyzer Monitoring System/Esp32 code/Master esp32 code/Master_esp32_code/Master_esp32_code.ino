#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Wi-Fi credentials
const char* ssid = "HUAWEI Y7a";
const char* password = "20011125";

// Web server & TCP server
AsyncWebServer server(80);
WiFiServer tcpServer(502);
WiFiClient client;

// Slave configuration
#define NUM_SLAVES 14
const uint8_t SLAVE_IDS[NUM_SLAVES] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};
#define FUNCTION_CODE 0x04  // Read Input Registers
#define REG_COUNT 136       // 68 float parameters = 136 registers

// Register buffers per slave
uint16_t holdingRegs[NUM_SLAVES][REG_COUNT];

// WebServer setup
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start Modbus TCP server
  tcpServer.begin();
  tcpServer.setNoDelay(true);

  // HTTP GET /send handler
  server.on("/send", HTTP_GET, [](AsyncWebServerRequest *request) {
    int slaveIndex = -1;
    int slaveID = -1;

    // Identify the slave
    for (int i = 0; i < request->params(); i++) {
      const AsyncWebParameter* p = request->getParam(i);
      String name = p->name();
      String value = p->value();
      if (name == "slave") {
        slaveID = value.toInt();
        for (int j = 0; j < NUM_SLAVES; j++) {
          if (SLAVE_IDS[j] == slaveID) {
            slaveIndex = j;
            break;
          }
        }
      }
    }

    if (slaveIndex == -1) {
      Serial.println("❌ Invalid or missing slave ID");
      request->send(400, "text/plain", "Invalid slave ID");
      return;
    }

    Serial.print("📥 Received for Slave ID: ");
    Serial.println(SLAVE_IDS[slaveIndex]);

    // Process all parameters except "slave"
    int regCounter = 0;
    for (int i = 0; i < request->params(); i++) {
      const AsyncWebParameter* p = request->getParam(i);
      String name = p->name();
      String value = p->value();

      // Inside your /send handler
      if (name != "slave") {
        float f = value.toFloat();
        uint8_t* bytePtr = (uint8_t*)&f;

        // Swap to 32-bit Float Little-endian with byte-swapping inside each 16-bit register
        uint16_t reg1 = (bytePtr[1] << 8) | bytePtr[0]; // Low word: swapped little-endian
        uint16_t reg2 = (bytePtr[3] << 8) | bytePtr[2]; // High word: swapped little-endian

        Serial.print(name);
        Serial.print(" = ");
        Serial.print(f);
        Serial.printf(" => Reg1 (Low): 0x%04X, Reg2 (High): 0x%04X\n", reg1, reg2);

        holdingRegs[slaveIndex][regCounter * 2]     = reg1; // Low word
        holdingRegs[slaveIndex][regCounter * 2 + 1] = reg2; // High word

        regCounter++;
      }



    }

    // Final register snapshot
    Serial.println("📦 Registers for slave " + String(slaveID));
    for (int i = 0; i < regCounter * 2; i++) {
      Serial.print(holdingRegs[slaveIndex][i]);
      Serial.print(" ");
    }
    Serial.println();

    request->send(200, "text/plain", "✅ Data received");
  });




  server.begin();
  Serial.println("📡 Server started and listening...");
}

// Modbus TCP Handler
void handleTCPClient() {
  if (tcpServer.hasClient()) {
    if (client) client.stop();
    client = tcpServer.available();
    Serial.println("🔌 Modbus TCP client connected");
  }

  if (client && client.connected() && client.available() >= 12) {
    byte request[12];
    int bytesRead = client.readBytes(request, 12);

    if (bytesRead != 12) {
      Serial.println("❌ Incomplete request");
      return;
    }

    uint8_t unitID = request[6];
    int slaveIndex = -1;
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (SLAVE_IDS[i] == unitID) {
        slaveIndex = i;
        break;
      }
    }

    if (slaveIndex == -1) {
      Serial.println("❌ Unknown Slave ID");
      return;
    }

    uint16_t startAddr = (request[8] << 8) | request[9];
    uint16_t quantity  = (request[10] << 8) | request[11];

    if ((startAddr + quantity) > REG_COUNT) {
      Serial.println("❌ Invalid register range");
      return;
    }

    byte response[9 + quantity * 2];
    memcpy(response, request, 2); // Copy Transaction ID
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
    Serial.println("✅ Response sent");
  }
}

// Main loop
void loop() {
  handleTCPClient();
}
