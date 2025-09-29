#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>

// Serial configuration
#define SerialMon Serial
#define MODEM_RX 16
#define MODEM_TX 17
#define SerialAT Serial2

TinyGsm modem(SerialAT);

void setup() {
  SerialMon.begin(115200);
  delay(10);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  SerialMon.println("=== SIM7600 SIGNAL DIAGNOSTIC TOOL ===");
  
  // Initialize modem
  SerialMon.println("Initializing modem...");
  modem.restart();
  delay(5000);

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Check SIM card
  SerialMon.println("Checking SIM card...");
  if (modem.getSimStatus() != 1) {
    SerialMon.println("SIM card not ready!");
    while (true) delay(1000);
  }
  SerialMon.println("SIM card OK");

  // Comprehensive signal analysis
  performSignalDiagnostic();
  
  // Check network bands and registration
  checkNetworkDetails();
  
  // Power and antenna checks
  checkHardwareStatus();
}

void performSignalDiagnostic() {
  SerialMon.println("\n=== SIGNAL QUALITY ANALYSIS ===");
  
  int signalReadings[20];
  int validReadings = 0;
  int sumSignal = 0;
  
  SerialMon.println("Taking 20 signal readings (1 per second):");
  
  for(int i = 0; i < 20; i++) {
    int signal = modem.getSignalQuality();
    signalReadings[i] = signal;
    
    SerialMon.print("Reading ");
    SerialMon.print(i+1);
    SerialMon.print(": ");
    SerialMon.print(signal);
    
    if(signal > 0 && signal < 32) {
      SerialMon.print(" (Valid - ");
      if(signal < 10) SerialMon.print("Very Poor");
      else if(signal < 15) SerialMon.print("Poor");
      else if(signal < 20) SerialMon.print("Fair");
      else if(signal < 25) SerialMon.print("Good");
      else SerialMon.print("Excellent");
      SerialMon.println(")");
      
      validReadings++;
      sumSignal += signal;
    } else if(signal == 99) {
      SerialMon.println(" (Invalid/No Antenna)");
    } else {
      SerialMon.println(" (No Signal)");
    }
    
    delay(1000);
  }
  
  // Analysis
  SerialMon.println("\n--- SIGNAL ANALYSIS ---");
  SerialMon.print("Valid readings: ");
  SerialMon.print(validReadings);
  SerialMon.print("/20");
  
  if(validReadings > 0) {
    int avgSignal = sumSignal / validReadings;
    SerialMon.print(" | Average: ");
    SerialMon.println(avgSignal);
    
    if(avgSignal < 10) {
      SerialMon.println("DIAGNOSIS: Very poor signal - Check antenna and location");
    } else if(avgSignal < 15) {
      SerialMon.println("DIAGNOSIS: Poor signal - Try different location or external antenna");
    } else if(avgSignal < 20) {
      SerialMon.println("DIAGNOSIS: Fair signal - Should work but may be unstable");
    } else {
      SerialMon.println("DIAGNOSIS: Good signal - Connection should be stable");
    }
  } else {
    SerialMon.println("DIAGNOSIS: No valid signal readings - Check antenna connection!");
  }
}

void checkNetworkDetails() {
  SerialMon.println("\n=== NETWORK REGISTRATION DETAILS ===");
  
  // Check registration status
  SerialMon.print("Registration Status: ");
  int regStatus = modem.getRegistrationStatus();
  SerialMon.print(regStatus);
  switch(regStatus) {
    case 0: SerialMon.println(" - Not searching"); break;
    case 1: SerialMon.println(" - Registered (Home)"); break;
    case 2: SerialMon.println(" - Searching"); break;
    case 3: SerialMon.println(" - Registration denied"); break;
    case 5: SerialMon.println(" - Registered (Roaming)"); break;
    default: SerialMon.println(" - Unknown status"); break;
  }
  
  // Get operator info
  String operatorName = modem.getOperator();
  SerialMon.print("Operator: ");
  SerialMon.println(operatorName);
  
  // Check network mode
  SerialMon.println("Getting network mode...");
  modem.sendAT("+CNMP?"); // Network mode preference
  delay(1000);
  
  // Check band selection
  SerialMon.println("Checking band selection...");
  modem.sendAT("+CBANDCFG?"); // Band configuration
  delay(1000);
}

void checkHardwareStatus() {
  SerialMon.println("\n=== HARDWARE STATUS CHECK ===");
  
  // Check power supply voltage
  SerialMon.println("Checking power supply...");
  modem.sendAT("+CBC"); // Battery/power status
  delay(1000);
  
  // Check temperature
  SerialMon.println("Checking temperature...");
  modem.sendAT("+CPMUTEMP"); // Module temperature
  delay(1000);
  
  // Antenna detection test
  SerialMon.println("Testing antenna detection...");
  modem.sendAT("+CIND?"); // Signal indicators
  delay(1000);
}

void loop() {
  SerialMon.println("\n=== CONTINUOUS MONITORING ===");
  SerialMon.println("Press any key to stop monitoring and get recommendations...");
  
  for(int i = 0; i < 60; i++) { // Monitor for 1 minute
    if(SerialMon.available()) {
      SerialMon.read(); // Clear buffer
      break;
    }
    
    int signal = modem.getSignalQuality();
    SerialMon.print("Signal: ");
    SerialMon.print(signal);
    SerialMon.print(" | Network: ");
    SerialMon.print(modem.isNetworkConnected() ? "OK" : "FAIL");
    SerialMon.print(" | Reg: ");
    SerialMon.println(modem.getRegistrationStatus());
    
    delay(1000);
  }
  
  // Provide recommendations
  giveRecommendations();
  
  delay(10000); // Wait 10 seconds before next cycle
}

void giveRecommendations() {
  SerialMon.println("\n=== TROUBLESHOOTING RECOMMENDATIONS ===");
  SerialMon.println("1. ANTENNA: Ensure antenna is firmly connected");
  SerialMon.println("2. LOCATION: Try moving to a window or outdoors");
  SerialMon.println("3. POWER: Check 3.7-4.2V stable power supply (2A capable)");
  SerialMon.println("4. INTERFERENCE: Move away from WiFi routers, computers");
  SerialMon.println("5. ANTENNA TYPE: Consider external antenna if signal is consistently low");
  SerialMon.println("6. CARRIER: Verify Dialog coverage in your area");
  SerialMon.println("\nIf signal readings show 99 frequently = Antenna problem");
  SerialMon.println("If signal is consistently <15 = Location/coverage problem");
  SerialMon.println("If signal varies wildly = Power supply problem");
}