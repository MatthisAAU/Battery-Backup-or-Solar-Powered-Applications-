#include <Adafruit_INA260.h>

// INA260 instances for Solar and Battery/Grid
Adafruit_INA260 ina260_solar;  // Address 0x45
Adafruit_INA260 ina260_battery;  // Address 0x40

// Pin definitions
const int relayControlPin = 8;       // Digital pin to control the relay
const int gridActiveLedPin = 6;      // LED for Grid Active
const int solarBatteryActiveLedPin = 5; // LED for Solar/Battery Active

// Battery and SOC parameters
const float V_max = 13.2;              // Open-circuit voltage at 100% SOC
const float V_min = 10.5;              // Open-circuit voltage at 0% SOC
const float batteryCapacity = 6.34;    // Total capacity in Ah
const float usableCapacity = batteryCapacity * 0.5; // 50% usable capacity (3.17 Ah)
const float internalResistance = 0.018; // Internal resistance in Ohms

float socOpenCircuit = 100.0;             // SOC based on open-circuit voltage
float socDynamic = 100.0;              // SOC based on current integration
float socUse = 100.0;                  // SOC relative to 50% usable capacity

// Time management
unsigned long lastDischarge = 0;       // Last discharge time in ms
unsigned long lastCharge = 0;          // Last charge time in ms
unsigned long lastUpdate = 0;          // Last update time in ms
const unsigned long updateInterval = 1000; // 1-second interval
const unsigned long dischargeTimeout = 30 * 60 * 1000; // 30 minutes
const unsigned long chargeTimeout = 5 * 60 * 60 * 1000; // 5 hours

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  while (!Serial);

  // Initialize INA260 sensors
  if (!ina260_solar.begin(0x45)) {
    Serial.println("Failed to initialize INA260 for Solar! Check connections.");
    while (1);
  }
  Serial.println("INA260 for Solar initialized.");

  if (!ina260_battery.begin(0x40)) {
    Serial.println("Failed to initialize INA260 for Battery! Check connections.");
    while (1);
  }
  Serial.println("INA260 for Battery initialized.");

  // Configure pins
  pinMode(relayControlPin, OUTPUT);
  pinMode(gridActiveLedPin, OUTPUT);
  pinMode(solarBatteryActiveLedPin, OUTPUT);

  // Ensure initial states
  digitalWrite(relayControlPin, LOW);        // NC active initially
  digitalWrite(gridActiveLedPin, LOW);       // Grid LED off initially
  digitalWrite(solarBatteryActiveLedPin, HIGH); // Solar/Battery LED on initially

  // Initial SOC calculation from open-circuit voltage
  socLeerlauf = calculateSOCFromVoltage(ina260_battery.readBusVoltage() / 1000.0);
}

void loop() {
  unsigned long currentTime = millis();

  // Update every second
  if (currentTime - lastUpdate >= updateInterval) {
    lastUpdate = currentTime;

    // Read solar and battery voltages
    float solarVoltage = ina260_solar.readBusVoltage() / 1000.0;  // Convert to Volts
    float batteryVoltage = ina260_battery.readBusVoltage() / 1000.0;  // Convert to Volts

    // Read solar and battery currents
    float solarCurrent = ina260_solar.readCurrent() / 1000.0;  // Convert to Amps
    float batteryCurrent = ina260_battery.readCurrent() / 1000.0;  // Convert to Amps

    float deltaTime = updateInterval / 3600000.0; // Time in hours

    // Update SOC dynamically based on charging or discharging
    if (batteryCurrent > 0.005) { // Discharging only if current > 5mA
      float dischargedAh = batteryCurrent * deltaTime; // Discharged capacity in Ah
      socDynamic -= (dischargedAh / batteryCapacity) * 100.0;

      // Calculate energy loss due to internal resistance
      float energyLoss = pow(batteryCurrent, 2) * internalResistance * deltaTime;
      socDynamic -= (energyLoss / (batteryCapacity * 12.0)) * 100.0;

      lastDischarge = currentTime; // Update last discharge time
    } else if (batteryCurrent < -0.005) { // Charging only if current > 5mA
      float chargedAh = (-batteryCurrent) * deltaTime; // Charged capacity in Ah
      socDynamic += (chargedAh / batteryCapacity) * 100.0;

      // Calculate energy loss due to internal resistance
      float energyLoss = pow(batteryCurrent, 2) * internalResistance * deltaTime;
      socDynamic -= (energyLoss / (batteryCapacity * 12.0)) * 100.0;

      lastCharge = currentTime; // Update last charge time
    }

    // Use open-circuit SOC if no charge/discharge activity for specified time
    if ((currentTime - lastDischarge >= dischargeTimeout) && 
        (currentTime - lastCharge >= chargeTimeout)) {
      socLeerlauf = calculateSOCFromVoltage(batteryVoltage);
    }

    // Combine open-circuit and dynamic SOC
    socUse = socLeerlauf + socDynamic - 100.0;
    socUse = constrain(socUse, 0.0, 100.0); // Constrain SOC Use to 0–100%

    // Relay control based on SOC and solar voltage
    if (socUse < 50.0 && solarVoltage < 12.0) {
      digitalWrite(relayControlPin, HIGH); // Switch to NO (Grid Active)
      digitalWrite(gridActiveLedPin, HIGH); // Turn on Grid LED
      digitalWrite(solarBatteryActiveLedPin, LOW); // Turn off Solar/Battery LED
      Serial.println("Grid Active: Low SoC and insufficient solar.");
    } else {
      digitalWrite(relayControlPin, LOW); // Switch to NC (Solar/Battery Active)
      digitalWrite(gridActiveLedPin, LOW); // Turn off Grid LED
      digitalWrite(solarBatteryActiveLedPin, HIGH); // Turn on Solar/Battery LED
      Serial.println("Solar/Battery Active: Sufficient solar or SOC.");
    }

    // Serial Monitor: Display all relevant data
    Serial.println("--- Monitoring Values ---");
    Serial.print("Solar Voltage: ");
    Serial.println(solarVoltage, 2);
    Serial.print("Solar Current: ");
    Serial.println(solarCurrent, 3);

    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage, 2);
    Serial.print("Battery Current: ");
    Serial.println(batteryCurrent, 3);

    Serial.print("SoC Leerlauf: ");
    Serial.print(socLeerlauf);
    Serial.println(" %");

    Serial.print("SoC Use: ");
    Serial.print(socUse);
    Serial.println(" %");

    Serial.print("Relay Status: ");
    Serial.println(digitalRead(relayControlPin) == HIGH ? "Grid Active" : "Solar/Battery Active");

    Serial.println("-----------------------");
  }

  delay(1000); // Stability delay
}

// Function to calculate SOC from open-circuit voltage
float calculateSOCFromVoltage(float voltage) {
  if (voltage >= V_max) return 100.0; // Above 100%
  if (voltage <= V_min) return 0.0;   // Below 0%
  return (voltage - V_min) / (V_max - V_min) * 100.0;
}
