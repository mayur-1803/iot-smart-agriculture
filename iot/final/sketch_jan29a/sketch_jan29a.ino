#define BLYNK_TEMPLATE_ID "TMPL3ktSztXVk"
#define BLYNK_TEMPLATE_NAME "GREENHOUSE"
#define BLYNK_AUTH_TOKEN "nN_5WzDcNd1744jQ6fUjqrc3JzFp2ma7"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

char ssid[] = "GANPATI";
char pass[] = "PAPAKOPATAHAI";
char auth[] = "nN_5WzDcNd1744jQ6fUjqrc3JzFp2ma7";

#define DHTPIN 4    // Pin where the DHT sensor is connected (GPIO 4)
#define SOIL_MOISTURE_PIN 34
#define RAIN_SENSOR_PIN 13
#define PUMP_RELAY_PIN 14
#define RAIN_LED_PIN V5
#define PUMP_LED_PIN V0
#define PUMP_SWITCH_PIN V6
#define BATTERY_PIN 35

#define DHTTYPE DHT11  // DHT sensor type

DHT dht(DHTPIN, DHTTYPE);

unsigned long lastRelayChangeTime = 0;
unsigned long debounceDelay = 5000;
bool pumpStatus = false;
bool manualMode = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Change the I2C address if needed

void turnOnPump() {
  digitalWrite(PUMP_RELAY_PIN, LOW);
  digitalWrite(PUMP_LED_PIN, HIGH);
  pumpStatus = true;
  Blynk.virtualWrite(PUMP_LED_PIN, HIGH);
  Serial.println("Pump turned on");
}

void turnOffPump() {
  digitalWrite(PUMP_RELAY_PIN, HIGH);
  digitalWrite(PUMP_LED_PIN, LOW);
  pumpStatus = false;
  Blynk.virtualWrite(PUMP_LED_PIN, LOW);
  Serial.println("Pump turned off");
}

float getBatteryPercentage() {
  float batteryVoltage = getBatteryVoltage();
  // Assuming linear relationship: minVoltage corresponds to 0%, maxVoltage corresponds to 100%
  float minVoltage = 1.65; // minimum voltage (empty battery)
  float maxVoltage = 4.2;  // maximum voltage (fully charged battery)
  
  // Calculate battery percentage
  float batteryPercentage = ((batteryVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
  // Ensure the battery percentage is within the range 0-100
  batteryPercentage = constrain(batteryPercentage, 0, 100);

  return batteryPercentage;
}

float getBatteryVoltage() {
  int batteryLevel = analogRead(BATTERY_PIN);
  return map(batteryLevel, 0, 4095, 165, 575) / 100.0;
}

BLYNK_WRITE(PUMP_SWITCH_PIN) {
  int pumpSwitchState = param.asInt();
  if (pumpSwitchState == HIGH) {
    manualMode = true;
    turnOnPump();
  } else {
    manualMode = false;
    turnOffPump();
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("Initializing...");

  Blynk.begin(auth, ssid, pass);

  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(RAIN_LED_PIN, OUTPUT);
  pinMode(PUMP_LED_PIN, OUTPUT);

  // Set initial states to ensure the relay is off
  digitalWrite(PUMP_RELAY_PIN, HIGH);
  digitalWrite(RAIN_LED_PIN, LOW);
  digitalWrite(PUMP_LED_PIN, LOW);

  lcd.init();
  lcd.backlight();

  displayInitialParameters();
}

void loop() {
  Blynk.run();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int soilMoisture = analogRead(SOIL_MOISTURE_PIN);
  int rainSensorValue = digitalRead(RAIN_SENSOR_PIN);
  int mappedSoilMoisture = map(soilMoisture, 2300, 4095, 100, 0);
  float batteryPercentage = getBatteryPercentage();

  Blynk.syncVirtual(PUMP_SWITCH_PIN);

  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.println("Reading Sensors...");

    Serial.print("Temperature: ");
    Serial.println(temperature);

    Serial.print("Humidity: ");
    Serial.println(humidity);

    Serial.print("Soil Moisture: ");
    Serial.println(mappedSoilMoisture);

    Serial.print("Battery Percentage: ");
    Serial.print(batteryPercentage);
    Serial.println("%");

    Blynk.virtualWrite(V2, temperature);
    Blynk.virtualWrite(V3, humidity);
    Blynk.virtualWrite(V4, mappedSoilMoisture);
    Blynk.virtualWrite(V1, batteryPercentage);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" T=");
    lcd.print(temperature);
    lcd.print("  H=");
    lcd.print(humidity);
    lcd.print("  BAT=");
    lcd.print(batteryPercentage);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("  SM=");
    lcd.print(mappedSoilMoisture);
    lcd.print("%  RAIN=");
    lcd.print(rainSensorValue == LOW ? "Yes " : "No  ");
    lcd.print("PUMP=");
    lcd.print(pumpStatus ? "On " : "Off");

    if (!rainSensorValue) {
      Blynk.virtualWrite(V5, HIGH);
      digitalWrite(RAIN_LED_PIN, HIGH);
      Serial.println("Rain Detected!");
    } else {
      Blynk.virtualWrite(V5, LOW);
      digitalWrite(RAIN_LED_PIN, LOW);
      Serial.println("No Rain Detected.");

      if (!manualMode) {
        // Check soil moisture conditions only in automatic mode
        if (!pumpStatus && mappedSoilMoisture < 20 && (millis() - lastRelayChangeTime >= debounceDelay)) {
          turnOnPump();
          lastRelayChangeTime = millis();
        } else if (pumpStatus && mappedSoilMoisture > 80 && (millis() - lastRelayChangeTime >= debounceDelay)) {
          turnOffPump();
          lastRelayChangeTime = millis();
        }
      }
    }

    scrollLCD();

    Serial.println("Waiting...");
  } else {
    Serial.println("Error reading DHT sensor. Check connections and sensor type.");
  }

  delay(1000);
}

void displayInitialParameters() {
  Serial.println("Initial Parameters:");
  Serial.print("Temperature: ");
  Serial.println(dht.readTemperature());
  Serial.print("Humidity: ");
  Serial.println(dht.readHumidity());
  Serial.println("--------------");
}

void scrollLCD() {
  for (int position = 0; position < 16; position++) {
    lcd.scrollDisplayLeft();
    delay(250);
  }
}
