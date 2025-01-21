#include <Arduino.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <Update.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266httpUpdate.h>
#endif

#include <Firebase_ESP_Client.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Token and Helper files
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Wi-Fi and Firebase credentials
#define WIFI_SSID "MCS"
#define WIFI_PASSWORD "mcs@413438"
#define API_KEY "AIzaSyDqZWuNj93LQYBMNPOsHa3SXfcam6EKvg0"
#define DATABASE_URL "https://iot-8266-5e007-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Relay and LED Pins
#define RELAY_PIN 5
#define RELAY_PIN2 4
#define LED_PIN LED_BUILTIN

// DHT sensor pins and type
#define DHT_PIN D7
#define DHT_TYPE DHT11
DHT sensor(DHT_PIN, DHT_TYPE);

// LCD I2C Address and dimensions
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Timers
unsigned long lastPollTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastScreenSwitchTime = 0;

// Intervals
const unsigned long pollInterval = 5000;  // Poll Firebase every 5 seconds
const unsigned long sensorInterval = 10000; // Read sensor data every 10 seconds
const unsigned long screenInterval = 5000;  // Switch screens every 5 seconds

// Current screen state
bool showRelayScreen = true;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);  // 300 ms delay for blinking
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  Serial.println("\nConnected with IP: " + WiFi.localIP().toString());
}

void setupFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase authentication successful");
  } else {
    Serial.printf("Firebase auth error: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// Poll Firebase for the current relay value
void pollFirebase() {
  if (Firebase.RTDB.getInt(&fbdo, "/relay")) {
    int intValue = fbdo.intData();
    digitalWrite(RELAY_PIN, intValue ? LOW : HIGH);
  }

  if (Firebase.RTDB.getInt(&fbdo, "/relay2")) {
    int intValue = fbdo.intData();
    digitalWrite(RELAY_PIN2, intValue ? LOW : HIGH);
  }
}

// Function to check for OTA updates in Firebase and apply it
// Function to check for OTA updates in Firebase and apply it
void checkForOTA() {
  String updateUrl;
  
  if (Firebase.RTDB.getString(&fbdo, "/ota/updateUrl")) {
    updateUrl = fbdo.stringData();
    Serial.print("Update URL: ");
    Serial.println(updateUrl);
    
    if (updateUrl != "") {
      Serial.println("Downloading firmware...");
      
      WiFiClient client;
      t_httpUpdate_return ret = ESPhttpUpdate.update(client, updateUrl);
      
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.println("Update Failed!");
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("No Update Available!");
          break;
        case HTTP_UPDATE_OK:
          Serial.println("Update Successful!");
          ESP.restart();
          break;
      }
    }
  }
}


// Display relay statuses on the LCD
void displayRelayStatus() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Outdoor Bulb: ");
  lcd.print(digitalRead(RELAY_PIN) == LOW ? "ON" : "OFF");
  
  lcd.setCursor(0, 1);
  lcd.print("Indoor Bulb: ");
  lcd.print(digitalRead(RELAY_PIN2) == LOW ? "ON " : "OFF");
}

// Display temperature and humidity on the LCD
void displayTempHumidity(float temperature, float humidity) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature, 1);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity, 1);
  lcd.print(" %");
}

void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);  // Initialize relay off
  digitalWrite(RELAY_PIN2, HIGH);  // Initialize relay off
  digitalWrite(LED_PIN, HIGH);     // Initialize LED off
  
  connectToWiFi();
  setupFirebase();

  // Initialize DHT sensor
  sensor.begin();

  // Initialize I2C on D0 (SDA) and D1 (SCL)
  Wire.begin(D5, D6);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
}

void loop() {
  // Poll Firebase for relay statuses
  if (millis() - lastPollTime >= pollInterval) {
    lastPollTime = millis();
    pollFirebase();
  }

  // Check for OTA update
  checkForOTA();

  // Reconnect Wi-Fi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  // Read sensor data every 10 seconds
  static float lastTemp = 0.0;
  static float lastHum = 0.0;

  if (millis() - lastSensorReadTime >= sensorInterval) {
    lastSensorReadTime = millis();
    float temperature = sensor.readTemperature();
    float humidity = sensor.readHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      lastTemp = temperature;
      lastHum = humidity;

      // Send to Firebase
      Firebase.RTDB.setFloat(&fbdo, "/temp", temperature);
      Firebase.RTDB.setFloat(&fbdo, "/hum", humidity);
    }
  }

  // Alternate screens every 5 seconds
  if (millis() - lastScreenSwitchTime >= screenInterval) {
    lastScreenSwitchTime = millis();
    showRelayScreen = !showRelayScreen;
  }

  if (showRelayScreen) {
    displayRelayStatus();
  } else {
    displayTempHumidity(lastTemp, lastHum);
  }
}
