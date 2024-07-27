#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "deneyap.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include "utils/screen.cpp" 
#include <TimeLib.h>
#include "DHT.h"

#include <Stepper.h>

#define DHTTYPE DHT22
int tamtur = 2048;

#define ampulPin D8
#define ampulFanPin D0
#define ampulFanSpeedPin D1
#define wifiConnectedLedPin DAC1
#define highLowTempLedPin DAC2
#define highLowHumLedPin D9
#define lowWaterLevelLedPin MOSI
#define waterSensorEnablePin MISO
#define waterSensorDataPin A0
#define egzozFanEnablePin A5
#define dht22Pin D4

#define stepIN1 D12
#define stepIN2 D13
#define stepIN3 D14
#define stepIN4 D15

#include <analogWrite.h>

// LCD ve diğer global değişkenler
LiquidCrystal_I2C lcd(0x27, 20, 4);
LcdUtil lcdUtil;
AsyncWebServer server(80);
Preferences preferences;
unsigned long lastUpdateTime = 0; // Son güncelleme zamanı
const unsigned long updateInterval = 60000; // 1 dakika (60,000 milisaniye)
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 30000;
unsigned long dhtdataAttempt = 0;
const unsigned long dhtDataInterval = 3000;
unsigned long lastTurnTime = 0;
const unsigned long turnInterval = 4 * 60 * 60 * 1000;

float targetTemp = 37.5;
float lowerTempLimit = 37.2;
float upperTempLimit = 37.8;

float targetHumidity = 55.0;
float lowerHumidityLimit = 50.0;
float upperHumidityLimit = 60.0;

//gun 18>
int oldingDay = 18;
float targetTempA = 37.0;
float lowerTempLimitA = 36.7;
float upperTempLimitA = 37.3;

float targetHumidityA = 65.0;
float lowerHumidityLimitA = 60.0;
float upperHumidityLimitA = 70.0;

//danger

float tempUp = 38.0;
float tempdown = 36.0;

float humUp = 75.0;
float humdown = 50.0;


int stepSayisi = 800;

int suSeviyeLimiti = 1000;

String ssid;
String password;
IPAddress local_IP(192, 168, 1, 32);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

Stepper step_motorum = Stepper(tamtur, stepIN1, stepIN2, stepIN3, stepIN4);

float globalTemp = 0;
float globalHum = 0;
int globalWaterLevel = 0;
String globalMotor = "";
int globalFanSpeed = 0;
boolean globalAmpulEnable = false;
boolean globalEgzozEnable = false;
DHT dht(dht22Pin, DHTTYPE);

int waterLevel = 0;


int readWaterSensor() {
	digitalWrite(waterSensorEnablePin, HIGH);	// Turn the sensor ON
	delay(10);							// wait 10 milliseconds
	waterLevel = analogRead(waterSensorDataPin);		// Read the analog value form sensor
	digitalWrite(waterSensorEnablePin, LOW);		// Turn the sensor OFF
	return waterLevel;							// send current reading
}

void notFound(AsyncWebServerRequest *request) {
    JsonDocument doc;
    String response;
    doc["success"] = 0;
    doc["message"] = "Not Found";
    serializeJson(doc, response);
    request->send(404, "application/json; charset=utf-8", response);
}
long startTime;

void setup() {
  Serial.begin(9600);

  
  // Pin ayarları
  pinMode(ampulPin, OUTPUT);
  pinMode(ampulFanPin, OUTPUT);
  pinMode(ampulFanSpeedPin, OUTPUT);
  
  pinMode(wifiConnectedLedPin, OUTPUT);
  pinMode(highLowTempLedPin, OUTPUT);
  pinMode(highLowHumLedPin, OUTPUT);
  pinMode(lowWaterLevelLedPin, OUTPUT);
  pinMode(waterSensorEnablePin, OUTPUT);
  pinMode(egzozFanEnablePin, OUTPUT);

  digitalWrite(waterSensorEnablePin, LOW);

  digitalWrite(wifiConnectedLedPin, HIGH);
  digitalWrite(ampulPin, HIGH);
  digitalWrite(highLowTempLedPin, HIGH);
  digitalWrite(highLowHumLedPin, HIGH);
  digitalWrite(lowWaterLevelLedPin, HIGH);
  delay(2000);


  digitalWrite(wifiConnectedLedPin, LOW);
  digitalWrite(ampulPin, LOW);
  digitalWrite(highLowTempLedPin, LOW);
  digitalWrite(highLowHumLedPin, LOW);
  digitalWrite(lowWaterLevelLedPin, LOW);

  


  preferences.begin("my-app", false);
  
  ssid = preferences.getString("ssid", "CAZEB LOCAL SERVER");
  password = preferences.getString("password", "cazeb4657");
  setTime(preferences.getInt("hour", 0), preferences.getInt("minute", 0), preferences.getInt("seconds", 0),
          preferences.getInt("day", 1), preferences.getInt("month", 1), preferences.getInt("year", 2024));

  // LCD ekranını başlatma
  lcd.init(SDA, SCL); // Bu metod genellikle init içinde gerekli parametrelerle çağrılır
  lcd.backlight();
  lcdUtil.begin(lcd);
  lcdUtil.printCentered("Kulucka v3", 0);
  lcdUtil.printCentered("Hosgeldiniz", 2);
  delay(3000);
  lcd.clear();

  // WiFi bağlantısı
  lcdUtil.printCentered("Wi-fiye baglaniliyor", 0);
  lcdUtil.printCentered(ssid.c_str(), 2);
  lcdUtil.printCentered(password.c_str(), 3);

  WiFi.mode(WIFI_STA);
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    lcdUtil.printCentered("STA Failed to configure", 0);
  }
  WiFi.begin(ssid.c_str(), password.c_str());

  // WiFi bağlantı kontrolü
  startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) { // 30 saniye zaman aşımı
    delay(500); // Yarım saniye bekle
    lcdUtil.printCentered("Wi-fiye baglaniliyor", 0);
    lcdUtil.printCentered(ssid.c_str(), 2);
    lcdUtil.printCentered(WiFi.localIP().toString().c_str(), 3);
  }

  if (WiFi.status() == WL_CONNECTED) {
    lcdUtil.printCentered("Wi-fiye baglanildi", 0);
  } else {
    lcdUtil.printCentered("Wifiye baglanilamadi", 0);
  }
  dht.begin();
  char buffer[20]; 
    snprintf(buffer, sizeof(buffer), "G:%02d S:%02d D:%02d", day(), hour(), minute());
    lcdUtil.printCentered(buffer, 3);
  server.on("/getdata", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String response;
        JsonDocument doc;
        doc["success"] = 1;
        doc["message"] = "no error";
        doc["day"] = day();
        doc["hour"] = hour();
        doc["minute"] = minute();
        doc["ssid"] = ssid;
        doc["password"] = password;

        doc["lastUpdateTime"] = lastUpdateTime;
        doc["updateInterval"] = updateInterval;
        doc["lastReconnectAttempt"] = lastReconnectAttempt;
        doc["reconnectInterval"] = reconnectInterval;
        doc["dhtdataAttempt"] = dhtdataAttempt;
        doc["dhtDataInterval"] = dhtDataInterval;
        doc["lastTurnTime"] = lastTurnTime;
        doc["turnInterval"] = turnInterval;

        doc["targetTemp"] = targetTemp;
        doc["lowerTempLimit"] = lowerTempLimit;
        doc["upperTempLimit"] = upperTempLimit;

        doc["targetHumidity"] = targetHumidity;
        doc["lowerHumidityLimit"] = lowerHumidityLimit;
        doc["upperHumidityLimit"] = upperHumidityLimit;

        doc["oldingDay"] = oldingDay;
        doc["targetTempA"] = targetTempA;
        doc["lowerTempLimitA"] = lowerTempLimitA;
        doc["upperTempLimitA"] = upperTempLimitA;

        doc["targetHumidityA"] = targetHumidityA;
        doc["lowerHumidityLimitA"] = lowerHumidityLimitA;
        doc["upperHumidityLimitA"] = upperHumidityLimitA;

        doc["tempUp"] = tempUp;
        doc["tempdown"] = tempdown;

        doc["humUp"] = humUp;
        doc["humdown"] = humdown;

        doc["stepSayisi"] = stepSayisi;
        doc["suSeviyeLimiti"] = suSeviyeLimiti;

        doc["temp"] = globalTemp;
        doc["hum"] = globalHum;
        doc["waterLevel"] = globalWaterLevel;
        doc["motor"] = globalMotor;

        doc["fanSpeed"] = globalFanSpeed; 
        doc["ampulEnable"] = globalAmpulEnable;
        doc["egzozEnable"] = globalEgzozEnable;


        serializeJson(doc, response);

        request->send(200, "application/json; charset=utf-8", response);
    });
  server.onNotFound(notFound);
  server.begin();

  
}

void loop() {
  if(timeStatus() != timeSet){
    setTime(preferences.getInt("hour", 0),preferences.getInt("minute", 0),preferences.getInt("seconds", 0),preferences.getInt("day", 1),preferences.getInt("month", 1),preferences.getInt("year", 2024));
  }
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdateTime >= updateInterval) {
    preferences.putInt("hour", hour());
    preferences.putInt("minute", minute());
    preferences.putInt("seconds", second());
    preferences.putInt("day", day());
    preferences.putInt("month", month());
    preferences.putInt("year", year());

    lastUpdateTime = currentTime;

    char buffer[20]; 
    snprintf(buffer, sizeof(buffer), "G:%02d S:%02d D:%02d", day(), hour(), minute());
    lcdUtil.printCentered(buffer, 3);
  }

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(wifiConnectedLedPin, LOW);

    // Try to reconnect at specified intervals
    if (currentTime - lastReconnectAttempt >= reconnectInterval) {
      WiFi.begin(ssid.c_str(), password.c_str());
      lastReconnectAttempt = currentTime;
    }
  } else {
    digitalWrite(wifiConnectedLedPin, HIGH);
  }

  if (currentTime - dhtdataAttempt >= dhtDataInterval) {
    dhtdataAttempt = currentTime;

    waterLevel = readWaterSensor();
    globalWaterLevel = waterLevel;
    if(waterLevel < suSeviyeLimiti) {
      digitalWrite(lowWaterLevelLedPin, HIGH);
    }else {
      digitalWrite(lowWaterLevelLedPin, LOW);
    }
    Serial.println(waterLevel);
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    globalTemp = t;
    globalHum = h;
    String firstLine = "ISI: " + String(t);
    String secondLine = "Nem: " + String(h) + "%";
    lcdUtil.printCentered(firstLine.c_str(), 0);
    lcdUtil.printCentered(secondLine.c_str(), 1);

    if(day() < oldingDay){
      unsigned long timeRemaining = turnInterval - (currentTime - lastTurnTime);
      int hoursRemaining = timeRemaining / (60 * 60 * 1000);
      int minutesRemaining = (timeRemaining % (60 * 60 * 1000)) / (60 * 1000);
      char remainingBuffer[20];
      snprintf(remainingBuffer, sizeof(remainingBuffer), "S:%02d D:%02d", hoursRemaining, minutesRemaining);
      globalMotor = remainingBuffer;
      lcdUtil.printCentered(remainingBuffer, 2);
    }else {
      lcdUtil.printCentered("Motor Çevrilmiyor", 2); 
    }


    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      globalHum = 0.0;
      globalTemp = 0.0;
      digitalWrite(ampulPin, LOW);
      globalAmpulEnable = false;
      return;
    }
    digitalWrite(ampulFanPin, HIGH);

    
    if(day() < oldingDay){
      if(t < lowerTempLimit){
        digitalWrite(ampulPin, HIGH);
        globalAmpulEnable = true;
      }else if(t > upperTempLimit){
        digitalWrite(ampulPin, LOW);
        globalAmpulEnable = false;
      }
    }else{
      if(t < lowerTempLimitA){
        digitalWrite(ampulPin, HIGH);
        globalAmpulEnable = true;
      }else if(t > upperTempLimitA){
        digitalWrite(ampulPin, LOW);
        globalAmpulEnable = false;
      }
    }

    
    if(day() < oldingDay){
      if(t > targetTemp){
        analogWrite(ampulFanSpeedPin, 255, 255);
        globalFanSpeed = 255;
      }else if(t > lowerTempLimit && t < targetTemp) {
        globalFanSpeed = 128;
        analogWrite(ampulFanSpeedPin, 128, 255);
      }else {
        analogWrite(ampulFanSpeedPin, 255, 255);
        globalFanSpeed = 255;
      }
    }else {
      if(t > targetTempA){
        analogWrite(ampulFanSpeedPin, 255, 255);
        globalFanSpeed = 255;
      }else if(t > lowerTempLimitA && t < targetTempA) {
        analogWrite(ampulFanSpeedPin, 128, 255);
        globalFanSpeed = 128;
      }else {
        analogWrite(ampulFanSpeedPin, 255, 255);
        globalFanSpeed = 255;
      }
    }
    

    if(day() < oldingDay){
      if(h > upperHumidityLimit){
        digitalWrite(egzozFanEnablePin, HIGH);
        globalEgzozEnable = true;
      }else{
        digitalWrite(egzozFanEnablePin, LOW);
        globalEgzozEnable = false;
      }
    }else {
      if(h > upperHumidityLimitA){
        digitalWrite(egzozFanEnablePin, HIGH);
        globalEgzozEnable = true;
      }else{
        digitalWrite(egzozFanEnablePin, LOW);
        globalEgzozEnable = false;
      }
    }

    if(t>tempUp){
      digitalWrite(egzozFanEnablePin, HIGH);
      globalEgzozEnable = true;
      digitalWrite(highLowTempLedPin, HIGH);
      
    }

    if(t < tempdown){
      digitalWrite(egzozFanEnablePin, LOW);
      globalEgzozEnable = false;
      digitalWrite(highLowTempLedPin, HIGH);
    }

    if(t< tempUp && t> tempdown){
      digitalWrite(highLowTempLedPin, LOW);
    }

    if(h < humdown){
      digitalWrite(highLowHumLedPin, HIGH);
    }

    if(h > humUp){
      digitalWrite(highLowHumLedPin, HIGH);
    }

    if(h<humUp && h>humdown){
      digitalWrite(highLowHumLedPin, LOW);
    }
    
  }

  if(currentTime - lastTurnTime > turnInterval){
    lastTurnTime = currentTime;
    step_motorum.setSpeed(5);
    step_motorum.step(stepSayisi);
  }


  
}
