// Inclusies
//githubtest
#include <WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_LTR390.h"

// WiFi-configuratie
const char* ssid = "Devadam";
const char* password = "hallo123";

// MQTT-configuratie
const char* mqtt_server = "broker.emqx.io";
const char* mqtt_topic_temp = "esp32/temperature";
const char* mqtt_topic_cadence = "esp32/cadence";
const char* mqtt_topic_distance = "esp32/distance";
const char* mqtt_topic_speed = "esp32/speed";
const char* mqtt_topic_bpm = "esp32/bpm";
const char* mqtt_topic_lux = "esp32/lux";

WiFiClient espClient;
PubSubClient client(espClient);

// LM35 Temperatuursensor
#define LM35_PIN 4
unsigned long tempPreviousMillis = 0;
const unsigned long tempInterval = 10000;

// Cadans en Afstand
int sensor = 1;     // Cadanssensor
int sensor1 = 2;    // Afstandsensor
int cadCounter = 0; // Cadans counter
int distCounter = 0; // Afstand counter
float afstandTussenMagneten, omtrekWiel = 0;
int aantalMagneet = 4;
unsigned long speedPreviousMillis = 0;
const unsigned long speedInterval = 10000;

// BPM (Hartslag)
int PulseSensorPurplePin = 0;
int Threshold = 2600;
int BPM;
unsigned long lastBeatTime = 0;

// Licht Sensor
Adafruit_LTR390 ltr = Adafruit_LTR390();
int Als_data = 0;
const int Voorlicht = 6;
const int Achterlicht = 7;
unsigned long luxPreviousMillis = 0;
const unsigned long luxInterval = 20000;

// Functie om verbinding met WiFi te maken
void setup_wifi() {
  Serial.print("Verbinden met WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi verbonden!");
  Serial.print("IP-adres: ");
  Serial.println(WiFi.localIP());
}

// Callback voor MQTT-berichten
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Bericht ontvangen op topic: ");
  Serial.println(topic);
}

// Functie om opnieuw verbinding te maken met de MQTT-broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Verbinden met MQTT-broker...");
    if (client.connect("ESP32_Integrated_Client")) { // Unieke Client-ID
      Serial.println("Verbonden!");
    } else {
      Serial.print("Verbinden mislukt, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// Initialisatie ////////////
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(LM35_PIN, INPUT);
  pinMode(sensor, INPUT);
  pinMode(sensor1, INPUT);
  pinMode(Voorlicht, OUTPUT);
  pinMode(Achterlicht, OUTPUT);

  // Licht Sensor Initialisatie
  if (!ltr.begin()) {
    Serial.println("LTR390-sensor niet gevonden!");
    while (1) delay(10);
  }
  ltr.setMode(LTR390_MODE_ALS);
  ltr.setGain(LTR390_GAIN_3);
  ltr.setResolution(LTR390_RESOLUTION_16BIT);

  // Berekening wielomtrek en afstand tussen magneten
  float pi = 3.1416;
  float diameterWiel = 12.0; // cm
  omtrekWiel = pi * diameterWiel;
  afstandTussenMagneten = omtrekWiel / aantalMagneet;
}

// Hoofdprogramma
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  // Temperatuurmeting
  if (currentMillis - tempPreviousMillis >= tempInterval) {
    tempPreviousMillis = currentMillis;

    int analogValue = analogRead(LM35_PIN);
    float voltage = analogValue * (3.3 / 4095.0);
    float temperatureC = voltage * 78;

    char tempString[8];
    dtostrf(temperatureC, 6, 2, tempString);
    Serial.print("Temperatuur: ");
    Serial.println(tempString);
    client.publish(mqtt_topic_temp, tempString);
  }

  // Cadans en Afstand
  int val = digitalRead(sensor);
  int val1 = digitalRead(sensor1);

  if (val == LOW) {
    cadCounter++;
    Serial.print("Cadans: ");
    Serial.println(cadCounter);
    client.publish(mqtt_topic_cadence, String(cadCounter).c_str());
  }

  if (val1 == LOW) {
    distCounter++;
    float afstand = afstandTussenMagneten * distCounter;
    Serial.print("Afstand: ");
    Serial.print(afstand);
    Serial.println(" cm");
    client.publish(mqtt_topic_distance, String(afstand).c_str());
  }

  // Snelheid
  if (currentMillis - speedPreviousMillis >= speedInterval) {
    speedPreviousMillis = currentMillis;

    static float vorigeAfstand = 0;
    float afgelegdeAfstand = afstandTussenMagneten * distCounter;
    float deltaAfstand = afgelegdeAfstand - vorigeAfstand;
    vorigeAfstand = afgelegdeAfstand;

    float snelheid = deltaAfstand / (speedInterval / 1000.0); // m/s
    Serial.print("Snelheid: ");
    Serial.print(snelheid);
    Serial.println(" m/s");
    client.publish(mqtt_topic_speed, String(snelheid).c_str());
  }

  // BPM Meting
  int Signal = analogRead(PulseSensorPurplePin);
  if (Signal > Threshold && currentMillis - lastBeatTime > 300) {
    unsigned long beatInterval = currentMillis - lastBeatTime;
    lastBeatTime = currentMillis;
    BPM = 60000 / beatInterval;
    Serial.print("BPM: ");
    Serial.println(BPM);
    client.publish(mqtt_topic_bpm, String(BPM).c_str());
  }

  // Licht Sensor
  if (currentMillis - luxPreviousMillis >= luxInterval) {
    luxPreviousMillis = currentMillis;

    if (ltr.newDataAvailable()) {
      Als_data = ltr.readALS() - 250;
      Als_data = max(Als_data, 0);
      Serial.print("Lichtintensiteit: ");
      Serial.print(Als_data);
      Serial.println(" LUX");

      client.publish(mqtt_topic_lux, String(Als_data).c_str());

      if (Als_data < 50) {
        digitalWrite(Voorlicht, HIGH);
        digitalWrite(Achterlicht, HIGH);
      } else {
        digitalWrite(Voorlicht, LOW);
        digitalWrite(Achterlicht, LOW);
      }
    }
  }
}