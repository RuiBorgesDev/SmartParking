#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// --- WiFi Configuration ---
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// --- MQTT Configuration ---
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32UltrasonicSensor";

// MQTT topics for each sensor
const char* mqtt_topic_sensor1 = "ultrasonic/sensor1/carParked";
const char* mqtt_topic_sensor2 = "ultrasonic/sensor2/carParked";
const char* mqtt_topic_servo1_control = "servo/region1/status";
const char* mqtt_topic_servo2_control = "servo/region2/status";

// --- Servo Definitions ---
static const int servo1Pin = 12;
static const int servo2Pin = 5;

Servo servo1;
Servo servo2;

// --- Ultrasonic Sensor Definitions ---
const int trig1Pin = 14;
const int echo1Pin = 27;
const int trig2Pin = 17;
const int echo2Pin = 16;

float duration1, duration2;
float distanceCm1, distanceCm2;

// --- Non-blocking Timing Variables ---
unsigned long previousMillisSensor = 0;
const long sensorInterval = 500;

// --- State Variables for MQTT Publishing ---
bool sensor1_below_threshold_reported = false;
bool sensor2_below_threshold_reported = false;

// --- MQTT Client Setup ---
WiFiClient espClient;
PubSubClient client(espClient);

// --- Function Prototypes ---
void setup_wifi();
void reconnect_mqtt();
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  Serial.begin(9600);

  // --- WiFi Setup ---
  setup_wifi();

  // --- MQTT Setup ---
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // --- Pin Modes ---
  pinMode(trig1Pin, OUTPUT);
  pinMode(echo1Pin, INPUT);
  pinMode(trig2Pin, OUTPUT);
  pinMode(echo2Pin, INPUT);

  // --- Servo Attach ---
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
}

void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  unsigned long currentMillis = millis();

  // --- Non-blocking sensor readings and MQTT publishing ---
  if (currentMillis - previousMillisSensor >= sensorInterval) {
    previousMillisSensor = currentMillis;

    // --- Sensor 1 reading ---
    digitalWrite(trig1Pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig1Pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig1Pin, LOW);
    duration1 = pulseIn(echo1Pin, HIGH, 100000);

    if (duration1 > 0) {
      distanceCm1 = (duration1 * 0.03402) / 2;

      if (distanceCm1 < 150.0 && !sensor1_below_threshold_reported) {
        Serial.print("Publishing 'true' to ");
        Serial.println(mqtt_topic_sensor2);
        client.publish(mqtt_topic_sensor2, "true");
        sensor1_below_threshold_reported = true;
      }
      else if (distanceCm1 >= 150.0 && sensor1_below_threshold_reported) {
        sensor1_below_threshold_reported = false;
        Serial.println("Sensor 1: Distance now above 1.5m, resetting report flag and sending 'false'.");
        client.publish(mqtt_topic_sensor2, "false");
      }
    } else {
      Serial.println("Distance 1: No echo received or out of range.");
      if (sensor1_below_threshold_reported) {
        sensor1_below_threshold_reported = false;
        Serial.println("Sensor 1: Out of range, resetting report flag and sending 'false'.");
        client.publish(mqtt_topic_sensor2, "false");
      }
    }

    // --- Sensor 2 reading ---
    digitalWrite(trig2Pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig2Pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig2Pin, LOW);
    duration2 = pulseIn(echo2Pin, HIGH, 100000);

    if (duration2 > 0) {
      distanceCm2 = (duration2 * 0.03402) / 2;

      if (distanceCm2 < 150.0 && !sensor2_below_threshold_reported) {
        Serial.print("Publishing 'true' to ");
        Serial.println(mqtt_topic_sensor1);
        client.publish(mqtt_topic_sensor1, "true");
        sensor2_below_threshold_reported = true;
      }
      else if (distanceCm2 >= 150.0 && sensor2_below_threshold_reported) {
        sensor2_below_threshold_reported = false;
        Serial.println("Sensor 2: Distance now above 1.5m, resetting report flag and sending 'false'.");
        client.publish(mqtt_topic_sensor1, "false");
      }
    } else {
      Serial.println("Distance 2: No echo received or out of range.");
      if (sensor2_below_threshold_reported) {
        sensor2_below_threshold_reported = false;
        Serial.println("Sensor 2: Out of range, resetting report flag and sending 'false'.");
        client.publish(mqtt_topic_sensor1, "false");
      }
    }
  }
}

// --- WiFi Connection Function ---
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// --- MQTT Reconnection Function ---
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("connected");
      client.subscribe(mqtt_topic_servo1_control);
      client.subscribe(mqtt_topic_servo2_control);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(2500);
    }
  }
}

// --- MQTT Callback Function ---
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (strcmp(topic, mqtt_topic_servo1_control) == 0) {
    if (message == "open") {
      servo1.write(0);
      Serial.println("Servo do setor 1 aberto");
    } else if (message == "close") {
      servo1.write(90);
      Serial.println("Servo do setor 1 fechado");
    }
  } else if (strcmp(topic, mqtt_topic_servo2_control) == 0) {
    if (message == "open") {
      servo2.write(0);
      Serial.println("Servo do setor 2 aberto");
    } else if (message == "close") {
      servo2.write(90);
      Serial.println("Servo do setor 2 fechado");
    }
  }
}