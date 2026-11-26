/************************************************************
 PROJECT:
 IoT-Based Smart Industrial Machine Health Monitoring
 and Early Fault Detection System

 BOARD: ESP32 DevKit V1

 PLATFORMS:
 1. Blynk IoT
 2. MQTT Explorer / MQTTX

 MQTT BROKER:
 broker.emqx.io
 Port: 1883

 MQTT TOPIC:
 smart-machine/telemetry

 MOTOR DRIVER:
 L298N

 MOTOR CONTROL:
 NORMAL TEMPERATURE       = RUN
 TEMPERATURE WARNING      = RUN
 TEMPERATURE CRITICAL     = RUN
 OTHER SENSOR FAULT       = STOP

 TEMPERATURE:
 Below 40 C = NORMAL
 40-49 C    = WARNING
 50 C+      = CRITICAL
 60 C+      = FUTURE FAN ACTIVATION

 ALERTS:
 WARNING / CRITICAL:
 RED LED BLINKING
 5V ACTIVE BUZZER ON

 IMPORTANT:
 Temperature alone never stops the motor.
 Other sensor faults stop the motor.
************************************************************/

#define BLYNK_TEMPLATE_ID "TMPL38fKmZU4z"
#define BLYNK_TEMPLATE_NAME "Smart Machine Monitor"
#define BLYNK_AUTH_TOKEN "q6R5LHpUWsHy4j_vHcnNpyktbpGyACJC"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <math.h>

// ------------ WIFI CREDENTIALS ------------

char ssid[] = "Y16";
char pass[] = "deekshith";

// ------------ MQTT CONFIGURATION ------------

const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_topic = "smart-machine/telemetry";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ------------ PIN CONFIGURATION ------------

#define SDA_PIN 21
#define SCL_PIN 22

#define DHT_PIN 19
#define DHT_TYPE DHT11

#define SW420_PIN 34
#define ACS_PIN 35

#define GREEN_LED 16
#define YELLOW_LED 17
#define RED_LED 23
#define BUZZER_PIN 18

// L298N MOTOR DRIVER

#define ENA_PIN 25
#define IN1_PIN 26
#define IN2_PIN 27

// ------------ OLED CONFIGURATION ------------

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C
#define OLED_RESET -1

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// ------------ SENSOR OBJECTS ------------

DHT dht(DHT_PIN, DHT_TYPE);
BlynkTimer timer;

// ------------ SETTINGS ------------

#define ENABLE_LOCAL_ALERTS true

#define ACS_SENSITIVITY 0.100f
#define ACS_DIVIDER_RATIO (20.0f / 32.0f)

#define CURRENT_WARNING 2.0f
#define CURRENT_CRITICAL 3.0f

#define VIBRATION_WARNING 0.25f
#define VIBRATION_CRITICAL 0.50f

#define TEMP_WARNING 40.0f
#define TEMP_CRITICAL 50.0f

// Future cooling fan activation threshold
#define FUTURE_FAN_TEMP 60.0f

#define MPU_ADDRESS 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B

// ------------ GLOBAL VARIABLES ------------

bool mpuOK = false;
bool oledOK = false;
bool motorRunning = false;

bool redLedState = false;

float temperature = NAN;
float humidity = NAN;

float motorCurrent = 0.0f;
float accelerationG = 0.0f;
float vibrationDeviation = 0.0f;

float acsZeroVoltage = 0.0f;

uint32_t vibrationEvents = 0;

bool previousSW420State = HIGH;

unsigned long lastVibrationEvent = 0;
unsigned long lastMPURetry = 0;
unsigned long lastRedBlink = 0;

const unsigned long RED_BLINK_INTERVAL = 500;

String systemStatus = "WARNING";
String faultCode = "STARTING";

// ------------ MPU REGISTER FUNCTIONS ------------

bool writeMPURegister(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool readMPURegisters(
  uint8_t startRegister,
  uint8_t *buffer,
  uint8_t length
) {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(startRegister);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(
    (uint8_t)MPU_ADDRESS,
    length
  );

  if (received != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

// ------------ MPU INITIALIZATION ------------

bool initializeMPU() {

  Serial.println("Initializing MPU6050...");

  if (!writeMPURegister(MPU_PWR_MGMT_1, 0x00)) {

    mpuOK = false;
    Serial.println("MPU communication failed!");

    return false;
  }

  delay(100);

  if (!writeMPURegister(0x1C, 0x00)) {

    mpuOK = false;
    return false;
  }

  delay(50);

  uint8_t testData[6];

  if (!readMPURegisters(
        MPU_ACCEL_XOUT_H,
        testData,
        6
      )) {

    mpuOK = false;
    Serial.println("MPU reading failed!");

    return false;
  }

  mpuOK = true;

  Serial.println("MPU6050 initialized!");

  return true;
}

// ------------ CONVERT MPU DATA ------------

int16_t makeSignedValue(
  uint8_t highByte,
  uint8_t lowByte
) {

  uint16_t combined =
    ((uint16_t)highByte << 8) | lowByte;

  return (int16_t)combined;
}

// ------------ READ MPU6050 ------------

void readMPU6050() {

  if (!mpuOK) {
    accelerationG = 0;
    vibrationDeviation = 0;
    return;
  }

  uint8_t data[6];

  if (!readMPURegisters(
        MPU_ACCEL_XOUT_H,
        data,
        6
      )) {

    Serial.println("MPU reading failed!");

    mpuOK = false;
    return;
  }

  int16_t rawX = makeSignedValue(data[0], data[1]);
  int16_t rawY = makeSignedValue(data[2], data[3]);
  int16_t rawZ = makeSignedValue(data[4], data[5]);

  float x = rawX / 16384.0f;
  float y = rawY / 16384.0f;
  float z = rawZ / 16384.0f;

  accelerationG = sqrt(x*x + y*y + z*z);

  vibrationDeviation = fabs(accelerationG - 1.0f);
}

// ------------ ACS712 CALIBRATION ------------

void calibrateACS712() {

  Serial.println("ACS712 calibration...");
  Serial.println("Keep motor disconnected!");

  float totalVoltage = 0;

  const int samples = 300;

  for (int i = 0; i < samples; i++) {

    uint32_t pinMilliVolts =
      analogReadMilliVolts(ACS_PIN);

    float moduleVoltage =
      (pinMilliVolts / 1000.0f) /
      ACS_DIVIDER_RATIO;

    totalVoltage += moduleVoltage;

    delay(5);
  }

  acsZeroVoltage = totalVoltage / samples;

  Serial.print("ACS zero voltage: ");
  Serial.println(acsZeroVoltage, 3);
}

// ------------ READ MOTOR CURRENT ------------

float readMotorCurrent() {

  uint32_t pinMilliVolts =
    analogReadMilliVolts(ACS_PIN);

  float moduleVoltage =
    (pinMilliVolts / 1000.0f) /
    ACS_DIVIDER_RATIO;

  float current =
    (moduleVoltage - acsZeroVoltage) /
    ACS_SENSITIVITY;

  return fabs(current);
}

// ------------ READ SW420 ------------

void readSW420() {

  bool currentState = digitalRead(SW420_PIN);

  if (
    previousSW420State == HIGH &&
    currentState == LOW
  ) {

    if (millis() - lastVibrationEvent > 150) {

      vibrationEvents++;

      lastVibrationEvent = millis();

      Serial.println("Vibration event detected!");
    }
  }

  previousSW420State = currentState;
}

// ------------ READ DHT11 ------------

void readDHT11() {

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t) && !isnan(h)) {

    temperature = t;
    humidity = h;

  } else {

    temperature = NAN;
    humidity = NAN;

    Serial.println("DHT11 reading failed!");
  }
}

// ------------ MACHINE STATUS ------------

void calculateMachineStatus() {

  bool warning = false;
  bool critical = false;

  String fault = "NONE";

  if (!mpuOK || isnan(temperature) || isnan(humidity)) {

    warning = true;
    fault = "SENSOR_ERROR";
  }

  if (motorCurrent >= CURRENT_CRITICAL) {

    critical = true;
    fault = "OVERCURRENT";

  } else if (motorCurrent >= CURRENT_WARNING) {

    warning = true;

    if (fault == "NONE") {
      fault = "HIGH_CURRENT";
    }
  }

  if (mpuOK &&
      vibrationDeviation >= VIBRATION_CRITICAL) {

    critical = true;
    fault = "HIGH_VIBRATION";

  } else if (
    mpuOK &&
    vibrationDeviation >= VIBRATION_WARNING
  ) {

    warning = true;

    if (fault == "NONE") {
      fault = "VIBRATION_WARNING";
    }
  }

  if (
    lastVibrationEvent != 0 &&
    millis() - lastVibrationEvent < 2000
  ) {

    warning = true;

    if (fault == "NONE") {
      fault = "SW420_EVENT";
    }
  }

  // TEMPERATURE MONITORING

  if (!isnan(temperature)) {

    if (temperature >= TEMP_CRITICAL) {

      critical = true;
      fault = "HIGH_TEMP";

    } else if (temperature >= TEMP_WARNING) {

      warning = true;

      if (fault == "NONE") {
        fault = "TEMP_WARNING";
      }
    }
  }

  // OVERALL SYSTEM STATUS

  if (critical) {

    systemStatus = "CRITICAL";

  } else if (warning) {

    systemStatus = "WARNING";

  } else {

    systemStatus = "NORMAL";
    fault = "NONE";
  }

  faultCode = fault;
}

// ------------ MOTOR CONTROL ------------

void stopMotor() {

  digitalWrite(ENA_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  motorRunning = false;
}

void controlMotor() {

  // CHECK OTHER SENSOR FAULTS
  // TEMPERATURE IS EXCLUDED HERE.

  bool otherSensorFault = false;

  // MPU6050 ERROR

  if (!mpuOK) {
    otherSensorFault = true;
  }

  // DHT11 ERROR

  if (isnan(temperature) || isnan(humidity)) {
    otherSensorFault = true;
  }

  // ACS712 HIGH CURRENT

  if (motorCurrent >= CURRENT_WARNING) {
    otherSensorFault = true;
  }

  // MPU6050 HIGH VIBRATION

  if (mpuOK &&
      vibrationDeviation >= VIBRATION_WARNING) {

    otherSensorFault = true;
  }

  // SW420 VIBRATION EVENT

  if (
    lastVibrationEvent != 0 &&
    millis() - lastVibrationEvent < 2000
  ) {

    otherSensorFault = true;
  }

  // MOTOR CONTROL DECISION

  if (otherSensorFault) {

    stopMotor();

    Serial.println("MOTOR STOPPED: OTHER SENSOR FAULT");

  } else {

    // TEMPERATURE WARNING OR CRITICAL
    // WILL NOT STOP THE MOTOR.

    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(ENA_PIN, HIGH);

    motorRunning = true;

    Serial.println("MOTOR OUTPUT: RUNNING");

    if (temperature >= FUTURE_FAN_TEMP) {

      Serial.println(
        "HIGH TEMPERATURE: FUTURE FAN SHOULD ACTIVATE"
      );
    }
  }
}

// ------------ OLED DISPLAY ------------

void updateOLED() {

  if (!oledOK) return;

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("SMART MACHINE MONITOR");

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 13);
  display.print("Temp: ");

  if (isnan(temperature)) {
    display.println("ERROR");
  } else {
    display.print(temperature, 1);
    display.println(" C");
  }

  display.setCursor(0, 23);
  display.print("Humidity: ");

  if (isnan(humidity)) {
    display.println("ERROR");
  } else {
    display.print(humidity, 1);
    display.println(" %");
  }

  display.setCursor(0, 33);
  display.print("Current: ");
  display.print(motorCurrent, 2);
  display.println(" A");

  display.setCursor(0, 43);
  display.print("Accel: ");
  display.print(accelerationG, 2);
  display.println(" g");

  display.setCursor(0, 54);
  display.print("Motor: ");
  display.print(motorRunning ? "RUNNING" : "STOPPED");

  display.display();
}

// ------------ SERIAL MONITOR ------------

void printReadings() {

  Serial.println();
  Serial.println("====== MACHINE DATA ======");

  Serial.print("Temperature: ");
  Serial.println(temperature);

  Serial.print("Humidity: ");
  Serial.println(humidity);

  Serial.print("Motor Current: ");
  Serial.println(motorCurrent);

  Serial.print("Acceleration: ");
  Serial.println(accelerationG);

  Serial.print("Vibration Deviation: ");
  Serial.println(vibrationDeviation);

  Serial.print("SW420 Events: ");
  Serial.println(vibrationEvents);

  Serial.print("MPU: ");
  Serial.println(mpuOK ? "OK" : "ERROR");

  Serial.print("System Status: ");
  Serial.println(systemStatus);

  Serial.print("Fault Code: ");
  Serial.println(faultCode);

  Serial.print("Motor: ");
  Serial.println(motorRunning ? "RUNNING" : "STOPPED");

  Serial.print("Blynk: ");
  Serial.println(Blynk.connected() ? "CONNECTED" : "DISCONNECTED");

  Serial.print("MQTT: ");
  Serial.println(mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");

  Serial.println("==========================");
}

// ------------ BLYNK DATA UPDATE ------------

void updateBlynk() {

  if (!Blynk.connected()) return;

  if (!isnan(temperature)) {
    Blynk.virtualWrite(V0, temperature);
  }

  if (!isnan(humidity)) {
    Blynk.virtualWrite(V1, humidity);
  }

  Blynk.virtualWrite(V2, motorCurrent);
  Blynk.virtualWrite(V3, (int)vibrationEvents);
  Blynk.virtualWrite(V4, accelerationG);
  Blynk.virtualWrite(V5, systemStatus);
  Blynk.virtualWrite(V6, faultCode);
}

// ------------ MQTT DATA PUBLISH ------------

void updateMQTT() {

  if (!mqttClient.connected()) {
    return;
  }

  char payload[400];

  snprintf(
    payload,
    sizeof(payload),

    "{"
    "\"temperature\":%.2f,"
    "\"humidity\":%.2f,"
    "\"current\":%.3f,"
    "\"vibration\":%.3f,"
    "\"acceleration\":%.3f,"
    "\"vibration_events\":%lu,"
    "\"status\":\"%s\","
    "\"fault\":\"%s\","
    "\"mpu\":\"%s\","
    "\"motor\":\"%s\""
    "}",

    isnan(temperature) ? -1.0f : temperature,
    isnan(humidity) ? -1.0f : humidity,
    motorCurrent,
    vibrationDeviation,
    accelerationG,
    (unsigned long)vibrationEvents,
    systemStatus.c_str(),
    faultCode.c_str(),
    mpuOK ? "OK" : "ERROR",
    motorRunning ? "RUNNING" : "STOPPED"
  );

  bool published = mqttClient.publish(
    mqtt_topic,
    payload
  );

  if (published) {

    Serial.println("MQTT DATA PUBLISHED:");
    Serial.println(payload);

  } else {

    Serial.println("MQTT publish failed!");
  }
}

// ------------ RED LED + BUZZER ALERTS ------------

void updateLocalAlerts() {

  if (!ENABLE_LOCAL_ALERTS) return;

  // NORMAL CONDITION

  if (systemStatus == "NORMAL") {

    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    redLedState = false;

    return;
  }

  // WARNING CONDITION

  if (systemStatus == "WARNING") {

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
  }

  // CRITICAL CONDITION

  if (systemStatus == "CRITICAL") {

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
  }

  // RED LED BLINKING

  if (millis() - lastRedBlink >= RED_BLINK_INTERVAL) {

    lastRedBlink = millis();

    redLedState = !redLedState;

    digitalWrite(RED_LED, redLedState);
  }

  // 5V ACTIVE BUZZER ALARM

  digitalWrite(BUZZER_PIN, HIGH);
}

// ------------ VIBRATION TASK ------------

void vibrationTask() {

  readSW420();
}

// ------------ MAIN MONITORING TASK ------------

void monitoringTask() {

  if (!mpuOK &&
      millis() - lastMPURetry >= 5000) {

    lastMPURetry = millis();

    initializeMPU();
  }

  readDHT11();

  readMPU6050();

  motorCurrent = readMotorCurrent();

  calculateMachineStatus();

  controlMotor();

  updateOLED();

  printReadings();

  updateBlynk();

  updateMQTT();
}

// ------------ CONNECTION TASK ------------

void connectionTask() {

  if (WiFi.status() != WL_CONNECTED) {

    Serial.println("WiFi disconnected!");

    WiFi.reconnect();

    return;
  }

  // BLYNK RECONNECTION

  if (!Blynk.connected()) {

    Serial.println("Connecting to Blynk...");

    Blynk.connect(1000);
  }

  // MQTT RECONNECTION

  if (!mqttClient.connected()) {

    Serial.println("Connecting to MQTT...");

    if (mqttClient.connect("ESP32_Machine_01")) {

      Serial.println("MQTT Connected!");

    } else {

      Serial.print("MQTT connection failed. State: ");
      Serial.println(mqttClient.state());
    }
  }
}

// ------------ SETUP ------------

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("SMART MACHINE MONITOR STARTING...");

  // MOTOR DRIVER INITIALIZATION

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  stopMotor();

  // SENSOR CONFIGURATION

  pinMode(SW420_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(ACS_PIN, ADC_11db);

  // I2C INITIALIZATION

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("I2C initialized.");

  // OLED INITIALIZATION

  if (display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS
      )) {

    oledOK = true;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 15);
    display.println("SMART MACHINE MONITOR");
    display.setCursor(0, 30);
    display.println("Starting...");
    display.display();

    Serial.println("OLED initialized!");

  } else {

    oledOK = false;

    Serial.println("OLED initialization failed!");
  }

  // MPU INITIALIZATION

  initializeMPU();

  // DHT INITIALIZATION

  dht.begin();

  Serial.println("DHT11 initialized.");

  // ACS712 CALIBRATION
  // MOTOR MUST BE DISCONNECTED

  calibrateACS712();

  // SW420 INITIAL STATE

  previousSW420State = digitalRead(SW420_PIN);

  // LOCAL ALERT INITIALIZATION

  if (ENABLE_LOCAL_ALERTS) {

    pinMode(GREEN_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // WIFI INITIALIZATION

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  Serial.print("Connecting to WiFi");

  unsigned long wifiStart = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - wifiStart < 15000
  ) {

    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println();
    Serial.println("WiFi Connected!");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

  } else {

    Serial.println();
    Serial.println("WiFi connection pending.");
  }

  // BLYNK INITIALIZATION

  Blynk.config(BLYNK_AUTH_TOKEN);

  // MQTT INITIALIZATION

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(3);

  // TIMER CONFIGURATION

  timer.setInterval(20L, vibrationTask);

  timer.setInterval(2000L, monitoringTask);

  timer.setInterval(10000L, connectionTask);

  // INITIAL CONNECTION

  connectionTask();

  Serial.println();
  Serial.println("SYSTEM SETUP COMPLETED!");

  Serial.println("Blynk + MQTT enabled.");

  Serial.println("Temperature monitoring enabled.");

  Serial.println("Motor runs during temperature warnings.");

  Serial.println("Other sensor faults stop the motor.");

  Serial.println("Automatic local alerts enabled.");

  Serial.println("Future cooling fan threshold: 60 C.");
}

// ------------ MAIN LOOP ------------

void loop() {

  if (WiFi.status() == WL_CONNECTED) {

    Blynk.run();
  }

  if (mqttClient.connected()) {

    mqttClient.loop();
  }

  timer.run();

  // CONTINUOUS ALERT CONTROL

  updateLocalAlerts();
}