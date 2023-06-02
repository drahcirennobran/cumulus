#include <RBDdimmer.h>
#include <ArduinoMqttClient.h>

#include <EEPROM.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "secrets.h"

#define USE_SERIAL Serial
#define outputPin 5
#define zerocross 4

#define EPROM_ON 0
#define EPROM_AUTO 1

const char* ssid = STASSID;
const char* password = STAPSK;
const bool retained = false;
const int qos = 1;
const bool dup = false;

dimmerLamp dimmer(outputPin, zerocross);

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "192.168.0.34";
int port = 1883;
char topicConso[] = "shellies/shellyem-34945477FB60/emeter/0/power";
char topicLoad[] = "cumulus/load";
char topicCommand[] = "cumulus/command";
char topicAuto[] = "cumulus/auto";
char topicCommandState[] = "cumulus/commandState";
char topicAutoState[] = "cumulus/autoState";
char topicLoadHA[] = "cumulus/loadHA";

int load = 10;
int previousLoad = load;
int conso;
int loadHA;
bool commandState = false;
bool autoState = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial.println("Booting");

  dimmer.begin(NORMAL_MODE, ON);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setHostname("cumulus");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1)
      ;
  }

  Serial.println("Connected to the MQTT broker!");
  mqttClient.onMessage(onMqttMessage);
  Serial.println("Subscribing to topics");

  mqttClient.subscribe(topicConso);
  mqttClient.subscribe(topicCommand);
  mqttClient.subscribe(topicAuto);
  mqttClient.subscribe(topicLoadHA);
}

void loop() {
  mqttClient.poll();
  ArduinoOTA.handle();
  static int cpt = 0;
  cpt++;
  cpt %= 10;

  delay(500);

  if (cpt == 0) {
    String payload = String(load);
    mqttClient.beginMessage(topicLoad, payload.length(), retained, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();

    payload = String(commandState ? "on" : "off");
    mqttClient.beginMessage(topicCommandState, payload.length(), retained, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();

    payload = String(autoState ? "on" : "off");
    mqttClient.beginMessage(topicAutoState, payload.length(), retained, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();
  }
  if (load != previousLoad) {
    String payload = String(load);
    mqttClient.beginMessage(topicLoad, payload.length(), retained, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();
    previousLoad = load;
  }
}

void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();
  Serial.print("Topic : ");
  Serial.println(topic);
  uint8_t buf[500];
  int available = mqttClient.available();
  if (available > 0) {
    mqttClient.read(buf, available);
    buf[available] = '\n';
  }
  String stringBuf = String((char*)buf);
  //Serial.print("buf : ");
  //Serial.println(stringBuf);
  char* stateTopic;
  bool on;

  on = stringBuf.indexOf("on") >= 0 ? true : false;

  if (topic.indexOf("auto") >= 0) {
    autoState = on;
    setState(topicAutoState, on);
  } else if (topic.indexOf("command") >= 0) {
    commandState = on;
    setState(topicCommandState, on);
  } else if (topic.indexOf("emeter") >= 0) {
    conso = stringBuf.toInt();
    Serial.printf("Conso : %d\n", conso);
  } else if (topic.indexOf("loadHA") >= 0) {
    loadHA = stringBuf.toInt();
    Serial.printf("loadHA : %d\n", loadHA);
  }
  command();
}

void command() {
  if (commandState) {
    if (autoState) {
      load -= conso / 30;
    } else {
      load = loadHA;
    }

    if (load < 4) {
      load = 0;
      dimmer.setState(OFF);
      Serial.println("dimmer.setState off");
    } else {
      dimmer.setState(ON);
      Serial.println("dimmer.setState on");
    }
    if (load > 100) {
      load = 100;
    }
    dimmer.setPower(load);
    Serial.print("dimmer.setPower : ");
    Serial.println(load);

  } else {
    dimmer.setState(OFF);
    load = 0;
    Serial.println("dimmer.setState off");
  }
}

inline void setState(const char* topic, bool s) {
  String payload = s ? "on" : "off";
  Serial.print("setState ");
  Serial.print(payload);
  Serial.print(" (");
  Serial.print(s);
  Serial.println(")");

  mqttClient.beginMessage(topic, payload.length(), retained, qos, dup);
  mqttClient.print(payload);
  mqttClient.endMessage();
}
