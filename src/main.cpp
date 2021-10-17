#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <ArduinoJson.h>
#include <MqttClient.h>
#include <FotaClient.h>
#include <ESPWifiClient.h>
#include <RemotePrint.h>
#include <TemperatureClient.h>
#include <Lamp.h>
#include "settings.h"
#include "utils/utils.h"


MqttClient *mqttClient               = NULL;
TemperatureClient *temperatureClient = new TemperatureClient();
FotaClient *fotaClient               = new FotaClient(DEVICE_NAME);
ESPWifiClient *wifiClient            = new ESPWifiClient(WIFI_SSID, WIFI_PASS);
String topics[LAMPS_COUNT];
long   lampsLastStatusMsgSentAt           = 0;
volatile bool interruptFromSwitchReceived = true;


// TODO: Move the method below to utils class
void getAllTopics(String action, String topics[]) {
  for (int i = 0; i < LAMPS_COUNT; i++) {
    Lamp  *lamp = &lamps[i];
    String tmp;
    tmp       = action + lamp->mqttTopic;
    topics[i] = tmp;
  }
}

void getAllOutputRelayPins(uint8_t outputRelayPins[]) {
  for (int i = 0; i < LAMPS_COUNT; i++) {
    Lamp *lamp = &lamps[i];
    outputRelayPins[i] = lamp->outputRelayPin;
  }
}

// TODO: Move the method below to utils class
Lamp* getLampFromTopic(String topic) {
  String *lowerTopic = new String(topic.c_str());

  lowerTopic->toLowerCase();

  for (int i = 0; i < LAMPS_COUNT; i++) {
    String *lowerTopicNext = new String(lamps[i].mqttTopic.c_str());
    lowerTopicNext->toLowerCase();

    if (*lowerTopic == *lowerTopicNext) {
      return &lamps[i];
    }
  }
  PRINT("LAMP: Could not find Lamp with topic: ");
  PRINTLN(topic);
  return NULL;
}

// TODO: Move the method below to utils class
Lamp* getLampWithInputSwitchPin(uint8_t inputSwitchPin) {
  for (int i = 0; i < LAMPS_COUNT; i++) {
    if (inputSwitchPin == lamps[i].inputSwitchPin) {
      return &lamps[i];
    }
  }
  PRINT("LAMP: Could not find Lamp with input switch pin: ");
  PRINTLN(inputSwitchPin);
  return NULL;
}

// TODO: Move the method below to utils class
void turnOffLampWithOutputRelayPin(uint8_t outputRelayPin) {
  for (int i = 0; i < LAMPS_COUNT; i++) {
    if (outputRelayPin == lamps[i].outputRelayPin) {
      if (!lamps[i].isInputSwitchOn()) lamps[i].turnOff();

      // we need to stop all lamps with same outputRelayPin
    }
  }
}

void updateLampsState() {
  if (!interruptFromSwitchReceived) return;
  interruptFromSwitchReceived = false;

  for (int i = 0; i < LAMPS_COUNT; i++) {
    Lamp *lamp = &lamps[i];

    if (lamp->isInputSwitchOn()) {
      lamp->turnOn();
    } else {
      // First turn off the LED
      lamp->turnOffLED();
    }
  }

  uint8_t outputRelayPins[LAMPS_COUNT];
  getAllOutputRelayPins(outputRelayPins);

  uint8_t outputRelayPin;
  bool    turnOffOutputRelay;

  for (int j = 0; j < LAMPS_COUNT; j++) {
    outputRelayPin     = outputRelayPins[j];
    turnOffOutputRelay = true;

    for (int i = 0; i < LAMPS_COUNT; i++) {
      Lamp *lamp = &lamps[i];

      if ((!lamp->hasLED()) || (lamp->outputRelayPin != outputRelayPin)) continue;

      if (lamp->isLEDStateOn) {
        turnOffOutputRelay = false;
        break;
      }
    }

    if (turnOffOutputRelay) {
      PRINTLN("___TURN OFF OUTPUT");
      turnOffLampWithOutputRelayPin(outputRelayPin);
    }
  }

  // FIXME: set it to 0 to force the publish
  lampsLastStatusMsgSentAt = 0;
}

// The interrupt routine handle
void IRAM_ATTR interruptServiceRoutine() {
  uint8_t pinNumber, pinValue;

  noInterrupts();

  // Debounce. Slow I2C: extra debounce between interrupts anyway.
  // Can not use delay() in interrupt code.
  delayMicroseconds(500);

  // Stop interrupts from external pin.
  detachInterrupt(digitalPinToInterrupt(PIN_ESP_INTERUPT_FROM_MCP));
  interrupts(); // re-start interrupts for mcp

  pinNumber = mcp.getLastInterruptPin();

  // This one resets the interrupt state as it reads from reg INTCAPA(B).
  pinValue = mcp.digitalRead(pinNumber);

  PRINT_D("Interrupt from input ");
  PRINT_D(pinNumber);
  PRINT_D(" with value ");
  PRINTLN_D(pinValue);

  // Just mark that there is an interrupt and processed it in the loop
  interruptFromSwitchReceived = true;

  // Reinstate interrupts from external pin.
  attachInterrupt(digitalPinToInterrupt(PIN_ESP_INTERUPT_FROM_MCP), \
                  interruptServiceRoutine,                          \
                  FALLING);
}

void setPinsMode() {
  // Initialise for interrupts.
  // mcp.readGPIOAB();

  // Enable ESP interrupt control
  pinMode(PIN_ESP_INTERUPT_FROM_MCP, INPUT);

  // ???
  attachInterrupt(digitalPinToInterrupt(PIN_ESP_INTERUPT_FROM_MCP), \
                  interruptServiceRoutine,                          \
                  FALLING);

  // On interrupt, polariy is set HIGH/LOW (last parameter).
  mcp.setupInterrupts(MCP_INT_MIRROR, MCP_INT_ODR, LOW); // The mcp output interrupt pin.
  mcp.setupInterruptPin(PIN_MCP_INPUT_SWITCH1, CHANGE);  // The mcp action that causes an interrupt.
  mcp.setupInterruptPin(PIN_MCP_INPUT_SWITCH2, CHANGE);
  mcp.setupInterruptPin(PIN_MCP_INPUT_SWITCH3, CHANGE);

  // Output pins for relays
  mcp.pinMode(PIN_MCP_OUTPUT_RELAY1, OUTPUT);
  mcp.digitalWrite(PIN_MCP_OUTPUT_RELAY1, LOW);
  mcp.pinMode(PIN_MCP_OUTPUT_RELAY2, OUTPUT);
  mcp.digitalWrite(PIN_MCP_OUTPUT_RELAY2, LOW);
  mcp.pinMode(PIN_MCP_OUTPUT_RELAY3, OUTPUT);
  mcp.digitalWrite(PIN_MCP_OUTPUT_RELAY3, LOW);

  // Output pins for LED
  mcp.pinMode(PIN_MCP_OUTPUT_LED1_WHITE, OUTPUT);
  mcp.pinMode(PIN_MCP_OUTPUT_LED1_RED,   OUTPUT);
  mcp.pinMode(PIN_MCP_OUTPUT_LED1_GREEN, OUTPUT);
  mcp.pinMode(PIN_MCP_OUTPUT_LED1_BLUE,  OUTPUT);

  mcp.pinMode(PIN_MCP_OUTPUT_LED2_WHITE, OUTPUT);
  mcp.pinMode(PIN_MCP_OUTPUT_LED2_RED,   OUTPUT);
  mcp.pinMode(PIN_MCP_OUTPUT_LED2_GREEN, OUTPUT);
  mcp.pinMode(PIN_MCP_OUTPUT_LED2_BLUE,  OUTPUT);

  // Input pins
  mcp.pinMode(PIN_MCP_INPUT_SWITCH1,     INPUT_PULLUP);
  mcp.pinMode(PIN_MCP_INPUT_SWITCH2,     INPUT_PULLUP);
  mcp.pinMode(PIN_MCP_INPUT_SWITCH3,     INPUT_PULLUP);
}

void lampPublishStatus(bool forcePublish = false,
                       const char *messageId = NULL, String topic = "") {
  long now = millis();

  if ((forcePublish) or (now - lampsLastStatusMsgSentAt >
                         MQTT_LAMP_PUBLISH_STATUS_INTERVAL)) {
    int count = LAMPS_COUNT;

    if (topic.length()) {
      count = 1;
    }

    for (int i = 0; i < count; i++) {
      Lamp *lamp = NULL;

      if (topic.length()) {
        lamp = getLampFromTopic(topic);
      } else {
        lamp = &lamps[i];
      }

      const size_t capacity = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(5);
      DynamicJsonDocument root(capacity);
      JsonObject status = root.createNestedObject("status");

      if (messageId != NULL) {
        root["messageId"] = messageId;
      }

      root["name"]      = lamp->name;
      status["powerOn"] = lamp->isStateOn;

      if (lamp->outputLedWhitePin != PIN_INVALID_NUMBER) status["white"] = lamp->isColorWhiteOn;

      if (lamp->outputLedWhitePin != PIN_INVALID_NUMBER) status["red"] = lamp->isColorRedOn;

      if (lamp->outputLedWhitePin != PIN_INVALID_NUMBER) status["green"] = lamp->isColorGreenOn;

      if (lamp->outputLedWhitePin != PIN_INVALID_NUMBER) status["blue"] = lamp->isColorBlueOn;

      // convert to String
      String outString;
      serializeJson(root, outString);

      // publish the message
      String topic = String("get");
      topic.concat(lamp->mqttTopic);
      mqttClient->publish(topic, outString);
    }
    lampsLastStatusMsgSentAt = now;
  }
}

void lampSendToLamp(Lamp *lamp, String payload) {
  PRINTLN("LAMP: Send to lamp '" + lamp->name + "'");

  // deserialize the payload to JSON
  const size_t capacity = 2 * JSON_OBJECT_SIZE(3) + 100;
  DynamicJsonDocument  jsonDoc(capacity);
  DeserializationError error = deserializeJson(jsonDoc, payload);

  if (error) {
    PRINT_E("Failed to deserialize the received payload. Error: ");
    PRINTLN_E(error.c_str());
    PRINTLN_E("The payload is: ");
    PRINTLN_E(payload)
    return;
  }
  JsonObject root   = jsonDoc.as<JsonObject>();
  JsonObject status = root["status"];

  if (status.isNull()) {
    PRINTLN_E(
      "LAMP: The received payload is valid JSON, but \"status\" key is not found.");
    PRINTLN_E("The payload is: ");
    PRINTLN_E(payload)
    return;
  }

  JsonVariant powerOnJV = status["powerOn"];

  if (!powerOnJV.isNull()) {
    bool powerOn = powerOnJV.as<bool>();

    PRINT("LAMP: Power On: ");
    PRINTLN(powerOn);

    if (powerOn) {
      lamp->turnOn();

      // lampPublishStatus(true, messageId, lamp->mqttTopic);
    } else {
      lamp->turnOff();

      // lampPublishStatus(true, messageId, lamp->mqttTopic);
    }
  }

  JsonVariant colorWhiteJV = status["white"];

  if (!colorWhiteJV.isNull()) {
    bool colorWhite = colorWhiteJV.as<bool>();

    if (colorWhite) lamp->changeColorWhite(HIGH); else lamp->changeColorWhite(LOW);
  }

  JsonVariant colorRedJV = status["red"];

  if (!colorRedJV.isNull()) {
    bool colorRed = colorRedJV.as<bool>();

    if (colorRed) lamp->changeColorRed(HIGH); else lamp->changeColorRed(LOW);
  }

  JsonVariant colorGreenJV = status["green"];

  if (!colorGreenJV.isNull()) {
    bool colorGreen = colorGreenJV.as<bool>();

    if (colorGreen) lamp->changeColorGreen(HIGH); else lamp->changeColorGreen(LOW);
  }

  JsonVariant colorBlueJV = status["blue"];

  if (!colorBlueJV.isNull()) {
    bool colorBlue = colorBlueJV.as<bool>();

    if (colorBlue) lamp->changeColorBlue(HIGH); else lamp->changeColorBlue(LOW);
  }

  const char *messageId = root["messageId"];
  lampPublishStatus(true, messageId, lamp->mqttTopic);
}

void lampMqttCallback(char *topic, String payload) {
  PRINTLN("LAMP: Callback called.");

  char *ptr = strchr(topic, '/');

  if (ptr != NULL) {
    String lowerTopic = String(ptr);
    lowerTopic.toLowerCase();
    Lamp *lamp = getLampFromTopic(lowerTopic);

    if (lamp == NULL) {
      // Lamp not found
      return;
    }
    lampSendToLamp(lamp, payload);
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  PRINT("MQTT Message arrived [");
  PRINT(topic);
  PRINTLN("] ");

  // Convert the payload to string
  char spayload[length + 1];
  memcpy(spayload, payload, length);
  spayload[length] = '\0';
  String payloadString = String(spayload);

  lampMqttCallback(topic, payloadString);
}

void setup() {
  mcp.begin_I2C(MCP_ADDRESS);
  setPinsMode();
  wifiClient->init();
  fotaClient->init();

  getAllTopics("set", topics);
  mqttClient = new MqttClient(MQTT_SERVER,
                              MQTT_SERVER_PORT,
                              DEVICE_NAME,
                              MQTT_USERNAME,
                              MQTT_PASS,
                              topics,
                              LAMPS_COUNT,
                              MQTT_SERVER_FINGERPRINT,
                              mqttCallback);

  temperatureClient->init(DEVICE_NAME,
                          PIN_ESP_DHT,
                          TEMP_SENSOR_TYPE,
                          mqttClient,
                          MQTT_TOPIC_TEMPERATURE_GET,
                          TEMP_SENSOR_CORRECTION);
}

void loop() {
  wifiClient->reconnectIfNeeded();
  RemotePrint::instance()->handle();
  fotaClient->loop();
  temperatureClient->loop();
  mqttClient->loop();

  updateLampsState();
  lampPublishStatus();
}
