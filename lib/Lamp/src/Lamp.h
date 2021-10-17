#ifndef LAMP_H
#define LAMP_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

#define PIN_INVALID_NUMBER 99

class Lamp {
public:

  Lamp();
  Lamp(String            name,
       String            mqttTopic,
       Adafruit_MCP23X17 mcp,
       uint8_t           inputSwitchPin,
       uint8_t           outputRelayPin,
       uint8_t           outputLedWhitePin = PIN_INVALID_NUMBER,
       uint8_t           outputLedRedPin   = PIN_INVALID_NUMBER,
       uint8_t           outputLedGreenPin = PIN_INVALID_NUMBER,
       uint8_t           outputLedBluePin  = PIN_INVALID_NUMBER);
  void turnOnOff(bool isOn);
  void turnOn();
  void turnOff();
  void turnOffLED();
  void changeColor(uint8_t white,
                   uint8_t red,
                   uint8_t green,
                   uint8_t blue);
  void changeColorWhite(uint8_t white);
  void changeColorRed(uint8_t red);
  void changeColorGreen(uint8_t green);
  void changeColorBlue(uint8_t blue);
  void changeColorRandom();
  void updateLampState();
  bool isInputSwitchOn();
  bool hasLED();

  String name;
  String mqttTopic;
  uint8_t inputSwitchPin;
  bool isStateOn;
  bool isLEDStateOn;
  bool isColorWhiteOn;
  bool isColorRedOn;
  bool isColorGreenOn;
  bool isColorBlueOn;
  uint8_t outputLedWhitePin;
  uint8_t outputLedRedPin;
  uint8_t outputLedGreenPin;
  uint8_t outputLedBluePin;
  uint8_t outputRelayPin;

private:

  Adafruit_MCP23X17 mcp;
};

#endif // ifndef LAMP_H
