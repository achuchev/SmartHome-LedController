#include <Arduino.h>
#include <RemotePrint.h>
#include "Adafruit_MCP23017.h"
#include "Lamp.h"


Lamp::Lamp() {
  this->name      = "";
  this->mqttTopic = "";
}

Lamp::Lamp(String            name,
           String            mqttTopic,
           Adafruit_MCP23017 mcp,
           uint8_t           inputSwitchPin,
           uint8_t           outputRelayPin,
           uint8_t           outputLedWhitePin,
           uint8_t           outputLedRedPin,
           uint8_t           outputLedGreenPin,
           uint8_t           outputLedBluePin) {
  this->name              = name;
  this->mqttTopic         = mqttTopic;
  this->mcp               = mcp;
  this->inputSwitchPin    = inputSwitchPin;
  this->outputRelayPin    = outputRelayPin;
  this->outputLedWhitePin = outputLedWhitePin;
  this->outputLedRedPin   = outputLedRedPin;
  this->outputLedGreenPin = outputLedGreenPin;
  this->outputLedBluePin  = outputLedBluePin;
};

void Lamp::turnOnOff(bool isOn) {
  if (isOn) this->turnOn(); else this->turnOff();
}

void Lamp::turnOn()      {
  PRINT("LAMP: Turning ON ");
  PRINTLN(this->name);

  // this->changeColorRandom();
  this->changeColorWhite(HIGH);

  // this->changeColorRed(HIGH);
  // this->changeColorGreen(HIGH);
  // this->changeColorBlue(HIGH);
  this->mcp.digitalWrite(this->outputRelayPin, HIGH);
  this->isStateOn = true;
}

void Lamp::turnOff()     {
  PRINT("LAMP: Turning OFF ");
  PRINTLN(this->name);
  this->mcp.digitalWrite(this->outputRelayPin, LOW);
  this->isStateOn = false;

  this->changeColor(LOW, LOW, LOW, LOW);
}

void Lamp::turnOffLED()     {
  PRINT_D("LAMP: Turning OFF LED of ");
  PRINTLN_D(this->name);

  this->changeColor(LOW, LOW, LOW, LOW);
  isLEDStateOn = false;
}

void Lamp::changeColorRandom() {
  randomSeed(millis());
  uint8_t color[4];
  uint8_t sum;

  while (true) {
    sum = 0;

    for (uint8_t i = 0; i < 4; i++) {
      color[i] = random(0, 2);
      sum     += color[i];
    }

    if (sum != 0) {
      break;
    }
  }
  this->changeColor(color[0], color[1], color[2], color[3]);
}

void Lamp::changeColorWhite(uint8_t white) {
  if (this->outputLedWhitePin != PIN_INVALID_NUMBER) {
    PRINT("LAMP:    White color set to ");
    PRINTLN(white);
    this->mcp.digitalWrite(this->outputLedWhitePin, white);
    this->isColorWhiteOn = white;
    this->isLEDStateOn   = true;
  }
}

void Lamp::changeColorRed(uint8_t red) {
  if (this->outputLedRedPin != PIN_INVALID_NUMBER) {
    PRINT("LAMP:    Red color set to ");
    PRINTLN(red);
    this->mcp.digitalWrite(this->outputLedRedPin, red);
    this->isColorRedOn = red;
    this->isLEDStateOn = true;
  }
}

void Lamp::changeColorGreen(uint8_t green) {
  if (this->outputLedGreenPin != PIN_INVALID_NUMBER) {
    PRINT("LAMP:    Green color set to ");
    PRINTLN(green);
    this->mcp.digitalWrite(this->outputLedGreenPin, green);
    this->isColorGreenOn = green;
    this->isLEDStateOn   = true;
  }
}

void Lamp::changeColorBlue(uint8_t blue) {
  if (this->outputLedBluePin != PIN_INVALID_NUMBER) {
    PRINT("LAMP:    Blue color set to ");
    PRINTLN(blue);
    this->mcp.digitalWrite(this->outputLedBluePin, blue);
    this->isColorBlueOn = blue;
    this->isLEDStateOn  = true;
  }
}

void Lamp::changeColor(uint8_t white, uint8_t red, uint8_t green, uint8_t blue) {
  this->changeColorWhite(white);
  this->changeColorRed(red);
  this->changeColorGreen(green);
  this->changeColorBlue(blue);
}

bool Lamp::isInputSwitchOn() {
  if (this->mcp.digitalRead(this->inputSwitchPin) == LOW) return true;
  return false;
}

void Lamp::updateLampState() {
  this->turnOnOff(this->isInputSwitchOn());
}

bool Lamp::hasLED() {
  if ((this->outputLedWhitePin != PIN_INVALID_NUMBER) && (this->outputLedRedPin != PIN_INVALID_NUMBER) &&
      (this->outputLedGreenPin != PIN_INVALID_NUMBER) && (this->outputLedBluePin != PIN_INVALID_NUMBER)) return true;
  return false;
}
