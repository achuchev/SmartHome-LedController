#include "utils.h"
#include <Arduino.h>
#include <RemotePrint.h>
#include <Lamp.h>

// #include "settings.h"


// TODO: Move the method below to utils class
// void Utils::getAllTopics(String action, String topics[]) {
//   for (int i = 0; i < LAMPS_COUNT; i++) {
//     Lamp  *lamp = &lamps[i];
//     String tmp;
//     tmp       = action + lamp->mqttTopic;
//     topics[i] = tmp;
//   }
// }

// TODO: Move the method below to utils class
// Lamp * Utils::getLampFromTopic(String topic) {
//   String *lowerTopic = new String(topic.c_str());
//
//   lowerTopic->toLowerCase();
//
//   for (int i = 0; i < LAMPS_COUNT; i++) {
//     String *lowerTopicNext = new String(lamps[i].mqttTopic.c_str());
//     lowerTopicNext->toLowerCase();
//
//     if (*lowerTopic == *lowerTopicNext) {
//       return &lamps[i];
//     }
//   }
//   PRINT("LAMP: Could not find Lamp with topic: ");
//   PRINTLN(topic);
//   return NULL;
// }

// TODO: Move the method below to utils class
// Lamp * Utils::getLampWithInputSwitchPin(uint8_t inputSwitchPin) {
//   for (int i = 0; i < LAMPS_COUNT; i++) {
//     if (inputSwitchPin == lamps[i].inputSwitchPin) {
//       return &lamps[i];
//     }
//   }
//   PRINT("LAMP: Could not find Lamp with input switch pin: ");
//   PRINTLN(inputSwitchPin);
//   return NULL;
// }
