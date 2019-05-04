#ifndef SETTINGS_H
#define SETTINGS_H

#include <Lamp.h>
#include "DHT.h"
#include "Adafruit_MCP23017.h"

#define DEVICE_NAME "LivingRoomLamp"

#define MQTT_LAMP_PUBLISH_STATUS_INTERVAL 60000

// Temperature sensor
#define PIN_ESP_DHT D4
#define TEMP_SENSOR_TYPE DHT22
#define TEMP_SENSOR_CORRECTION 0 // Correction in degrees
#define MQTT_TOPIC_TEMPERATURE_GET "get/temperature/apartment/livingRoom/lamp"

// MCP23017
#define MCP_ADDRESS 0                // Equal to I2C address 0x20 (pins A0, A1, and A2 are LOW)
#define MCP_INT_MIRROR true          // Mirror inta to intb.
#define MCP_INT_ODR    false         // Open drain.
#define PIN_ESP_INTERUPT_FROM_MCP D3 // D3

// pin A0 = 0; A1 = 1; B0 = 8;
// Switches
#define PIN_MCP_INPUT_SWITCH1 15 // Big lamp
#define PIN_MCP_INPUT_SWITCH2 14 // Small lamp
#define PIN_MCP_INPUT_SWITCH3 13 // LED

// Relays
#define PIN_MCP_OUTPUT_RELAY1 6
#define PIN_MCP_OUTPUT_RELAY2 7
#define PIN_MCP_OUTPUT_RELAY3 5

// LED big
#define PIN_MCP_OUTPUT_LED1_WHITE 8
#define PIN_MCP_OUTPUT_LED1_RED 10
#define PIN_MCP_OUTPUT_LED1_GREEN 9
#define PIN_MCP_OUTPUT_LED1_BLUE 11

// LED small
#define PIN_MCP_OUTPUT_LED2_WHITE 0
#define PIN_MCP_OUTPUT_LED2_RED 2
#define PIN_MCP_OUTPUT_LED2_GREEN 1
#define PIN_MCP_OUTPUT_LED2_BLUE 3


Adafruit_MCP23017 mcp;
Lamp *lamp1 = new Lamp("Big Lamp",
                       "/apartment/livingRoom/lamp/1",
                       mcp,
                       PIN_MCP_INPUT_SWITCH1,
                       PIN_MCP_OUTPUT_RELAY1);
Lamp *lamp2 = new Lamp("Small Lamp",
                       "/apartment/livingRoom/lamp/2",
                       mcp,
                       PIN_MCP_INPUT_SWITCH2,
                       PIN_MCP_OUTPUT_RELAY2);

Lamp *lamp3 = new Lamp("LED Big",
                       "/apartment/livingRoom/lamp/3",
                       mcp,
                       PIN_MCP_INPUT_SWITCH1,
                       PIN_MCP_OUTPUT_RELAY3,
                       PIN_MCP_OUTPUT_LED1_WHITE,
                       PIN_MCP_OUTPUT_LED1_RED,
                       PIN_MCP_OUTPUT_LED1_GREEN,
                       PIN_MCP_OUTPUT_LED1_BLUE);

Lamp *lamp4 = new Lamp("LED Small",
                       "/apartment/livingRoom/lamp/4",
                       mcp,
                       PIN_MCP_INPUT_SWITCH3,
                       PIN_MCP_OUTPUT_RELAY3,
                       PIN_MCP_OUTPUT_LED2_WHITE,
                       PIN_MCP_OUTPUT_LED2_RED,
                       PIN_MCP_OUTPUT_LED2_GREEN,
                       PIN_MCP_OUTPUT_LED2_BLUE);
#define LAMPS_COUNT 4
Lamp lamps[LAMPS_COUNT] = { *lamp1, *lamp2, *lamp3, *lamp4 };

#endif // ifndef SETTINGS_H
