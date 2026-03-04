#include <Homie.h>
#include <Arduino.h>
#include <WiFiUdp.h>
#include <string>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <ArduinoJson.h>
#include "IRelectra.h"

// Debug logging - controlled by DEBUG_SERIAL and DEBUG_MQTT build flags
#if DEBUG_SERIAL && DEBUG_MQTT
  #define DEBUG_LOG(x) do { Serial.println(x); debugNode.setProperty("log").send(x); } while(0)
#elif DEBUG_SERIAL
  #define DEBUG_LOG(x) Serial.println(x)
#elif DEBUG_MQTT
  #define DEBUG_LOG(x) debugNode.setProperty("log").send(x)
#else
  inline void DEBUG_LOG(const String&) { }
#endif

HomieNode temperatureNode("temperature", "temperature","temperature");
HomieNode modeNode("mode", "mode","mode");
HomieNode fanNode("fan", "fan","fan");
HomieNode swingNode("swing", "swing","swing");
HomieNode ifeelNode("ifeel", "ifeel","ifeel");
HomieNode ifeelTempNode("ifeel-temperature", "ifeel_temperature","ifeel_temperature");
HomieNode powerNode("power", "power","power");
HomieNode stateNode("state", "state","state");
HomieNode rebootNode("reboot", "reboot","reboot");
#if DEBUG_MQTT
HomieNode debugNode("debug", "debug","debug");
#endif

// Pin assignments - configure per-environment in platformio.ini build_flags
// e.g.  -D PIN_IR=23
#ifndef PIN_POWER
  #define PIN_POWER 5
#endif
#ifndef PIN_IR
  #define PIN_IR 4
#endif
#ifndef PIN_GREEN_LED
  #define PIN_GREEN_LED 12
#endif
#ifndef PIN_RED_LED
  #define PIN_RED_LED 15
#endif
#ifndef PIN_IR_RECV
  #define PIN_IR_RECV 14
#endif

const uint8_t POWER_PIN = PIN_POWER;
const uint8_t IR_PIN = PIN_IR;
#ifndef ARDUINO_ESP8266_ESP01
const uint8_t GREEN_LED_PIN = PIN_GREEN_LED;
const uint8_t RED_LED_PIN = PIN_RED_LED;
#endif

#ifndef ELECTRAWIFI_NO_IR_RCV
const uint8_t IR_RECV_PIN = PIN_IR_RECV;
#endif

const uint8_t kTimeout = 10;
const uint16_t kCaptureBufferSize = 300;

WiFiUDP udpClient;
IRelectra ac(IR_PIN);

#ifndef ELECTRAWIFI_NO_IR_RCV
IRrecv irrecv(IR_RECV_PIN, kCaptureBufferSize, kTimeout, true);
decode_results ir_ticks;
#endif

const int IFEEL_INTERVAL = 2 * 60000; // 2 min
const int POWER_DEBOUNCE = 2000; // 2 sec
const int UPDATES_INTERVAL = 10 * 60000; // 10 min
ulong power_change_time = 0;
ulong ifeel_send_time = 0;
ulong updates_send_time = 0;


void send_updates() {
  String fan, mode, swing;
  if (ac.fan == FAN_LOW) {
    fan = "low";
  } else if (ac.fan == FAN_MED) {
    fan = "med";
  } else if (ac.fan == FAN_HIGH) {
    fan = "high";
  } else if (ac.fan == FAN_AUTO) {
    fan = "auto";
  } 
  
  ac.power_real = ac.power_setting;

  if (ac.power_real) {
    if (ac.mode == MODE_COOL) {
      mode = "cool";
    } else if (ac.mode == MODE_HEAT) {
      mode = "heat";
    } else if (ac.mode == MODE_DRY) {
      mode = "dry";
    } else if (ac.mode == MODE_AUTO) {
      mode = "auto";
    } else if (ac.mode == MODE_FAN) {
      mode = "fan_only";
    }
  }
  else {
    mode = "off";
  }

  if (ac.swing == SWING_ON && ac.swing_h == SWING_H_ON) {
    swing = "both";
  } else if (ac.swing == SWING_ON && ac.swing_h == SWING_H_OFF) {
    swing = "on";
  } else if (ac.swing == SWING_OFF && ac.swing_h == SWING_H_ON) {
    swing = "hor";
  } else {
    swing = "off";
  }
  
  DEBUG_LOG("Sending state update: power=" + String(ac.power_real ? "on" : "off") + 
         " mode=" + mode + 
         " fan=" + fan + 
         " swing=" + swing + 
         " temp=" + String(ac.temperature) + 
         " ifeel=" + String(ac.ifeel == IFEEL_ON ? "on" : "off"));
  
  powerNode.setProperty("state").send(ac.power_real ? "on": "off");
  fanNode.setProperty("state").send(fan);
  modeNode.setProperty("state").send(mode);
  swingNode.setProperty("state").send(swing);
  temperatureNode.setProperty("state").send(String(ac.temperature));
  ifeelNode.setProperty("state").send(ac.ifeel == IFEEL_ON ? "on" : "off");
}

void loopHandler() {
  ulong now = millis();

  // Set power led and update the power state after it's stable for a while
  uint power_state = !digitalRead(POWER_PIN);
#ifndef ARDUINO_ESP8266_ESP01
  digitalWrite(GREEN_LED_PIN, power_state);
#endif
  if (power_state != ac.power_real) {
    if (power_change_time) {
      if (now - power_change_time > POWER_DEBOUNCE) {
        DEBUG_LOG("Power pin state changed: " + String(power_state ? "on" : "off"));
        ac.power_real = power_state;
        ac.power_setting = power_state;
        powerNode.setProperty("state").send(power_state ? "on": "off");
        send_updates();
        power_change_time = 0;
      }
    } else {
      power_change_time = now;
    }
  } else {
    power_change_time = 0;
  }

  // Send ifeel if relevant
  if (now - ifeel_send_time >= IFEEL_INTERVAL) {
    if (ac.ifeel == IFEEL_ON) {
      DEBUG_LOG("Sending ifeel temperature: " + String(ac.ifeel_temperature));
      ac.SendElectra(true);
    }
    ifeel_send_time = now;
  }

  // Send updates
  /*
  if (now - updates_send_time >= UPDATES_INTERVAL) {
    send_updates();
    updates_send_time = now;
  }
  */

  // Handle IR recv
#ifndef ELECTRAWIFI_NO_IR_RCV
  if (irrecv.decode(&ir_ticks)) {
    uint64_t code = 0;
    code = DecodeElectraIR(ir_ticks);
    irrecv.resume();
    if (code) {
      DEBUG_LOG("IR command received: 0x" + String((unsigned long)code, HEX));
      ac.UpdateFromIR(code);
      ac.SendElectra(false);
      send_updates();
    }
  }
#endif
}

bool powerHandler(const HomieRange& range, const String& value) {
  // This method is called when using the HA actions climate.turn_on and climate.turn_off
  DEBUG_LOG("MQTT received: power=" + value);
  if (value == "on") {
    ac.power_setting = true;
  } else if (value == "off") {
    ac.power_setting = false;
  } else {
    DEBUG_LOG("MQTT error: invalid power value");
    return false;
  }
  ac.SendElectra(false);
  ac.power_real = ac.power_setting;
  powerNode.setProperty("state").send(ac.power_real ? "on": "off");
  send_updates();
  return true;
}

bool temperatureHandler(const HomieRange& range, const String& value) {
  DEBUG_LOG("MQTT received: temperature=" + value);
  uint8_t temp = value.toInt();
  if (temp < 15 || temp > 30) { // setpoint temp has only 4 bits where 15 == 0b0000 and 30 == 0b1111
    DEBUG_LOG("MQTT error: temperature out of range (15-30)");
    return false;
  }
  ac.temperature = temp;
  ac.SendElectra(false);
  send_updates();
  return true;
}

bool modeHandler(const HomieRange& range, const String& value) {
  // This method is called when using the HA action climate.set_hvac_mode
  // Since climate.set_hvac_mode can be used to switch between `off` and other modes instead of climate.turn_on/off, it needs to call powerHandler.
  DEBUG_LOG("MQTT received: mode=" + value);
  if (value == "cool") {
    ac.mode = MODE_COOL;
  } else if (value == "heat") {
    ac.mode = MODE_HEAT;
  } else if (value == "auto") {
    ac.mode = MODE_AUTO;
  } else if (value == "dry") {
    ac.mode = MODE_DRY;
  } else if (value == "fan" || value == "fan_only") {
    ac.mode = MODE_FAN;
  } else if (value == "off") {
    return powerHandler(range, "off");
  } else {
    DEBUG_LOG("MQTT error: invalid mode value");
    return false;
  }
  return powerHandler(range, "on");
}

bool fanHandler(const HomieRange& range, const String& value) {
  DEBUG_LOG("MQTT received: fan=" + value);
  if (value == "low") {
    ac.fan = FAN_LOW;
  } else if (value == "med") {
    ac.fan = FAN_MED;
  } else if (value == "high") {
    ac.fan = FAN_HIGH;
  } else if (value == "auto") {
    ac.fan = FAN_AUTO;
  } else {
    DEBUG_LOG("MQTT error: invalid fan value");
    return false;
  }
  ac.SendElectra(false);
  send_updates();
  return true;
}

bool ifeelHandler(const HomieRange& range, const String& value) {
  DEBUG_LOG("MQTT received: ifeel=" + value);
  if (value == "on") {
    ac.ifeel = IFEEL_ON;
  } else if (value == "off") {
    ac.ifeel = IFEEL_OFF;
  } else {
    DEBUG_LOG("MQTT error: invalid ifeel value");
    return false;
  }
  ac.SendElectra(false);
  send_updates();
  return true;
}

bool ifeelTempHandler(const HomieRange& range, const String& value) {
  DEBUG_LOG("MQTT received: ifeel_temperature=" + value);
  uint8_t temp = value.toInt();
  if (temp < 5 || temp > 36) { // ifeel temp has only 5 bits where 0 == 0b00000 and 36 == 0b11111
    DEBUG_LOG("MQTT error: ifeel_temperature out of range (5-36)");
    return false;
  }
  ac.ifeel_temperature = temp;
  send_updates();
  return true;
}

bool swingHandler(const HomieRange& range, const String& value) {
  DEBUG_LOG("MQTT received: swing=" + value);
  if (value == "on") {
    ac.swing = SWING_ON;
    ac.swing_h = SWING_H_OFF;
  } else if (value == "both") {
    ac.swing = SWING_ON;
    ac.swing_h = SWING_H_ON;
  } else if (value == "hor") {
    ac.swing = SWING_OFF;
    ac.swing_h = SWING_H_ON;
  }  else if (value == "off"){
    ac.swing = SWING_OFF;
    ac.swing_h = SWING_H_OFF;
  } else {
    DEBUG_LOG("MQTT error: invalid swing value");
    return false;
  }
  ac.SendElectra(false);
  send_updates();
  return true;
}


bool jsonHandler(const HomieRange& range, const String& value) {
  DEBUG_LOG("MQTT received: json=" + value);
  StaticJsonDocument<200> parsed;
  auto error = deserializeJson(parsed,value);
  if (error) {
    DEBUG_LOG("MQTT error: failed to parse JSON");
    return false;
  }

  String fan = parsed["fan"];
  String mode = parsed["mode"];
  String power = parsed["power"];
  String ifeel = parsed["ifeel"];
  String temp_str = parsed["temperature"];
  String swing = parsed["swing"];
  uint8_t temp = temp_str.toInt();

  if (mode == "cool") {
    ac.mode = MODE_COOL;
  } else if (mode == "heat") {
    ac.mode = MODE_HEAT;
  } else if (mode == "auto") {
    ac.mode = MODE_AUTO;
  } else if (mode == "dry") {
    ac.mode = MODE_DRY;
  } else if (mode == "fan") {
    ac.mode = MODE_FAN;
  } else {
    return false;
  }

  if (fan == "low") {
    ac.fan = FAN_LOW;
  } else if (fan == "med") {
    ac.fan = FAN_MED;
  } else if (fan == "high") {
    ac.fan = FAN_HIGH;
  } else if (fan == "auto") {
    ac.fan = FAN_AUTO;
  } else {
    return false;
  }

  if (power == "on") {
    ac.power_setting = true;
  } else if (power == "off") {
    ac.power_setting = false;
  } else {
    return false;
  }

  if (swing == "on") {
    ac.swing = SWING_ON;
    ac.swing_h = SWING_H_OFF;
  } else if (swing == "both") {
    ac.swing = SWING_ON;
    ac.swing_h = SWING_H_ON;
  } else if (swing == "hor") {
    ac.swing = SWING_OFF;
    ac.swing_h = SWING_H_ON;
  }  else if (swing == "off") {
    ac.swing = SWING_OFF;
    ac.swing_h = SWING_H_OFF;
  } else {
    return false;
  }

  if (ifeel == "on") {
    ac.ifeel = IFEEL_ON;
  } else if (ifeel == "off") {
    ac.ifeel = IFEEL_OFF;
  } else {
    return false;
  }

  if (temp < 15 || temp > 30) { // setpoint temp has only 4 bits where 15 == 0b0000 and 30 == 0b1111
    return false;
  }
  ac.temperature = temp;

  ac.SendElectra(false);
  ac.power_real = ac.power_setting;

  send_updates();
  return true;
}

bool rebootHandler(const HomieRange& range, const String& value) {
  if (value == "true") {
    DEBUG_LOG("Rebooting...");
    ESP.restart();
    return true;
  }
  DEBUG_LOG("MQTT error: invalid reboot value (expected \"true\")");
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;
  //Homie.disableLogging();
  
#if DEBUG_SERIAL || DEBUG_MQTT
  // Set up IR debug logging callback
  setIRDebugCallback([](const String& msg) {
    #if DEBUG_SERIAL
      Serial.println(msg);
    #endif
    #if DEBUG_MQTT
      debugNode.setProperty("log").send(msg);
    #endif
  });
#endif

#ifdef ARDUINO_ESP8266_ESP01
  Homie.disableLedFeedback();
#endif
  Homie_setFirmware("ElectraWifi", "1.0.0");
  Homie.setLoopFunction(loopHandler);
  temperatureNode.advertise("state").settable(temperatureHandler);
  fanNode.advertise("state").settable(fanHandler);
  modeNode.advertise("state").settable(modeHandler);
  ifeelNode.advertise("state").settable(ifeelHandler);
  ifeelTempNode.advertise("state").settable(ifeelTempHandler);
  powerNode.advertise("state").settable(powerHandler);
  swingNode.advertise("state").settable(swingHandler);
  stateNode.advertise("json").settable(jsonHandler);
  rebootNode.advertise("trigger").settable(rebootHandler);
#if DEBUG_MQTT
  debugNode.advertise("log");
#endif
  
  pinMode(POWER_PIN, INPUT_PULLUP);
#ifndef ARDUINO_ESP8266_ESP01
  pinMode(GREEN_LED_PIN, OUTPUT);
  Homie.setLedPin(RED_LED_PIN, HIGH);
#endif
#ifndef ELECTRAWIFI_NO_IR_RCV
  irrecv.setUnknownThreshold(100); 
  irrecv.enableIRIn();
#endif
  Homie.setup();
}

void loop() {
  Homie.loop();
}