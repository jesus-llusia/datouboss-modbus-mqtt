// =============================================================================
//  Inverter Gateway — ESP32-C3
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <driver/uart.h>
#include "secrets.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ROM-level printf removed; avoid using ets_printf directly

// Debug Configuration
#define ENABLE_SERIAL_DEBUG 1

// Firmware version
#define FW_VERSION "3.0.0"

// MQTT
#define MQTT_BUFFER_SIZE 512
#define MQTT_MAX_RETRIES 3

// MQTT availability topic
#define MQTT_AVAIL_TOPIC "stat/solar/availability"

// Aggregated state topics
#define STAT_TOPIC_DATA "stat/solar/data"
#define STAT_TOPIC_DIAG_DATA "stat/solar/diag_data"

// Command topic (subscriptions)
#define CMND_TOPIC_RESTART "cmnd/solar/restart"

// Timing
#define PUBLISH_INTERVAL_MS (60UL * 1000UL)
// Heartbeat LED interval (ms)
#define HEARTBEAT_INTERVAL_MS 1000UL
// Heartbeat pin and polarity (override in secrets.h if needed)
#ifndef HEARTBEAT_PIN
#define HEARTBEAT_PIN 8
#endif
#ifndef HEARTBEAT_ACTIVE_LOW
#define HEARTBEAT_ACTIVE_LOW 1
#endif

#if HEARTBEAT_ACTIVE_LOW
#define HEARTBEAT_ON_LEVEL LOW
#define HEARTBEAT_OFF_LEVEL HIGH
#else
#define HEARTBEAT_ON_LEVEL HIGH
#define HEARTBEAT_OFF_LEVEL LOW
#endif

// Identity
#define UID_PREFIX "SolarGW-"

// WiFi restart policy: number of reconnect attempts (5s interval) before reboot
#define WIFI_RESET_AFTER_ATTEMPTS 36

// RS485 / Modbus
#define MODBUS_BAUD 9600
#define MODBUS_SLAVE_ADDR 1
#define MODBUS_UART_NUM 1
#define MODBUS_RX_PIN 5
#define MODBUS_TX_PIN 4
#define MODBUS_DE_RE_PIN -1

// Inverter register addresses — Holding Registers (function 0x03)
// VERY HIGH CONFIDENCE
#define REG_AC_OUTPUT_VOLTAGE 0x0002
#define REG_BUS_VOLTAGE 0x0004
#define REG_BATT_VOLTAGE 0x0008
#define REG_MPPT_HEATSINK_TEMP 0x000A
#define REG_DCDC_HEATSINK_TEMP 0x000F
#define REG_INV_HEATSINK_TEMP 0x0010
#define REG_BATT_CURRENT 0x0011
#define REG_LOAD_POWER_PCT 0x0013
#define REG_PV_MPPT_FLAG 0x0016
#define REG_LOAD_POWER_WATTS 0x0020
#define REG_BATT_SOC 0x0033

// HIGH CONFIDENCE
#define REG_BUS_CURRENT 0x0009
#define REG_PV1_CURRENT 0x000B
#define REG_PV1_VOLTAGE 0x000C
#define REG_INVERTER_ACTIVE_FLAG 0x0015
#define REG_CONFIG_INV_BIT 0x0017
#define REG_PV1_POWER 0x0037
#define REG_BATT_POWER 0x003E

// AC Grid / energy
#define REG_AC_INPUT_VOLTAGE 0x0005
#define REG_AC_INPUT_CURRENT 0x0007
#define REG_AC_INPUT_FREQ 0x0012
#define REG_PV1_GENERATION 0x0039
#define REG_AC_CHARGE_POWER 0x003B
#define REG_BATT_CHARGE_ENERGY 0x003F
#define REG_DISCHARGE_CAPACITY 0x0040
#define REG_LOADS_ELECTRICITY 0x0041
#define REG_AC_INPUT_POWER 0x0042

// Serial Debug Macros
#if ENABLE_SERIAL_DEBUG
#define DEBUG_BEGIN(baud) Serial.begin(baud)
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_BEGIN(baud)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif

// Global variables (ported)
static ModbusMaster charger;
static HardwareSerial modbusSerial(MODBUS_UART_NUM);

// Inverter live data
static float invAcOutputVoltage = 0;
static float invBusVoltage = 0;
static float invBattVoltage = 0;
static float invBusCurrent = 0;
static float invBattCurrent = 0;
static float invPv1Current = 0;
static float invPv1Voltage = 0;
static uint16_t invPv1Power = 0;
static uint16_t invMpptHeatsinkTemp = 0;
static uint16_t invDcdcHeatsinkTemp = 0;
static uint16_t invInvHeatsinkTemp = 0;
static uint16_t invLoadPowerPct = 0;
static uint16_t invLoadPowerWatts = 0;
static uint16_t invBattSoc = 0;
static uint16_t invBattPower = 0;
static uint16_t invPvMpptFlag = 0;
static uint16_t invInverterActiveFlag = 0;
static uint16_t invConfigInvBit = 0;
static float invAcInputVoltage = 0;
static float invAcInputCurrent = 0;
static float invAcInputFreq = 0;
static uint16_t invAcInputPower = 0;
static uint16_t invAcChargePower = 0;
static uint16_t invPv1Generation = 0;
static uint16_t invBattChargeEnergy = 0;
static uint16_t invDischargeCapacity = 0;
static uint16_t invLoadsElectricity = 0;

static WiFiClientSecure espClient;
static PubSubClient mqttClient(espClient);
static String mqttClientId;
static bool wifiConnected = false;

static uint8_t macAddr[6];
static char devUniqueID[30];

static char publishBuf[MQTT_BUFFER_SIZE];
static char topicBuf[128];
static char uidBuf[64];
static StaticJsonDocument<1024> jsonDoc;

static bool discoveryDone = false;
static unsigned long lastPublishMs = 0;
static bool modbusReadOk = false;

static bool hadInitialWiFiConnection = false;
static unsigned long wifiReconnectCount = 0;
static unsigned long mqttReconnectCount = 0;
static unsigned long modbusOkCount = 0;
static unsigned long modbusErrorCount = 0;
static char lastModbusErrorCode[24] = "Unknown";
static unsigned long lastModbusSuccessMs = 0;
static unsigned long lastModbusReadDurationMs = 0;
static char bootResetReason[48] = "";
static uint8_t modbusConsecutiveErrors = 0;
// Heartbeat state
static unsigned long lastHeartbeatMs = 0;
static bool heartbeatLedState = false;

static void onWiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    DEBUG_PRINTF("WiFi Got IP: %s\n", WiFi.localIP().toString().c_str());
    wifiConnected = true;
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    DEBUG_PRINTLN("WiFi Disconnected");
    wifiConnected = false;
    break;
  default:
    break;
  }
}

// FreeRTOS task prototypes
static void heartbeatTask(void *pvParameters);
static void mainTask(void *pvParameters);

static void buildDeviceUniqueID()
{
  int pos = snprintf(devUniqueID, sizeof(devUniqueID), "%s", UID_PREFIX);
  for (int i = 2; i >= 0; i--)
  {
    pos += snprintf(&devUniqueID[pos], sizeof(devUniqueID) - pos, "%02X", macAddr[i]);
  }
  DEBUG_PRINTF("Unique ID: %s\n", devUniqueID);
}

static void addDeviceBlock(JsonDocument &doc)
{
  JsonObject dev = doc.createNestedObject("device");
  dev["ids"] = devUniqueID;
  dev["name"] = "Solar Inverter";
  dev["mf"] = "Custom";
  dev["mdl"] = "ESP32-C3 + Modbus";
  dev["sw"] = FW_VERSION;
}

static bool publishDiscoveryMsg(PubSubClient &client, const char *label)
{
  size_t len = serializeJson(jsonDoc, publishBuf, sizeof(publishBuf));
  (void)len;
  DEBUG_PRINTF("  [%s] topic=%s  payload=%u bytes\n", label, topicBuf, (unsigned)len);
  bool ok = client.publish(topicBuf, publishBuf, true);
  DEBUG_PRINTF("  -> %s: %s\n", label, ok ? "published OK" : "publish FAILED");
  yield();
  return ok;
}

static void prepareEntity(const char *suffix)
{
  snprintf(topicBuf, sizeof(topicBuf), "homeassistant/sensor/%s%s/config", devUniqueID, suffix);
  snprintf(uidBuf, sizeof(uidBuf), "%s%s", devUniqueID, suffix);
}

static bool discoverEntity(PubSubClient &client, const char *suffix, const char *name, const char *entityId, const char *devCla, const char *statCla, const char *unit, const char *icon, int8_t precision, const char *statTopic, const char *jsonKey, bool isDiag)
{
  jsonDoc.clear();
  prepareEntity(suffix);
  jsonDoc["name"] = name;
  jsonDoc["uniq_id"] = uidBuf;
  if (entityId)
    jsonDoc["default_entity_id"] = entityId;
  if (devCla)
    jsonDoc["dev_cla"] = devCla;
  if (statCla)
    jsonDoc["stat_cla"] = statCla;
  if (unit)
    jsonDoc["unit_of_meas"] = unit;
  if (icon)
    jsonDoc["icon"] = icon;
  if (precision >= 0)
    jsonDoc["suggested_display_precision"] = precision;
  if (isDiag)
    jsonDoc["ent_cat"] = "diagnostic";
  jsonDoc["stat_t"] = statTopic;
  jsonDoc["avty_t"] = MQTT_AVAIL_TOPIC;
  char valTpl[48];
  snprintf(valTpl, sizeof(valTpl), "{{ value_json.%s }}", jsonKey);
  jsonDoc["val_tpl"] = valTpl;
  addDeviceBlock(jsonDoc);
  return publishDiscoveryMsg(client, name);
}

static uint16_t mqttKeepAliveSecondsFromPublishInterval()
{
  uint64_t keepAliveSeconds = (PUBLISH_INTERVAL_MS * 3) / 1000UL;
  if (keepAliveSeconds < 5ULL)
    keepAliveSeconds = 5ULL;
  if (keepAliveSeconds > 65535ULL)
    keepAliveSeconds = 65535ULL;
  return (uint16_t)keepAliveSeconds;
}

static void startDiscovery(PubSubClient &client)
{
  DEBUG_PRINTLN("Starting HA MQTT discovery for inverter...");

  discoverEntity(client, "BV", "Battery Voltage", "sensor.inverter_battery_voltage", "voltage", "measurement", "V", nullptr, 2, STAT_TOPIC_DATA, "bv", false);
  discoverEntity(client, "SoC", "Battery SoC", "sensor.inverter_battery_soc", "battery", "measurement", "%", nullptr, -1, STAT_TOPIC_DATA, "soc", false);
  discoverEntity(client, "AOV", "AC Output Voltage", "sensor.inverter_ac_output_voltage", "voltage", "measurement", "V", nullptr, 1, STAT_TOPIC_DATA, "aov", false);
  discoverEntity(client, "BUS", "Bus Voltage", "sensor.inverter_bus_voltage", "voltage", "measurement", "V", nullptr, 1, STAT_TOPIC_DATA, "busv", false);
  discoverEntity(client, "BC", "Bus Current", "sensor.inverter_bus_current", "current", "measurement", "A", nullptr, -1, STAT_TOPIC_DATA, "busc", false);
  discoverEntity(client, "BATTC", "Battery Current", "sensor.inverter_battery_current", "current", "measurement", "A", nullptr, -1, STAT_TOPIC_DATA, "bc", false);
  discoverEntity(client, "BATTP", "Battery Power", "sensor.inverter_battery_power", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "bp", false);
  discoverEntity(client, "PV1V", "PV1 Voltage", "sensor.inverter_pv1_voltage", "voltage", "measurement", "V", nullptr, 1, STAT_TOPIC_DATA, "pv1v", false);
  discoverEntity(client, "PV1C", "PV1 Current", "sensor.inverter_pv1_current", "current", "measurement", "A", nullptr, -1, STAT_TOPIC_DATA, "pv1c", false);
  discoverEntity(client, "PV1P", "PV1 Power", "sensor.inverter_pv1_power", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "pv1p", false);
  discoverEntity(client, "LP", "Power Consumption", "sensor.inverter_power_consumption", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "lw", false);
  discoverEntity(client, "LPP", "Load Power Percentage", "sensor.inverter_load_power_pct", nullptr, "measurement", "%", "mdi:gauge", -1, STAT_TOPIC_DATA, "lp", false);
  discoverEntity(client, "MPPT", "MPPT Heatsink Temp", "sensor.inverter_mppt_heatsink_temp", "temperature", "measurement", "°C", nullptr, -1, STAT_TOPIC_DATA, "mt", false);
  discoverEntity(client, "IHT", "Inverter Heatsink Temp", "sensor.inverter_heatsink_temp", "temperature", "measurement", "°C", nullptr, -1, STAT_TOPIC_DATA, "it", false);
  discoverEntity(client, "DCDC", "DC/DC Heatsink Temp", "sensor.inverter_dcdc_heatsink_temp", "temperature", "measurement", "°C", nullptr, -1, STAT_TOPIC_DATA, "dt", false);
  discoverEntity(client, "AIV", "AC Input Voltage", "sensor.inverter_ac_input_voltage", "voltage", "measurement", "V", nullptr, -1, STAT_TOPIC_DATA, "aiv", false);
  discoverEntity(client, "AIC", "AC Input Current", "sensor.inverter_ac_input_current", "current", "measurement", "A", nullptr, -1, STAT_TOPIC_DATA, "aic", false);
  discoverEntity(client, "AIF", "AC Input Frequency", "sensor.inverter_ac_input_frequency", "frequency", "measurement", "Hz", nullptr, 1, STAT_TOPIC_DATA, "aif", false);
  discoverEntity(client, "AIP", "AC Input Power", "sensor.inverter_ac_input_power", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "aip", false);
  discoverEntity(client, "ACP", "AC Charging Power", "sensor.inverter_ac_charging_power", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "acp", false);
  discoverEntity(client, "PV1G", "PV1 Generation", "sensor.inverter_pv1_generation", "energy", "total_increasing", "kWh", nullptr, -1, STAT_TOPIC_DATA, "pg", false);
  discoverEntity(client, "BCE", "Battery Charge Energy", "sensor.inverter_battery_charge_energy", "energy", "total_increasing", "kWh", nullptr, -1, STAT_TOPIC_DATA, "bce", false);
  discoverEntity(client, "DC", "Discharge Capacity", "sensor.inverter_discharge_capacity", "energy", "total_increasing", "kWh", nullptr, -1, STAT_TOPIC_DATA, "dc", false);
  discoverEntity(client, "LE", "Loads Electricity", "sensor.inverter_loads_electricity", "energy", "total_increasing", "kWh", nullptr, -1, STAT_TOPIC_DATA, "le", false);

  discoverEntity(client, "IP", "IP Address", nullptr, nullptr, nullptr, nullptr, "mdi:ip-network", -1, STAT_TOPIC_DIAG_DATA, "ip", true);
  discoverEntity(client, "HEAP", "Free Heap", "sensor.inverter_free_heap", nullptr, "measurement", "B", "mdi:memory", -1, STAT_TOPIC_DIAG_DATA, "heap", true);
  discoverEntity(client, "MBLK", "Max Free Block", "sensor.inverter_max_free_block", nullptr, "measurement", "B", "mdi:memory", -1, STAT_TOPIC_DIAG_DATA, "mblk", true);
  discoverEntity(client, "FRAG", "Heap Fragmentation", "sensor.inverter_heap_fragmentation", nullptr, "measurement", "%", "mdi:chart-sankey", -1, STAT_TOPIC_DIAG_DATA, "frag", true);
  discoverEntity(client, "RSSI", "WiFi RSSI", "sensor.inverter_wifi_rssi", "signal_strength", "measurement", "dBm", "mdi:wifi", -1, STAT_TOPIC_DIAG_DATA, "rssi", true);
  discoverEntity(client, "WRC", "WiFi Reconnect Count", "sensor.inverter_wifi_reconnect_count", nullptr, "total_increasing", nullptr, "mdi:wifi-refresh", -1, STAT_TOPIC_DIAG_DATA, "wrc", true);
  discoverEntity(client, "MRC", "MQTT Reconnect Count", "sensor.inverter_mqtt_reconnect_count", nullptr, "total_increasing", nullptr, "mdi:wifi-refresh", -1, STAT_TOPIC_DIAG_DATA, "mqrc", true);
  discoverEntity(client, "MS", "Modbus Status", "sensor.inverter_modbus_status", nullptr, nullptr, nullptr, "mdi:swap-horizontal", -1, STAT_TOPIC_DIAG_DATA, "ms", true);
  discoverEntity(client, "MOK", "Modbus OK Count", "sensor.inverter_modbus_ok_count", nullptr, "total_increasing", nullptr, "mdi:counter", -1, STAT_TOPIC_DIAG_DATA, "mok", true);
  discoverEntity(client, "MER", "Modbus Error Count", "sensor.inverter_modbus_error_count", nullptr, "total_increasing", nullptr, "mdi:counter", -1, STAT_TOPIC_DIAG_DATA, "mer", true);
  discoverEntity(client, "RST", "Reset Reason", "sensor.inverter_reset_reason", nullptr, nullptr, nullptr, "mdi:restart-alert", -1, STAT_TOPIC_DIAG_DATA, "rr", true);

  // -- Restart button entity --
  jsonDoc.clear();
  snprintf(topicBuf, sizeof(topicBuf), "homeassistant/button/%sRST_BTN/config", devUniqueID);
  snprintf(uidBuf, sizeof(uidBuf), "%sRST_BTN", devUniqueID);
  jsonDoc["name"] = "Restart Device";
  jsonDoc["uniq_id"] = uidBuf;
  jsonDoc["default_entity_id"] = "button.inverter_restart";
  jsonDoc["ent_cat"] = "config";
  jsonDoc["icon"] = "mdi:restart";
  jsonDoc["cmd_t"] = CMND_TOPIC_RESTART;
  jsonDoc["pl_prs"] = "1";
  jsonDoc["avty_t"] = MQTT_AVAIL_TOPIC;
  addDeviceBlock(jsonDoc);
  publishDiscoveryMsg(client, "Restart Device");

  // -- Availability entity (special: own topic, no val_tpl) --
  jsonDoc.clear();
  prepareEntity("AVAIL");
  jsonDoc["name"] = "Device Availability";
  jsonDoc["uniq_id"] = uidBuf;
  jsonDoc["default_entity_id"] = "sensor.inverter_device_availability";
  jsonDoc["ent_cat"] = "diagnostic";
  jsonDoc["icon"] = "mdi:lan-connect";
  jsonDoc["stat_t"] = MQTT_AVAIL_TOPIC;
  addDeviceBlock(jsonDoc);
  publishDiscoveryMsg(client, "Device Availability");
}

static void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  DEBUG_PRINTF("MQTT msg: %s (%u bytes)\n", topic, length);
  if (strcmp(topic, CMND_TOPIC_RESTART) == 0)
  {
    DEBUG_PRINTLN("Restart command received via MQTT!");
    mqttClient.publish(MQTT_AVAIL_TOPIC, "offline", true);
    delay(200);
    ESP.restart();
  }
}

static bool mqttReconnect()
{
  for (int i = 0; i < MQTT_MAX_RETRIES; i++)
  {
    DEBUG_PRINT("MQTT connecting... ");
    yield();
    if (mqttClient.connect(mqttClientId.c_str(), MQTT_USER, MQTT_PASSWORD, MQTT_AVAIL_TOPIC, 1, true, "offline"))
    {
      DEBUG_PRINTLN("connected!");
      mqttClient.publish(MQTT_AVAIL_TOPIC, "online", true);
      mqttClient.subscribe(CMND_TOPIC_RESTART);
      return true;
    }
    DEBUG_PRINTF("failed, rc=%d\n", mqttClient.state());
    delay(1000);
    mqttReconnectCount++;
  }
  return false;
}

static void modbusPreTransmission()
{
  if (MODBUS_DE_RE_PIN >= 0)
    digitalWrite(MODBUS_DE_RE_PIN, HIGH);
}
static void modbusPostTransmission()
{
  if (MODBUS_DE_RE_PIN >= 0)
    digitalWrite(MODBUS_DE_RE_PIN, LOW);
}

static void updateModbusErrorCode(uint8_t error)
{
  const char *msg;
  switch (error)
  {
  case ModbusMaster::ku8MBSuccess:
    msg = "Success";
    break;
  case ModbusMaster::ku8MBInvalidSlaveID:
    msg = "Invalid Slave ID";
    break;
  case ModbusMaster::ku8MBInvalidFunction:
    msg = "Invalid Function";
    break;
  case ModbusMaster::ku8MBResponseTimedOut:
    msg = "Timeout";
    break;
  case ModbusMaster::ku8MBInvalidCRC:
    msg = "Invalid CRC";
    break;
  default:
    msg = "Unknown";
    break;
  }
  strncpy(lastModbusErrorCode, msg, sizeof(lastModbusErrorCode) - 1);
  lastModbusErrorCode[sizeof(lastModbusErrorCode) - 1] = '\0';
}

#define MODBUS_READ_RETRIES 3
#define MODBUS_REINIT_THRESHOLD 5
#define IDX(reg, base) ((reg) - (base))

static bool readHoldingBlockWithRetry(uint16_t startReg, uint16_t regCount, uint8_t &lastError)
{
  lastError = ModbusMaster::ku8MBResponseTimedOut;
  for (uint8_t attempt = 0; attempt < MODBUS_READ_RETRIES; ++attempt)
  {
    while (modbusSerial.available())
      modbusSerial.read();
    delay(20);
    esp_task_wdt_reset();
    uint8_t res = charger.readHoldingRegisters(startReg, regCount);
    if (res == ModbusMaster::ku8MBSuccess)
    {
      lastError = res;
      return true;
    }
    lastError = res;
    DEBUG_PRINTF("Modbus block 0x%04X len=%u failed attempt %u (err=%u)\n", startReg, regCount, attempt + 1, res);
    delay(50 + (attempt * 50));
  }
  return false;
}

static void reinitModbus()
{
  DEBUG_PRINTLN("Reinitializing Modbus after consecutive failures...");
  modbusSerial.end();
  delay(200);
  modbusSerial.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  delay(100);
  charger.begin(MODBUS_SLAVE_ADDR, modbusSerial);
  charger.preTransmission(modbusPreTransmission);
  charger.postTransmission(modbusPostTransmission);
  charger.idle([]()
               { esp_task_wdt_reset(); });
  modbusConsecutiveErrors = 0;
  DEBUG_PRINTLN("Modbus re-initialized.");
}

static bool readInverterRegisters()
{
  const unsigned long readStartMs = millis();
  uint8_t lastErr = ModbusMaster::ku8MBSuccess;

  if (!readHoldingBlockWithRetry(0x0001, 0x0020, lastErr))
  {
    lastModbusReadDurationMs = millis() - readStartMs;
    modbusErrorCount++;
    updateModbusErrorCode(lastErr);
    modbusConsecutiveErrors++;
    DEBUG_PRINTF("Modbus block 1 failed: %s (consecutive: %u)\n", lastModbusErrorCode, modbusConsecutiveErrors);
    if (modbusConsecutiveErrors >= MODBUS_REINIT_THRESHOLD)
      reinitModbus();
    return false;
  }

  invAcOutputVoltage = charger.getResponseBuffer(IDX(REG_AC_OUTPUT_VOLTAGE, 0x0001));
  invBusVoltage = charger.getResponseBuffer(IDX(REG_BUS_VOLTAGE, 0x0001)) / 10.0f;
  invAcInputVoltage = charger.getResponseBuffer(IDX(REG_AC_INPUT_VOLTAGE, 0x0001));
  invAcInputCurrent = charger.getResponseBuffer(IDX(REG_AC_INPUT_CURRENT, 0x0001)) / 10.0f;
  invBattVoltage = charger.getResponseBuffer(IDX(REG_BATT_VOLTAGE, 0x0001)) / 10.0f;
  invBusCurrent = (int16_t)charger.getResponseBuffer(IDX(REG_BUS_CURRENT, 0x0001)) / 100.0f;
  invMpptHeatsinkTemp = charger.getResponseBuffer(IDX(REG_MPPT_HEATSINK_TEMP, 0x0001));
  invPv1Current = charger.getResponseBuffer(IDX(REG_PV1_CURRENT, 0x0001)) / 10.0f;
  invPv1Voltage = charger.getResponseBuffer(IDX(REG_PV1_VOLTAGE, 0x0001)) / 10.0f;
  invDcdcHeatsinkTemp = charger.getResponseBuffer(IDX(REG_DCDC_HEATSINK_TEMP, 0x0001));
  invInvHeatsinkTemp = charger.getResponseBuffer(IDX(REG_INV_HEATSINK_TEMP, 0x0001));
  invBattCurrent = (int16_t)charger.getResponseBuffer(IDX(REG_BATT_CURRENT, 0x0001)) / 10.0f;
  invAcInputFreq = charger.getResponseBuffer(IDX(REG_AC_INPUT_FREQ, 0x0001)) / 10.0f;
  invLoadPowerPct = charger.getResponseBuffer(IDX(REG_LOAD_POWER_PCT, 0x0001));
  invInverterActiveFlag = charger.getResponseBuffer(IDX(REG_INVERTER_ACTIVE_FLAG, 0x0001));
  invPvMpptFlag = charger.getResponseBuffer(IDX(REG_PV_MPPT_FLAG, 0x0001));
  invConfigInvBit = charger.getResponseBuffer(IDX(REG_CONFIG_INV_BIT, 0x0001));
  invLoadPowerWatts = charger.getResponseBuffer(IDX(REG_LOAD_POWER_WATTS, 0x0001));

  delay(30);

  if (!readHoldingBlockWithRetry(REG_BATT_SOC, (REG_AC_INPUT_POWER - REG_BATT_SOC + 1), lastErr))
  {
    lastModbusReadDurationMs = millis() - readStartMs;
    modbusErrorCount++;
    updateModbusErrorCode(lastErr);
    modbusConsecutiveErrors++;
    DEBUG_PRINTF("Modbus block 2 failed: %s (consecutive: %u)\n", lastModbusErrorCode, modbusConsecutiveErrors);
    if (modbusConsecutiveErrors >= MODBUS_REINIT_THRESHOLD)
      reinitModbus();
    return false;
  }

  invBattSoc = charger.getResponseBuffer(IDX(REG_BATT_SOC, REG_BATT_SOC));
  invPv1Power = charger.getResponseBuffer(IDX(REG_PV1_POWER, REG_BATT_SOC));
  invPv1Generation = charger.getResponseBuffer(IDX(REG_PV1_GENERATION, REG_BATT_SOC));
  invAcChargePower = charger.getResponseBuffer(IDX(REG_AC_CHARGE_POWER, REG_BATT_SOC));
  invBattPower = charger.getResponseBuffer(IDX(REG_BATT_POWER, REG_BATT_SOC));
  invBattChargeEnergy = charger.getResponseBuffer(IDX(REG_BATT_CHARGE_ENERGY, REG_BATT_SOC));
  invDischargeCapacity = charger.getResponseBuffer(IDX(REG_DISCHARGE_CAPACITY, REG_BATT_SOC));
  invLoadsElectricity = charger.getResponseBuffer(IDX(REG_LOADS_ELECTRICITY, REG_BATT_SOC));
  invAcInputPower = charger.getResponseBuffer(IDX(REG_AC_INPUT_POWER, REG_BATT_SOC));

  lastModbusReadDurationMs = millis() - readStartMs;
  modbusOkCount++;
  lastModbusSuccessMs = millis();
  updateModbusErrorCode(ModbusMaster::ku8MBSuccess);
  modbusConsecutiveErrors = 0;

  DEBUG_PRINTF("Inverter: Batt=%.1fV/%.1fA/%uW (%u%%) PV=%.1fV/%.1fA/%uW AC=%.1fV Load=%uW(%u%%) Temps: MPPT=%uC INV=%uC DCDC=%uC\n",
               invBattVoltage, invBattCurrent, invBattPower, invBattSoc,
               invPv1Voltage, invPv1Current, invPv1Power,
               invAcOutputVoltage,
               invLoadPowerWatts, invLoadPowerPct,
               invMpptHeatsinkTemp, invInvHeatsinkTemp, invDcdcHeatsinkTemp);
  DEBUG_PRINTF("  ACgrid: In=%.0fV/%.2fA/%.1fHz/%uW ChgPow=%uW PV1Gen=%ukWh BattChgE=%ukWh\n",
               invAcInputVoltage, invAcInputCurrent, invAcInputFreq, invAcInputPower,
               invAcChargePower, invPv1Generation, invBattChargeEnergy);

  return true;
}

static void publishSensorData()
{
  modbusReadOk = readInverterRegisters();
  if (!modbusReadOk)
  {
    DEBUG_PRINTLN("Modbus read failed — publishing last known values.");
  }

  bool ok = true;
  size_t len;
  (void)len;

  jsonDoc.clear();
  jsonDoc["bv"] = invBattVoltage;
  jsonDoc["soc"] = invBattSoc;
  jsonDoc["aov"] = invAcOutputVoltage;
  jsonDoc["busv"] = invBusVoltage;
  jsonDoc["busc"] = invBusCurrent;
  jsonDoc["bc"] = invBattCurrent;
  jsonDoc["bp"] = invBattPower;
  jsonDoc["pv1v"] = invPv1Voltage;
  jsonDoc["pv1c"] = invPv1Current;
  jsonDoc["pv1p"] = invPv1Power;
  jsonDoc["pg"] = invPv1Generation;
  jsonDoc["lw"] = invLoadPowerWatts;
  jsonDoc["lp"] = invLoadPowerPct;
  jsonDoc["mt"] = invMpptHeatsinkTemp;
  jsonDoc["it"] = invInvHeatsinkTemp;
  jsonDoc["dt"] = invDcdcHeatsinkTemp;
  jsonDoc["aiv"] = invAcInputVoltage;
  jsonDoc["aic"] = invAcInputCurrent;
  jsonDoc["aif"] = invAcInputFreq;
  jsonDoc["aip"] = invAcInputPower;
  jsonDoc["acp"] = invAcChargePower;
  jsonDoc["bce"] = invBattChargeEnergy;
  jsonDoc["dc"] = invDischargeCapacity;
  jsonDoc["le"] = invLoadsElectricity;
  len = serializeJson(jsonDoc, publishBuf, sizeof(publishBuf));
  ok &= mqttClient.publish(STAT_TOPIC_DATA, publishBuf, true);
  DEBUG_PRINTF("Sensor JSON: %u bytes\n", (unsigned)len);

  jsonDoc.clear();
  jsonDoc["ip"] = WiFi.localIP().toString();
  jsonDoc["rssi"] = WiFi.RSSI();
  jsonDoc["heap"] = ESP.getFreeHeap();
  size_t mblk = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  jsonDoc["mblk"] = (unsigned long)mblk;
  jsonDoc["frag"] = (mblk == 0) ? 100 : (int)(100.0f * (1.0f - ((float)mblk / (float)ESP.getFreeHeap())));
  jsonDoc["wrc"] = wifiReconnectCount;
  jsonDoc["mqrc"] = mqttReconnectCount;
  jsonDoc["ms"] = lastModbusErrorCode;
  jsonDoc["mok"] = modbusOkCount;
  jsonDoc["mer"] = modbusErrorCount;
  jsonDoc["rr"] = bootResetReason;
  len = serializeJson(jsonDoc, publishBuf, sizeof(publishBuf));
  ok &= mqttClient.publish(STAT_TOPIC_DIAG_DATA, publishBuf, true);
  DEBUG_PRINTF("Diag JSON: %u bytes\n", (unsigned)len);

  DEBUG_PRINTLN(ok ? "Sensor data published OK." : "Sensor data publish: one or more FAILED!");
}

void setup()
{
  // Very early ROM-level print removed.

  DEBUG_BEGIN(115200);
  // Ensure ESP core debug output is routed to Serial
  Serial.setDebugOutput(true);
  // Initialize heartbeat LED
  pinMode(HEARTBEAT_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_PIN, HEARTBEAT_OFF_LEVEL);
  // Initialize heartbeat timer so LED remains in OFF state until first toggle
  lastHeartbeatMs = millis();
  delay(1000);
  DEBUG_PRINTLN("\n\nInverter Gateway (ESP32-C3) starting...");

  esp_reset_reason_t r = esp_reset_reason();
  const char *reason = "Unknown";
  switch (r)
  {
  case ESP_RST_POWERON:
    reason = "Power-on";
    break;
  case ESP_RST_SW:
    reason = "Software";
    break;
  case ESP_RST_PANIC:
    reason = "Panic/Exception";
    break;
  case ESP_RST_INT_WDT:
    reason = "Interrupt WDT";
    break;
  case ESP_RST_TASK_WDT:
    reason = "Task WDT";
    break;
  case ESP_RST_WDT:
    reason = "Other WDT";
    break;
  case ESP_RST_DEEPSLEEP:
    reason = "Deep Sleep";
    break;
  case ESP_RST_BROWNOUT:
    reason = "Brownout";
    break;
  default:
    reason = "Unknown";
    break;
  }
  strncpy(bootResetReason, reason, sizeof(bootResetReason) - 1);

  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWiFiEvent);
  // Ensure the ESP32 WiFi driver will attempt to reconnect automatically
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  DEBUG_PRINTF("Connecting to WiFi: %s\n", WIFI_SSID);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40)
  {
    delay(500);
    DEBUG_PRINT(".");
    esp_task_wdt_reset();
    attempts++;
  }
  DEBUG_PRINTLN();

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiConnected = true;
    hadInitialWiFiConnection = true;
    DEBUG_PRINTF("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  }
  else
  {
    DEBUG_PRINTLN("WiFi connection failed!");
  }

  WiFi.macAddress(macAddr);
  mqttClientId = UID_PREFIX + String(macAddr[3], HEX) + String(macAddr[4], HEX) + String(macAddr[5], HEX);
  buildDeviceUniqueID();

  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  uint16_t mqttKeepAliveSeconds = mqttKeepAliveSecondsFromPublishInterval();
  mqttClient.setKeepAlive(mqttKeepAliveSeconds);
  DEBUG_PRINTF("MQTT keep alive: %u s\n", mqttKeepAliveSeconds);
  if (strstr(MQTT_CA_CERT, "BEGIN CERTIFICATE") != nullptr)
  {
    espClient.setCACert(MQTT_CA_CERT);
    DEBUG_PRINTLN("MQTT TLS: CA certificate validation enabled.");
  }
  else
  {
    espClient.setInsecure();
    DEBUG_PRINTLN("MQTT TLS: insecure mode (no CA cert configured).");
  }
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  if (MODBUS_DE_RE_PIN >= 0)
  {
    pinMode(MODBUS_DE_RE_PIN, OUTPUT);
    digitalWrite(MODBUS_DE_RE_PIN, LOW);
  }
  modbusSerial.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  delay(100);
  charger.begin(MODBUS_SLAVE_ADDR, modbusSerial);
  charger.preTransmission(modbusPreTransmission);
  charger.postTransmission(modbusPostTransmission);
  charger.idle([]()
               { esp_task_wdt_reset(); });

  // Increase WDT timeout to allow longer Modbus/TLS operations
  esp_task_wdt_init(30, true);
  DEBUG_PRINTLN("Task WDT enabled.");

  // Ensure WDT is reset before potentially blocking Modbus reads
  esp_task_wdt_reset();

  // Create FreeRTOS tasks: heartbeat and main
  xTaskCreate(heartbeatTask, "heartbeat", 2048, NULL, 2, NULL);
  xTaskCreate(mainTask, "mainTask", 8192, NULL, 1, NULL);
}

// Empty Arduino loop: main work is done in FreeRTOS tasks
void loop()
{
}

// Heartbeat task: toggles heartbeat pin at HEARTBEAT_INTERVAL_MS
static void heartbeatTask(void *pvParameters)
{
  (void)pvParameters;
  // Register this task with the task watchdog
  esp_task_wdt_add(NULL);
  for (;;)
  {
    esp_task_wdt_reset();
    heartbeatLedState = !heartbeatLedState;
    digitalWrite(HEARTBEAT_PIN, heartbeatLedState ? HEARTBEAT_ON_LEVEL : HEARTBEAT_OFF_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
  }
}

// Main task: contains previous loop() logic, runs as FreeRTOS task
static void mainTask(void *pvParameters)
{
  (void)pvParameters;
  // Register this task with the task watchdog
  esp_task_wdt_add(NULL);
  static unsigned long lastWifiReconnectAttemptMs = 0;
  static int wifiReconnectAttemptCounter = 0;
  for (;;)
  {
    esp_task_wdt_reset();

    // Track connection state locally to avoid modifying the global
    // `wifiConnected` which is owned by the WiFi event handler.
    static bool prevWifiConnected = false;
    bool nowWifiConnected = (WiFi.status() == WL_CONNECTED);

    if (!nowWifiConnected)
    {
      if (prevWifiConnected)
      {
        DEBUG_PRINTLN("WiFi disconnected!");
      }
      unsigned long now = millis();
      if ((now - lastWifiReconnectAttemptMs) >= 5000)
      {
        lastWifiReconnectAttemptMs = now;
        wifiReconnectAttemptCounter++;
        DEBUG_PRINTLN("Attempting WiFi reconnect...");
        // Try a light reconnect first; every few attempts do a full WiFi.begin()
        if ((wifiReconnectAttemptCounter % 3) == 0)
        {
          DEBUG_PRINTLN("Rebootstrapping WiFi: disconnect + reconnect");
          WiFi.disconnect();
          delay(100);
          WiFi.reconnect();
        }
        else
        {
          WiFi.reconnect();
        }

        // If we've retried too many times, reboot the device to recover
        if (wifiReconnectAttemptCounter >= WIFI_RESET_AFTER_ATTEMPTS)
        {
          DEBUG_PRINTF("WiFi failed to reconnect after %d attempts — restarting...\n", WIFI_RESET_AFTER_ATTEMPTS);
          delay(200);
          ESP.restart();
        }
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (nowWifiConnected && !prevWifiConnected)
    {
      if (hadInitialWiFiConnection)
      {
        wifiReconnectCount++;
      }
      else
      {
        hadInitialWiFiConnection = true;
      }
      // Reset reconnect attempt counter on successful connection
      wifiReconnectAttemptCounter = 0;
      DEBUG_PRINTF("WiFi reconnected! IP: %s\n", WiFi.localIP().toString().c_str());
    }

    // Update previous-state snapshot for next iteration
    prevWifiConnected = nowWifiConnected;

    if (!mqttClient.connected())
    {
      if (!mqttReconnect())
      {
        DEBUG_PRINTLN("MQTT connect failed, retrying next loop.");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
      if (!discoveryDone)
      {
        startDiscovery(mqttClient);
        discoveryDone = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
        publishSensorData();
        lastPublishMs = millis();
      }
    }

    mqttClient.loop();

    unsigned long now = millis();
    if ((now - lastPublishMs) >= PUBLISH_INTERVAL_MS)
    {
      lastPublishMs = now;
      publishSensorData();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
