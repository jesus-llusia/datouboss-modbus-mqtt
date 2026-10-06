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
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include "secrets.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Debug Configuration
#define ENABLE_SERIAL_DEBUG 1

// Firmware version
#define FW_VERSION "3.1.0"

// MQTT
#define MQTT_BUFFER_SIZE 768
#define MQTT_MAX_RETRIES 3
// Ignore commands for this long after (re)connecting, to skip retained messages
#define MQTT_CMD_GRACE_MS 5000UL
// Connection timeouts. One connect attempt can take up to
// TLS_CONNECT_TIMEOUT_S + TLS_HANDSHAKE_TIMEOUT_S + MQTT_SOCKET_TIMEOUT_S,
// which must stay well under the 30 s task watchdog.
#define TLS_CONNECT_TIMEOUT_S 8
#define TLS_HANDSHAKE_TIMEOUT_S 8
#define MQTT_SOCKET_TIMEOUT_S 5

// MQTT availability topic
#define MQTT_AVAIL_TOPIC "stat/solar/availability"

// Home Assistant birth/LWT topic: "online" means HA (re)started and needs discovery
#define HA_STATUS_TOPIC "homeassistant/status"
// If a discovery run fails, retry it after this long
#define DISCOVERY_RETRY_MS (60UL * 1000UL)

// Aggregated state topics
#define STAT_TOPIC_DATA "stat/solar/data"
#define STAT_TOPIC_DIAG_DATA "stat/solar/diag_data"

// Command topic (subscriptions)
#define CMND_TOPIC_RESTART "cmnd/solar/restart"

// Timing
#define PUBLISH_INTERVAL_MS (60UL * 1000UL)
// HA marks inverter sensors unavailable if no fresh data arrives for 3 publish cycles
#define SENSOR_EXPIRE_AFTER_S ((PUBLISH_INTERVAL_MS * 3UL) / 1000UL)

// Status LED: the pattern repeats every LED_CYCLE_TICKS * LED_TICK_MS (2 s)
//   1 blink  = all OK
//   2 blinks = MQTT not connected
//   3 blinks = Modbus reads failing
//   fast continuous blink = WiFi not connected
#define LED_TICK_MS 100UL
#define LED_CYCLE_TICKS 20
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

// WiFi recovery policy. The driver's auto-reconnect does the normal work; if
// still disconnected, force a full disconnect + begin every WIFI_RETRY_INTERVAL_MS,
// and reboot once disconnected for WIFI_RESET_AFTER_MS.
#define WIFI_RETRY_INTERVAL_MS (30UL * 1000UL)
#define WIFI_RESET_AFTER_MS (3UL * 60UL * 1000UL)

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
#define REG_BATT_CURRENT 0x0011 // signed: + charging, - discharging
#define REG_LOAD_POWER_PCT 0x0013
#define REG_LOAD_POWER_WATTS 0x0020
#define REG_BATT_SOC 0x0033

// HIGH CONFIDENCE
#define REG_BUS_CURRENT 0x0009
#define REG_PV1_CURRENT 0x000B
#define REG_PV1_VOLTAGE 0x000C
#define REG_PV1_POWER 0x0037
#define REG_BATT_POWER 0x003E

// LOW CONFIDENCE (AC grid / energy)
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

// Global variables
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
static int16_t invMpptHeatsinkTemp = 0;
static int16_t invDcdcHeatsinkTemp = 0;
static int16_t invInvHeatsinkTemp = 0;
static uint16_t invLoadPowerPct = 0;
static uint16_t invLoadPowerWatts = 0;
static uint16_t invBattSoc = 0;
static uint16_t invBattPower = 0;
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

static uint8_t macAddr[6];
static char devUniqueID[30];

static char publishBuf[MQTT_BUFFER_SIZE];
static char topicBuf[128];
static char uidBuf[64];
static JsonDocument jsonDoc;

static bool discoveryDone = false;
static bool discoveryAttempted = false;
static unsigned long lastDiscoveryAttemptMs = 0;
// Set from the MQTT callback; acted on by mainTask outside of mqttClient.loop()
static bool restartRequested = false;
static bool mqttTlsConfigured = false;
static bool mqttHadConnection = false;
static unsigned long lastPublishMs = 0;

static bool hadInitialWiFiConnection = false;
static unsigned long wifiReconnectCount = 0;
static unsigned long mqttReconnectCount = 0;
static unsigned long mqttConnectedAtMs = 0;
static unsigned long modbusOkCount = 0;
static unsigned long modbusErrorCount = 0;
static char lastModbusErrorCode[24] = "Unknown";
static char bootResetReason[48] = "";
static uint8_t modbusConsecutiveErrors = 0;
static bool lastModbusReadOk = false;

// Lifetime energy counters must never go down; a drop means a bad read.
// After this many consecutive drops, accept the new values (e.g. counters were reset on the inverter).
#define ENERGY_DROP_ACCEPT_AFTER 3
static bool haveEnergyBaseline = false;
static uint8_t energyDropCount = 0;

// Status shown on the heartbeat LED; written by mainTask, read by heartbeatTask
enum LedStatus : uint8_t
{
  LED_STATUS_OK,
  LED_STATUS_MQTT_DOWN,
  LED_STATUS_MODBUS_FAIL,
  LED_STATUS_WIFI_DOWN,
};
static volatile LedStatus ledStatus = LED_STATUS_WIFI_DOWN;

static void onWiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    DEBUG_PRINTF("WiFi Got IP: %s\n", WiFi.localIP().toString().c_str());
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    DEBUG_PRINTLN("WiFi Disconnected");
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
  // Use the NIC-specific half of the MAC (bytes 3..5); bytes 0..2 are the vendor OUI
  snprintf(devUniqueID, sizeof(devUniqueID), "%s%02X%02X%02X", UID_PREFIX, macAddr[3], macAddr[4], macAddr[5]);
  DEBUG_PRINTF("Unique ID: %s\n", devUniqueID);
}

static void addDeviceBlock(JsonDocument &doc)
{
  JsonObject dev = doc["device"].to<JsonObject>();
  dev["ids"] = devUniqueID;
  dev["name"] = "Solar Inverter";
  dev["mf"] = "jesus-llusia";
  dev["mdl"] = "ESP32-C3 + Modbus";
  dev["sw"] = FW_VERSION;
}

// Serialize jsonDoc into publishBuf and publish it. Refuses to publish if the
// JSON would not fit (serializeJson would silently truncate it).
static bool publishJsonDoc(PubSubClient &client, const char *topic, bool retained)
{
  size_t needed = measureJson(jsonDoc);
  if (needed >= sizeof(publishBuf))
  {
    DEBUG_PRINTF("JSON for %s too large: %u bytes (buffer %u) — not published\n", topic, (unsigned)needed, (unsigned)sizeof(publishBuf));
    return false;
  }
  serializeJson(jsonDoc, publishBuf, sizeof(publishBuf));
  DEBUG_PRINTF("  topic=%s  payload=%u bytes\n", topic, (unsigned)needed);
  return client.publish(topic, publishBuf, retained);
}

static bool publishDiscoveryMsg(PubSubClient &client, const char *label)
{
  bool ok = publishJsonDoc(client, topicBuf, true);
  DEBUG_PRINTF("  -> %s: %s\n", label, ok ? "published OK" : "publish FAILED");
  yield();
  return ok;
}

static void prepareEntity(const char *suffix)
{
  snprintf(topicBuf, sizeof(topicBuf), "homeassistant/sensor/%s%s/config", devUniqueID, suffix);
  snprintf(uidBuf, sizeof(uidBuf), "%s%s", devUniqueID, suffix);
}

struct SensorEntity
{
  const char *suffix;
  const char *name;
  const char *entityId;
  const char *devCla;
  const char *statCla;
  const char *unit;
  const char *icon;
  int8_t precision; // -1 = let HA decide
  const char *statTopic;
  const char *jsonKey;
  bool isDiag;
};

// Home Assistant sensors. jsonKey must match the keys published in publishSensorData().
static const SensorEntity SENSOR_ENTITIES[] = {
    // suffix, name, default entity id, device class, state class, unit, icon, precision, state topic, json key, diagnostic
    {"BV", "Battery Voltage", "sensor.inverter_battery_voltage", "voltage", "measurement", "V", nullptr, 2, STAT_TOPIC_DATA, "bv", false},
    {"SoC", "Battery SoC", "sensor.inverter_battery_soc", "battery", "measurement", "%", nullptr, -1, STAT_TOPIC_DATA, "soc", false},
    {"AOV", "AC Output Voltage", "sensor.inverter_ac_output_voltage", "voltage", "measurement", "V", nullptr, 1, STAT_TOPIC_DATA, "aov", false},
    {"BUS", "Bus Voltage", "sensor.inverter_bus_voltage", "voltage", "measurement", "V", nullptr, 1, STAT_TOPIC_DATA, "busv", false},
    {"BC", "Bus Current", "sensor.inverter_bus_current", "current", "measurement", "A", nullptr, -1, STAT_TOPIC_DATA, "busc", false},
    {"BATTC", "Battery Current", "sensor.inverter_battery_current", "current", "measurement", "A", nullptr, -1, STAT_TOPIC_DATA, "bc", false},
    {"BATTP", "Battery Power", "sensor.inverter_battery_power", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "bp", false},
    {"PV1V", "PV1 Voltage", "sensor.inverter_pv1_voltage", "voltage", "measurement", "V", nullptr, 1, STAT_TOPIC_DATA, "pv1v", false},
    {"PV1C", "PV1 Current", "sensor.inverter_pv1_current", "current", "measurement", "A", nullptr, -1, STAT_TOPIC_DATA, "pv1c", false},
    {"PV1P", "PV1 Power", "sensor.inverter_pv1_power", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "pv1p", false},
    {"LP", "Power Consumption", "sensor.inverter_power_consumption", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "lw", false},
    {"LPP", "Load Power Percentage", "sensor.inverter_load_power_pct", nullptr, "measurement", "%", "mdi:gauge", -1, STAT_TOPIC_DATA, "lp", false},
    {"MPPT", "MPPT Heatsink Temp", "sensor.inverter_mppt_heatsink_temp", "temperature", "measurement", "°C", nullptr, -1, STAT_TOPIC_DATA, "mt", false},
    {"IHT", "Inverter Heatsink Temp", "sensor.inverter_heatsink_temp", "temperature", "measurement", "°C", nullptr, -1, STAT_TOPIC_DATA, "it", false},
    {"DCDC", "DC/DC Heatsink Temp", "sensor.inverter_dcdc_heatsink_temp", "temperature", "measurement", "°C", nullptr, -1, STAT_TOPIC_DATA, "dt", false},
    {"AIV", "AC Input Voltage", "sensor.inverter_ac_input_voltage", "voltage", "measurement", "V", nullptr, -1, STAT_TOPIC_DATA, "aiv", false},
    {"AIC", "AC Input Current", "sensor.inverter_ac_input_current", "current", "measurement", "A", nullptr, -1, STAT_TOPIC_DATA, "aic", false},
    {"AIF", "AC Input Frequency", "sensor.inverter_ac_input_frequency", "frequency", "measurement", "Hz", nullptr, 1, STAT_TOPIC_DATA, "aif", false},
    {"AIP", "AC Input Power", "sensor.inverter_ac_input_power", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "aip", false},
    {"ACP", "AC Charging Power", "sensor.inverter_ac_charging_power", "power", "measurement", "W", nullptr, -1, STAT_TOPIC_DATA, "acp", false},
    {"PV1G", "PV1 Generation", "sensor.inverter_pv1_generation", "energy", "total_increasing", "kWh", nullptr, -1, STAT_TOPIC_DATA, "pg", false},
    {"BCE", "Battery Charge Energy", "sensor.inverter_battery_charge_energy", "energy", "total_increasing", "kWh", nullptr, -1, STAT_TOPIC_DATA, "bce", false},
    {"DC", "Discharge Capacity", "sensor.inverter_discharge_capacity", "energy", "total_increasing", "kWh", nullptr, -1, STAT_TOPIC_DATA, "dc", false},
    {"LE", "Loads Electricity", "sensor.inverter_loads_electricity", "energy", "total_increasing", "kWh", nullptr, -1, STAT_TOPIC_DATA, "le", false},

    {"IP", "IP Address", nullptr, nullptr, nullptr, nullptr, "mdi:ip-network", -1, STAT_TOPIC_DIAG_DATA, "ip", true},
    {"HEAP", "Free Heap", "sensor.inverter_free_heap", nullptr, "measurement", "B", "mdi:memory", -1, STAT_TOPIC_DIAG_DATA, "heap", true},
    {"MBLK", "Max Free Block", "sensor.inverter_max_free_block", nullptr, "measurement", "B", "mdi:memory", -1, STAT_TOPIC_DIAG_DATA, "mblk", true},
    {"FRAG", "Heap Fragmentation", "sensor.inverter_heap_fragmentation", nullptr, "measurement", "%", "mdi:chart-sankey", -1, STAT_TOPIC_DIAG_DATA, "frag", true},
    {"RSSI", "WiFi RSSI", "sensor.inverter_wifi_rssi", "signal_strength", "measurement", "dBm", "mdi:wifi", -1, STAT_TOPIC_DIAG_DATA, "rssi", true},
    {"WRC", "WiFi Reconnect Count", "sensor.inverter_wifi_reconnect_count", nullptr, "total_increasing", nullptr, "mdi:wifi-refresh", -1, STAT_TOPIC_DIAG_DATA, "wrc", true},
    {"MRC", "MQTT Reconnect Count", "sensor.inverter_mqtt_reconnect_count", nullptr, "total_increasing", nullptr, "mdi:wifi-refresh", -1, STAT_TOPIC_DIAG_DATA, "mqrc", true},
    {"MS", "Modbus Status", "sensor.inverter_modbus_status", nullptr, nullptr, nullptr, "mdi:swap-horizontal", -1, STAT_TOPIC_DIAG_DATA, "ms", true},
    {"MOK", "Modbus OK Count", "sensor.inverter_modbus_ok_count", nullptr, "total_increasing", nullptr, "mdi:counter", -1, STAT_TOPIC_DIAG_DATA, "mok", true},
    {"MER", "Modbus Error Count", "sensor.inverter_modbus_error_count", nullptr, "total_increasing", nullptr, "mdi:counter", -1, STAT_TOPIC_DIAG_DATA, "mer", true},
    {"RST", "Reset Reason", "sensor.inverter_reset_reason", nullptr, nullptr, nullptr, "mdi:restart-alert", -1, STAT_TOPIC_DIAG_DATA, "rr", true},
};

static bool discoverEntity(PubSubClient &client, const SensorEntity &e)
{
  jsonDoc.clear();
  prepareEntity(e.suffix);
  jsonDoc["name"] = e.name;
  jsonDoc["uniq_id"] = uidBuf;
  if (e.entityId)
    jsonDoc["default_entity_id"] = e.entityId;
  if (e.devCla)
    jsonDoc["dev_cla"] = e.devCla;
  if (e.statCla)
    jsonDoc["stat_cla"] = e.statCla;
  if (e.unit)
    jsonDoc["unit_of_meas"] = e.unit;
  if (e.icon)
    jsonDoc["icon"] = e.icon;
  if (e.precision >= 0)
    jsonDoc["suggested_display_precision"] = e.precision;
  if (e.isDiag)
    jsonDoc["ent_cat"] = "diagnostic";
  jsonDoc["stat_t"] = e.statTopic;
  jsonDoc["avty_t"] = MQTT_AVAIL_TOPIC;
  // Inverter data is skipped when Modbus fails; let HA show it as unavailable
  // instead of displaying the last retained value indefinitely.
  if (strcmp(e.statTopic, STAT_TOPIC_DATA) == 0)
    jsonDoc["exp_aft"] = SENSOR_EXPIRE_AFTER_S;
  char valTpl[48];
  snprintf(valTpl, sizeof(valTpl), "{{ value_json.%s }}", e.jsonKey);
  jsonDoc["val_tpl"] = valTpl;
  addDeviceBlock(jsonDoc);
  return publishDiscoveryMsg(client, e.name);
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

// Returns true only if every discovery message was published.
static bool startDiscovery(PubSubClient &client)
{
  DEBUG_PRINTLN("Starting HA MQTT discovery for inverter...");
  bool ok = true;

  for (const SensorEntity &e : SENSOR_ENTITIES)
    ok &= discoverEntity(client, e);

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
  ok &= publishDiscoveryMsg(client, "Restart Device");

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
  ok &= publishDiscoveryMsg(client, "Device Availability");

  return ok;
}

static void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  DEBUG_PRINTF("MQTT msg: %s (%u bytes)\n", topic, length);
  if (strcmp(topic, HA_STATUS_TOPIC) == 0)
  {
    // HA's retained "online" arrives right after we subscribe; discovery is
    // already handled on connect, so only react to a later HA restart.
    if (length == 6 && memcmp(payload, "online", 6) == 0 && (millis() - mqttConnectedAtMs) >= MQTT_CMD_GRACE_MS)
    {
      DEBUG_PRINTLN("Home Assistant came online — re-sending discovery.");
      discoveryDone = false;
      discoveryAttempted = false;
    }
    return;
  }
  if (strcmp(topic, CMND_TOPIC_RESTART) == 0)
  {
    if (length != 1 || payload[0] != '1')
    {
      DEBUG_PRINTLN("Restart command ignored: unexpected payload.");
      return;
    }
    // Messages arriving right after subscribing are likely retained; acting on
    // them would cause a reboot loop.
    if ((millis() - mqttConnectedAtMs) < MQTT_CMD_GRACE_MS)
    {
      DEBUG_PRINTLN("Restart command ignored: received during post-connect grace period (likely retained).");
      return;
    }
    DEBUG_PRINTLN("Restart command received via MQTT!");
    restartRequested = true;
  }
}

static bool mqttReconnect()
{
  if (!mqttTlsConfigured)
  {
    DEBUG_PRINTLN("MQTT disabled: no CA certificate and MQTT_ALLOW_INSECURE not set.");
    return false;
  }
  for (int i = 0; i < MQTT_MAX_RETRIES; i++)
  {
    DEBUG_PRINT("MQTT connecting... ");
    // Each attempt can block for up to ~21 s (see TLS_*_TIMEOUT_S), so feed the WDT per attempt
    esp_task_wdt_reset();
    if (mqttClient.connect(devUniqueID, MQTT_USER, MQTT_PASSWORD, MQTT_AVAIL_TOPIC, 1, true, "offline"))
    {
      DEBUG_PRINTLN("connected!");
      mqttConnectedAtMs = millis();
      // Count successful reconnects only (the first connection after boot isn't one)
      if (mqttHadConnection)
        mqttReconnectCount++;
      mqttHadConnection = true;
      mqttClient.publish(MQTT_AVAIL_TOPIC, "online", true);
      mqttClient.subscribe(CMND_TOPIC_RESTART);
      mqttClient.subscribe(HA_STATUS_TOPIC);
      return true;
    }
    DEBUG_PRINTF("failed, rc=%d\n", mqttClient.state());
    vTaskDelay(pdMS_TO_TICKS(1000));
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

static void initModbus()
{
  modbusSerial.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  delay(100);
  charger.begin(MODBUS_SLAVE_ADDR, modbusSerial);
  charger.preTransmission(modbusPreTransmission);
  charger.postTransmission(modbusPostTransmission);
  // Called while waiting for a response; runs in mainTask, which is WDT-registered
  charger.idle([]()
               { esp_task_wdt_reset(); });
}

static void reinitModbus()
{
  DEBUG_PRINTLN("Reinitializing Modbus after consecutive failures...");
  modbusSerial.end();
  delay(200);
  initModbus();
  modbusConsecutiveErrors = 0;
  DEBUG_PRINTLN("Modbus re-initialized.");
}

static void recordModbusFailure(uint8_t blockNum, uint8_t err)
{
  modbusErrorCount++;
  updateModbusErrorCode(err);
  modbusConsecutiveErrors++;
  DEBUG_PRINTF("Modbus block %u failed: %s (consecutive: %u)\n", blockNum, lastModbusErrorCode, modbusConsecutiveErrors);
  if (modbusConsecutiveErrors >= MODBUS_REINIT_THRESHOLD)
    reinitModbus();
}

// True if a lifetime counter went down. A wrap of the 16-bit register
// (near 65535 -> small value) is allowed; HA treats it as a meter reset.
static bool energyCounterDropped(uint16_t prev, uint16_t now)
{
  if (now >= prev)
    return false;
  bool wrapped = (prev > 65000) && (now < 1000);
  return !wrapped;
}

static bool readInverterRegisters()
{
  uint8_t lastErr = ModbusMaster::ku8MBSuccess;

  if (!readHoldingBlockWithRetry(0x0001, 0x0020, lastErr))
  {
    recordModbusFailure(1, lastErr);
    return false;
  }

  invAcOutputVoltage = charger.getResponseBuffer(IDX(REG_AC_OUTPUT_VOLTAGE, 0x0001));
  invBusVoltage = charger.getResponseBuffer(IDX(REG_BUS_VOLTAGE, 0x0001)) / 10.0f;
  invAcInputVoltage = charger.getResponseBuffer(IDX(REG_AC_INPUT_VOLTAGE, 0x0001));
  invAcInputCurrent = charger.getResponseBuffer(IDX(REG_AC_INPUT_CURRENT, 0x0001)) / 10.0f;
  invBattVoltage = charger.getResponseBuffer(IDX(REG_BATT_VOLTAGE, 0x0001)) / 10.0f;
  invBusCurrent = (int16_t)charger.getResponseBuffer(IDX(REG_BUS_CURRENT, 0x0001)) / 100.0f;
  invMpptHeatsinkTemp = (int16_t)charger.getResponseBuffer(IDX(REG_MPPT_HEATSINK_TEMP, 0x0001));
  invPv1Current = charger.getResponseBuffer(IDX(REG_PV1_CURRENT, 0x0001)) / 10.0f;
  invPv1Voltage = charger.getResponseBuffer(IDX(REG_PV1_VOLTAGE, 0x0001)) / 10.0f;
  invDcdcHeatsinkTemp = (int16_t)charger.getResponseBuffer(IDX(REG_DCDC_HEATSINK_TEMP, 0x0001));
  invInvHeatsinkTemp = (int16_t)charger.getResponseBuffer(IDX(REG_INV_HEATSINK_TEMP, 0x0001));
  invBattCurrent = (int16_t)charger.getResponseBuffer(IDX(REG_BATT_CURRENT, 0x0001)) / 10.0f;
  invAcInputFreq = charger.getResponseBuffer(IDX(REG_AC_INPUT_FREQ, 0x0001)) / 10.0f;
  invLoadPowerPct = charger.getResponseBuffer(IDX(REG_LOAD_POWER_PCT, 0x0001));
  invLoadPowerWatts = charger.getResponseBuffer(IDX(REG_LOAD_POWER_WATTS, 0x0001));

  delay(30);

  if (!readHoldingBlockWithRetry(REG_BATT_SOC, (REG_AC_INPUT_POWER - REG_BATT_SOC + 1), lastErr))
  {
    recordModbusFailure(2, lastErr);
    return false;
  }

  // Lifetime energy counters (1 kWh units). Validate before accepting: a
  // spurious lower value would look like a meter reset to HA and get
  // double-counted in the Energy dashboard.
  uint16_t pv1Gen = charger.getResponseBuffer(IDX(REG_PV1_GENERATION, REG_BATT_SOC));
  uint16_t battChgE = charger.getResponseBuffer(IDX(REG_BATT_CHARGE_ENERGY, REG_BATT_SOC));
  uint16_t dischCap = charger.getResponseBuffer(IDX(REG_DISCHARGE_CAPACITY, REG_BATT_SOC));
  uint16_t loadsE = charger.getResponseBuffer(IDX(REG_LOADS_ELECTRICITY, REG_BATT_SOC));
  if (haveEnergyBaseline &&
      (energyCounterDropped(invPv1Generation, pv1Gen) ||
       energyCounterDropped(invBattChargeEnergy, battChgE) ||
       energyCounterDropped(invDischargeCapacity, dischCap) ||
       energyCounterDropped(invLoadsElectricity, loadsE)))
  {
    energyDropCount++;
    DEBUG_PRINTF("Energy counter decreased (PV %u->%u, BattChg %u->%u, Disch %u->%u, Loads %u->%u), strike %u/%u\n",
                 invPv1Generation, pv1Gen, invBattChargeEnergy, battChgE,
                 invDischargeCapacity, dischCap, invLoadsElectricity, loadsE,
                 energyDropCount, ENERGY_DROP_ACCEPT_AFTER);
    if (energyDropCount < ENERGY_DROP_ACCEPT_AFTER)
    {
      modbusErrorCount++;
      strncpy(lastModbusErrorCode, "Implausible data", sizeof(lastModbusErrorCode) - 1);
      return false;
    }
    DEBUG_PRINTLN("Energy counter drop persisted — accepting new values.");
  }
  energyDropCount = 0;
  haveEnergyBaseline = true;
  invPv1Generation = pv1Gen;
  invBattChargeEnergy = battChgE;
  invDischargeCapacity = dischCap;
  invLoadsElectricity = loadsE;

  invBattSoc = charger.getResponseBuffer(IDX(REG_BATT_SOC, REG_BATT_SOC));
  invPv1Power = charger.getResponseBuffer(IDX(REG_PV1_POWER, REG_BATT_SOC));
  invAcChargePower = charger.getResponseBuffer(IDX(REG_AC_CHARGE_POWER, REG_BATT_SOC));
  invBattPower = charger.getResponseBuffer(IDX(REG_BATT_POWER, REG_BATT_SOC));
  invAcInputPower = charger.getResponseBuffer(IDX(REG_AC_INPUT_POWER, REG_BATT_SOC));

  modbusOkCount++;
  updateModbusErrorCode(ModbusMaster::ku8MBSuccess);
  modbusConsecutiveErrors = 0;

  DEBUG_PRINTF("Inverter: Batt=%.1fV/%.1fA/%uW (%u%%) PV=%.1fV/%.1fA/%uW AC=%.1fV Load=%uW(%u%%) Temps: MPPT=%dC INV=%dC DCDC=%dC\n",
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
  lastModbusReadOk = readInverterRegisters();

  bool ok = true;

  // Only publish sensor data from a fresh, complete read. Publishing defaults or
  // stale values would make total_increasing sensors look like meter resets in HA.
  if (!lastModbusReadOk)
  {
    DEBUG_PRINTLN("Modbus read failed — skipping sensor data publish.");
  }
  else
  {
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
    ok &= publishJsonDoc(mqttClient, STAT_TOPIC_DATA, true);
  }

  jsonDoc.clear();
  jsonDoc["ip"] = WiFi.localIP().toString();
  jsonDoc["rssi"] = WiFi.RSSI();
  // Use the same capability set for both values so the fragmentation ratio is consistent
  size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t mblk = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  jsonDoc["heap"] = (unsigned long)freeHeap;
  jsonDoc["mblk"] = (unsigned long)mblk;
  jsonDoc["frag"] = (freeHeap == 0) ? 100 : (int)(100.0f * (1.0f - ((float)mblk / (float)freeHeap)));
  jsonDoc["wrc"] = wifiReconnectCount;
  jsonDoc["mqrc"] = mqttReconnectCount;
  jsonDoc["ms"] = lastModbusErrorCode;
  jsonDoc["mok"] = modbusOkCount;
  jsonDoc["mer"] = modbusErrorCount;
  jsonDoc["rr"] = bootResetReason;
  ok &= publishJsonDoc(mqttClient, STAT_TOPIC_DIAG_DATA, true);

  DEBUG_PRINTLN(ok ? "Sensor data published OK." : "Sensor data publish: one or more FAILED!");
}

void setup()
{
  DEBUG_BEGIN(115200);
#if ENABLE_SERIAL_DEBUG
  // Ensure ESP core debug output is routed to Serial
  Serial.setDebugOutput(true);
#endif
  // Initialize heartbeat LED
  pinMode(HEARTBEAT_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_PIN, HEARTBEAT_OFF_LEVEL);
  delay(1000);
  DEBUG_PRINTLN("\n\nInverter Gateway (ESP32-C3) starting...");

  esp_reset_reason_t r = esp_reset_reason();
  const char *reason = "Unknown";
  switch (r)
  {
  case ESP_RST_POWERON:
    reason = "Power-on";
    break;
  case ESP_RST_EXT:
    reason = "External pin";
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
  case ESP_RST_SDIO:
    reason = "SDIO";
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
  // Don't wait here: mainTask tracks the connection and handles recovery
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  DEBUG_PRINTF("Connecting to WiFi: %s\n", WIFI_SSID);

  WiFi.macAddress(macAddr);
  buildDeviceUniqueID();

  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  uint16_t mqttKeepAliveSeconds = mqttKeepAliveSecondsFromPublishInterval();
  mqttClient.setKeepAlive(mqttKeepAliveSeconds);
  DEBUG_PRINTF("MQTT keep alive: %u s\n", mqttKeepAliveSeconds);
  // Defaults are 30 s connect + 120 s handshake, longer than the task WDT
  espClient.setTimeout(TLS_CONNECT_TIMEOUT_S);
  espClient.setHandshakeTimeout(TLS_HANDSHAKE_TIMEOUT_S);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
  if (strstr(MQTT_CA_CERT, "BEGIN CERTIFICATE") != nullptr)
  {
    espClient.setCACert(MQTT_CA_CERT);
    mqttTlsConfigured = true;
    DEBUG_PRINTLN("MQTT TLS: CA certificate validation enabled.");
  }
  else
  {
#ifdef MQTT_ALLOW_INSECURE
    espClient.setInsecure();
    mqttTlsConfigured = true;
    DEBUG_PRINTLN("MQTT TLS: insecure mode (MQTT_ALLOW_INSECURE set, broker not verified).");
#else
    DEBUG_PRINTLN("MQTT TLS: no CA certificate configured and MQTT_ALLOW_INSECURE not set — MQTT disabled.");
#endif
  }
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  if (MODBUS_DE_RE_PIN >= 0)
  {
    pinMode(MODBUS_DE_RE_PIN, OUTPUT);
    digitalWrite(MODBUS_DE_RE_PIN, LOW);
  }
  initModbus();

  // Increase WDT timeout to allow longer Modbus/TLS operations
  esp_task_wdt_init(30, true);
  DEBUG_PRINTLN("Task WDT enabled.");

  // Each task registers itself with the WDT and feeds it; setup()'s task is
  // not registered, so it must not call esp_task_wdt_reset().
  // Create FreeRTOS tasks: heartbeat and main
  xTaskCreate(heartbeatTask, "heartbeat", 2048, NULL, 2, NULL);
  xTaskCreate(mainTask, "mainTask", 8192, NULL, 1, NULL);
}

// Main work is done in FreeRTOS tasks; delete the Arduino loop task so it
// doesn't spin and compete with mainTask for the single core.
void loop()
{
  vTaskDelete(NULL);
}

// Heartbeat task: blinks the status LED pattern for the current ledStatus
static void heartbeatTask(void *pvParameters)
{
  (void)pvParameters;
  // Register this task with the task watchdog
  esp_task_wdt_add(NULL);
  for (uint8_t tick = 0;; tick = (tick + 1) % LED_CYCLE_TICKS)
  {
    esp_task_wdt_reset();
    LedStatus status = ledStatus;
    bool on;
    if (status == LED_STATUS_WIFI_DOWN)
    {
      // Fast blink: 200 ms on / 200 ms off
      on = ((tick / 2) % 2) == 0;
    }
    else
    {
      // N short blinks (100 ms on, 200 ms off) at the start of each cycle, then a pause
      uint8_t blinks = (status == LED_STATUS_OK) ? 1 : (status == LED_STATUS_MQTT_DOWN) ? 2 : 3;
      on = (tick % 3 == 0) && (tick / 3 < blinks);
    }
    digitalWrite(HEARTBEAT_PIN, on ? HEARTBEAT_ON_LEVEL : HEARTBEAT_OFF_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(LED_TICK_MS));
  }
}

// Main task: contains previous loop() logic, runs as FreeRTOS task
static void mainTask(void *pvParameters)
{
  (void)pvParameters;
  // Register this task with the task watchdog
  esp_task_wdt_add(NULL);
  unsigned long wifiDownSinceMs = 0;
  unsigned long lastWifiRetryMs = 0;
  // Previous WiFi state, used to detect connect/disconnect transitions.
  // The first connection after boot is not counted as a reconnect
  // (see hadInitialWiFiConnection).
  bool prevWifiConnected = false;
  for (;;)
  {
    esp_task_wdt_reset();

    bool nowWifiConnected = (WiFi.status() == WL_CONNECTED);

    if (!nowWifiConnected)
    {
      ledStatus = LED_STATUS_WIFI_DOWN;
      unsigned long now = millis();
      if (prevWifiConnected || wifiDownSinceMs == 0)
      {
        DEBUG_PRINTLN(hadInitialWiFiConnection ? "WiFi disconnected!" : "Waiting for WiFi...");
        wifiDownSinceMs = now;
        lastWifiRetryMs = now;
        prevWifiConnected = false;
      }

      // Auto-reconnect handles most drops, but not every disconnect reason
      // (e.g. AUTH_FAIL), so occasionally force a fresh connection attempt.
      // Spaced well apart so we don't abort a handshake that is in progress.
      if ((now - lastWifiRetryMs) >= WIFI_RETRY_INTERVAL_MS)
      {
        lastWifiRetryMs = now;
        DEBUG_PRINTLN("WiFi still down: forcing disconnect + begin");
        WiFi.disconnect();
        vTaskDelay(pdMS_TO_TICKS(100));
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      }

      if ((now - wifiDownSinceMs) >= WIFI_RESET_AFTER_MS)
      {
        DEBUG_PRINTF("WiFi down for %lu s — restarting...\n", WIFI_RESET_AFTER_MS / 1000UL);
        delay(200);
        ESP.restart();
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (nowWifiConnected && !prevWifiConnected)
    {
      DEBUG_PRINTF("WiFi %s! IP: %s\n", hadInitialWiFiConnection ? "reconnected" : "connected", WiFi.localIP().toString().c_str());
      if (hadInitialWiFiConnection)
      {
        wifiReconnectCount++;
      }
      else
      {
        hadInitialWiFiConnection = true;
      }
      wifiDownSinceMs = 0;
    }

    // Update previous-state snapshot for next iteration
    prevWifiConnected = nowWifiConnected;

    if (!mqttClient.connected())
    {
      ledStatus = LED_STATUS_MQTT_DOWN;
      if (!mqttReconnect())
      {
        DEBUG_PRINTLN("MQTT connect failed, retrying next loop.");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }

    mqttClient.loop();

    // Discovery runs after boot, after HA restarts (see mqttCallback), and is
    // retried every DISCOVERY_RETRY_MS if any message failed.
    if (!discoveryDone && (!discoveryAttempted || (millis() - lastDiscoveryAttemptMs) >= DISCOVERY_RETRY_MS))
    {
      discoveryAttempted = true;
      lastDiscoveryAttemptMs = millis();
      discoveryDone = startDiscovery(mqttClient);
      // Give HA a moment to subscribe to the new entities before sending state
      vTaskDelay(pdMS_TO_TICKS(1000));
      publishSensorData();
      lastPublishMs = millis();
    }

    ledStatus = lastModbusReadOk ? LED_STATUS_OK : LED_STATUS_MODBUS_FAIL;

    if (restartRequested)
    {
      // Announce offline and close the session cleanly so the message is
      // flushed over TLS before rebooting.
      mqttClient.publish(MQTT_AVAIL_TOPIC, "offline", true);
      mqttClient.disconnect();
      espClient.stop();
      vTaskDelay(pdMS_TO_TICKS(200));
      ESP.restart();
    }

    unsigned long now = millis();
    if ((now - lastPublishMs) >= PUBLISH_INTERVAL_MS)
    {
      lastPublishMs = now;
      publishSensorData();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
