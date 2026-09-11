/*
  Dual-mode RFID Boom Barrier Controller V4
  ------------------------------------------------
  Product: RFID Gate Controller V4 (4 MB flash recommended)

  FLOW:
  1. Loop detector sees vehicle.
  2. The controller triggers the RFID reader over its internal serial link.
  3. Standalone mode checks the EPC against LittleFS. Plate Program mode sends
     it to Plate Program and waits for an authorized/denied decision.
  4. Authorized EPC -> pulse OPEN relay.
  5. Traffic signal goes GREEN.
  6. IR beam detects vehicle under barrier.
  7. After IR beam is clear for IR_CLEAR_DELAY_MS:
       traffic signal goes RED and CLOSE relay is pulsed.
  8. System waits for loop + IR to clear, then resets.

  WEB ADMIN:
  - The controller creates its own Wi-Fi access point.
  - Connect to Wi-Fi and open http://192.168.4.1
  - Select Standalone or Plate Program mode from the System Mode page.
  - Standalone: enroll/search/edit/delete local LittleFS members.
  - Plate Program: all registration and enrollment is performed centrally;
    this controller stores only mode/network settings, never server member data.

  IMPORTANT:
  - Loop detector and IR inputs must be DRY CONTACT / 3.3 V SAFE.
    Never feed 12 V / 24 V directly into a controller signal pin.
  - OPEN/CLOSE outputs should drive relay modules whose dry contacts
    mimic the boom barrier's push buttons.
  - D8/GPIO15 is used for traffic signal:
       LOW = RED, HIGH = GREEN.
    The external light interface MUST NOT pull GPIO15 HIGH at boot.
  - In Plate Program mode, D3/GPIO0 blinks while station Wi-Fi is
    disconnected. After Wi-Fi connects, it resumes normal RFID indication.
  - If the RFID reader uses true RS232, use a MAX3232 between it
    and the controller. Do not connect RS232 voltage directly to a signal pin.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <LittleFS.h>
#include <SoftwareSerial.h>
#include <time.h>

// Hardware TX/RX are dedicated status outputs in this build. Debug output is
// intentionally disabled so TX remains a stable indicator signal.
struct DisabledSerialDebug {
  void begin(unsigned long) {}
  template <typename T> size_t print(const T&) { return 0; }
  template <typename T> size_t print(const T&, int) { return 0; }
  template <typename T> size_t println(const T&) { return 0; }
  size_t println() { return 0; }
};
DisabledSerialDebug disabledSerialDebug;
#define Serial disabledSerialDebug

// ============================================================
// PIN ASSIGNMENTS
// ============================================================

static const uint8_t LOOP_PIN          = 5;   // D1 / GPIO5
static const uint8_t IR_PIN            = 4;   // D2 / GPIO4
static const uint8_t RFID_RX_PIN       = 14;  // D5 / GPIO14 (ESP RX)
static const uint8_t RFID_TX_PIN       = 12;  // D6 / GPIO12 (ESP TX)
static const uint8_t OPEN_RELAY_PIN    = 13;  // D7 / GPIO13
static const uint8_t CLOSE_RELAY_PIN   = 16;  // D0 / GPIO16
static const uint8_t RFID_STATUS_PIN   = 0;   // D3 / GPIO0
static const uint8_t RED_STATUS_PIN    = 2;   // D4 / GPIO2
static const uint8_t GREEN_STATUS_PIN  = 3;   // RX / GPIO3
static const uint8_t VEHICLE_STATUS_PIN = 1;  // TX / GPIO1
static const uint8_t TRAFFIC_LIGHT_PIN = 15;  // D8 / GPIO15: LOW=RED, HIGH=GREEN
// ============================================================
// ELECTRICAL LOGIC
// Change these if your modules use opposite polarity.
// ============================================================

// Loop detector: recommended wiring is relay contact to GND + INPUT_PULLUP.
static const uint8_t LOOP_ACTIVE_LEVEL = LOW;

// IR beam: recommended wiring is relay contact to GND + INPUT_PULLUP.
// LOW means beam is BLOCKED.
static const uint8_t IR_BLOCKED_LEVEL = LOW;

// Most relay boards are active LOW. Change to HIGH if yours is active HIGH.
static const uint8_t RELAY_ACTIVE_LEVEL   = LOW;
static const uint8_t RELAY_INACTIVE_LEVEL = HIGH;

// User-requested outdoor traffic-light interface:
// LOW = RED / STOP
// HIGH = GREEN / GO
static const uint8_t LIGHT_RED_LEVEL   = LOW;
static const uint8_t LIGHT_GREEN_LEVEL = HIGH;

// ============================================================
// WIFI ACCESS POINT / WEB ADMIN
// CHANGE THESE PASSWORDS BEFORE INSTALLING.
// ============================================================

const char* AP_SSID     = "RFID-GATE";
const char* AP_PASSWORD = "GateRFID123";  // minimum 8 characters

const char* WEB_USERNAME = "admin";
const char* WEB_PASSWORD = "change-me-now";

// ============================================================
// OPERATING MODE
// ============================================================

enum OperatingMode : uint8_t {
  MODE_STANDALONE = 0,
  MODE_PLATE_PROGRAM = 1
};

// The selected mode and network settings are stored in internal flash. RFID member
// records remain local only in standalone mode. In Plate Program mode all RFID
// registration, expiry, and authorization decisions belong to Plate Program.
const char* MODE_CONFIG_FILE = "/mode.cfg";
OperatingMode operatingMode = MODE_STANDALONE;
String programWifiSsid = "";
String programWifiPassword = "";
String plateProgramBaseUrl = "http://192.168.1.3:8080";
String controllerId = "";
String controllerKey = "";
String plateProgramStatus = "Standalone mode";
bool plateProgramReachable = false;
unsigned long lastProgramReconnectAt = 0;
unsigned long lastProgramHeartbeatAt = 0;
static const unsigned long PROGRAM_CONNECT_TIMEOUT_MS = 15000;
static const unsigned long PROGRAM_HTTP_TIMEOUT_MS = 10000;
static const unsigned long PROGRAM_RECONNECT_MS = 10000;
static const unsigned long PROGRAM_HEARTBEAT_MS = 5000;
static const unsigned long WIFI_STATUS_BLINK_MS = 500;

// ============================================================
// RFID SERIAL SETTINGS
// ============================================================

// Change to match the reader.
static const uint32_t RFID_BAUD = 9600;

// Debug: print each RFID response as ONE readable packet instead of one line per byte.
// A packet is considered complete after this many milliseconds with no new bytes.
static const bool PRINT_RFID_RAW_BYTES = true;
static const unsigned long RFID_PACKET_IDLE_MS = 15;
static const size_t RFID_DEBUG_PACKET_MAX = 128;

SoftwareSerial rfidSerial(RFID_RX_PIN, RFID_TX_PIN);

// IMPORTANT:
// I could not verify the exact RD903M inventory/trigger command bytes from
// a trustworthy protocol manual.
//
// Put the reader's REAL trigger/inventory command below.
// Leave RFID_TRIGGER_COMMAND_LENGTH = 0 if your reader automatically
// outputs EPCs / is already configured for triggered auto-output.
//
// Example format ONLY:
// const uint8_t RFID_TRIGGER_COMMAND[] = { 0xAA, 0xBB, 0xCC };
// const size_t RFID_TRIGGER_COMMAND_LENGTH = 3;

const uint8_t RFID_TRIGGER_COMMAND[] = { 0x06, 0x00, 0x01, 0x02, 0x00, 0x7C, 0x62 };
const size_t RFID_TRIGGER_COMMAND_LENGTH = 7;

// ============================================================
// TIMING
// ============================================================

static const unsigned long RELAY_PULSE_MS       = 500;
static const unsigned long RFID_RETRY_MS        = 1500;
static const unsigned long GREEN_DELAY_MS       = 1200;
static const unsigned long IR_CLEAR_DELAY_MS    = 1000;
static const unsigned long DUPLICATE_IGNORE_MS  = 1000;

// ============================================================
// FILESYSTEM / MEMBER DATABASE
// ============================================================

// Full member records are stored as fixed-size binary rows in internal flash.
// This avoids rewriting the whole database every time last_access_at changes.
const char* MEMBER_DB_FILE       = "/members.dat";
const char* LEGACY_RFID_DB_FILE = "/rfids.jsonl";

static const size_t OWNER_NAME_MAX    = 48;
static const size_t VEHICLE_MODEL_MAX = 32;
static const size_t VEHICLE_BRAND_MAX = 32;
static const size_t PLATE_NUMBER_MAX  = 16;
static const size_t RFID_VALUE_MAX    = 64;
static const size_t EXPIRY_MAX        = 10;  // YYYY-MM-DD
static const size_t TIMESTAMP_MAX     = 25;  // YYYY-MM-DDTHH:MM:SS+08:00

#pragma pack(push, 1)
struct MemberRecord {
  uint8_t active;
  char ownerName[OWNER_NAME_MAX + 1];
  char vehicleModel[VEHICLE_MODEL_MAX + 1];
  char vehicleBrand[VEHICLE_BRAND_MAX + 1];
  char plateNumber[PLATE_NUMBER_MAX + 1];
  char rfid[RFID_VALUE_MAX + 1];
  char expiry[EXPIRY_MAX + 1];
  char createdAt[TIMESTAMP_MAX + 1];
  char updatedAt[TIMESTAMP_MAX + 1];
  char lastAccessAt[TIMESTAMP_MAX + 1];
};

#pragma pack(pop)

struct PlateProgramDecision {
  bool requestCompleted;
  bool authorized;
  String owner;
  String plate;
  String vehicleType;
  String make;
  String model;
  String reason;
};

// ============================================================
// WEB SERVER
// ============================================================

ESP8266WebServer server(80);

// ============================================================
// GATE STATE
// ============================================================

enum GateState {
  WAITING_FOR_VEHICLE,
  WAITING_FOR_RFID,
  OPENING_DELAY,
  WAITING_FOR_IR_BLOCK,
  WAITING_FOR_IR_CLEAR,
  WAITING_FOR_AREA_CLEAR
};

GateState gateState = WAITING_FOR_VEHICLE;

unsigned long stateStartedAt = 0;
unsigned long lastRFIDTriggerAt = 0;
unsigned long lastAuthorizationPollAt = 0;
unsigned long lastAuthorizationRetryAt = 0;
unsigned long irClearStartedAt = 0;

// ============================================================
// RELAY PULSE STATE
// ============================================================

int activeRelayPin = -1;
unsigned long relayPulseStartedAt = 0;

// ============================================================
// RFID INPUT / ENROLLMENT STATE
// ============================================================

String rfidLineBuffer;

// Raw packet debug buffer. Used only for clean Serial Monitor output.
uint8_t rfidDebugPacket[RFID_DEBUG_PACKET_MAX];
size_t rfidDebugPacketLength = 0;
unsigned long rfidDebugLastByteAt = 0;

String lastRFID = "";
String lastRFIDResult = "None";
unsigned long lastRFIDSeenAt = 0;
bool rfidDetected = false;

// Live vehicle information shown prominently on the dashboard while an
// authorized vehicle is moving through the gate cycle.
String activeVehiclePlate = "";
String activeVehicleOwner = "";
String activeVehicleBrand = "";
String activeVehicleModel = "";
String activeVehicleRFID = "";
String activeAttemptUid = "";

bool enrollMode = false;
String enrollOwnerName = "";
String enrollVehicleModel = "";
String enrollVehicleBrand = "";
String enrollPlateNumber = "";
String enrollExpiry = "";
String enrollmentStatus = "Off";

size_t authorizedCount = 0;

// The controller has no battery-backed real-time clock.
// The dashboard automatically synchronizes the controller from the browser.
bool clockSynced = false;
uint32_t clockBaseEpochUtc = 0;
unsigned long clockBaseMillis = 0;
int16_t clockTimezoneOffsetMinutes = 480; // Philippines default: UTC+8

// ============================================================
// HELPERS
// ============================================================

bool vehicleDetected() {
  return digitalRead(LOOP_PIN) == LOOP_ACTIVE_LEVEL;
}

bool irBlocked() {
  return digitalRead(IR_PIN) == IR_BLOCKED_LEVEL;
}

void signalStop() {
  digitalWrite(TRAFFIC_LIGHT_PIN, LIGHT_RED_LEVEL);
  digitalWrite(GREEN_STATUS_PIN, LOW);
  digitalWrite(RED_STATUS_PIN, HIGH);
}

void signalGo() {
  digitalWrite(TRAFFIC_LIGHT_PIN, LIGHT_GREEN_LEVEL);
  digitalWrite(GREEN_STATUS_PIN, HIGH);
  digitalWrite(RED_STATUS_PIN, LOW);
}

void updateStatusOutputs() {
  digitalWrite(VEHICLE_STATUS_PIN, vehicleDetected() ? HIGH : LOW);

  // In Plate Program mode, make a missing Wi-Fi connection immediately
  // visible without blocking RFID or gate processing. Once station Wi-Fi is
  // connected, D3 returns to its normal RFID-detected indication.
  if (operatingMode == MODE_PLATE_PROGRAM && WiFi.status() != WL_CONNECTED) {
    bool blinkOn = ((millis() / WIFI_STATUS_BLINK_MS) % 2U) != 0U;
    digitalWrite(RFID_STATUS_PIN, blinkOn ? HIGH : LOW);
  } else {
    digitalWrite(RFID_STATUS_PIN, rfidDetected ? HIGH : LOW);
  }
}

String stateName() {
  switch (gateState) {
    case WAITING_FOR_VEHICLE:    return "Waiting for vehicle";
    case WAITING_FOR_RFID:       return "Waiting for RFID";
    case OPENING_DELAY:          return "Barrier opening";
    case WAITING_FOR_IR_BLOCK:   return "Green - waiting for vehicle to cross";
    case WAITING_FOR_IR_CLEAR:   return "Vehicle under barrier";
    case WAITING_FOR_AREA_CLEAR: return "Barrier closing / clearing";
  }
  return "Unknown";
}

// Compact state names shared with the Plate Program hardware indicators.
String plateProgramGateState() {
  switch (gateState) {
    case WAITING_FOR_VEHICLE:    return "idle_closed";
    case WAITING_FOR_RFID:       return "recognizing";
    case OPENING_DELAY:          return "opening";
    case WAITING_FOR_IR_BLOCK:   return "open_waiting_passage";
    case WAITING_FOR_IR_CLEAR:   return "vehicle_under_barrier";
    case WAITING_FOR_AREA_CLEAR: return "closing";
  }
  return "unknown";
}

bool barrierReportedOpen() {
  return gateState == OPENING_DELAY ||
         gateState == WAITING_FOR_IR_BLOCK ||
         gateState == WAITING_FOR_IR_CLEAR;
}

String htmlEscape(String s) {
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  s.replace("'", "&#39;");
  return s;
}

String jsonEscape(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\r", "");
  s.replace("\n", " ");
  return s;
}

String cleanField(String value, size_t maxLength) {
  value.trim();
  value.replace("\r", " ");
  value.replace("\n", " ");
  while (value.indexOf("  ") >= 0) {
    value.replace("  ", " ");
  }
  if (value.length() > maxLength) {
    value = value.substring(0, maxLength);
  }
  return value;
}

String cleanPlate(String value) {
  value = cleanField(value, PLATE_NUMBER_MAX);
  value.toUpperCase();
  return value;
}

void copyToField(char* destination, size_t destinationSize, const String& value) {
  if (destinationSize == 0) return;
  memset(destination, 0, destinationSize);
  value.substring(0, destinationSize - 1).toCharArray(destination, destinationSize);
}

String normalizeUID(String uid) {
  uid.trim();
  uid.toUpperCase();

  String out;
  out.reserve(uid.length());

  for (size_t i = 0; i < uid.length(); i++) {
    char c = uid.charAt(i);

    if (c == ' ' || c == ':' || c == '-') {
      continue;
    }

    bool isHex =
      (c >= '0' && c <= '9') ||
      (c >= 'A' && c <= 'F');

    if (!isHex) {
      return "";
    }

    out += c;
  }

  if (out.length() < 8 || out.length() > RFID_VALUE_MAX) {
    return "";
  }

  return out;
}

String operatingModeName() {
  return operatingMode == MODE_PLATE_PROGRAM
    ? "Plate Program"
    : "Standalone";
}

String sanitizeConfigLine(String value, size_t maximumLength) {
  value.trim();
  value.replace("\r", "");
  value.replace("\n", "");
  if (value.length() > maximumLength) {
    value = value.substring(0, maximumLength);
  }
  return value;
}

void loadModeConfiguration() {
  if (!LittleFS.exists(MODE_CONFIG_FILE)) return;
  File file = LittleFS.open(MODE_CONFIG_FILE, "r");
  if (!file) return;

  String mode = file.readStringUntil('\n'); mode.trim();
  programWifiSsid = file.readStringUntil('\n'); programWifiSsid.trim();
  programWifiPassword = file.readStringUntil('\n'); programWifiPassword.trim();
  plateProgramBaseUrl = file.readStringUntil('\n'); plateProgramBaseUrl.trim();
  String savedControllerId = file.readStringUntil('\n'); savedControllerId.trim();
  controllerKey = file.readStringUntil('\n'); controllerKey.trim();
  file.close();

  // Older mode.cfg files contain only the first four lines. Keep the stable
  // chip-derived ID until Plate Program credentials are provisioned.
  if (savedControllerId.length() > 0) {
    controllerId = savedControllerId;
  }

  operatingMode = mode == "plate-program"
    ? MODE_PLATE_PROGRAM
    : MODE_STANDALONE;
  if (plateProgramBaseUrl.length() == 0) {
    plateProgramBaseUrl = "http://192.168.1.3:8080";
  }
  while (plateProgramBaseUrl.endsWith("/")) {
    plateProgramBaseUrl.remove(plateProgramBaseUrl.length() - 1);
  }
}

bool saveModeConfiguration(
  OperatingMode mode,
  String wifiSsid,
  String wifiPassword,
  String programUrl,
  String configuredControllerId,
  String configuredControllerKey
) {
  wifiSsid = sanitizeConfigLine(wifiSsid, 32);
  wifiPassword = sanitizeConfigLine(wifiPassword, 64);
  programUrl = sanitizeConfigLine(programUrl, 120);
  configuredControllerId = sanitizeConfigLine(configuredControllerId, 80);
  configuredControllerKey = sanitizeConfigLine(configuredControllerKey, 160);
  while (programUrl.endsWith("/")) programUrl.remove(programUrl.length() - 1);

  if (mode == MODE_PLATE_PROGRAM &&
      (wifiSsid.length() == 0 ||
       (!programUrl.startsWith("http://") && !programUrl.startsWith("https://")) ||
       configuredControllerId.length() == 0 ||
       configuredControllerKey.length() == 0)) {
    return false;
  }

  File file = LittleFS.open(MODE_CONFIG_FILE, "w");
  if (!file) return false;
  file.println(mode == MODE_PLATE_PROGRAM ? "plate-program" : "standalone");
  file.println(wifiSsid);
  file.println(wifiPassword);
  file.println(programUrl);
  file.println(configuredControllerId);
  file.println(configuredControllerKey);
  file.close();
  return true;
}

String formEncode(const String& value) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(value.length() * 3U);
  for (size_t i = 0; i < value.length(); i++) {
    uint8_t c = (uint8_t)value.charAt(i);
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.') {
      encoded += (char)c;
    } else {
      encoded += '%';
      encoded += HEX_DIGITS[(c >> 4U) & 0x0FU];
      encoded += HEX_DIGITS[c & 0x0FU];
    }
  }
  return encoded;
}

int postToPlateProgram(
  const String& endpoint,
  const String& body,
  String* responseBody = nullptr
) {
  HTTPClient http;
  int responseCode = -1;

  if (endpoint.startsWith("https://")) {
    BearSSL::WiFiClientSecure client;
    // The ESP8266 trust store is not maintained in the field. The HTTPS
    // transport is encrypted, while controller identity is independently
    // verified by the provisioned server key on every request.
    client.setInsecure();
    if (!http.begin(client, endpoint)) return -1;
    http.setTimeout(PROGRAM_HTTP_TIMEOUT_MS);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    http.addHeader("X-Controller-Key", controllerKey);
    responseCode = http.POST(body);
    if (responseBody != nullptr && responseCode > 0) {
      *responseBody = http.getString();
    }
    http.end();
    return responseCode;
  }

  WiFiClient client;
  if (!http.begin(client, endpoint)) return -1;
  http.setTimeout(PROGRAM_HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("X-Controller-Key", controllerKey);
  responseCode = http.POST(body);
  if (responseBody != nullptr && responseCode > 0) {
    *responseBody = http.getString();
  }
  http.end();
  return responseCode;
}

bool jsonBoolean(const String& json, const String& key, bool fallback = false) {
  String marker = "\"" + key + "\":";
  int start = json.indexOf(marker);
  if (start < 0) return fallback;
  start += marker.length();
  while (start < (int)json.length() && json.charAt(start) == ' ') start++;
  return json.substring(start).startsWith("true");
}

String jsonStringValue(const String& json, const String& key) {
  String marker = "\"" + key + "\":";
  int start = json.indexOf(marker);
  if (start < 0) return "";
  start += marker.length();
  while (start < (int)json.length() && json.charAt(start) == ' ') start++;
  if (json.substring(start).startsWith("null") || json.charAt(start) != '"') {
    return "";
  }
  start++;
  String result;
  bool escaped = false;
  for (int i = start; i < (int)json.length(); i++) {
    char c = json.charAt(i);
    if (escaped) { result += c; escaped = false; continue; }
    if (c == '\\') { escaped = true; continue; }
    if (c == '"') break;
    result += c;
  }
  return result;
}

void connectToPlateProgramNetwork() {
  if (operatingMode != MODE_PLATE_PROGRAM) {
    plateProgramStatus = "Standalone mode";
    plateProgramReachable = false;
    return;
  }
  if (programWifiSsid.length() == 0) {
    plateProgramStatus = "Wi-Fi not configured";
    plateProgramReachable = false;
    return;
  }

  WiFi.begin(programWifiSsid.c_str(), programWifiPassword.c_str());
  unsigned long startedAt = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - startedAt < PROGRAM_CONNECT_TIMEOUT_MS) {
    // The main loop is not running yet, so refresh D3 here as well. This
    // keeps the Wi-Fi warning blink active throughout the initial attempt.
    updateStatusOutputs();
    delay(50);
    yield();
  }
  updateStatusOutputs();
  plateProgramStatus = WiFi.status() == WL_CONNECTED
    ? "Connected to Plate Program network"
    : "Plate Program Wi-Fi unavailable";
  plateProgramReachable = false;
}

void maintainPlateProgramConnection() {
  if (operatingMode != MODE_PLATE_PROGRAM || WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastProgramReconnectAt < PROGRAM_RECONNECT_MS) return;
  lastProgramReconnectAt = millis();
  plateProgramStatus = "Reconnecting to Plate Program Wi-Fi";
  WiFi.disconnect();
  WiFi.begin(programWifiSsid.c_str(), programWifiPassword.c_str());
}

void sendPlateProgramHeartbeat() {
  if (operatingMode != MODE_PLATE_PROGRAM || WiFi.status() != WL_CONNECTED) {
    plateProgramReachable = false;
    return;
  }
  if (millis() - lastProgramHeartbeatAt < PROGRAM_HEARTBEAT_MS) return;
  lastProgramHeartbeatAt = millis();

  String endpoint = plateProgramBaseUrl + "/api/rfid-controller/status";
  String body = "controller_id=" + formEncode(controllerId);
  body += "&gate_state=" + formEncode(plateProgramGateState());
  body += "&loop_active=" + String(vehicleDetected() ? "1" : "0");
  body += "&ir_blocked=" + String(irBlocked() ? "1" : "0");
  body += "&barrier_open=" + String(barrierReportedOpen() ? "1" : "0");
  body += "&traffic_green=" + String(
    digitalRead(TRAFFIC_LIGHT_PIN) == LIGHT_GREEN_LEVEL ? "1" : "0"
  );
  body += "&credential_unrecognized=" + String(
    rfidDetected && activeVehiclePlate.length() == 0 ? "1" : "0"
  );
  int responseCode = postToPlateProgram(endpoint, body);
  plateProgramReachable = responseCode >= 200 && responseCode < 300;
  if (plateProgramReachable) {
    plateProgramStatus = "Plate Program connected";
  } else {
    plateProgramStatus = responseCode > 0
      ? "Plate Program HTTP " + String(responseCode)
      : "Plate Program did not respond";
  }
}

String newCaptureAttemptUid() {
  String uid = controllerId;
  uid += "-";
  uid += String(millis());
  uid += "-";
  uid += String(random(0xFFFF), HEX);
  uid.toUpperCase();
  return uid;
}

bool requestServerCapture() {
  if (operatingMode != MODE_PLATE_PROGRAM || WiFi.status() != WL_CONNECTED) {
    plateProgramStatus = "DENIED - Plate Program network unavailable";
    return false;
  }

  activeAttemptUid = newCaptureAttemptUid();
  String endpoint = plateProgramBaseUrl + "/api/controller/capture-request";
  String body = "controller_id=" + formEncode(controllerId);
  body += "&attempt_uid=" + formEncode(activeAttemptUid);
  String response;
  int responseCode = postToPlateProgram(endpoint, body, &response);
  if (responseCode < 200 || responseCode >= 300) {
    plateProgramStatus = responseCode > 0
      ? "DENIED - Camera capture HTTP " + String(responseCode)
      : "DENIED - Camera capture did not respond";
    return false;
  }

  plateProgramStatus = "Camera capture requested for " + activeAttemptUid;
  return true;
}

void openForServerAuthorization(const String& response) {
  activeVehiclePlate = jsonStringValue(response, "plate");
  activeVehicleOwner = jsonStringValue(response, "owner");
  activeVehicleBrand = "";
  activeVehicleModel = "";
  activeVehicleRFID = jsonStringValue(response, "rfid");
  if (activeVehiclePlate.length() == 0) activeVehiclePlate = "PLATE ACCESS";
  if (activeVehicleOwner.length() == 0) activeVehicleOwner = "Registered vehicle";
  if (activeVehicleRFID.length() == 0) activeVehicleRFID = "";

  lastRFIDResult = "AUTHORIZED by Plate Program";
  signalStop();
  requestOpenBarrier();
  gateState = OPENING_DELAY;
  stateStartedAt = millis();
}

void pollServerAuthorization() {
  if (activeAttemptUid.length() == 0 ||
      millis() - lastAuthorizationPollAt < 500) {
    return;
  }
  lastAuthorizationPollAt = millis();
  if (operatingMode != MODE_PLATE_PROGRAM || WiFi.status() != WL_CONNECTED) {
    return;
  }

  String endpoint = plateProgramBaseUrl + "/api/controller/access-result";
  String body = "controller_id=" + formEncode(controllerId);
  body += "&attempt_uid=" + formEncode(activeAttemptUid);
  String response;
  int responseCode = postToPlateProgram(endpoint, body, &response);
  if (responseCode < 200 || responseCode >= 300) {
    plateProgramReachable = false;
    return;
  }

  plateProgramReachable = true;
  String status = jsonStringValue(response, "status");
  if (status == "authorized") {
    openForServerAuthorization(response);
    return;
  }

  if (status == "denied" && millis() - lastAuthorizationRetryAt >= RFID_RETRY_MS) {
    lastAuthorizationRetryAt = millis();
    lastRFIDResult = "DENIED - retrying vehicle attempt";
    requestServerCapture();
    triggerRFIDReader();
  }
}

// Used only for migrating the older JSONL database.
String getJsonField(const String& line, const String& field) {
  String key = "\"" + field + "\":\"";
  int start = line.indexOf(key);
  if (start < 0) return "";

  start += key.length();
  String value;
  bool escaped = false;

  for (int i = start; i < (int)line.length(); i++) {
    char c = line.charAt(i);
    if (escaped) {
      value += c;
      escaped = false;
      continue;
    }
    if (c == '\\') {
      escaped = true;
      continue;
    }
    if (c == '"') break;
    value += c;
  }
  return value;
}

bool validDate(const String& value) {
  if (value.length() != 10) return false;
  if (value.charAt(4) != '-' || value.charAt(7) != '-') return false;
  for (int i = 0; i < 10; i++) {
    if (i == 4 || i == 7) continue;
    if (!isDigit(value.charAt(i))) return false;
  }

  int year = value.substring(0, 4).toInt();
  int month = value.substring(5, 7).toInt();
  int day = value.substring(8, 10).toInt();
  if (year < 2020 || year > 2099 || month < 1 || month > 12 || day < 1) return false;

  static const uint8_t daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int maxDay = daysInMonth[month - 1];
  bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  if (month == 2 && leap) maxDay = 29;
  return day <= maxDay;
}

void syncControllerClock(uint32_t epochUtc, int16_t timezoneOffsetMinutes) {
  clockBaseEpochUtc = epochUtc;
  clockBaseMillis = millis();
  clockTimezoneOffsetMinutes = timezoneOffsetMinutes;
  clockSynced = true;
}

uint32_t currentEpochUtc() {
  if (!clockSynced) return 0;
  return clockBaseEpochUtc + ((millis() - clockBaseMillis) / 1000UL);
}

String formatTimestampFromEpoch(uint32_t epochUtc) {
  if (!clockSynced || epochUtc == 0) return "";

  int32_t offsetSeconds = (int32_t)clockTimezoneOffsetMinutes * 60L;
  time_t localEpoch = (time_t)((int64_t)epochUtc + offsetSeconds);
  struct tm timeInfo;
  gmtime_r(&localEpoch, &timeInfo);

  // Leave formatting headroom so strict ESP8266 compiler warnings do not
  // treat the integer fields as potentially truncated. Valid output remains
  // the 25-character ISO-8601 value stored by MemberRecord.
  char buffer[80];
  int offsetAbs = abs(clockTimezoneOffsetMinutes);
  int offsetHours = offsetAbs / 60;
  int offsetMinutes = offsetAbs % 60;
  char sign = clockTimezoneOffsetMinutes >= 0 ? '+' : '-';

  snprintf(
    buffer,
    sizeof(buffer),
    "%04d-%02d-%02dT%02d:%02d:%02d%c%02d:%02d",
    timeInfo.tm_year + 1900,
    timeInfo.tm_mon + 1,
    timeInfo.tm_mday,
    timeInfo.tm_hour,
    timeInfo.tm_min,
    timeInfo.tm_sec,
    sign,
    offsetHours,
    offsetMinutes
  );

  return String(buffer);
}

String currentTimestamp() {
  return formatTimestampFromEpoch(currentEpochUtc());
}

String currentDate() {
  String ts = currentTimestamp();
  if (ts.length() < 10) return "";
  return ts.substring(0, 10);
}

bool isExpired(const String& expiry) {
  if (expiry.length() == 0) return false; // Legacy records may not have expiry yet.
  if (!clockSynced) return false;
  String today = currentDate();
  if (today.length() != 10) return false;
  return expiry < today;
}

String formatDisplayTimestamp(const String& value) {
  if (value.length() < 19) return value;
  return value.substring(0, 10) + " " + value.substring(11, 19);
}

// ============================================================
// MEMBER DATABASE
// ============================================================

size_t memberRecordCount(File& file) {
  return file.size() / sizeof(MemberRecord);
}

bool readMemberRecord(File& file, size_t index, MemberRecord& record) {
  size_t offset = index * sizeof(MemberRecord);
  if (!file.seek(offset, SeekSet)) return false;
  return file.read((uint8_t*)&record, sizeof(MemberRecord)) == sizeof(MemberRecord);
}

bool writeMemberRecord(size_t index, const MemberRecord& record) {
  File file = LittleFS.open(MEMBER_DB_FILE, "r+");
  if (!file) return false;

  size_t offset = index * sizeof(MemberRecord);
  bool ok = file.seek(offset, SeekSet) &&
            file.write((const uint8_t*)&record, sizeof(MemberRecord)) == sizeof(MemberRecord);
  file.close();
  return ok;
}

bool appendMemberRecord(const MemberRecord& record) {
  File file = LittleFS.open(MEMBER_DB_FILE, "a");
  if (!file) return false;
  bool ok = file.write((const uint8_t*)&record, sizeof(MemberRecord)) == sizeof(MemberRecord);
  file.close();
  return ok;
}

bool findMember(const String& rfid, MemberRecord* recordOut = nullptr, size_t* indexOut = nullptr) {
  File file = LittleFS.open(MEMBER_DB_FILE, "r");
  if (!file) return false;

  MemberRecord record;
  size_t index = 0;
  while (file.available() >= (int)sizeof(MemberRecord)) {
    if (file.read((uint8_t*)&record, sizeof(MemberRecord)) != sizeof(MemberRecord)) break;

    if (record.active && String(record.rfid) == rfid) {
      if (recordOut) *recordOut = record;
      if (indexOut) *indexOut = index;
      file.close();
      return true;
    }

    index++;
    if (index % 50 == 0) yield();
  }

  file.close();
  return false;
}

size_t countMembers() {
  File file = LittleFS.open(MEMBER_DB_FILE, "r");
  if (!file) return 0;

  size_t count = 0;
  size_t index = 0;
  MemberRecord record;
  while (file.available() >= (int)sizeof(MemberRecord)) {
    if (file.read((uint8_t*)&record, sizeof(MemberRecord)) != sizeof(MemberRecord)) break;
    if (record.active) count++;
    if (++index % 50 == 0) yield();
  }

  file.close();
  return count;
}

bool findReusableSlot(size_t& indexOut) {
  File file = LittleFS.open(MEMBER_DB_FILE, "r");
  if (!file) return false;

  MemberRecord record;
  size_t index = 0;
  while (file.available() >= (int)sizeof(MemberRecord)) {
    if (file.read((uint8_t*)&record, sizeof(MemberRecord)) != sizeof(MemberRecord)) break;
    if (!record.active) {
      indexOut = index;
      file.close();
      return true;
    }
    index++;
    if (index % 50 == 0) yield();
  }

  file.close();
  return false;
}

MemberRecord makeMemberRecord(
  const String& ownerName,
  const String& vehicleModel,
  const String& vehicleBrand,
  const String& plateNumber,
  const String& rfid,
  const String& expiry,
  const String& createdAt,
  const String& updatedAt,
  const String& lastAccessAt
) {
  MemberRecord record;
  memset(&record, 0, sizeof(record));
  record.active = 1;
  copyToField(record.ownerName, sizeof(record.ownerName), ownerName);
  copyToField(record.vehicleModel, sizeof(record.vehicleModel), vehicleModel);
  copyToField(record.vehicleBrand, sizeof(record.vehicleBrand), vehicleBrand);
  copyToField(record.plateNumber, sizeof(record.plateNumber), plateNumber);
  copyToField(record.rfid, sizeof(record.rfid), rfid);
  copyToField(record.expiry, sizeof(record.expiry), expiry);
  copyToField(record.createdAt, sizeof(record.createdAt), createdAt);
  copyToField(record.updatedAt, sizeof(record.updatedAt), updatedAt);
  copyToField(record.lastAccessAt, sizeof(record.lastAccessAt), lastAccessAt);
  return record;
}

bool addMember(
  String ownerName,
  String vehicleModel,
  String vehicleBrand,
  String plateNumber,
  String rfid,
  String expiry
) {
  ownerName = cleanField(ownerName, OWNER_NAME_MAX);
  vehicleModel = cleanField(vehicleModel, VEHICLE_MODEL_MAX);
  vehicleBrand = cleanField(vehicleBrand, VEHICLE_BRAND_MAX);
  plateNumber = cleanPlate(plateNumber);
  rfid = normalizeUID(rfid);
  expiry = cleanField(expiry, EXPIRY_MAX);

  if (ownerName.length() == 0 || vehicleModel.length() == 0 ||
      vehicleBrand.length() == 0 || plateNumber.length() == 0 ||
      rfid.length() == 0 || !validDate(expiry)) {
    return false;
  }

  if (findMember(rfid)) return false;

  String now = currentTimestamp();
  MemberRecord record = makeMemberRecord(
    ownerName, vehicleModel, vehicleBrand, plateNumber, rfid, expiry,
    now, now, ""
  );

  size_t reusableIndex = 0;
  bool ok = findReusableSlot(reusableIndex)
              ? writeMemberRecord(reusableIndex, record)
              : appendMemberRecord(record);

  if (ok) authorizedCount++;
  return ok;
}

bool updateMember(
  String rfid,
  String ownerName,
  String vehicleModel,
  String vehicleBrand,
  String plateNumber,
  String expiry
) {
  rfid = normalizeUID(rfid);
  ownerName = cleanField(ownerName, OWNER_NAME_MAX);
  vehicleModel = cleanField(vehicleModel, VEHICLE_MODEL_MAX);
  vehicleBrand = cleanField(vehicleBrand, VEHICLE_BRAND_MAX);
  plateNumber = cleanPlate(plateNumber);
  expiry = cleanField(expiry, EXPIRY_MAX);

  if (rfid.length() == 0 || ownerName.length() == 0 || vehicleModel.length() == 0 ||
      vehicleBrand.length() == 0 || plateNumber.length() == 0 || !validDate(expiry)) {
    return false;
  }

  MemberRecord record;
  size_t index;
  if (!findMember(rfid, &record, &index)) return false;

  copyToField(record.ownerName, sizeof(record.ownerName), ownerName);
  copyToField(record.vehicleModel, sizeof(record.vehicleModel), vehicleModel);
  copyToField(record.vehicleBrand, sizeof(record.vehicleBrand), vehicleBrand);
  copyToField(record.plateNumber, sizeof(record.plateNumber), plateNumber);
  copyToField(record.expiry, sizeof(record.expiry), expiry);
  copyToField(record.updatedAt, sizeof(record.updatedAt), currentTimestamp());

  return writeMemberRecord(index, record);
}

bool updateMemberLastAccess(size_t index, MemberRecord& record) {
  String now = currentTimestamp();
  if (now.length() == 0) return false;
  copyToField(record.lastAccessAt, sizeof(record.lastAccessAt), now);
  return writeMemberRecord(index, record);
}

bool removeMember(String rfid) {
  rfid = normalizeUID(rfid);
  if (rfid.length() == 0) return false;

  MemberRecord record;
  size_t index;
  if (!findMember(rfid, &record, &index)) return false;

  record.active = 0;
  bool ok = writeMemberRecord(index, record);
  if (ok && authorizedCount > 0) authorizedCount--;
  return ok;
}

bool memberMatchesSearch(const MemberRecord& record, String query) {
  query.trim();
  query.toLowerCase();
  if (query.length() == 0) return true;

  String haystack = String(record.ownerName) + " " +
                    String(record.vehicleBrand) + " " +
                    String(record.vehicleModel) + " " +
                    String(record.plateNumber) + " " +
                    String(record.rfid);
  haystack.toLowerCase();
  return haystack.indexOf(query) >= 0;
}

String memberJson(const MemberRecord& record) {
  String json = "{";
  json += "\"owner_name\":\"" + jsonEscape(String(record.ownerName)) + "\",";
  json += "\"vehicle_model\":\"" + jsonEscape(String(record.vehicleModel)) + "\",";
  json += "\"vehicle_brand\":\"" + jsonEscape(String(record.vehicleBrand)) + "\",";
  json += "\"plate_number\":\"" + jsonEscape(String(record.plateNumber)) + "\",";
  json += "\"rfid\":\"" + jsonEscape(String(record.rfid)) + "\",";
  json += "\"expiry\":\"" + jsonEscape(String(record.expiry)) + "\",";
  json += "\"created_at\":\"" + jsonEscape(String(record.createdAt)) + "\",";
  json += "\"updated_at\":\"" + jsonEscape(String(record.updatedAt)) + "\",";
  json += "\"last_access_at\":\"" + jsonEscape(String(record.lastAccessAt)) + "\",";
  json += "\"expired\":" + String(isExpired(String(record.expiry)) ? "true" : "false");
  json += "}";
  return json;
}

void migrateLegacyDatabaseIfNeeded() {
  if (LittleFS.exists(MEMBER_DB_FILE)) return;

  File newDb = LittleFS.open(MEMBER_DB_FILE, "w");
  if (!newDb) {
    Serial.println(F("[FS] Could not create member database"));
    return;
  }
  newDb.close();

  if (!LittleFS.exists(LEGACY_RFID_DB_FILE)) return;

  File legacy = LittleFS.open(LEGACY_RFID_DB_FILE, "r");
  if (!legacy || legacy.size() == 0) {
    if (legacy) legacy.close();
    return;
  }

  Serial.println(F("[FS] Migrating legacy RFID records..."));
  size_t migrated = 0;

  while (legacy.available()) {
    String line = legacy.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    String oldRfid = normalizeUID(getJsonField(line, "uid"));
    String oldName = cleanField(getJsonField(line, "name"), OWNER_NAME_MAX);
    if (oldRfid.length() == 0) continue;
    if (oldName.length() == 0) oldName = "Legacy member";

    MemberRecord record = makeMemberRecord(
      oldName, "Not set", "Not set", "Not set", oldRfid, "", "", "", ""
    );
    if (appendMemberRecord(record)) migrated++;
    if (migrated % 25 == 0) yield();
  }

  legacy.close();
  Serial.print(F("[FS] Legacy records migrated: "));
  Serial.println(migrated);
}

// ============================================================
// RELAY CONTROL
// ============================================================

void updateRelayPulse() {
  if (activeRelayPin < 0) {
    return;
  }

  if (millis() - relayPulseStartedAt >= RELAY_PULSE_MS) {
    digitalWrite(activeRelayPin, RELAY_INACTIVE_LEVEL);
    activeRelayPin = -1;
  }
}

bool startRelayPulse(uint8_t pin) {
  if (activeRelayPin >= 0) {
    return false;
  }

  digitalWrite(pin, RELAY_ACTIVE_LEVEL);
  activeRelayPin = pin;
  relayPulseStartedAt = millis();

  return true;
}

void requestOpenBarrier() {
  // Safety: never energize OPEN and CLOSE at the same time.
  digitalWrite(CLOSE_RELAY_PIN, RELAY_INACTIVE_LEVEL);

  if (startRelayPulse(OPEN_RELAY_PIN)) {
    Serial.println(F("[BARRIER] OPEN pulse"));
  }
}

void requestCloseBarrier() {
  digitalWrite(OPEN_RELAY_PIN, RELAY_INACTIVE_LEVEL);

  if (startRelayPulse(CLOSE_RELAY_PIN)) {
    Serial.println(F("[BARRIER] CLOSE pulse"));
  }
}

// ============================================================
// RFID READER
// ============================================================

void clearRFIDSerialBuffer() {
  while (rfidSerial.available()) {
    rfidSerial.read();
    yield();
  }

  rfidLineBuffer = "";
  rfidDebugPacketLength = 0;
  rfidDebugLastByteAt = 0;
}

void printRFIDDebugPacket() {
  if (!PRINT_RFID_RAW_BYTES || rfidDebugPacketLength == 0) {
    return;
  }

  Serial.print(F("[RFID RX] HEX: "));

  for (size_t i = 0; i < rfidDebugPacketLength; i++) {
    if (rfidDebugPacket[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(rfidDebugPacket[i], HEX);

    if (i + 1 < rfidDebugPacketLength) {
      Serial.print(' ');
    }
  }

  Serial.println();
}

// Parse the binary inventory packet observed from the RD903M-style reader.
// Sample packet:
// 14 00 01 01 01 0C E2 84 36 11 00 00 10 00 00 09 49 44 AA C7 0D CB
//                   ^^ 12-byte EPC starts here ----------------------^
//
// For this packet format:
//   byte 5 = EPC length in bytes (0x0C = 12 bytes)
//   byte 6 = first EPC byte
bool extractEPCFromBinaryPacket(const uint8_t* packet, size_t length, String& epcOut) {
  if (length < 7) {
    return false;
  }

  const size_t EPC_LENGTH_INDEX = 5;
  const size_t EPC_START_INDEX  = 6;

  uint8_t epcLength = packet[EPC_LENGTH_INDEX];

  // Normal EPCs are usually several bytes long. Keep a safe upper bound.
  if (epcLength < 4 || epcLength > 32) {
    return false;
  }

  if (EPC_START_INDEX + epcLength > length) {
    return false;
  }

  String epc;
  epc.reserve(epcLength * 2);

  const char hexChars[] = "0123456789ABCDEF";
  for (size_t i = 0; i < epcLength; i++) {
    uint8_t b = packet[EPC_START_INDEX + i];
    epc += hexChars[(b >> 4) & 0x0F];
    epc += hexChars[b & 0x0F];
  }

  epcOut = epc;
  return true;
}

// Fallback for readers configured to send a plain ASCII EPC followed by CR/LF.
bool extractASCIIEPC(const uint8_t* packet, size_t length, String& epcOut) {
  String text;
  text.reserve(length);

  for (size_t i = 0; i < length; i++) {
    uint8_t b = packet[i];

    if (b == '\r' || b == '\n') {
      continue;
    }

    // If any other non-printable byte exists, this is not a plain ASCII packet.
    if (b < 32 || b > 126) {
      return false;
    }

    text += (char)b;
  }

  String candidate = normalizeUID(text);
  if (candidate.length() == 0) {
    return false;
  }

  epcOut = candidate;
  return true;
}

void triggerRFIDReader() {
  clearRFIDSerialBuffer();

  if (RFID_TRIGGER_COMMAND_LENGTH > 0) {
    rfidSerial.write(
      RFID_TRIGGER_COMMAND,
      RFID_TRIGGER_COMMAND_LENGTH
    );
    // A response can arrive before this function returns. Never clear the
    // receive buffer here or a fast reader response could be discarded.
  }

  lastRFIDTriggerAt = millis();

  Serial.println(F("[RFID] Trigger requested"));
}

// Collect one complete UART response, then extract the EPC.
// The reader's response is considered complete after RFID_PACKET_IDLE_MS
// with no new bytes.
bool pollRFID(String& uidOut) {
  while (rfidSerial.available()) {
    uint8_t rawByte = (uint8_t)rfidSerial.read();

    if (rfidDebugPacketLength < RFID_DEBUG_PACKET_MAX) {
      rfidDebugPacket[rfidDebugPacketLength++] = rawByte;
    }

    rfidDebugLastByteAt = millis();
  }

  // Wait until the UART has been quiet long enough to consider the packet complete.
  if (rfidDebugPacketLength == 0 ||
      millis() - rfidDebugLastByteAt < RFID_PACKET_IDLE_MS) {
    return false;
  }

  String epc;
  bool found = extractEPCFromBinaryPacket(
    rfidDebugPacket,
    rfidDebugPacketLength,
    epc
  );

  // Keep compatibility with plain-text reader output too.
  if (!found) {
    found = extractASCIIEPC(
      rfidDebugPacket,
      rfidDebugPacketLength,
      epc
    );
  }

  if (PRINT_RFID_RAW_BYTES) {
    printRFIDDebugPacket();
  }

  if (found) {
    Serial.print(F("[RFID EPC] "));
    Serial.println(epc);
  } else {
    Serial.println(F("[RFID] Packet received, but no EPC was recognized"));
  }

  // Packet has now been consumed.
  rfidDebugPacketLength = 0;
  rfidDebugLastByteAt = 0;

  if (!found) {
    return false;
  }

  uidOut = epc;
  return true;
}

void clearEnrollmentData() {
  enrollOwnerName = "";
  enrollVehicleModel = "";
  enrollVehicleBrand = "";
  enrollPlateNumber = "";
  enrollExpiry = "";
}

PlateProgramDecision requestPlateProgramDecision(const String& uid) {
  PlateProgramDecision decision{false, false, "", "", "", "", "", "server_unavailable"};
  if (WiFi.status() != WL_CONNECTED) {
    plateProgramReachable = false;
    plateProgramStatus = "DENIED - Plate Program network unavailable";
    return decision;
  }

  String endpoint = plateProgramBaseUrl + "/api/rfid-controller/recognitions";
  String body = "rfid=" + formEncode(uid);
  body += "&controller_id=" + formEncode(controllerId);
  if (activeAttemptUid.length() > 0) {
    body += "&attempt_uid=" + formEncode(activeAttemptUid);
  }
  String response = "";
  int responseCode = postToPlateProgram(endpoint, body, &response);

  if (responseCode < 200 || responseCode >= 300) {
    plateProgramReachable = false;
    plateProgramStatus = responseCode > 0
      ? "DENIED - Plate Program HTTP " + String(responseCode)
      : "DENIED - Plate Program did not respond";
    return decision;
  }

  decision.requestCompleted = true;
  plateProgramReachable = true;
  decision.authorized = jsonBoolean(response, "authorized", false);
  decision.owner = jsonStringValue(response, "owner");
  decision.plate = jsonStringValue(response, "plate");
  decision.vehicleType = jsonStringValue(response, "vehicle_type");
  decision.make = jsonStringValue(response, "make");
  decision.model = jsonStringValue(response, "model");
  decision.reason = jsonStringValue(response, "authorization_reason");
  if (decision.reason.length() == 0) {
    decision.reason = jsonStringValue(response, "decision");
  }
  plateProgramStatus = decision.authorized
    ? "Plate Program authorized " + uid
    : "Plate Program denied " + uid;
  return decision;
}

void processScannedRFID(String uid) {
  uid = normalizeUID(uid);
  if (uid.length() == 0) return;

  rfidDetected = true;
  updateStatusOutputs();

  // Enrollment has priority and never opens the barrier.
  // Exactly one valid EPC is accepted; enrollment ends immediately afterwards.
  if (operatingMode == MODE_STANDALONE && enrollMode) {
    lastRFID = uid;
    lastRFIDSeenAt = millis();

    Serial.print(F("[ENROLL] EPC: "));
    Serial.println(uid);

    MemberRecord existing;
    if (findMember(uid, &existing)) {
      lastRFIDResult = "Enrollment failed - RFID already belongs to " + String(existing.ownerName);
      enrollmentStatus = "RFID already enrolled: " + uid;
      Serial.println(F("[ENROLL] RFID already exists"));
    } else if (addMember(
                 enrollOwnerName,
                 enrollVehicleModel,
                 enrollVehicleBrand,
                 enrollPlateNumber,
                 uid,
                 enrollExpiry
               )) {
      lastRFIDResult = "Enrollment successful - " + enrollOwnerName;
      enrollmentStatus = "Enrolled: " + uid;

      Serial.print(F("[ENROLL] ADDED: "));
      Serial.print(uid);
      Serial.print(F(" | Owner: "));
      Serial.println(enrollOwnerName);
    } else {
      lastRFIDResult = "Enrollment failed - could not save member";
      enrollmentStatus = "Failed to save member";
      Serial.println(F("[ENROLL] ERROR: Failed to save member"));
    }

    enrollMode = false;
    clearEnrollmentData();
    return;
  }

  if (uid == lastRFID && millis() - lastRFIDSeenAt < DUPLICATE_IGNORE_MS) {
    return;
  }

  lastRFID = uid;
  lastRFIDSeenAt = millis();

  Serial.print(F("[RFID] "));
  Serial.println(uid);

  if (gateState != WAITING_FOR_RFID) {
    lastRFIDResult = "Ignored - no vehicle waiting";
    return;
  }

  if (operatingMode == MODE_PLATE_PROGRAM) {
    lastRFIDResult = "Waiting for Plate Program decision";
    PlateProgramDecision decision = requestPlateProgramDecision(uid);
    if (!decision.requestCompleted) {
      lastRFIDResult = plateProgramStatus;
      return;
    }
    if (!decision.authorized) {
      lastRFIDResult = "DENIED by Plate Program";
    if (decision.reason.length() > 0) {
        lastRFIDResult += " - " + decision.reason;
      }
      return;
    }

    activeVehiclePlate = decision.plate.length() > 0
      ? decision.plate
      : "RFID ACCESS";
    activeVehicleOwner = decision.owner.length() > 0
      ? decision.owner
      : "Registered RFID member";
    activeVehicleBrand = decision.make.length() > 0
      ? decision.make
      : decision.vehicleType;
    activeVehicleModel = decision.model;
    activeVehicleRFID = uid;
    lastRFIDResult = "AUTHORIZED by Plate Program";
    if (decision.owner.length() > 0) lastRFIDResult += " - " + decision.owner;

    signalStop();
    requestOpenBarrier();
    gateState = OPENING_DELAY;
    stateStartedAt = millis();
    return;
  }

  MemberRecord member;
  size_t memberIndex = 0;

  if (!findMember(uid, &member, &memberIndex)) {
    lastRFIDResult = "DENIED - RFID not enrolled";
    Serial.println(F("[ACCESS] DENIED - NOT ENROLLED"));
    return;
  }

  // Expiry requires a real date. The dashboard automatically synchronizes the clock.
  // Fail closed after a reboot until the browser has synchronized time again.
  if (String(member.expiry).length() > 0 && !clockSynced) {
    lastRFIDResult = "DENIED - controller clock not synchronized";
    Serial.println(F("[ACCESS] DENIED - CLOCK NOT SYNCED"));
    return;
  }

  if (isExpired(String(member.expiry))) {
    lastRFIDResult = "DENIED - membership expired - " + String(member.ownerName);
    Serial.println(F("[ACCESS] DENIED - EXPIRED"));
    return;
  }

  lastRFIDResult = "AUTHORIZED - " + String(member.ownerName) +
                   " - " + String(member.plateNumber);

  // Keep the identified vehicle visible on the dashboard for the entire
  // active gate cycle. It is cleared only after the vehicle has fully passed.
  activeVehiclePlate = String(member.plateNumber);
  activeVehicleOwner = String(member.ownerName);
  activeVehicleBrand = String(member.vehicleBrand);
  activeVehicleModel = String(member.vehicleModel);
  activeVehicleRFID = String(member.rfid);

  Serial.print(F("[ACCESS] AUTHORIZED - PLATE: "));
  Serial.println(activeVehiclePlate);

  // Update only this fixed-size row, not the entire database.
  updateMemberLastAccess(memberIndex, member);

  signalStop();
  requestOpenBarrier();
  gateState = OPENING_DELAY;
  stateStartedAt = millis();
}

// ============================================================
// GATE STATE MACHINE
// ============================================================

void updateGate() {
  // Scan enrollment is independent of the loop detector.
  // While enrollment is active, do not begin a vehicle access cycle.
  if (operatingMode == MODE_STANDALONE && enrollMode) {
    signalStop();
    return;
  }

  switch (gateState) {

    case WAITING_FOR_VEHICLE:
      signalStop();

      if (!vehicleDetected()) {
        rfidDetected = false;
      }

      if (vehicleDetected()) {
        Serial.println(F("[LOOP] Vehicle detected"));

        gateState = WAITING_FOR_RFID;
        stateStartedAt = millis();

        requestServerCapture();
        triggerRFIDReader();
      }
      break;

    case WAITING_FOR_RFID:
      signalStop();

      // Vehicle left before authorization.
      if (!vehicleDetected()) {
        Serial.println(F("[LOOP] Vehicle left before authorization"));

        gateState = WAITING_FOR_VEHICLE;
        stateStartedAt = millis();
        break;
      }

      // Re-trigger reader while vehicle remains present.
      if (millis() - lastRFIDTriggerAt >= RFID_RETRY_MS) {
        triggerRFIDReader();
      }
      pollServerAuthorization();
      break;

    case OPENING_DELAY:
      // Give the barrier time to start moving before GREEN.
      if (millis() - stateStartedAt >= GREEN_DELAY_MS) {
        signalGo();

        gateState = WAITING_FOR_IR_BLOCK;
        stateStartedAt = millis();

        Serial.println(F("[LIGHT] GREEN"));
      }
      break;

    case WAITING_FOR_IR_BLOCK:
      signalGo();

      if (irBlocked()) {
        Serial.println(F("[IR] Beam blocked"));

        gateState = WAITING_FOR_IR_CLEAR;
        stateStartedAt = millis();
        irClearStartedAt = 0;
      }
      break;

    case WAITING_FOR_IR_CLEAR:
      signalGo();

      if (irBlocked()) {
        irClearStartedAt = 0;
      } else {
        if (irClearStartedAt == 0) {
          irClearStartedAt = millis();
        }

        if (millis() - irClearStartedAt >= IR_CLEAR_DELAY_MS) {
          // RED first, then command barrier to close.
          signalStop();
          Serial.println(F("[LIGHT] RED"));

          requestCloseBarrier();

          gateState = WAITING_FOR_AREA_CLEAR;
          stateStartedAt = millis();
          irClearStartedAt = 0;
        }
      }
      break;

    case WAITING_FOR_AREA_CLEAR:
      signalStop();

      // Do not accept the next cycle until both sensing areas are clear.
      if (!vehicleDetected() && !irBlocked()) {
        Serial.println(F("[SYSTEM] Cycle complete"));

        // The vehicle has cleared both detection areas, so remove the live
        // plate display before accepting the next vehicle.
        activeVehiclePlate = "";
        activeVehicleOwner = "";
        activeVehicleBrand = "";
        activeVehicleModel = "";
        activeVehicleRFID = "";
        activeAttemptUid = "";
        rfidDetected = false;

        gateState = WAITING_FOR_VEHICLE;
        stateStartedAt = millis();
      }
      break;
  }
}

// ============================================================
// WEB AUTHENTICATION
// ============================================================

bool requireWebAuth() {
  if (server.authenticate(WEB_USERNAME, WEB_PASSWORD)) {
    return true;
  }

  server.requestAuthentication();
  return false;
}

// ============================================================
// WEB PAGE - STANDALONE SYSTEM DASHBOARD
// ============================================================

const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>RFID Gate Dashboard</title>
  <style>
    *{box-sizing:border-box}
    :root{--bg:#f4f7fb;--card:#fff;--text:#172033;--muted:#6b7280;--line:#e5e7eb;--blue:#2563eb;--green:#059669;--red:#dc2626;--amber:#d97706;--nav:#111827}
    body{margin:0;font-family:Arial,sans-serif;background:var(--bg);color:var(--text)}
    .topbar{background:var(--nav);color:#fff;padding:16px 0;position:sticky;top:0;z-index:20;box-shadow:0 2px 10px rgba(0,0,0,.12)}
    .topinner,.wrap{width:min(96%,1180px);margin:auto}
    .topinner{display:flex;align-items:center;justify-content:space-between;gap:12px}
    .brand{font-weight:800;font-size:19px}.subtitle{font-size:12px;color:#cbd5e1;margin-top:3px}
    .wrap{padding:20px 0 35px}.toolbar{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:16px}
    h1,h2,h3,p{margin-top:0}h2{font-size:18px;margin-bottom:14px}
    .btn{border:0;border-radius:9px;padding:10px 14px;font-weight:700;cursor:pointer;font-size:14px}.btn-primary{background:var(--blue);color:#fff}.btn-green{background:var(--green);color:#fff}.btn-red{background:var(--red);color:#fff}.btn-gray{background:#475569;color:#fff}.btn-light{background:#eef2f7;color:#334155}.btn:disabled{opacity:.5;cursor:not-allowed}
    .cards{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;margin-bottom:14px}.card{background:var(--card);border:1px solid var(--line);border-radius:13px;padding:16px;box-shadow:0 2px 7px rgba(15,23,42,.04)}
    .stat-label{font-size:12px;color:var(--muted);margin-bottom:7px}.stat-value{font-size:16px;font-weight:800;word-break:break-word}.ok{color:var(--green)}.bad{color:var(--red)}.warn{color:var(--amber)}
    .section{margin-bottom:14px}.system-grid{display:grid;grid-template-columns:1.2fr .8fr;gap:12px}.last-rfid{font-family:monospace;font-size:14px;word-break:break-all;background:#f8fafc;border-radius:8px;padding:10px}
    .vehicle-live{margin-bottom:14px;background:linear-gradient(135deg,#111827,#1f2937);color:#fff;border:0;display:grid;grid-template-columns:minmax(250px,.8fr) 1.2fr;gap:18px;align-items:center}.vehicle-live.waiting{background:linear-gradient(135deg,#334155,#475569)}.vehicle-live.active-vehicle{background:linear-gradient(135deg,#064e3b,#047857)}.vehicle-kicker{font-size:12px;text-transform:uppercase;letter-spacing:.12em;color:#cbd5e1;font-weight:800;margin-bottom:9px}.plate-display{display:inline-block;min-width:230px;text-align:center;background:#fff;color:#111827;border:5px solid #111827;outline:2px solid #fff;border-radius:9px;padding:12px 18px;font-family:Arial,sans-serif;font-size:clamp(28px,5vw,48px);font-weight:900;letter-spacing:.08em;line-height:1;box-shadow:0 5px 16px rgba(0,0,0,.2)}.vehicle-detail-title{font-size:22px;font-weight:800;margin-bottom:6px}.vehicle-detail-line{color:#d1fae5;font-size:14px;line-height:1.6}.vehicle-placeholder{color:#cbd5e1}
    .tablebar{display:flex;gap:8px;justify-content:space-between;align-items:center;flex-wrap:wrap;margin-bottom:12px}.search{width:min(100%,380px);padding:10px 12px;border:1px solid #cbd5e1;border-radius:9px;font-size:14px}
    .tablewrap{overflow:auto;border:1px solid var(--line);border-radius:10px}table{width:100%;border-collapse:collapse;min-width:1120px;background:#fff}th,td{padding:10px 9px;border-bottom:1px solid var(--line);text-align:left;font-size:12px;vertical-align:top}th{background:#f8fafc;color:#475569;font-weight:800;position:sticky;top:0}td.rfid{font-family:monospace;word-break:break-all}.badge{display:inline-block;border-radius:999px;padding:4px 8px;font-size:11px;font-weight:800}.active{background:#dcfce7;color:#166534}.expired{background:#fee2e2;color:#991b1b}.legacy{background:#fef3c7;color:#92400e}.actions{display:flex;gap:5px}.mini{border:0;border-radius:7px;padding:6px 8px;font-size:11px;font-weight:700;cursor:pointer}
    .pagination{display:flex;align-items:center;justify-content:flex-end;gap:8px;margin-top:11px;font-size:13px;color:var(--muted)}
    .modal{display:none;position:fixed;inset:0;background:rgba(15,23,42,.62);z-index:100;align-items:center;justify-content:center;padding:16px}.modal.show{display:flex}.dialog{background:#fff;border-radius:15px;width:min(96%,560px);max-height:92vh;overflow:auto;box-shadow:0 20px 60px rgba(0,0,0,.25)}.dialog-head{padding:17px 18px;border-bottom:1px solid var(--line);display:flex;align-items:center;justify-content:space-between}.dialog-body{padding:18px}.close{border:0;background:transparent;font-size:24px;cursor:pointer;color:#64748b}
    .form-grid{display:grid;grid-template-columns:1fr 1fr;gap:11px}.field label{display:block;font-size:12px;font-weight:700;color:#475569;margin-bottom:5px}.field input{width:100%;padding:11px;border:1px solid #cbd5e1;border-radius:8px;font-size:14px}.full{grid-column:1/-1}.modal-actions{display:flex;justify-content:flex-end;gap:8px;margin-top:18px}.help{font-size:12px;line-height:1.5;color:var(--muted)}
    .scanbox{text-align:center;padding:18px 8px}.scanner{width:78px;height:78px;border:6px solid #dbeafe;border-top-color:var(--blue);border-radius:50%;margin:0 auto 18px;animation:spin 1s linear infinite}@keyframes spin{to{transform:rotate(360deg)}}.scan-epc{font-family:monospace;word-break:break-all;background:#f8fafc;padding:10px;border-radius:8px;margin-top:12px}
    .toast{position:fixed;right:18px;bottom:18px;max-width:360px;background:#111827;color:#fff;padding:12px 14px;border-radius:10px;display:none;z-index:200;font-size:13px}.toast.show{display:block}
    @media(max-width:900px){.cards{grid-template-columns:repeat(2,1fr)}.system-grid,.vehicle-live{grid-template-columns:1fr}}@media(max-width:560px){.cards,.form-grid{grid-template-columns:1fr}.topinner{align-items:flex-start}.toolbar{align-items:stretch}.toolbar .btn{width:100%}.plate-display{width:100%;min-width:0}}
  </style>
</head>
<body>
  <div class="topbar"><div class="topinner"><div><div class="brand">RFID Gate System</div><div class="subtitle" id="modeSubtitle">Access-control dashboard</div></div><div id="clockLabel" class="subtitle">Clock: syncing...</div></div></div>
  <main class="wrap">
    <div class="toolbar"><div><h1 style="font-size:24px;margin-bottom:4px">System Dashboard</h1><div class="help">Live gate and RFID status</div></div><div><a class="btn btn-gray" href="/mode" style="text-decoration:none;display:inline-block">System Mode</a> <button class="btn btn-green standalone-only" onclick="openEnrollModal()">+ Enroll Member</button></div></div>

    <section class="cards">
      <div class="card"><div class="stat-label">Gate state</div><div class="stat-value" id="gate">Loading...</div></div>
      <div class="card"><div class="stat-label">Traffic signal</div><div class="stat-value" id="light">Loading...</div></div>
      <div class="card"><div class="stat-label">Loop detector</div><div class="stat-value" id="loop">Loading...</div></div>
      <div class="card"><div class="stat-label">IR beam</div><div class="stat-value" id="ir">Loading...</div></div>
      <div class="card"><div class="stat-label">Members</div><div class="stat-value" id="count">0</div></div>
      <div class="card"><div class="stat-label">Enrollment</div><div class="stat-value" id="enrollState">Off</div></div>
      <div class="card"><div class="stat-label">Internal storage</div><div class="stat-value" id="storage">Loading...</div></div>
      <div class="card"><div class="stat-label">Controller clock</div><div class="stat-value" id="clockState">Loading...</div></div>
      <div class="card"><div class="stat-label">Operating mode</div><div class="stat-value" id="operatingMode">Loading...</div></div>
      <div class="card"><div class="stat-label">Plate Program</div><div class="stat-value" id="programStatus">Loading...</div></div>
    </section>

    <section class="card vehicle-live waiting" id="vehiclePanel">
      <div>
        <div class="vehicle-kicker">Vehicle at gate</div>
        <div class="plate-display" id="currentPlate">NO VEHICLE</div>
      </div>
      <div>
        <div class="vehicle-detail-title" id="currentVehicleTitle">Waiting for vehicle</div>
        <div class="vehicle-detail-line vehicle-placeholder" id="currentVehicleInfo">The authorized vehicle's plate number will appear here during the gate cycle.</div>
      </div>
    </section>

    <section class="system-grid section">
      <div class="card"><h2>Last RFID activity</h2><div class="last-rfid" id="lastUid">None</div><div class="help" id="lastResult" style="margin-top:9px">None</div></div>
      <div class="card standalone-only"><h2>Database</h2><div class="help">Every row contains owner, vehicle, RFID, expiry, and timestamps.</div><div style="margin-top:12px"><a class="btn btn-gray" href="/export" style="text-decoration:none;display:inline-block">Download JSONL</a></div></div>
      <div class="card program-only" style="display:none"><h2>Central database</h2><div class="help">RFID enrollment, member records, expiry, and authorization are managed by Plate Program. This controller stores no server-mode members.</div></div>
    </section>

    <section class="card section standalone-only">
      <div class="tablebar"><div><h2 style="margin-bottom:4px">Members</h2><div class="help" id="memberSummary">Loading records...</div></div><input id="memberSearch" class="search" placeholder="Search owner, vehicle, plate or RFID" oninput="scheduleSearch()"></div>
      <div class="tablewrap"><table><thead><tr><th>Owner</th><th>Brand</th><th>Model</th><th>Plate</th><th>RFID</th><th>Expiry</th><th>Status</th><th>Created</th><th>Updated</th><th>Last Access</th><th>Actions</th></tr></thead><tbody id="memberRows"><tr><td colspan="11">Loading...</td></tr></tbody></table></div>
      <div class="pagination"><button class="btn btn-light" id="prevBtn" onclick="changePage(-1)">Previous</button><span id="pageLabel">Page 1</span><button class="btn btn-light" id="nextBtn" onclick="changePage(1)">Next</button></div>
    </section>
  </main>

  <div class="modal standalone-only" id="enrollModal"><div class="dialog"><div class="dialog-head"><h2 style="margin:0" id="enrollTitle">Enroll Member</h2><button class="close" onclick="cancelEnrollment(true)">&times;</button></div><div class="dialog-body">
    <div id="enrollInfoStep">
      <p class="help">Enter the membership information first. The RFID value cannot be typed manually. After you continue, the controller will wait until exactly one RFID is scanned.</p>
      <div class="form-grid">
        <div class="field full"><label>Owner Name</label><input id="enOwner" maxlength="48" required></div>
        <div class="field"><label>Vehicle Brand</label><input id="enBrand" maxlength="32" placeholder="Toyota" required></div>
        <div class="field"><label>Vehicle Model</label><input id="enModel" maxlength="32" placeholder="Vios" required></div>
        <div class="field"><label>Plate Number</label><input id="enPlate" maxlength="16" placeholder="ABC1234" required></div>
        <div class="field"><label>Expiry Date</label><input id="enExpiry" type="date" required></div>
      </div>
      <div class="modal-actions"><button class="btn btn-gray" onclick="closeModal('enrollModal')">Cancel</button><button class="btn btn-green" onclick="beginEnrollment()">Continue to RFID Scan</button></div>
    </div>
    <div id="enrollScanStep" style="display:none"><div class="scanbox"><div class="scanner" id="scanSpinner"></div><h3 id="scanHeading">Waiting for RFID</h3><p class="help" id="scanMessage">Present one RFID tag to the reader. There is no countdown; the system will keep waiting until one tag is scanned or you cancel.</p><div class="scan-epc" id="scanEpc">No RFID scanned yet</div></div><div class="modal-actions"><button class="btn btn-red" id="scanCancel" onclick="cancelEnrollment(true)">Cancel Enrollment</button><button class="btn btn-primary" id="scanDone" style="display:none" onclick="finishEnrollmentModal()">Done</button></div></div>
  </div></div>

  <div class="modal standalone-only" id="editModal"><div class="dialog"><div class="dialog-head"><h2 style="margin:0">Edit Member</h2><button class="close" onclick="closeModal('editModal')">&times;</button></div><div class="dialog-body">
    <input type="hidden" id="editRfid"><div class="form-grid"><div class="field full"><label>Owner Name</label><input id="editOwner" maxlength="48"></div><div class="field"><label>Vehicle Brand</label><input id="editBrand" maxlength="32"></div><div class="field"><label>Vehicle Model</label><input id="editModel" maxlength="32"></div><div class="field"><label>Plate Number</label><input id="editPlate" maxlength="16"></div><div class="field"><label>Expiry Date</label><input id="editExpiry" type="date"></div><div class="field full"><label>RFID (read only)</label><input id="editRfidDisplay" readonly></div></div>
    <div class="modal-actions"><button class="btn btn-gray" onclick="closeModal('editModal')">Cancel</button><button class="btn btn-primary" onclick="saveMemberEdit()">Save Changes</button></div>
  </div></div></div>

  <div class="toast" id="toast"></div>

<script>
let page=1,limit=25,total=0,currentItems=[],searchTimer=null,lastEnrollState=false;
const $=id=>document.getElementById(id);
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const showToast=msg=>{const t=$('toast');t.textContent=msg;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),3200)};
const openModal=id=>$(id).classList.add('show');const closeModal=id=>$(id).classList.remove('show');

const syncClock=async()=>{try{const b=new URLSearchParams();b.set('epoch',Math.floor(Date.now()/1000));b.set('tz',-new Date().getTimezoneOffset());await fetch('/sync-time',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.toString()})}catch(e){}};

const refreshStatus=async()=>{
  try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)return;const s=await r.json();
    $('gate').textContent=s.gate;$('loop').textContent=s.vehicle?'VEHICLE DETECTED':'Clear';$('loop').className='stat-value '+(s.vehicle?'warn':'ok');$('ir').textContent=s.irBlocked?'BLOCKED':'Clear';$('ir').className='stat-value '+(s.irBlocked?'warn':'ok');
    $('light').textContent=s.green?'GREEN / GO':'RED / STOP';$('light').className='stat-value '+(s.green?'ok':'bad');$('count').textContent=s.serverMode?'Managed centrally':s.count;$('enrollState').textContent=s.serverMode?'Plate Program only':(s.enroll?'Waiting for RFID':'Off');$('enrollState').className='stat-value '+(s.enroll?'warn':'');
    $('lastUid').textContent=s.lastUid||'None';$('lastResult').textContent=s.lastResult||'None';$('storage').textContent=formatBytes(s.fsUsed)+' / '+formatBytes(s.fsTotal);$('clockState').textContent=s.clockSynced?'Synchronized':'SYNC REQUIRED';$('clockState').className='stat-value '+(s.clockSynced?'ok':'bad');$('clockLabel').textContent=s.clockSynced?('Controller time: '+s.now):'Clock: sync required';
    $('operatingMode').textContent=s.modeName;$('operatingMode').className='stat-value '+(s.serverMode?'warn':'ok');$('programStatus').textContent=s.programStatus;$('programStatus').className='stat-value '+(s.serverConnected?'ok':(s.serverMode?'bad':''));$('modeSubtitle').textContent=s.serverMode?'Plate Program client mode':'Standalone controller mode';document.querySelectorAll('.standalone-only').forEach(e=>e.style.display=s.serverMode?'none':'');document.querySelectorAll('.program-only').forEach(e=>e.style.display=s.serverMode?'':'none');

    const vp=$('vehiclePanel');
    if(s.currentPlate){
      vp.className='card vehicle-live active-vehicle';
      $('currentPlate').textContent=s.currentPlate;
      $('currentVehicleTitle').textContent=(s.currentBrand||'')+(s.currentBrand&&s.currentModel?' ':'')+(s.currentModel||'');
      $('currentVehicleInfo').className='vehicle-detail-line';
      $('currentVehicleInfo').textContent='Owner: '+(s.currentOwner||'Unknown')+'  •  RFID: '+(s.currentRfid||'Unknown');
    }else if(s.vehicle){
      vp.className='card vehicle-live waiting';
      $('currentPlate').textContent='SCAN RFID';
      $('currentVehicleTitle').textContent='Vehicle detected';
      $('currentVehicleInfo').className='vehicle-detail-line vehicle-placeholder';
      $('currentVehicleInfo').textContent='Waiting for an authorized RFID before displaying the plate number.';
    }else{
      vp.className='card vehicle-live waiting';
      $('currentPlate').textContent='NO VEHICLE';
      $('currentVehicleTitle').textContent='Waiting for vehicle';
      $('currentVehicleInfo').className='vehicle-detail-line vehicle-placeholder';
      $('currentVehicleInfo').textContent="The authorized vehicle's plate number will appear here during the gate cycle.";
    }

    if(lastEnrollState && !s.enroll){completeEnrollmentUI(s.enrollmentStatus,s.lastUid)}lastEnrollState=s.enroll;
  }catch(e){}
};
const formatBytes=n=>{if(!n)return'0 KB';return n>=1048576?(n/1048576).toFixed(2)+' MB':(n/1024).toFixed(0)+' KB'};

const openEnrollModal=()=>{
  $('enrollInfoStep').style.display='block';$('enrollScanStep').style.display='none';$('enrollTitle').textContent='Enroll Member';$('enOwner').value='';$('enBrand').value='';$('enModel').value='';$('enPlate').value='';$('enExpiry').value='';openModal('enrollModal')
};
const beginEnrollment=async()=>{
  const fields={owner_name:$('enOwner').value.trim(),vehicle_brand:$('enBrand').value.trim(),vehicle_model:$('enModel').value.trim(),plate_number:$('enPlate').value.trim(),expiry:$('enExpiry').value};
  if(Object.values(fields).some(v=>!v)){showToast('Please complete all membership fields.');return}
  await syncClock();
  const b=new URLSearchParams(fields);
  try{const r=await fetch('/enroll',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.toString()});const text=await r.text();if(!r.ok){showToast(text);return}
    $('enrollInfoStep').style.display='none';$('enrollScanStep').style.display='block';$('enrollTitle').textContent='Scan RFID';$('scanSpinner').style.display='block';$('scanHeading').textContent='Waiting for RFID';$('scanMessage').textContent='Present one RFID tag to the reader. There is no countdown; the system will keep waiting until one tag is scanned or you cancel.';$('scanEpc').textContent='No RFID scanned yet';$('scanCancel').style.display='inline-block';$('scanDone').style.display='none';lastEnrollState=true;refreshStatus();
  }catch(e){showToast('Could not start enrollment.')}
};
const completeEnrollmentUI=(status,rfid)=>{
  if(!$('enrollModal').classList.contains('show'))return;$('scanSpinner').style.display='none';$('scanCancel').style.display='none';$('scanDone').style.display='inline-block';$('scanEpc').textContent=rfid||'No RFID';
  if(status&&status.startsWith('Enrolled:')){$('scanHeading').textContent='Enrollment Complete';$('scanMessage').textContent='The member and RFID were saved successfully.';showToast('Member enrolled successfully.');loadMembers()}else{$('scanHeading').textContent='Enrollment Stopped';$('scanMessage').textContent=status||'Enrollment ended.'}
};
const cancelEnrollment=async closeAfter=>{try{await fetch('/cancel-enroll',{method:'POST'})}catch(e){}lastEnrollState=false;if(closeAfter)closeModal('enrollModal');refreshStatus()};
const finishEnrollmentModal=()=>{closeModal('enrollModal');loadMembers();refreshStatus()};

const scheduleSearch=()=>{clearTimeout(searchTimer);searchTimer=setTimeout(()=>{page=1;loadMembers()},350)};
const loadMembers=async()=>{
  const q=$('memberSearch').value.trim();try{const r=await fetch('/members?page='+page+'&limit='+limit+'&q='+encodeURIComponent(q),{cache:'no-store'});if(!r.ok)return;const data=await r.json();total=data.total;currentItems=data.items||[];renderMembers();}catch(e){$('memberRows').innerHTML='<tr><td colspan="11">Could not load member records.</td></tr>'}
};
const renderMembers=()=>{
  if(!currentItems.length){$('memberRows').innerHTML='<tr><td colspan="11">No matching members.</td></tr>'}else{$('memberRows').innerHTML=currentItems.map((m,i)=>{let st=m.expiry?(m.expired?'<span class="badge expired">EXPIRED</span>':'<span class="badge active">ACTIVE</span>'):'<span class="badge legacy">NO EXPIRY</span>';return `<tr><td>${esc(m.owner_name)}</td><td>${esc(m.vehicle_brand)}</td><td>${esc(m.vehicle_model)}</td><td>${esc(m.plate_number)}</td><td class="rfid">${esc(m.rfid)}</td><td>${esc(m.expiry||'Not set')}</td><td>${st}</td><td>${esc(displayTs(m.created_at))}</td><td>${esc(displayTs(m.updated_at))}</td><td>${esc(displayTs(m.last_access_at)||'Never')}</td><td><div class="actions"><button class="mini btn-light" onclick="openEdit(${i})">Edit</button><button class="mini btn-red" onclick="deleteMember(${i})">Delete</button></div></td></tr>`}).join('')}
  const pages=Math.max(1,Math.ceil(total/limit));if(page>pages){page=pages;loadMembers();return}$('memberSummary').textContent=total+' member'+(total===1?'':'s')+' found';$('pageLabel').textContent='Page '+page+' of '+pages;$('prevBtn').disabled=page<=1;$('nextBtn').disabled=page>=pages;
};
const displayTs=v=>{if(!v)return'';return v.length>=19?v.slice(0,10)+' '+v.slice(11,19):v};
const changePage=d=>{const pages=Math.max(1,Math.ceil(total/limit));page=Math.min(pages,Math.max(1,page+d));loadMembers()};
const openEdit=i=>{const m=currentItems[i];if(!m)return;$('editRfid').value=m.rfid;$('editRfidDisplay').value=m.rfid;$('editOwner').value=m.owner_name;$('editBrand').value=m.vehicle_brand;$('editModel').value=m.vehicle_model;$('editPlate').value=m.plate_number;$('editExpiry').value=m.expiry;openModal('editModal')};
const saveMemberEdit=async()=>{const data={rfid:$('editRfid').value,owner_name:$('editOwner').value.trim(),vehicle_brand:$('editBrand').value.trim(),vehicle_model:$('editModel').value.trim(),plate_number:$('editPlate').value.trim(),expiry:$('editExpiry').value};if(Object.values(data).some(v=>!v)){showToast('All member fields are required.');return}await syncClock();const b=new URLSearchParams(data);try{const r=await fetch('/member/update',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.toString()});const t=await r.text();if(!r.ok){showToast(t);return}closeModal('editModal');showToast('Member updated.');loadMembers()}catch(e){showToast('Could not update member.')}};
const deleteMember=async i=>{const m=currentItems[i];if(!m||!confirm('Delete '+m.owner_name+' and RFID '+m.rfid+'?'))return;const b=new URLSearchParams();b.set('rfid',m.rfid);try{const r=await fetch('/member/delete',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.toString()});const t=await r.text();if(!r.ok){showToast(t);return}showToast('Member deleted.');loadMembers();refreshStatus()}catch(e){showToast('Could not delete member.')}};

syncClock().then(refreshStatus);loadMembers();setInterval(refreshStatus,1000);setInterval(syncClock,60000);
</script>
</body></html>
)rawliteral";

String modeConfigurationPage(const String& message = "") {
  String page = F(
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>RFID Controller Mode</title><style>body{font-family:Arial;background:#f4f7fb;color:#172033;margin:0}"
    ".wrap{max-width:680px;margin:30px auto;padding:16px}.card{background:#fff;border:1px solid #e5e7eb;"
    "border-radius:14px;padding:20px;box-shadow:0 3px 12px #0001}label{display:block;font-weight:700;margin-top:14px}"
    "input,select{width:100%;padding:11px;margin-top:6px;border:1px solid #cbd5e1;border-radius:8px;box-sizing:border-box}"
    "button,a{display:inline-block;margin-top:18px;padding:11px 15px;border:0;border-radius:8px;background:#2563eb;color:#fff;"
    "font-weight:700;text-decoration:none}.back{background:#475569}.note{background:#eff6ff;padding:12px;border-radius:9px;font-size:13px;line-height:1.5}"
    ".message{background:#ecfdf5;color:#065f46;padding:10px;border-radius:8px}</style></head><body><div class='wrap'><div class='card'>"
    "<h1>System Mode</h1>"
  );
  if (message.length() > 0) {
    page += "<p class='message'>" + htmlEscape(message) + "</p>";
  }
  page += F(
    "<p class='note'><strong>Standalone:</strong> members and enrollment remain in internal storage.<br>"
    "<strong>Plate Program:</strong> the controller only reads RFID and obeys the server decision. Enrollment and records are managed on Plate Program.</p>"
    "<form method='post' action='/mode'><label>Operating mode<select name='mode'>"
  );
  page += operatingMode == MODE_STANDALONE
    ? "<option value='standalone' selected>Standalone</option><option value='plate-program'>Plate Program</option>"
    : "<option value='standalone'>Standalone</option><option value='plate-program' selected>Plate Program</option>";
  page += "</select></label><label>Local Wi-Fi name (SSID)<input name='ssid' maxlength='32' value='" +
          htmlEscape(programWifiSsid) + "'></label>";
  page += F("<label>Local Wi-Fi password<input type='password' name='password' maxlength='64' placeholder='Leave blank to keep saved password'></label>");
  page += "<label>Plate Program address<input name='program_url' maxlength='120' value='" +
          htmlEscape(plateProgramBaseUrl) + "' placeholder='https://server.example.com'></label>";
  page += "<label>Provisioned controller ID<input name='controller_id' maxlength='80' value='" +
          htmlEscape(controllerId) + "' placeholder='Copy from Plate Program Sites'></label>";
  page += F("<label>Controller key<input type='password' name='controller_key' maxlength='160' "
            "placeholder='Leave blank to keep saved key'></label>");
  page += F("<button type='submit'>Save and restart</button> <a class='back' href='/'>Back</a></form></div></div></body></html>");
  return page;
}

// ============================================================
// WEB HELPERS / HANDLERS
// ============================================================

void handleRoot() {
  if (!requireWebAuth()) return;
  server.send_P(200, "text/html", MAIN_PAGE);
}

bool requireStandaloneMode() {
  if (operatingMode == MODE_STANDALONE) return true;
  server.send(
    409,
    "text/plain",
    "This function is disabled in Plate Program mode. Manage RFID members in Plate Program."
  );
  return false;
}

void handleModeConfiguration() {
  if (!requireWebAuth()) return;
  if (server.method() == HTTP_GET) {
    server.send(200, "text/html", modeConfigurationPage());
    return;
  }

  OperatingMode requestedMode = server.arg("mode") == "plate-program"
    ? MODE_PLATE_PROGRAM
    : MODE_STANDALONE;
  String password = server.arg("password");
  if (password.length() == 0) password = programWifiPassword;
  String submittedControllerKey = server.arg("controller_key");
  if (submittedControllerKey.length() == 0) submittedControllerKey = controllerKey;
  if (!saveModeConfiguration(
        requestedMode,
        server.arg("ssid"),
        password,
        server.arg("program_url"),
        server.arg("controller_id"),
        submittedControllerKey
      )) {
    server.send(400, "text/html", modeConfigurationPage(
      "Could not save. Plate Program mode requires Wi-Fi, an http:// or https:// server address, and the provisioned controller ID and key."
    ));
    return;
  }

  server.send(
    200,
    "text/html",
    "<!doctype html><meta name='viewport' content='width=device-width'><h2>Configuration saved.</h2>"
    "<p>The controller is restarting. Reconnect to RFID-GATE, then open 192.168.4.1.</p>"
  );
  delay(500);
  ESP.restart();
}

void handleSyncTime() {
  if (!requireWebAuth()) return;
  if (!server.hasArg("epoch") || !server.hasArg("tz")) {
    server.send(400, "text/plain", "Missing clock data.");
    return;
  }

  uint32_t epoch = (uint32_t)strtoul(server.arg("epoch").c_str(), nullptr, 10);
  int tz = server.arg("tz").toInt();
  if (epoch < 1700000000UL || tz < -720 || tz > 840) {
    server.send(400, "text/plain", "Invalid clock data.");
    return;
  }

  syncControllerClock(epoch, (int16_t)tz);
  server.send(200, "text/plain", "Clock synchronized.");
}

void handleStatus() {
  if (!requireWebAuth()) return;

  FSInfo fsInfo;
  size_t fsTotal = 0;
  size_t fsUsed = 0;
  if (LittleFS.info(fsInfo)) {
    fsTotal = fsInfo.totalBytes;
    fsUsed = fsInfo.usedBytes;
  }

  String json = "{";
  json += "\"gate\":\"" + jsonEscape(stateName()) + "\",";
  json += "\"modeName\":\"" + jsonEscape(operatingModeName()) + "\",";
  json += "\"serverMode\":" + String(operatingMode == MODE_PLATE_PROGRAM ? "true" : "false") + ",";
  json += "\"serverConnected\":" + String(plateProgramReachable ? "true" : "false") + ",";
  json += "\"programStatus\":\"" + jsonEscape(plateProgramStatus) + "\",";
  json += "\"vehicle\":" + String(vehicleDetected() ? "true" : "false") + ",";
  json += "\"irBlocked\":" + String(irBlocked() ? "true" : "false") + ",";
  json += "\"green\":" + String(digitalRead(TRAFFIC_LIGHT_PIN) == LIGHT_GREEN_LEVEL ? "true" : "false") + ",";
  json += "\"count\":" + String(authorizedCount) + ",";
  json += "\"enroll\":" + String(enrollMode ? "true" : "false") + ",";
  json += "\"enrollmentStatus\":\"" + jsonEscape(enrollmentStatus) + "\",";
  json += "\"lastUid\":\"" + jsonEscape(lastRFID) + "\",";
  json += "\"lastResult\":\"" + jsonEscape(lastRFIDResult) + "\",";
  json += "\"currentPlate\":\"" + jsonEscape(activeVehiclePlate) + "\",";
  json += "\"currentOwner\":\"" + jsonEscape(activeVehicleOwner) + "\",";
  json += "\"currentBrand\":\"" + jsonEscape(activeVehicleBrand) + "\",";
  json += "\"currentModel\":\"" + jsonEscape(activeVehicleModel) + "\",";
  json += "\"currentRfid\":\"" + jsonEscape(activeVehicleRFID) + "\",";
  json += "\"clockSynced\":" + String(clockSynced ? "true" : "false") + ",";
  json += "\"now\":\"" + jsonEscape(currentTimestamp()) + "\",";
  json += "\"fsTotal\":" + String(fsTotal) + ",";
  json += "\"fsUsed\":" + String(fsUsed);
  json += "}";
  server.send(200, "application/json", json);
}

void handleEnroll() {
  if (!requireWebAuth()) return;
  if (!requireStandaloneMode()) return;

  if (gateState != WAITING_FOR_VEHICLE || vehicleDetected()) {
    server.send(409, "text/plain", "Cannot enroll while a vehicle access cycle is active.");
    return;
  }

  if (!clockSynced) {
    server.send(409, "text/plain", "Controller clock is not synchronized. Reload the dashboard and try again.");
    return;
  }

  const char* required[] = {"owner_name", "vehicle_model", "vehicle_brand", "plate_number", "expiry"};
  for (const char* field : required) {
    if (!server.hasArg(field) || server.arg(field).length() == 0) {
      server.send(400, "text/plain", "All membership fields are required.");
      return;
    }
  }

  enrollOwnerName = cleanField(server.arg("owner_name"), OWNER_NAME_MAX);
  enrollVehicleModel = cleanField(server.arg("vehicle_model"), VEHICLE_MODEL_MAX);
  enrollVehicleBrand = cleanField(server.arg("vehicle_brand"), VEHICLE_BRAND_MAX);
  enrollPlateNumber = cleanPlate(server.arg("plate_number"));
  enrollExpiry = cleanField(server.arg("expiry"), EXPIRY_MAX);

  if (!validDate(enrollExpiry)) {
    server.send(400, "text/plain", "Invalid expiry date.");
    clearEnrollmentData();
    return;
  }

  enrollMode = true;
  enrollmentStatus = "Waiting for RFID";
  lastRFIDResult = "Enrollment - waiting for one RFID";
  triggerRFIDReader();

  Serial.print(F("[ENROLL] Waiting for RFID for: "));
  Serial.println(enrollOwnerName);

  server.send(200, "text/plain", "Enrollment started. Present one RFID tag.");
}

void handleCancelEnroll() {
  if (!requireWebAuth()) return;
  if (!requireStandaloneMode()) return;
  enrollMode = false;
  clearEnrollmentData();
  enrollmentStatus = "Cancelled";
  lastRFIDResult = "Enrollment cancelled";
  clearRFIDSerialBuffer();
  Serial.println(F("[ENROLL] Cancelled"));
  server.send(200, "text/plain", "Enrollment cancelled.");
}

void handleMembers() {
  if (!requireWebAuth()) return;
  if (!requireStandaloneMode()) return;

  int page = server.hasArg("page") ? server.arg("page").toInt() : 1;
  int limit = server.hasArg("limit") ? server.arg("limit").toInt() : 25;
  String query = server.hasArg("q") ? server.arg("q") : "";
  if (page < 1) page = 1;
  if (limit < 1) limit = 25;
  if (limit > 50) limit = 50;

  size_t startAt = (size_t)(page - 1) * (size_t)limit;
  size_t matched = 0;
  size_t returned = 0;

  File file = LittleFS.open(MEMBER_DB_FILE, "r");
  if (!file) {
    server.send(500, "application/json", "{\"error\":\"Database unavailable\"}");
    return;
  }

  String json = "{\"page\":" + String(page) + ",\"limit\":" + String(limit) + ",\"items\":[";
  bool first = true;
  MemberRecord record;
  size_t scanned = 0;

  while (file.available() >= (int)sizeof(MemberRecord)) {
    if (file.read((uint8_t*)&record, sizeof(MemberRecord)) != sizeof(MemberRecord)) break;
    if (!record.active || !memberMatchesSearch(record, query)) {
      if (++scanned % 50 == 0) yield();
      continue;
    }

    if (matched >= startAt && returned < (size_t)limit) {
      if (!first) json += ',';
      json += memberJson(record);
      first = false;
      returned++;
    }
    matched++;
    if (++scanned % 50 == 0) yield();
  }
  file.close();

  json += "],\"total\":" + String(matched) + "}";
  server.send(200, "application/json", json);
}

void handleMemberUpdate() {
  if (!requireWebAuth()) return;
  if (!requireStandaloneMode()) return;
  if (!clockSynced) {
    server.send(409, "text/plain", "Controller clock is not synchronized.");
    return;
  }

  const char* required[] = {"rfid", "owner_name", "vehicle_model", "vehicle_brand", "plate_number", "expiry"};
  for (const char* field : required) {
    if (!server.hasArg(field) || server.arg(field).length() == 0) {
      server.send(400, "text/plain", "All fields are required.");
      return;
    }
  }

  if (updateMember(
        server.arg("rfid"),
        server.arg("owner_name"),
        server.arg("vehicle_model"),
        server.arg("vehicle_brand"),
        server.arg("plate_number"),
        server.arg("expiry")
      )) {
    server.send(200, "text/plain", "Member updated.");
  } else {
    server.send(400, "text/plain", "Member could not be updated.");
  }
}

void handleMemberDelete() {
  if (!requireWebAuth()) return;
  if (!requireStandaloneMode()) return;
  if (!server.hasArg("rfid")) {
    server.send(400, "text/plain", "Missing RFID.");
    return;
  }

  if (removeMember(server.arg("rfid"))) {
    server.send(200, "text/plain", "Member deleted.");
  } else {
    server.send(404, "text/plain", "Member not found.");
  }
}

void handleExport() {
  if (!requireWebAuth()) return;
  if (!requireStandaloneMode()) return;

  File file = LittleFS.open(MEMBER_DB_FILE, "r");
  if (!file) {
    server.send(404, "text/plain", "Member database not found.");
    return;
  }

  server.sendHeader("Content-Disposition", "attachment; filename=members.jsonl");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/x-ndjson", "");

  MemberRecord record;
  size_t rows = 0;
  while (file.available() >= (int)sizeof(MemberRecord)) {
    if (file.read((uint8_t*)&record, sizeof(MemberRecord)) != sizeof(MemberRecord)) break;
    if (!record.active) continue;
    server.sendContent(memberJson(record) + "\n");
    if (++rows % 20 == 0) yield();
  }
  file.close();
  server.sendContent("");
}

void handleNotFound() {
  if (!requireWebAuth()) return;
  server.send(404, "text/plain", "Not found.");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/mode", HTTP_GET, handleModeConfiguration);
  server.on("/mode", HTTP_POST, handleModeConfiguration);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/sync-time", HTTP_POST, handleSyncTime);
  server.on("/enroll", HTTP_POST, handleEnroll);
  server.on("/cancel-enroll", HTTP_POST, handleCancelEnroll);
  server.on("/members", HTTP_GET, handleMembers);
  server.on("/member/update", HTTP_POST, handleMemberUpdate);
  server.on("/member/delete", HTTP_POST, handleMemberDelete);
  server.on("/export", HTTP_GET, handleExport);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("[WEB] Dashboard server started"));
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  delay(50);

  char controllerIdBuffer[24];
  snprintf(
    controllerIdBuffer,
    sizeof(controllerIdBuffer),
    "rfid-%06X",
    ESP.getChipId()
  );
  controllerId = controllerIdBuffer;

  Serial.println();
  Serial.println(F("===================================="));
  Serial.println(F(" RFID GATE CONTROLLER V4"));
  Serial.println(F("===================================="));

  // Inputs
  pinMode(LOOP_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT_PULLUP);

  // Outputs
  pinMode(OPEN_RELAY_PIN, OUTPUT);
  pinMode(CLOSE_RELAY_PIN, OUTPUT);
  pinMode(RFID_STATUS_PIN, OUTPUT);
  pinMode(RED_STATUS_PIN, OUTPUT);
  pinMode(GREEN_STATUS_PIN, OUTPUT);
  pinMode(VEHICLE_STATUS_PIN, OUTPUT);
  pinMode(TRAFFIC_LIGHT_PIN, OUTPUT);

  digitalWrite(RFID_STATUS_PIN, LOW);
  digitalWrite(VEHICLE_STATUS_PIN, LOW);
  
  // Safe startup state
  digitalWrite(OPEN_RELAY_PIN, RELAY_INACTIVE_LEVEL);
  digitalWrite(CLOSE_RELAY_PIN, RELAY_INACTIVE_LEVEL);
  signalStop();

  // RFID serial
  rfidSerial.begin(RFID_BAUD);
  rfidLineBuffer.reserve(100);

  Serial.print(F("[RFID] Reader baud: "));
  Serial.println(RFID_BAUD);
  Serial.println(F("[RFID] Clean packet debug output enabled"));

  // Internal flash stores mode/network settings in both modes. Member records are
  // opened only when standalone mode is selected.
  bool fileSystemReady = LittleFS.begin();
  if (!fileSystemReady) {
    Serial.println(F("[ERROR] Internal storage unavailable."));
    Serial.println(F("[ERROR] Mode configuration unavailable."));
  } else {
    Serial.println(F("[FS] Internal storage ready"));
    loadModeConfiguration();

    if (operatingMode == MODE_STANDALONE) {
      migrateLegacyDatabaseIfNeeded();

      if (!LittleFS.exists(MEMBER_DB_FILE)) {
        File file = LittleFS.open(MEMBER_DB_FILE, "w");
        if (file) file.close();
      }

      authorizedCount = countMembers();
      Serial.print(F("[FS] Member count: "));
      Serial.println(authorizedCount);
    }
  }

  // Keep the recovery/admin AP in both modes. Plate Program mode additionally
  // joins the configured local network as a station.
  WiFi.mode(operatingMode == MODE_PLATE_PROGRAM ? WIFI_AP_STA : WIFI_AP);

  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);

  if (apStarted) {
    Serial.print(F("[WIFI] SSID: "));
    Serial.println(AP_SSID);

    Serial.print(F("[WIFI] Admin page: http://"));
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(F("[ERROR] Wi-Fi AP failed to start"));
  }

  if (operatingMode == MODE_PLATE_PROGRAM) {
    connectToPlateProgramNetwork();
    Serial.print(F("[MODE] Plate Program: "));
    Serial.println(plateProgramStatus);
  } else {
    plateProgramStatus = "Standalone mode";
    Serial.println(F("[MODE] Standalone"));
  }

  setupWebServer();

  gateState = WAITING_FOR_VEHICLE;
  stateStartedAt = millis();

  Serial.println(F("[SYSTEM] Ready"));
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  updateStatusOutputs();
  // Keep web administration responsive.
  server.handleClient();
  maintainPlateProgramConnection();
  sendPlateProgramHeartbeat();

  // Finish relay pulses without blocking the main system.
  updateRelayPulse();

  // IMPORTANT: read any RFID response BEFORE deciding to re-trigger.
  // This prevents a retry from clearing a valid response that is already waiting.
  String scannedUID;
  bool gotRFID = pollRFID(scannedUID);

  if (gotRFID) {
    processScannedRFID(scannedUID);
  }

  // Enrollment has no countdown. Keep requesting inventory until exactly
  // one RFID is scanned or the user cancels from the dashboard.
  if (operatingMode == MODE_STANDALONE && enrollMode && !gotRFID &&
      rfidDebugPacketLength == 0 &&
      millis() - lastRFIDTriggerAt >= RFID_RETRY_MS) {
    triggerRFIDReader();
  }

  // Run gate logic. updateGate() pauses normal access while enrollment is active.
  updateGate();

  yield();
}
