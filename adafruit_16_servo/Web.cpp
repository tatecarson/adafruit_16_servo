#include "Web.h"
#include <WiFiS3.h>
#include "Secrets.h"
#include "Sync.h"
#include "storage.h"
#include "servo_calibration.h"
#include "servo_runtime.h"
#include "bake_validate.h"

// Forward decls from servo_control.h / servo_maintenance.h. Defined inline
// in those headers and included by the .ino translation unit, so we can
// reach them at link time without pulling the headers into Web.cpp.
extern Adafruit_PWMServoDriver pwm;
extern ServoConfig servoConfig[NUM_SERVOS];

// Provided by command_interface.h in the servo sketch.
void dispatchCommand(const char* cmd, bool fromNetwork);

// Provided by adafruit_16_servo.ino — needs access to the runtime globals.
void writeStatusJson(WiFiClient& client);

// Provided by adafruit_16_servo.ino — streams POST body to InternalStorage.
bool otaReceive(WiFiClient& client, int contentLength);
void otaApply();
// Maximum sketch size the OTA partition can stage. Smaller than total flash —
// see comment over otaMaxSize() in adafruit_16_servo.ino.
int otaMaxSize();

extern bool otaInProgress;

static WiFiServer controlServer(80);

static int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static String urlDecode(String value) {
  String decoded;
  for (int i = 0; i < (int)value.length(); i++) {
    char c = value.charAt(i);
    if (c == '+') {
      decoded += ' ';
    } else if (c == '%' && i + 2 < (int)value.length()) {
      int high = hexValue(value.charAt(i + 1));
      int low = hexValue(value.charAt(i + 2));
      if (high >= 0 && low >= 0) {
        decoded += char((high << 4) | low);
        i += 2;
      }
    } else {
      decoded += c;
    }
  }
  return decoded;
}

static String getQueryValue(String path, const char* key) {
  String marker = String(key) + "=";
  int start = path.indexOf(marker);
  if (start < 0) return "";
  start += marker.length();
  int end = path.indexOf('&', start);
  if (end < 0) end = path.length();
  return urlDecode(path.substring(start, end));
}

static void sendControlResponse(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Connection: close");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!doctype html><html><body>");
  client.println("<h1>Servo Cluster Control</h1>");
  client.println("<form action=\"/cmd\" method=\"get\">");
  client.println("<input name=\"c\" placeholder=\"PLAY 1\" autofocus>");
  client.println("<button type=\"submit\">Send</button>");
  client.println("</form>");
  client.println("<p>Examples: PLAY 1, SPLAY 0, STOP. The command is run locally and broadcast to peers.</p>");
  syncWritePeersHtml(client);
  client.println("</body></html>");
}

// Streams the POST body into a static buffer (up to STORAGE_PAYLOAD_MAX), then
// validates and persists. Writes the response on `client`.
static void handleSequencesPost(WiFiClient& client, int contentLength) {
    if (contentLength <= 0 || contentLength > (int)STORAGE_PAYLOAD_MAX) {
        client.println("HTTP/1.1 413 Payload Too Large");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.print("{\"ok\":false,\"error\":\"too-large\",\"limit\":");
        client.print((int)STORAGE_PAYLOAD_MAX); client.println("}");
        return;
    }
    // Use the shared 4 KB BSS scratch. The only other consumer is
    // storageHasPrevious(), and we don't call it until after storageWriteSlot
    // has completed and we no longer reference `buf`.
    uint8_t* buf = storageScratchBuffer();
    int received = 0;
    unsigned long deadline = millis() + 5000;
    while (received < contentLength && millis() < deadline && client.connected()) {
        int avail = client.available();
        if (avail <= 0) { delay(2); continue; }
        int toRead = avail; if (toRead > contentLength - received) toRead = contentLength - received;
        int n = client.read(buf + received, toRead);
        if (n > 0) { received += n; deadline = millis() + 2000; }
    }
    if (received != contentLength) {
        client.println("HTTP/1.1 400 Bad Request");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.println("{\"ok\":false,\"error\":\"incomplete\"}");
        return;
    }
    BakeValidateResult v = bakeValidate(buf, received);
    if (!v.ok) {
        client.println("HTTP/1.1 400 Bad Request");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.print("{\"ok\":false,\"error\":\""); client.print(v.error); client.println("\"}");
        return;
    }
    if (!storageWriteSlot(buf, (uint16_t)received)) {
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.println("{\"ok\":false,\"error\":\"write-failed\"}");
        return;
    }
    // After this point we may call storageHasPrevious(); it will clobber
    // the scratch buffer, which is fine because we no longer reference it.
    client.println("HTTP/1.1 200 OK");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Content-Type: application/json");
    client.println();
    client.print("{\"ok\":true,\"bytesUsed\":"); client.print(received);
    client.print(",\"slotsFree\":"); client.print((int)STORAGE_PAYLOAD_MAX - received);
    client.print(",\"boardId\":"); client.print(storageBoardId());
    client.print(",\"hasPrevious\":"); client.print(storageHasPrevious() ? "true" : "false");
    client.println("}");
}

static void handleSequencesInfo(WiFiClient& client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Content-Type: application/json");
    client.println();
    client.print("{\"ok\":true,\"boardId\":"); client.print(storageBoardId());
    client.print(",\"hasActive\":"); client.print(storageHasActive() ? "true" : "false");
    client.print(",\"hasPrevious\":"); client.print(storageHasPrevious() ? "true" : "false");
    client.print(",\"bytesUsed\":"); client.print(storageActiveBytesUsed());
    client.print(",\"slotPayloadMax\":"); client.print((int)STORAGE_PAYLOAD_MAX);
    client.println("}");
}

static void handleBoardIdGet(WiFiClient& client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Content-Type: application/json");
    client.println();
    client.print("{\"ok\":true,\"boardId\":"); client.print(storageBoardId()); client.println("}");
}

static void handleBoardIdPost(WiFiClient& client, String path) {
    String v = getQueryValue(path, "id");
    int id = v.toInt();
    bool ok = storageSetBoardId((uint8_t)id);
    client.println(ok ? "HTTP/1.1 200 OK" : "HTTP/1.1 400 Bad Request");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Content-Type: application/json");
    client.println();
    if (ok) { client.print("{\"ok\":true,\"boardId\":"); client.print(storageBoardId()); client.println("}"); }
    else    { client.println("{\"ok\":false,\"error\":\"invalid-id\"}"); }
}

static void handleCalibrationGet(WiFiClient& client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Content-Type: application/json");
    client.println();
    client.print("{\"ok\":true,\"boardId\":"); client.print(storageBoardId());
    client.print(",\"servos\":[");
    for (uint8_t ch = 0; ch < CALIB_NUM_CHANNELS; ch++) {
        CalibrationRecord r = calibrationGet(ch);
        if (ch > 0) client.print(",");
        client.print("{\"ch\":"); client.print(ch);
        client.print(",\"minUs\":"); client.print(r.minUs);
        client.print(",\"maxUs\":"); client.print(r.maxUs);
        client.print(",\"offsetDeg\":"); client.print(r.offsetDeg);
        client.print(",\"calibrated\":"); client.print(r.calibrated ? "true" : "false");
        client.print("}");
    }
    client.println("]}");
}

// Minimal positive-int extractor for our tightly-scoped POST shape. We
// don't bring in a JSON library because the firmware OTA partition is
// tight; the calibration payload is small and well-formed. Returns -1
// if the key isn't found or doesn't parse.
static int32_t jsonIntField(const char* body, int len, const char* key) {
    String haystack(body, len);
    String needle = String("\"") + key + "\"";
    int k = haystack.indexOf(needle);
    if (k < 0) return -1;
    int colon = haystack.indexOf(':', k);
    if (colon < 0) return -1;
    int i = colon + 1;
    while (i < (int)haystack.length() && (haystack[i] == ' ' || haystack[i] == '\t')) i++;
    bool neg = false;
    if (i < (int)haystack.length() && (haystack[i] == '-' || haystack[i] == '+')) {
        if (haystack[i] == '-') neg = true;
        i++;
    }
    int32_t val = 0;
    bool any = false;
    while (i < (int)haystack.length() && haystack[i] >= '0' && haystack[i] <= '9') {
        val = val * 10 + (haystack[i] - '0');
        i++;
        any = true;
    }
    if (!any) return -1;
    return neg ? -val : val;
}

// POST /calibration body: {"ch":0,"minUs":1000,"maxUs":2000,"offsetDeg":0}
// Updates one channel and persists to EEPROM. minUs/maxUs are mandatory;
// offsetDeg defaults to 0 if absent. Returns the updated record.
static void handleCalibrationPost(WiFiClient& client, int contentLength) {
    if (contentLength <= 0 || contentLength > 256) {
        client.println("HTTP/1.1 413 Payload Too Large");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.println("{\"ok\":false,\"error\":\"too-large\"}");
        return;
    }
    char buf[260];
    int received = 0;
    unsigned long deadline = millis() + 2000;
    while (received < contentLength && millis() < deadline && client.connected()) {
        int avail = client.available();
        if (avail <= 0) { delay(2); continue; }
        int toRead = avail;
        if (toRead > contentLength - received) toRead = contentLength - received;
        if (received + toRead >= (int)sizeof(buf)) toRead = sizeof(buf) - received - 1;
        if (toRead <= 0) break;
        int n = client.read((uint8_t*)buf + received, toRead);
        if (n > 0) { received += n; deadline = millis() + 1000; }
    }
    buf[received] = '\0';
    int32_t ch        = jsonIntField(buf, received, "ch");
    int32_t minUs     = jsonIntField(buf, received, "minUs");
    int32_t maxUs     = jsonIntField(buf, received, "maxUs");
    int32_t offsetDeg = jsonIntField(buf, received, "offsetDeg");
    if (offsetDeg == -1 && String(buf).indexOf("\"offsetDeg\"") < 0) offsetDeg = 0;
    if (ch < 0 || ch >= CALIB_NUM_CHANNELS || minUs <= 0 || maxUs <= 0 || minUs >= maxUs ||
        minUs < 400 || maxUs > 2600 || offsetDeg < -128 || offsetDeg > 127) {
        client.println("HTTP/1.1 400 Bad Request");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.println("{\"ok\":false,\"error\":\"invalid-fields\"}");
        return;
    }
    calibrationSet((uint8_t)ch, (uint16_t)minUs, (uint16_t)maxUs, (int8_t)offsetDeg);
    // Apply live so the next S<n> command honors the new calibration without
    // a reboot. Re-applies all channels; cheap.
    calibrationApplyToServoConfig(servoConfig);
    CalibrationRecord r = calibrationGet((uint8_t)ch);
    client.println("HTTP/1.1 200 OK");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Content-Type: application/json");
    client.println();
    client.print("{\"ok\":true,\"ch\":"); client.print(r.calibrated ? (int)ch : -1);
    client.print(",\"minUs\":"); client.print(r.minUs);
    client.print(",\"maxUs\":"); client.print(r.maxUs);
    client.print(",\"offsetDeg\":"); client.print(r.offsetDeg);
    client.print(",\"calibrated\":"); client.print(r.calibrated ? "true" : "false");
    client.println("}");
}

static void handleSequencesRestore(WiFiClient& client) {
    bool ok = storageRollback();
    client.println(ok ? "HTTP/1.1 200 OK" : "HTTP/1.1 409 Conflict");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Content-Type: application/json");
    client.println();
    if (ok) client.println("{\"ok\":true}");
    else    client.println("{\"ok\":false,\"error\":\"no-previous\"}");
}

void webBegin() {
  controlServer.begin();
}

void webPoll() {
  WiFiClient client = controlServer.available();
  if (!client) return;

  unsigned long _wpStart = millis();

  // Cap blocking reads so a stalled client can't freeze the servo loop.
  client.setTimeout(500);

  String requestLine = client.readStringUntil('\n');
  unsigned long _wpReqLine = millis() - _wpStart;
  requestLine.trim();
  if (requestLine.length() == 0) { client.stop(); return; }

  int contentLength = 0;
  unsigned long headerDeadline = millis() + 1000;
  while (client.connected() && millis() < headerDeadline) {
    String header = client.readStringUntil('\n');
    header.trim();
    if (header.length() == 0) break;
    if (header.startsWith("Content-Length:")) {
      contentLength = header.substring(15).toInt();
    }
  }

  // Parse method and path from request line.
  int sp1 = requestLine.indexOf(' ');
  int sp2 = (sp1 > 0) ? requestLine.indexOf(' ', sp1 + 1) : -1;
  if (sp1 < 0 || sp2 < 0) { client.stop(); return; }
  String method = requestLine.substring(0, sp1);
  String path   = requestLine.substring(sp1 + 1, sp2);

  // --- CORS preflight for /ota ---
  if (method == "OPTIONS" && path.startsWith("/ota")) {
    client.println("HTTP/1.1 204 No Content");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Access-Control-Allow-Methods: POST, OPTIONS");
    client.println("Access-Control-Allow-Headers: Content-Type");
    client.println("Access-Control-Max-Age: 86400");
    client.println();
    delay(5);
    client.stop();
    return;
  }

  // --- Browser OTA upload: POST /ota?p=<password> ---
  if (method == "POST" && path.startsWith("/ota")) {
    String pw = getQueryValue(path, "p");
    if (pw != OTA_PASSWORD) {
      client.println("HTTP/1.1 403 Forbidden");
      client.println("Connection: close");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("bad password");
      delay(5);
      client.stop();
      return;
    }
    if (contentLength <= 0 || contentLength > 256 * 1024) {
      client.println("HTTP/1.1 400 Bad Request");
      client.println("Connection: close");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("bad content-length");
      delay(5);
      client.stop();
      return;
    }

    // Refuse oversized images BEFORE draining the request body. The OTA
    // partition is smaller than total flash (~120 KB on UNO R4 WiFi). If we
    // let an oversized image through, InternalStorage.open() refuses, our
    // write loop no-ops, then apply() erases the running sketch's reset
    // vector and reboots into garbage — bricking the board.
    int maxOta = otaMaxSize();
    if (maxOta > 0 && contentLength > maxOta) {
      client.println("HTTP/1.1 413 Payload Too Large");
      client.println("Connection: close");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Content-Type: text/plain");
      client.println();
      client.print("sketch ");
      client.print(contentLength);
      client.print(" bytes exceeds OTA partition limit ");
      client.print(maxOta);
      client.println(" bytes; flash via USB or shrink the binary");
      delay(5);
      client.stop();
      return;
    }

    if (otaReceive(client, contentLength)) {
      client.println("HTTP/1.1 200 OK");
      client.println("Connection: close");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("OK");
      delay(100);
      client.stop();
      otaApply();
    } else {
      client.println("HTTP/1.1 500 Internal Server Error");
      client.println("Connection: close");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("upload incomplete");
      delay(5);
      client.stop();
    }
    return;
  }

  // --- CORS preflight for /boardId endpoints ---
  if (method == "OPTIONS" && path.startsWith("/boardId")) {
    client.println("HTTP/1.1 204 No Content");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Access-Control-Allow-Methods: POST, GET, OPTIONS");
    client.println("Access-Control-Allow-Headers: Content-Type");
    client.println("Access-Control-Max-Age: 86400");
    client.println();
    delay(5); client.stop();
    return;
  }

  // --- CORS preflight for /sequences endpoints ---
  if (method == "OPTIONS" && path.startsWith("/sequences")) {
    client.println("HTTP/1.1 204 No Content");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Access-Control-Allow-Methods: POST, GET, OPTIONS");
    client.println("Access-Control-Allow-Headers: Content-Type");
    client.println("Access-Control-Max-Age: 86400");
    client.println();
    delay(5); client.stop();
    return;
  }

  if (method == "POST" && path == "/sequences") {
    handleSequencesPost(client, contentLength);
    delay(5); client.stop();
    return;
  }

  if (method == "POST" && path == "/sequences/restore") {
    handleSequencesRestore(client);
    delay(5); client.stop();
    return;
  }
  if (method == "GET" && (path == "/sequences/info" || path.startsWith("/sequences/info?"))) {
    handleSequencesInfo(client);
    delay(5); client.stop();
    return;
  }
  if (method == "GET" && (path == "/boardId" || path.startsWith("/boardId?"))) {
    handleBoardIdGet(client); delay(5); client.stop(); return;
  }
  if (method == "POST" && path.startsWith("/boardId")) {
    handleBoardIdPost(client, path); delay(5); client.stop(); return;
  }
  // CORS preflight + GET/POST for /calibration (servo-2n9). GET returns
  // all three channels' records; POST updates one channel at a time so
  // partial saves don't clobber siblings.
  if (method == "OPTIONS" && path.startsWith("/calibration")) {
    client.println("HTTP/1.1 204 No Content");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Access-Control-Allow-Methods: POST, GET, OPTIONS");
    client.println("Access-Control-Allow-Headers: Content-Type");
    client.println("Access-Control-Max-Age: 86400");
    client.println();
    delay(5); client.stop();
    return;
  }
  if (method == "GET" && (path == "/calibration" || path.startsWith("/calibration?"))) {
    handleCalibrationGet(client); delay(5); client.stop(); return;
  }
  if (method == "POST" && path == "/calibration") {
    handleCalibrationPost(client, contentLength); delay(5); client.stop(); return;
  }

  // --- Existing GET handlers ---
  if (method == "GET") {
    if (path.startsWith("/cmd?")) {
      String cmd = getQueryValue(path, "c");
      if (cmd.length() > 0) {
        Serial.print("HTTP command: ");
        Serial.println(cmd);
        dispatchCommand(cmd.c_str(), false);
      }
      // Browser sends with mode:no-cors and discards the body. Send a
      // minimal response and return so /cmd isn't paying for the full
      // HTML control page on every command.
      static const char kCmdOk[] =
        "HTTP/1.1 200 OK\r\n"
        "Connection: close\r\n"
        "Content-Length: 0\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n";
      client.write((const uint8_t*)kCmdOk, sizeof(kCmdOk) - 1);
      delay(5);
      client.stop();
      return;
    } else if (path == "/status.json" || path.startsWith("/status.json?")) {
      // One write for the response head, then one for the body. Each
      // WiFiClient call is an SPI round-trip to the coprocessor; batching
      // these cut status latency from ~1s to under 100ms.
      static const char kStatusHead[] =
        "HTTP/1.1 200 OK\r\n"
        "Connection: close\r\n"
        "Content-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n";
      client.write((const uint8_t*)kStatusHead, sizeof(kStatusHead) - 1);
      writeStatusJson(client);
      delay(5);
      client.stop();
      {
        unsigned long _wpTotal = millis() - _wpStart;
        if (_wpTotal >= 250) {
          Serial.print(F("SLOW web ")); Serial.print(_wpTotal);
          Serial.print(F("ms reqLine=")); Serial.print(_wpReqLine);
          Serial.println(F(" path=/status.json"));
        }
      }
      return;
    } else if (path == "/peers.json" || path.startsWith("/peers.json?")) {
      client.println("HTTP/1.1 200 OK");
      client.println("Connection: close");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println();
      syncWritePeersJson(client);
      delay(5);
      client.stop();
      return;
    }
  }

  sendControlResponse(client);
  delay(5);
  client.stop();

  unsigned long _wpTotal = millis() - _wpStart;
  if (_wpTotal >= 250) {
    Serial.print(F("SLOW web "));
    Serial.print(_wpTotal); Serial.print(F("ms reqLine="));
    Serial.print(_wpReqLine); Serial.print(F(" path="));
    Serial.println(path);
  }
}
