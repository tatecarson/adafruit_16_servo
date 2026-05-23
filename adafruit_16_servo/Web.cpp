#include "Web.h"
#include <WiFiS3.h>
#include <EEPROM.h>
#include "Secrets.h"
#include "Sync.h"
#include "storage.h"
#include "bake_validate.h"

// Provided by command_interface.h in the servo sketch.
void dispatchCommand(const char* cmd, bool fromNetwork);

// Provided by adafruit_16_servo.ino — needs access to the runtime globals.
void writeStatusJson(WiFiClient& client);

// Provided by adafruit_16_servo.ino — streams POST body to InternalStorage.
bool otaReceive(WiFiClient& client, int contentLength);
void otaApply();

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
    // Heap-allocate the receive buffer so 4 KB only sits in RAM during the
    // upload, not permanently in BSS. (storage.h already has a 4 KB BSS
    // scratch; adding a second would overflow the linker's heap reservation
    // on UNO R4.)
    uint8_t* buf = (uint8_t*)malloc(STORAGE_PAYLOAD_MAX);
    if (!buf) {
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.println("{\"ok\":false,\"error\":\"oom\"}");
        return;
    }
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
        free(buf);
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
        free(buf);
        client.println("HTTP/1.1 400 Bad Request");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.print("{\"ok\":false,\"error\":\""); client.print(v.error); client.println("\"}");
        return;
    }
    if (!storageWriteSlot(buf, (uint16_t)received)) {
        free(buf);
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Connection: close");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Content-Type: application/json");
        client.println();
        client.println("{\"ok\":false,\"error\":\"write-failed\"}");
        return;
    }
    free(buf);
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
    // Read just the active slot's length header to report bytesUsed without
    // materializing the payload. Saves stack.
    int activeSlot = -1;
    uint8_t a = EEPROM.read(STORAGE_OFF_ACTIVE);
    if (a == 0 || a == 1) activeSlot = a;
    int bytesUsed = 0;
    if (activeSlot >= 0) {
        int off = (activeSlot == 0) ? STORAGE_SLOT_0_OFF : STORAGE_SLOT_1_OFF;
        bytesUsed = ((int)EEPROM.read(off) << 8) | EEPROM.read(off + 1);
        if (bytesUsed < 0 || bytesUsed > (int)STORAGE_PAYLOAD_MAX) bytesUsed = 0;
    }
    client.print("{\"ok\":true,\"boardId\":"); client.print(storageBoardId());
    client.print(",\"hasActive\":"); client.print(storageHasActive() ? "true" : "false");
    client.print(",\"hasPrevious\":"); client.print(storageHasPrevious() ? "true" : "false");
    client.print(",\"bytesUsed\":"); client.print(bytesUsed);
    client.print(",\"slotPayloadMax\":"); client.print((int)STORAGE_PAYLOAD_MAX);
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
