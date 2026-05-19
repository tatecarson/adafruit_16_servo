#include "Web.h"
#include <WiFiS3.h>
#include "Secrets.h"
#include "Sync.h"

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

void webBegin() {
  controlServer.begin();
}

void webPoll() {
  WiFiClient client = controlServer.available();
  if (!client) return;

  // Cap blocking reads so a stalled client can't freeze the servo loop.
  client.setTimeout(500);

  String requestLine = client.readStringUntil('\n');
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

  // --- Existing GET handlers ---
  if (method == "GET") {
    if (path.startsWith("/cmd?")) {
      String cmd = getQueryValue(path, "c");
      if (cmd.length() > 0) {
        Serial.print("HTTP command: ");
        Serial.println(cmd);
        dispatchCommand(cmd.c_str(), false);
      }
    } else if (path == "/status.json" || path.startsWith("/status.json?")) {
      client.println("HTTP/1.1 200 OK");
      client.println("Connection: close");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println();
      writeStatusJson(client);
      delay(5);
      client.stop();
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
}
