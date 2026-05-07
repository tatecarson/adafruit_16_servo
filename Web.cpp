#include "Web.h"
#include <WiFiS3.h>
#include "Sync.h"

// Provided by command_interface.h in the servo sketch.
void dispatchCommand(const char* cmd, bool fromNetwork);

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

  String requestLine = client.readStringUntil('\n');
  requestLine.trim();

  while (client.connected()) {
    String header = client.readStringUntil('\n');
    header.trim();
    if (header.length() == 0) break;
  }

  if (requestLine.startsWith("GET ")) {
    int pathStart = 4;
    int pathEnd = requestLine.indexOf(' ', pathStart);
    String path = requestLine.substring(pathStart, pathEnd);

    if (path.startsWith("/cmd?")) {
      String cmd = getQueryValue(path, "c");
      if (cmd.length() > 0) {
        Serial.print("HTTP command: ");
        Serial.println(cmd);
        // fromNetwork=false so dispatchCommand will broadcast it to peers.
        dispatchCommand(cmd.c_str(), false);
      }
    } else if (path == "/peers.json" || path.startsWith("/peers.json?")) {
      // CORS: only allow origins that browsers serialize as "null"
      // (file:// pages, sandboxed iframes). Tightens the previous "*" wildcard
      // so a random website on the LAN cannot fingerprint the cluster.
      // The bundled led_message_controller.html is opened from disk, so its
      // Origin header is "null" and the page works under this policy.
      client.println("HTTP/1.1 200 OK");
      client.println("Connection: close");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: null");
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
