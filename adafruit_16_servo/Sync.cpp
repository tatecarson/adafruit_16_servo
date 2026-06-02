#include "Sync.h"

// Provided by command_interface.h in the servo sketch. Declared extern so
// Sync.cpp stays decoupled from the command layer.
void dispatchCommand(const char* cmd, bool fromNetwork);
// servo-vna: arm a cluster-synchronized Motion. localStartMs is already in this
// board's millis() frame (Sync.cpp owns clockOffset, so it does the conversion).
void onMotionStartSync(const char* motionId, unsigned long localStartMs);

#define SYNC_MAGIC "INST1"
#define MAX_PEERS 4
#define HEARTBEAT_INTERVAL_MS 1000
#define PEER_ALIVE_MS 3000
// txSeq restarts at 0 when a board reboots, so an incoming seq far below the
// per-peer high-water mark means a new session. Without this, a non-rebooted
// receiver would keep its stale lastEventSeq/lastHeartbeatSeq/lastMotionSeq and
// silently drop the restarted peer's packets (stale clock sync, dropped synced
// Motion starts) until txSeq climbs back. The gap tolerates ordinary UDP
// reordering — a reboot drops thousands behind, a reorder only a few.
#define SEQ_REBOOT_GAP 16

struct Peer {
  uint8_t id;
  IPAddress ip;
  uint32_t lastEventSeq;
  uint32_t lastHeartbeatSeq;
  uint32_t lastMotionSeq;
  uint32_t seqHigh;
  uint32_t lastUptimeMs;
  unsigned long lastSeenMs;
};

static WiFiUDP udp;
static IPAddress broadcastIp(255, 255, 255, 255);
static uint8_t nodeId = 0;
static uint32_t txSeq = 0;
static long clockOffset = 0;
static unsigned long lastHeartbeatMs = 0;
static Peer peers[MAX_PEERS];
static int peerCount = 0;

uint8_t syncNodeId() { return nodeId; }

unsigned long syncMillis() {
  return millis() + clockOffset;
}

static void initNodeId() {
  byte mac[6];
  WiFi.macAddress(mac);
  nodeId = mac[5];
  if (nodeId == 0) {
    nodeId = mac[4] ? mac[4] : 1;
  }
}

static Peer* upsertPeer(uint8_t id, IPAddress ip) {
  for (int i = 0; i < peerCount; i++) {
    if (peers[i].id == id) {
      peers[i].ip = ip;
      peers[i].lastSeenMs = millis();
      return &peers[i];
    }
  }
  if (peerCount < MAX_PEERS) {
    peers[peerCount].id = id;
    peers[peerCount].ip = ip;
    peers[peerCount].lastEventSeq = 0;
    peers[peerCount].lastHeartbeatSeq = 0;
    peers[peerCount].lastMotionSeq = 0;
    peers[peerCount].seqHigh = 0;
    peers[peerCount].lastUptimeMs = 0;
    peers[peerCount].lastSeenMs = millis();
    return &peers[peerCount++];
  }
  return nullptr;
}

static uint8_t lowestAlivePeerId() {
  uint8_t lowest = nodeId;
  unsigned long now = millis();
  for (int i = 0; i < peerCount; i++) {
    if (now - peers[i].lastSeenMs < PEER_ALIVE_MS && peers[i].id < lowest) {
      lowest = peers[i].id;
    }
  }
  return lowest;
}

static void sendPacket(const char* type, const char* payload) {
  char buf[96];
  int n = snprintf(buf, sizeof(buf), "%s %s %u %lu %s",
                   SYNC_MAGIC, type, (unsigned)nodeId,
                   (unsigned long)(++txSeq), payload ? payload : "");
  if (n <= 0) return;
  udp.beginPacket(broadcastIp, SYNC_PORT);
  udp.write((const uint8_t*)buf, n);
  udp.endPacket();
}

void broadcastMessage(const char* text) { sendPacket("MSG", text); }
void broadcastEvent(const char* name)   { sendPacket("EVT", name); }

void broadcastMotionStart(const char* motionId, unsigned long leadMs) {
  char payload[64];
  snprintf(payload, sizeof(payload), "%s %lu", motionId ? motionId : "", leadMs);

  // MST is a one-shot trigger with no retransmit. WiFi BROADCAST frames get no
  // 802.11 ACK/retry and are dropped often (a single broadcast missed a
  // follower ~50% of the time — servo-dvi), so UNICAST the packet to each known
  // peer instead: unicast gets link-layer ACK + retransmission. Build it ONCE
  // (single seq) so the per-peer copies share a seq and any incidental
  // duplicate is deduped, not replayed as a second arm. Originator arms locally.
  char buf[96];
  int n = snprintf(buf, sizeof(buf), "%s %s %u %lu %s",
                   SYNC_MAGIC, "MST", (unsigned)nodeId,
                   (unsigned long)(++txSeq), payload);
  if (n <= 0) return;
  for (int i = 0; i < peerCount; i++) {
    udp.beginPacket(peers[i].ip, SYNC_PORT);
    udp.write((const uint8_t*)buf, n);
    udp.endPacket();
  }
}

void sendHeartbeat() {
  char payload[16];
  snprintf(payload, sizeof(payload), "%lu", (unsigned long)millis());
  sendPacket("HB", payload);
  lastHeartbeatMs = millis();
}

static void onSyncEvent(const char* name) {
  Serial.print("Sync EVT from peer: ");
  Serial.println(name);
  // Run the mirrored command locally; fromNetwork=true prevents re-broadcast.
  dispatchCommand(name, true);
}

static void handleSyncPacket() {
  int size = udp.parsePacket();
  if (size <= 0) return;

  char buf[128];
  int n = udp.read((uint8_t*)buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';

  const size_t magicLen = sizeof(SYNC_MAGIC) - 1;
  if (strncmp(buf, SYNC_MAGIC, magicLen) != 0 || buf[magicLen] != ' ') return;

  char type[8] = {0};
  unsigned originId = 0;
  unsigned long seq = 0;
  int payloadOffset = 0;
  int parsed = sscanf(buf + magicLen + 1, "%7s %u %lu %n",
                      type, &originId, &seq, &payloadOffset);
  if (parsed < 3) return;
  if ((uint8_t)originId == nodeId) return;

  const char* payload = buf + magicLen + 1 + payloadOffset;

  Peer* peer = upsertPeer((uint8_t)originId, udp.remoteIP());
  if (peer == nullptr) return;

  // Sender-reboot detection (shared by all packet types): if seq has fallen far
  // below this peer's high-water mark, the sender restarted (txSeq reset to 0).
  // Clear the per-type dedup counters so its restarted packets are accepted
  // instead of being rejected until seq climbs back past the stale value.
  if (peer->seqHigh != 0 && (uint32_t)seq + SEQ_REBOOT_GAP < peer->seqHigh) {
    peer->lastEventSeq = 0;
    peer->lastHeartbeatSeq = 0;
    peer->lastMotionSeq = 0;
    peer->seqHigh = 0;
  }
  if ((uint32_t)seq > peer->seqHigh) peer->seqHigh = (uint32_t)seq;

  if (strcmp(type, "EVT") == 0) {
    if (seq <= peer->lastEventSeq && peer->lastEventSeq != 0) return;
    peer->lastEventSeq = seq;
    onSyncEvent(payload);
  } else if (strcmp(type, "MST") == 0) {
    // servo-vna: "begin Motion <id> in <leadMs> ms". Relative start — arm on our
    // OWN clock at millis()+leadMs. No shared-clock conversion, so a stale/zero
    // clockOffset on any board can't desync or instant-complete the Motion.
    if (seq <= peer->lastMotionSeq && peer->lastMotionSeq != 0) return;
    peer->lastMotionSeq = seq;
    char motionId[40] = {0};
    unsigned long leadMs = 0;
    if (sscanf(payload, "%39s %lu", motionId, &leadMs) == 2 && motionId[0]) {
      onMotionStartSync(motionId, millis() + leadMs);
    }
  } else if (strcmp(type, "HB") == 0) {
    if (seq <= peer->lastHeartbeatSeq && peer->lastHeartbeatSeq != 0) return;
    peer->lastHeartbeatSeq = seq;
    unsigned long uptime = strtoul(payload, nullptr, 10);
    peer->lastUptimeMs = uptime;
    if ((uint8_t)originId == lowestAlivePeerId() && (uint8_t)originId < nodeId) {
      clockOffset = (long)uptime - (long)millis();
    }
  }
}

void syncBegin() {
  initNodeId();
  udp.begin(SYNC_PORT);
  sendHeartbeat();
}

void syncPoll() {
  handleSyncPacket();
  if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    sendHeartbeat();
  }
}

void syncWritePeersJson(WiFiClient& client) {
  unsigned long now = millis();
  String s;
  s.reserve(256);

  IPAddress selfIp = WiFi.localIP();
  char ipBuf[16];
  snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u",
           selfIp[0], selfIp[1], selfIp[2], selfIp[3]);

  s += F("{\"self\":{\"id\":"); s += nodeId;
  s += F(",\"ip\":\"");        s += ipBuf; s += '"';
  s += F(",\"uptimeMs\":");    s += now;
  s += F("},\"peers\":[");
  bool first = true;
  for (int i = 0; i < peerCount; i++) {
    unsigned long age = now - peers[i].lastSeenMs;
    if (age >= PEER_ALIVE_MS) continue;
    if (!first) s += ',';
    first = false;
    IPAddress pip = peers[i].ip;
    snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u", pip[0], pip[1], pip[2], pip[3]);
    s += F("{\"id\":");        s += peers[i].id;
    s += F(",\"ip\":\"");      s += ipBuf; s += '"';
    s += F(",\"lastSeenMs\":");s += age;
    s += F(",\"uptimeMs\":");  s += peers[i].lastUptimeMs;
    s += '}';
  }
  s += F("]}");

  client.write((const uint8_t*)s.c_str(), s.length());
}

void syncWritePeersHtml(WiFiClient& client) {
  client.print("<p>This node id: ");
  client.print(nodeId);
  client.print(" (");
  client.print(WiFi.localIP());
  client.println(")</p>");
  client.println("<h2>Peers</h2><ul>");
  unsigned long now = millis();
  int alive = 0;
  for (int i = 0; i < peerCount; i++) {
    unsigned long age = now - peers[i].lastSeenMs;
    if (age >= PEER_ALIVE_MS) continue;
    alive++;
    client.print("<li>id ");
    client.print(peers[i].id);
    client.print(" @ ");
    client.print(peers[i].ip);
    client.print(" (last seen ");
    client.print(age);
    client.println(" ms ago)</li>");
  }
  if (alive == 0) {
    client.println("<li>(no peers seen)</li>");
  }
  client.println("</ul>");
}
