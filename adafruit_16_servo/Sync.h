#pragma once
#include <Arduino.h>
#include <WiFiS3.h>

#define SYNC_PORT 4210

void syncBegin();
void syncPoll();
void sendHeartbeat();
void broadcastMessage(const char* text);
void broadcastEvent(const char* name);
// servo-vna: broadcast a MOTION_START — "begin Motion <id> in <leadMs> ms".
// Relative, not absolute: every board (originator included) arms the Motion at
// its OWN millis()+leadMs, so the start needs no shared-clock agreement and a
// stale/unsynced clockOffset on any board can't poison it. leadMs just has to
// exceed UDP broadcast propagation (~1-5ms on a LAN) so all boards arm together.
void broadcastMotionStart(const char* motionId, unsigned long leadMs);
unsigned long syncMillis();
uint8_t syncNodeId();
void syncWritePeersHtml(WiFiClient& client);
void syncWritePeersJson(WiFiClient& client);
