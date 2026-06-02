#pragma once
#include <Arduino.h>
#include <WiFiS3.h>

#define SYNC_PORT 4210

void syncBegin();
void syncPoll();
void sendHeartbeat();
void broadcastMessage(const char* text);
void broadcastEvent(const char* name);
// servo-vna: broadcast a MOTION_START — "begin Motion <id> at <syncStartMs>"
// where syncStartMs is on the shared synced clock (syncMillis()). Peers convert
// it into their own millis() frame and arm the Motion to start in lockstep.
void broadcastMotionStart(const char* motionId, unsigned long syncStartMs);
unsigned long syncMillis();
uint8_t syncNodeId();
void syncWritePeersHtml(WiFiClient& client);
void syncWritePeersJson(WiFiClient& client);
