#pragma once
#include <Arduino.h>
#include <WiFiS3.h>

#define SYNC_PORT 4210

void syncBegin();
void syncPoll();
void sendHeartbeat();
void broadcastEvent(const char* name);
// servo-vna: send a MOTION_START — "begin Motion <id> in <leadMs> ms".
// Relative, not absolute: every board (originator included) arms the Motion at
// its OWN millis()+leadMs, so the start needs no shared clock. Unicast to each
// known peer (broadcast frames are dropped too often for a one-shot trigger).
void broadcastMotionStart(const char* motionId, unsigned long leadMs);
uint8_t syncNodeId();
void syncWritePeersHtml(WiFiClient& client);
void syncWritePeersJson(WiFiClient& client);
