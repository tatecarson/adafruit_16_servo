#pragma once
#include <Arduino.h>
#include <WiFiS3.h>

#define SYNC_PORT 4210

void syncBegin();
void syncPoll();
void sendHeartbeat();
void broadcastMessage(const char* text);
void broadcastEvent(const char* name);
unsigned long syncMillis();
uint8_t syncNodeId();
void syncWritePeersHtml(WiFiClient& client);
void syncWritePeersJson(WiFiClient& client);
