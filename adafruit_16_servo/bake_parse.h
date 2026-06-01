#pragma once

#include <stdint.h>

static bool bakeAsciiEq(char a, char b) {
  if (a >= 'A' && a <= 'Z') a = (char)(a + ('a' - 'A'));
  if (b >= 'A' && b <= 'Z') b = (char)(b + ('a' - 'A'));
  return a == b;
}

static int bakeSkipWs(const uint8_t* data, int pos, int end) {
  while (pos < end) {
    char c = (char)data[pos];
    if (c != ' ' && c != '\t' && c != '\n' && c != '\r') break;
    pos++;
  }
  return pos;
}

static bool bakeKeyEquals(const uint8_t* data, int start, int end, const char* key) {
  int i = 0;
  for (int p = start; p < end; p++, i++) {
    if (key[i] == '\0' || (char)data[p] != key[i]) return false;
  }
  return key[i] == '\0';
}

static bool bakeStringEqualsIgnoreCase(const uint8_t* data, int pos, int end, const char* value) {
  if (pos >= end || data[pos] != '"') return false;
  int p = pos + 1;
  int i = 0;
  while (p < end && data[p] != '"') {
    if (data[p] == '\\') return false;
    if (value[i] == '\0' || !bakeAsciiEq((char)data[p], value[i])) return false;
    p++;
    i++;
  }
  return p < end && data[p] == '"' && value[i] == '\0';
}

static bool bakeCopyString(const uint8_t* data, int pos, int end, char* out, uint8_t outLen) {
  if (pos >= end || data[pos] != '"' || outLen == 0) return false;
  int p = pos + 1;
  uint8_t n = 0;
  while (p < end && data[p] != '"') {
    if (data[p] == '\\' || n >= outLen - 1) return false;
    out[n++] = (char)data[p++];
  }
  if (p >= end || data[p] != '"') return false;
  out[n] = '\0';
  return true;
}

static bool bakeFindValue(const uint8_t* data, int start, int end, const char* key, int& valuePos) {
  int curly = 0;
  int square = 0;
  bool inString = false;
  bool escape = false;
  int stringStart = -1;
  bool keyMatched = false;

  for (int p = start; p < end; p++) {
    char c = (char)data[p];
    if (escape) {
      escape = false;
      continue;
    }
    if (inString) {
      if (c == '\\') {
        escape = true;
      } else if (c == '"') {
        inString = false;
        if (curly == 0 && square == 0 && stringStart >= 0) {
          keyMatched = bakeKeyEquals(data, stringStart, p, key);
        }
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      stringStart = p + 1;
    } else if (c == '{') {
      curly++;
      keyMatched = false;
    } else if (c == '}') {
      curly--;
      keyMatched = false;
    } else if (c == '[') {
      square++;
      keyMatched = false;
    } else if (c == ']') {
      square--;
      keyMatched = false;
    } else if (c == ':' && curly == 0 && square == 0 && keyMatched) {
      valuePos = bakeSkipWs(data, p + 1, end);
      return valuePos < end;
    } else if (c == ',') {
      keyMatched = false;
    }
  }
  return false;
}

static int bakeFindContainerEnd(const uint8_t* data, int start, int end, char openChar, char closeChar) {
  if (start >= end || data[start] != openChar) return -1;
  int depth = 0;
  bool inString = false;
  bool escape = false;
  for (int p = start; p < end; p++) {
    char c = (char)data[p];
    if (escape) {
      escape = false;
      continue;
    }
    if (inString) {
      if (c == '\\') escape = true;
      else if (c == '"') inString = false;
      continue;
    }
    if (c == '"') inString = true;
    else if (c == openChar) depth++;
    else if (c == closeChar && --depth == 0) return p;
  }
  return -1;
}

static bool bakeParseInteger(const uint8_t* data, int pos, int end, long& value) {
  pos = bakeSkipWs(data, pos, end);
  bool neg = false;
  if (pos < end && data[pos] == '-') {
    neg = true;
    pos++;
  }
  if (pos >= end || data[pos] < '0' || data[pos] > '9') return false;
  long v = 0;
  while (pos < end && data[pos] >= '0' && data[pos] <= '9') {
    v = (v * 10) + (data[pos] - '0');
    pos++;
  }
  if (pos < end && data[pos] == '.') {
    pos++;
    while (pos < end && data[pos] >= '0' && data[pos] <= '9') pos++;
  }
  value = neg ? -v : v;
  return true;
}

static bool bakeNextObjectInArray(const uint8_t* data, int arrayStart, int arrayEnd,
                                  int& pos, int& objStart, int& objEnd) {
  if (pos <= arrayStart) pos = arrayStart + 1;
  pos = bakeSkipWs(data, pos, arrayEnd);
  if (pos < arrayEnd && data[pos] == ',') pos = bakeSkipWs(data, pos + 1, arrayEnd);
  if (pos >= arrayEnd || data[pos] == ']') return false;
  if (data[pos] != '{') return false;
  objStart = pos;
  objEnd = bakeFindContainerEnd(data, objStart, arrayEnd + 1, '{', '}');
  if (objEnd < 0) return false;
  pos = objEnd + 1;
  return true;
}
