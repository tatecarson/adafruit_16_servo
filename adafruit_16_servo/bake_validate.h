#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

struct BakeValidateResult {
    bool ok;
    const char* error;   // static string, no allocation
};

// Lightweight check. Confirms top-level is an object containing
// "schemaVersion":1 and that braces+brackets are balanced. Does NOT
// semantically validate referenced IDs etc. — that lives in the motion
// engine (servo-2cw).
inline BakeValidateResult bakeValidate(const uint8_t* data, size_t len) {
    if (len < 2 || data[0] != '{') return { false, "not-an-object" };

    int depthCurly = 0, depthSquare = 0;
    bool inString = false, escape = false;
    bool hasVersion = false;
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (escape) { escape = false; continue; }
        if (inString) {
            if (c == '\\') escape = true;
            else if (c == '"') inString = false;
            continue;
        }
        if (c == '"') { inString = true; continue; }
        if (c == '{') depthCurly++;
        else if (c == '}') { if (--depthCurly < 0) return { false, "brace-underflow" }; }
        else if (c == '[') depthSquare++;
        else if (c == ']') { if (--depthSquare < 0) return { false, "bracket-underflow" }; }

        // Detect "schemaVersion":1 at top level (depthCurly == 1, depthSquare == 0).
        if (!hasVersion && depthCurly == 1 && depthSquare == 0 && c == ':') {
            const char* key = "\"schemaVersion\"";
            size_t klen = strlen(key);
            if (i >= klen) {
                long back = (long)i - 1;
                while (back >= 0 && (data[back] == ' ' || data[back] == '\t' || data[back] == '\n')) back--;
                if (back + 1 >= (long)klen &&
                    memcmp(data + back + 1 - klen, key, klen) == 0) {
                    size_t fwd = i + 1;
                    while (fwd < len && (data[fwd] == ' ' || data[fwd] == '\t' || data[fwd] == '\n')) fwd++;
                    if (fwd < len && data[fwd] == '1' && (fwd + 1 == len ||
                        data[fwd+1] == ',' || data[fwd+1] == '}' || data[fwd+1] == ' ' ||
                        data[fwd+1] == '\t' || data[fwd+1] == '\n')) {
                        hasVersion = true;
                    } else {
                        return { false, "unsupported-schema-version" };
                    }
                }
            }
        }
    }
    if (depthCurly != 0 || depthSquare != 0) return { false, "unbalanced" };
    if (inString) return { false, "unterminated-string" };
    if (!hasVersion) return { false, "missing-schema-version" };
    return { true, nullptr };
}
