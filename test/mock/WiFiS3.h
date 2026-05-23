#pragma once

class WiFiClient {
public:
    operator bool() const { return false; }
    bool connected() { return false; }
    int available() { return 0; }
    int read() { return -1; }
    int read(unsigned char*, int) { return 0; }
    void setTimeout(unsigned long) {}
    void stop() {}
    void print(const char*) {}
    void println(const char*) {}
    void println() {}
    template<typename T> void print(T) {}
    template<typename T> void println(T) {}
    void write(const unsigned char*, unsigned long) {}
};
