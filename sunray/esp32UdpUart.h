#define DEBUG_UDP_MIRROR 1 // Set to 0 to disable UDP mirroring for debugging
// esp32UdpUart.h
// ConsoleMirrorStream: mirrors all print/println to UART with log markers for ESP32 UDP forwarding
// Usage: redefine CONSOLE as an instance of ConsoleMirrorStream in config.h

#ifndef ESP32_UDP_UART_H
#define ESP32_UDP_UART_H

#include <Arduino.h>

#ifdef __cplusplus
class ConsoleMirrorStream : public Stream {
public:
    Stream* realConsole;
    Stream* logUart;
    String lineBuffer;
    bool mirrorEnabled;
    String channel; // "log" or "proto"
    ConsoleMirrorStream(Stream* _realConsole, Stream* _logUart, const String& _channel = "log")
        : realConsole(_realConsole), logUart(_logUart), mirrorEnabled(true), channel(_channel) {}
    void begin(unsigned long baudrate) {
        #if defined(_SAM3XA_)
        if (realConsole == &SerialUSB) SerialUSB.begin(baudrate);
        #else
        if (realConsole) static_cast<HardwareSerial*>(realConsole)->begin(baudrate);
        #endif
        if (logUart) static_cast<HardwareSerial*>(logUart)->begin(baudrate);
    }
    virtual size_t write(uint8_t c) override {
        // Buffer output for line-based mirroring with channel marker
        if (DEBUG_UDP_MIRROR && mirrorEnabled && logUart) {
            lineBuffer += (char)c;
            // Emergency flush if buffer gets too large (e.g., 256 bytes)
            const size_t EMERGENCY_FLUSH_SIZE = 256;
            if (lineBuffer.length() >= EMERGENCY_FLUSH_SIZE) {
                flush();
            } else if (c == '\n') {
                flush();
            }
        }
        return realConsole->write(c);
    }
    virtual size_t write(const uint8_t *buffer, size_t size) override {
        size_t n = 0;
        for (size_t i = 0; i < size; i++) {
            n += write(buffer[i]);
        }
        return n;
    }
    virtual int available() override { return realConsole->available(); }
    virtual int read() override { return realConsole->read(); }
    virtual int peek() override { return realConsole->peek(); }
    virtual void flush() override {
        realConsole->flush();
        // Also flush any buffered line to logUart, even if not newline-terminated
        if (DEBUG_UDP_MIRROR && mirrorEnabled && logUart && lineBuffer.length() > 0) {
            String trimmed = lineBuffer;
            trimmed.trim();
            if (trimmed.length() > 0) {
                int logLen = lineBuffer.length();
                logUart->print("/ULs:");
                logUart->print(channel); // channel marker
                logUart->print(":");
                logUart->print(logLen);
                logUart->print("\n");
                logUart->print(lineBuffer);
                logUart->print("/ULe\n");
            }
            lineBuffer = "";
        }
    }
    void setMirrorEnabled(bool enabled) { mirrorEnabled = enabled; }
    void setChannel(const String& ch) { channel = ch; }
};
#endif // __cplusplus

#endif // ESP32_UDP_UART_H
