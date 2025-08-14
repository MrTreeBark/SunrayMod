// esp32UdpUart.cpp
// Only instantiate the global udpConsole object as per the header

#include "esp32UdpUart.h"
#include "config.h"

// Main console for user output
#if defined(_SAM3XA_)
#define MAIN_CONSOLE SerialUSB
#else
#define MAIN_CONSOLE Serial2
#endif

// Hardware UART for UDP log forwarding
#define LOG_UART Serial3

// Buffer size for log messages
#define LOG_BUFFER_SIZE 2048

// Instantiate the global udpConsole object for logs (default channel is "log")
ConsoleMirrorStream udpConsole(&MAIN_CONSOLE, &LOG_UART, "log");

// Example: If you want to mirror protocol data, you could instantiate another ConsoleMirrorStream with channel "proto"
// ConsoleMirrorStream protoConsole(&MAIN_CONSOLE, &LOG_UART, "proto");
