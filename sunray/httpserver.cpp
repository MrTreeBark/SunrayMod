#include "httpserver.h"
#include "config.h"
#include "robot.h"
#include "StateEstimator.h"
#include "LineTracker.h"
#include "Stats.h"
#include "src/op/op.h"
#include "reset.h"
#include "comm.h"

#ifdef __linux__
  #include <BridgeClient.h>
  #include <Process.h>
  #include <WiFi.h>
  #include <sys/resource.h>
#else
  #include "src/esp/WiFiEsp.h"
#endif
#include "RingBuffer.h"
#include "timetable.h"

// Debug
bool timeViolation = false;
unsigned long checkTime = 0;

// Hodor
bool hOdOr = false;
unsigned long hOdOrTimeOut = 0;

// wifi client
WiFiEspClient wifiClient;
unsigned long nextWifiClientCheckTime = 0;

// use a ring buffer to increase speed and reduce memory allocation
ERingBuffer buf(8);
int reqCount = 0;                // number of requests received
unsigned long stopClientTime = 0;
unsigned long wifiVerboseStopTime = 0;
unsigned long wifiLastClientAvailableWait = 0;
int wifiLastClientAvailable = 0;


// Add at file scope:
enum WifiRelayState {
  RELAY_IDLE,
  RELAY_CONNECTING,
  RELAY_SENDING_HEADER,
  RELAY_RECEIVING,
  RELAY_PROCESSING,
  RELAY_RESPONDING,
  RELAY_CLOSING
};
static WifiRelayState relayState = RELAY_IDLE;
static unsigned long relayStateStartTime = 0;
static String relayCmdBuffer = "";

// Non-blocking relay client handler
void processWifiRelayClient() {
  if (!wifiFound || !ENABLE_RELAY) return;

  unsigned long now = millis();

  switch (relayState) {
    case RELAY_IDLE:
      if (!wifiClient.connected()) {
        wifiClient.stop();
        CONSOLE.println("WIF: connecting..." RELAY_HOST);
        if (!wifiClient.connect(RELAY_HOST, RELAY_PORT)) {
          CONSOLE.println("WIF: connection failed");
          nextWifiClientCheckTime = now + 10000;
          return;
        }
        CONSOLE.println("WIF: connected!");
        relayState = RELAY_SENDING_HEADER;
        relayStateStartTime = now;
      }
      break;

    case RELAY_SENDING_HEADER: {
      String s = "GET / HTTP/1.1\r\n";
      s += "Host: " RELAY_USER "." RELAY_MACHINE "." RELAY_HOST ":";
      s += String(RELAY_PORT) + "\r\n";
      s += "Content-Length: 0\r\n";
      s += "\r\n\r\n";
      wifiClient.print(s);
      relayState = RELAY_RECEIVING;
      relayStateStartTime = now;
      buf.init();
      relayCmdBuffer = "";
      break;
    }

    case RELAY_RECEIVING:
      // Only read a few bytes per call to avoid blocking
      {
        int bytesRead = 0;
        const int maxBytesPerCall = 64; // or 32, or 128 if you want to push it
        while (wifiClient.available() && bytesRead < maxBytesPerCall) {
          char c = wifiClient.read();
          buf.push(c);
          bytesRead++;
          if (buf.endsWith("\r\n\r\n")) {
            relayState = RELAY_PROCESSING;
            relayStateStartTime = now;
            break;
          }
        }
      }
      // Timeout: if too much time has passed, close connection
      if (now - relayStateStartTime > 500) {
        relayState = RELAY_CLOSING;
      }
      break;

    case RELAY_PROCESSING:
      // Read command (non-blocking, just grab what's available)
      while (wifiClient.available()) {
        char ch = wifiClient.read();
        relayCmdBuffer += ch;
      }
      // Process command (could be split further if needed)
      processCmd("WIF", true, true, true);
      relayState = RELAY_RESPONDING;
      relayStateStartTime = now;
      break;

    case RELAY_RESPONDING: {
      String s = "HTTP/1.1 200 OK\r\n";
      s += "Host: " RELAY_USER "." RELAY_MACHINE "." RELAY_HOST ":";
      s += String(RELAY_PORT) + "\r\n";
      s += "Access-Control-Allow-Origin: *\r\n";
      s += "Content-Type: text/html\r\n";
      s += "Connection: close\r\n";
      s += "Content-length: ";
      s += String(cmdResponse.length());
      s += "\r\n\r\n";
      s += cmdResponse;
      wifiClient.print(s);
      relayState = RELAY_CLOSING;
      relayStateStartTime = now;
      break;
    }

    case RELAY_CLOSING:
      wifiClient.stop();
      relayState = RELAY_IDLE;
      break;
  }
}


// process WIFI input (App server)
// client (app) --->  server (robot)
void processWifiAppServer() {
  static enum {
    APP_IDLE,
    APP_HEADER,
    APP_BODY,
    APP_RESPONSE,
    APP_CLOSE
  } appState = APP_IDLE;

  static String buffer;
  static unsigned long stateStartTime = 0;
  static const unsigned long maxStateDuration = WIFI_DATA_TIMEOUT;  // ms
  
  if (!wifiFound || !ENABLE_SERVER) return;

  // reinitialize client if needed
  if (!client) client = server.available();

  switch (appState) {

    case APP_IDLE:
      if (client && client.connected()) {
        buffer = "";
        stateStartTime = millis();
        client.setTimeout(20);  // set short timeout
        appState = APP_HEADER;
      }
      break;

    case APP_HEADER: {
      if (millis() - stateStartTime > maxStateDuration) {
        appState = APP_CLOSE;
        break;
      }
        int bytesRead = 0;
        const int maxBytesPerCall = 32;
      while (client.available() && bytesRead < maxBytesPerCall) {
        char c = client.read();
        buffer += c;
        bytesRead++;
        if (buffer.endsWith("\r\n\r\n")) {
          cmd = "";
          stateStartTime = millis();
          appState = APP_BODY;
          break;
        }
      }
      break;
    }

    case APP_BODY: {
      if (millis() - stateStartTime > maxStateDuration) {
        appState = APP_CLOSE;
        break;
      }
      int bytesRead = 0;
      const int maxBytesPerCall = 32;
      while (client.available() && bytesRead < maxBytesPerCall) {
        char c = client.read();
        cmd += c;
        bytesRead++;
      }
      // Only process command if all expected data is received
      // (You may need to add a check here if you know the expected length)
      if (!client.available()) {
        processCmd("WIF", true, true, false);
        stateStartTime = millis();
        appState = APP_RESPONSE;
      }
      break;
    }

    case APP_RESPONSE: {
      if (!client.connected()) {
        appState = APP_CLOSE;
        break;
      }

      client.print(
        "HTTP/1.1 200 OK\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Type: text/html\r\n"
        "Connection: close\r\n"
      );
      client.print("Content-length: ");
      client.print(cmdResponse.length());
      client.print("\r\n\r\n");
      client.print(cmdResponse);

      stateStartTime = millis();
      appState = APP_CLOSE;
      break;
    }

    case APP_CLOSE:
      if (client) {
        client.stop();
        client = WiFiEspClient();  // reset client
      }
      appState = APP_IDLE;
      break;
  }
}


bool checkTimeViolation(unsigned long startTime, unsigned long maxTime, unsigned short breakPoint) {
  checkTime = millis() - startTime;
  if (checkTime > maxTime) {
    CONSOLE.print(" checkTimeViolation: maxTime violated! breakpoint: ");
    CONSOLE.print(breakPoint);
    CONSOLE.print("    duration: ");
    CONSOLE.print(checkTime);
    CONSOLE.print("    maxTime: ");
    CONSOLE.println(maxTime);
    return true;
  }
  return false;
}