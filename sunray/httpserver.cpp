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


// process WIFI input (relay client)
// a relay server allows to access the robot via the Internet by transferring data from app to robot and vice versa
// client (app) --->  relay server  <--- client (robot)
void processWifiRelayClient(){
  if (!wifiFound) return;
  if (!ENABLE_RELAY) return;
  if (!wifiClient.connected() || (wifiClient.available() == 0)){
    if (millis() > nextWifiClientCheckTime){   
      wifiClient.stop();
      CONSOLE.println("WIF: connecting..." RELAY_HOST);    
      if (!wifiClient.connect(RELAY_HOST, RELAY_PORT)) {
        CONSOLE.println("WIF: connection failed");
        nextWifiClientCheckTime = millis() + 10000;
        return;
      }
      CONSOLE.println("WIF: connected!");   
      String s = "GET / HTTP/1.1\r\n";
      s += "Host: " RELAY_USER "." RELAY_MACHINE "." RELAY_HOST ":";        
      s += String(RELAY_PORT) + "\r\n";
      s += "Content-Length: 0\r\n";
      s += "\r\n\r\n";
      wifiClient.print(s);
    } else return;
  }
  nextWifiClientCheckTime = millis() + 10000;     
  
  buf.init();                               // initialize the circular buffer   
  unsigned long timeout = millis() + 500;
    
  while (millis() < timeout) {              // loop while the client's connected    
    if (wifiClient.available()) {               // if there's bytes to read from the client,        
      char c = wifiClient.read();               // read a byte, then
      timeout = millis() + 200;
      buf.push(c);                          // push it to the ring buffer
      // you got two newline characters in a row
      // that's the end of the HTTP request, so send a response
      if (buf.endsWith("\r\n\r\n")) {
        cmd = "";
        while ((wifiClient.connected()) && (wifiClient.available()) && (millis() < timeout)) {
          char ch = wifiClient.read();
          timeout = millis() + 200;
          cmd = cmd + ch;
          gps.run();
        }
        CONSOLE.print("WIF:");
        CONSOLE.println(cmd);
        if (wifiClient.connected()) {
          processCmd("WIF", true,true,true);
          String s = "HTTP/1.1 200 OK\r\n";
            s += "Host: " RELAY_USER "." RELAY_MACHINE "." RELAY_HOST ":";        
            s += String(RELAY_PORT) + "\r\n";
            s += "Access-Control-Allow-Origin: *\r\n";              
            s += "Content-Type: text/html\r\n";              
            s += "Connection: close\r\n";  // the connection will be closed after completion of the response
            // "Refresh: 1\r\n"        // refresh the page automatically every 20 sec                                    
            s += "Content-length: ";
            s += String(cmdResponse.length());
            s += "\r\n\r\n";  
            s += cmdResponse;                      
            wifiClient.print(s);                                   
        }
        break;
      }
    }
  }
}



// process WIFI input (App server)
// client (app) --->  server (robot)
void processWifiAppServer()
{
  
  if (!wifiFound) return;
  if (!ENABLE_SERVER) return;

  if (hOdOr) {
    /* if (client){// && stateOp == OP_MOW){     //DO not ring my bell end leave me alone XD, FIDEL is busy. Fidel: Leave me Alone, I´m busy.
    CONSOLE.println("INFO: The Knocking and Pushing on the Door stopped.. for know, Run Hobbit... RUN! Hodor: Hodor?!");
    //stopClientTime = 0;
    //client.stop();
    hOdOrTimeOut = millis() + 100;
    } */
    //CONSOLE.print("millis > hodortimeout  ");CONSOLE.print(millis());CONSOLE.print(" > ");CONSOLE.println(hOdOrTimeOut);
    if (millis() > hOdOrTimeOut){
      hOdOr = false;
      CONSOLE.println("INFO: HODOR has fallen, but he´ll rise agaaain (if he must)! Hodor: hodorhodor");
    }
    CONSOLE.println("INFO: Hodor: Hodorhodor!!!!");
    return; //HooodooorHoooooodoooooor
  }

  if (wifiLastClientAvailableWait != 0){
    if (millis() < wifiLastClientAvailableWait) return;
    wifiLastClientAvailableWait = 0;
  }

  // listen for incoming clients    
  if (client){
    if (stopClientTime != 0) {
      if (millis() > stopClientTime){
        #ifdef VERBOSE 
          CONSOLE.println("app stopping client");
        #endif
        client.stop();
        stopClientTime = 0;                   
      }
      return;    
    }     
  }
  if (!client){
    //CONSOLE.println("client is NULL");
    client = server.available();      
  }
  if (client) {                               // if you get a client,
    #ifdef VERBOSE
      CONSOLE.println("New client");             // print a message out the serial port
    #endif
    //battery.resetIdle();
    buf.init();                               // initialize the circular buffer  
    if (client.available() != wifiLastClientAvailable) {
      wifiLastClientAvailable = client.available();
      wifiLastClientAvailableWait = millis() + 100;
      return;
    }
    unsigned long httpStartTime = millis();
    

    // The initial Cassandra connection to rover seems to take more than 1 seconds periodically. Unneccessary Data is transmitted from the client, it seems?
    // Then, on an unstable connection, Cassandra tries to reconnect every second or so to the rover spamming Data bytes without finishing and keeping the while loops busy for constant 1 second.
    // This second seems to be Cassandras Timeout?, but it will start again soon after -- THUS BLOCKING THE ROVER, ESPECIALLY LOW PERFORMANCE HARDWARE LIKE ALFRED`S BANANAPI.
    // The BananaPi will then loose its serial connection, doing CRC Errors and drops of data, sensors and the serialdriver. Mowmotor will be shut down, Rover might jerk, stop or act drunken.

    if (stateOp == OP_MOW) {
      hOdOrTimeOut = millis() + 50;                                  // Hodor is going to watch out for Orks and hold the door if he must!!
    } else hOdOrTimeOut = millis() + 1000;                                           // HODOR being gentle and patient
    
    String buffer = "";
    hOdOr = false;
    const unsigned long maxDuration = 50;
    
    while (client.connected()) {                              // loop while the client's connected
      //checkTimeViolation(httpStartTime, maxDuration, 1);
      
      if (millis() > hOdOrTimeOut){                  // Hodor is going to watch out for Orks and hold the door if he must!!
        hOdOr = true; //XD
        //hOdOrTimeOut = millis() + 5000;                            // Hodor will hold the door for 3 seconds XD
        CONSOLE.println("INFO: HODOR is going to hold the door to save your life!");
        //CONSOLE.print("INFO: act remaining HodorTimeOut: "); CONSOLE.println(millis() - hOdOrTimeOut);
        break;
      }

      if (client.available()) {               // if there's bytes to read from the client,        
        //checkTimeViolation(httpStartTime, maxDuration, 2);
                
        char c = client.read();               // read a byte, then
        //checkTimeViolation(httpStartTime, maxDuration, 3);
        if (stateOp != OP_MOW) hOdOrTimeOut = millis() + 100;
        
        buf.push(c);                          // push it to the ring buffer
        //checkTimeViolation(httpStartTime, maxDuration, 4);
        // you got two newline characters in a row
        // that's the end of the HTTP request, so send a response
        
        buffer = buffer + c;
        //checkTimeViolation(httpStartTime, maxDuration, 5);
        //CONSOLE.print("Buffer input: "); CONSOLE.println(buffer);

        if (buf.endsWith("\r\n\r\n")) {
          cmd = "";
          CONSOLE.print("Buffer input: "); CONSOLE.println(buffer);
          CONSOLE.print("Client read data startTime: "); CONSOLE.println(millis()-httpStartTime);
          
          while (client.connected() && client.available() && !hOdOr) {
            //checkTimeViolation(httpStartTime, maxDuration, 6);
            if (millis() > hOdOrTimeOut){
              hOdOr = true; //XD
              //hOdOrTimeOut = millis() + 5000;                            // Hodor will hold the door for 3 seconds XD
              CONSOLE.println("INFO: HODOR is holding the door to safe your life again!");
              //CONSOLE.print("INFO: act HodorTimeOut: "); CONSOLE.println(millis() - hOdOrTimeOut);
              break;
            }
            char ch = client.read();
            //checkTimeViolation(httpStartTime, maxDuration, 7);
            if (stateOp != OP_MOW) hOdOrTimeOut = millis() + 100;

            cmd = cmd + ch;
            //checkTimeViolation(httpStartTime, maxDuration, 8);
          }

          if (hOdOr) break;
          
          CONSOLE.print("Client read data endTime: "); CONSOLE.println(millis()-httpStartTime);
          CONSOLE.print("Client cmd data: ");CONSOLE.println(cmd);

          if (millis() > wifiVerboseStopTime){
            wifiVerboseStopTime = 0;
          }    
          if (wifiVerboseStopTime != 0){          
            //CONSOLE.print("WIF:");
            //CONSOLE.println(cmd);            
          }
          if (client.connected()) {
            //checkTimeViolation(httpStartTime, maxDuration, 9);
            CONSOLE.print("Client process cmd startTime: "); CONSOLE.println(millis()-httpStartTime);
            processCmd("WIF",true,true, (wifiVerboseStopTime != 0));
            //checkTimeViolation(httpStartTime, maxDuration, 10);

            client.print(
              "HTTP/1.1 200 OK\r\n"
              "Access-Control-Allow-Origin: *\r\n"              
              "Content-Type: text/html\r\n"              
              "Connection: close\r\n"  // the connection will be closed after completion of the response
              // "Refresh: 1\r\n"        // refresh the page automatically every 20 sec                        
              );
            //checkTimeViolation(httpStartTime, maxDuration, 11);
            client.print("Content-length: ");
            client.print(cmdResponse.length());
            //checkTimeViolation(httpStartTime, maxDuration, 12);
            client.print("\r\n\r\n");                      // <-- this is the [1b blob data]  
            client.print(cmdResponse);
            //checkTimeViolation(httpStartTime, maxDuration, 13);
            CONSOLE.print("Client process cmd endTime: "); CONSOLE.println(millis()-httpStartTime);

            if (DEBUG_HTTPSERVER) {
              CONSOLE.print("Content-length: ");
              CONSOLE.println(cmdResponse.length());
              CONSOLE.print("Content: ");
              CONSOLE.print("\r\n\r\n");                    // <-- this is the [1b blob data]    
              CONSOLE.println(cmdResponse);
            }
          }

          //if (hOdOr){
          //  hOdOrTimeOut = millis() + 5000;
          //}

          break;
        }
      }
      //Stop waiting
      if (hOdOr) {
        client.stop();
        client.setTimeout(25);
        hOdOrTimeOut = millis() + 100;
        break;
      }
    }
    
    // give the web browser time to receive the data
    stopClientTime = millis() + 100;
    unsigned long httpEndTime = millis();    
    int httpDuration = httpEndTime - httpStartTime;
    
    if (DEBUG_HTTPSERVER){

      CONSOLE.print("HODOR: "); CONSOLE.println(hOdOr);
      CONSOLE.print("CLIENT: "); CONSOLE.println(client.connected());
      CONSOLE.print("HTTP server duration: ");
      CONSOLE.println(httpDuration);
      CONSOLE.println(" -------------------------------------------- ");
    }
  
    if (httpDuration > 500){
      wifiVerboseStopTime = millis() + 30000;
      CONSOLE.print("HTTP WARN: high server duration: ");
      CONSOLE.println(httpDuration);
    }

    //gps.run(); //only run at end once?
    //delay(10);
    // close the connection
    //client.stop();
    //CONSOLE.println("Client disconnected");
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