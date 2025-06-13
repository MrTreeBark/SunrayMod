#ifndef HTTPSERVER_H
#define HTTPSERVER_H

#include <Arduino.h>


void processWifiRelayClient();
void processWifiAppServer();
bool checkTimeViolation(unsigned long, unsigned long, unsigned short);

#endif
