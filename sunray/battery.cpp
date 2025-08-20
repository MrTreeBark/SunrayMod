 // Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "battery.h"
#include "config.h"
#include "helper.h"
#include "motor.h"
#include "robot.h"
#include "buzzer.h"
#include <Arduino.h>

// lithium akkus sollten bis zu einem bestimmten wert CC also constant current geladen werden 
// und danach mit CV constant voltage
// um die lebensdauer zu erhöhen kann man die max spannung herabsetzen
// wir verwenden im Ardumower ein 7S (7 * 4.2V)
// voll geladen ist dann bei 29.4V schluss
// wenn man z.B. die spannung von 4.2V pro zelle auf 4.1V herab setzt (85–90% charged) kann man die lebensdauer 
// verdoppeln - das gleiche gilt bei der endladung

// lithium cells
// ardumower:  Sony US18650 VTC5, 7 cells in series, nominal volage 3.6v
// alfred:   Samsung INR18650-15M, 7 cells in series, nominal voltage 3.6v 

void Battery::begin() {
  startupPhase = 0;
  nextBatteryTime = 0;
  nextCheckTime = 0;
  nextEnableTime = 0;
  reEnableTime = 0;
  batteryVoltageSlopeLowCounter = 0;
  nextSlopeTime = 0;
  timeMinutes = 0;
  chargingVoltage = 0;
  chargingPower = 0;
  chargingCompletedDelay = 0;
  batteryVoltage = 0;
  chargerConnectedState = false;
  badChargerContactState = false;
  chargingCompleted = false;
  chargingEnabled = true;

  docked = false;

  batMonitor = true; // monitor battery and charge voltage?
  batGoHomeIfBelow = GO_HOME_VOLTAGE; // 21.5  drive home voltage (Volt)
  batSwitchOffIfBelow = BAT_SWITCH_OFF_VOLTAGE; // switch off battery if below voltage (Volt)
  batSwitchOffIfIdle = BAT_SWITCH_OFF_IDLE_TIME; // switch off battery if idle (seconds)
  // The battery will charge if both battery voltage is below that value and charging current is above that value.
  batFullCurrent = BAT_FULL_CURRENT; // 0.2  current flowing when battery is fully charged (A)
  batFullVoltage = BAT_FULL_VOLTAGE; // 28.7  voltage when battery is fully charged (we charge to only 90% to increase battery life time)
  enableChargingTimeout = 60 * BAT_CHARGE_TIMEOUT; // if battery is full, wait this time before enabling charging again (seconds)
  batteryVoltage = 0;
  batteryVoltageLast = 0;
  batteryVoltageSlope = 0;
  chargingVoltBatteryVoltDiff = 0;
  switchOffByOperator = false;
  switchOffAllowedUndervoltage = BAT_SWITCH_OFF_UNDERVOLTAGE;
  switchOffAllowedIdle = BAT_SWITCH_OFF_IDLE;

  enableCharging(false);
  resetIdle();
}


// controls charging relay
void Battery::enableCharging(bool flag) {
  if (chargingEnabled == flag) return;
  CONSOLE.print("enableCharging ");
  CONSOLE.println(flag);
  chargingEnabled = flag;
  batteryDriver.enableCharging(flag);
  nextPrintTime = 0;
}

bool Battery::chargerConnected(){
  return chargerConnectedState;  
}

bool Battery::isDocked(){
  return docked;
}

void Battery::setIsDocked(bool state){
  CONSOLE.print("battery.setIsDocked ");
  CONSOLE.println(state);
  docked = state;
}

bool Battery::badChargerContact(){
  return badChargerContactState;
}
 
bool Battery::chargingHasCompleted(){
  return chargingCompleted;
}
 

bool Battery::shouldGoHome(){
  if (startupPhase < 2) return false;  
  return (batteryVoltage < batGoHomeIfBelow);
}

bool Battery::underVoltage(){
  if (startupPhase < 2) return false;
  return (batteryVoltage < batSwitchOffIfBelow);
}

void Battery::resetIdle(){
  switchOffTime = millis() + batSwitchOffIfIdle * 1000;    
}

void Battery::switchOff(){
  CONSOLE.println("switching-off battery by operator...");
  switchOffByOperator = true;
}

void Battery::run() {
  if (startupPhase == 0) {
    // give some time to establish communication to external hardware etc.
    nextBatteryTime = millis() + 2000;
    startupPhase++;
    return;
  }
  if (millis() < nextBatteryTime) return;
  nextBatteryTime = millis() + 50;
  if (startupPhase == 1) startupPhase = 2;

  // charging voltage, use new voltage directly if e.g sudden charger connection and don’t filter the initial value of jump
  float voltage = batteryDriver.getChargeVoltage();
  if (abs(chargingVoltage - voltage) > 10) {
    chargingVoltage = voltage;
    chargingVoltBatteryVoltDiff = 0;
  }
  chargingVoltage = 0.9 * chargingVoltage + 0.1 * voltage;

  // battery voltage, use new voltage directly if e.g sudden charger connection and don’t filter the initial value of jump
  voltage = batteryDriver.getBatteryVoltage();
  if (abs(batteryVoltage - voltage) > 10) {
    batteryVoltage = voltage;
    batteryVoltageLast = voltage;
    chargingVoltBatteryVoltDiff = 0;
  }

  // lower filter if charger is connected (may not be necessary)
  float w = 0.995;
  if (chargerConnectedState) w = 0.9;
  batteryVoltage = w * batteryVoltage + (1 - w) * voltage;

  // difference of charging voltage and battery voltage
  chargingVoltBatteryVoltDiff = 0.99 * chargingVoltBatteryVoltDiff + 0.01 * (chargingVoltage - batteryVoltage);

  // current
  chargingCurrent = 0.9 * chargingCurrent + 0.1 * batteryDriver.getChargeCurrent();

  // charging power
  chargingPower = chargingVoltage * chargingCurrent;

  if (!chargerConnectedState) {
    //if (chargingVoltage > 7) { //try out over battery voltage
    if (chargingVoltage > batteryVoltage) { //try out over battery voltage
      chargerConnectedState = true;
      CONSOLE.print("CHARGER CONNECTED chgV=");
      CONSOLE.print(chargingVoltage);
      CONSOLE.print(" batV=");
      CONSOLE.println(batteryVoltage);
      buzzer.sound(SND_OVERCURRENT, true);
    }
  }

  if (millis() >= nextCheckTime) {
    nextCheckTime = millis() + 5000;
    if (chargerConnectedState) {
      //if (chargingVoltage <= 5) { try out under battery power off voltage
      if (chargingVoltage <= BAT_SWITCH_OFF_VOLTAGE) { //try out under battery undervoltage
        chargerConnectedState = false;
        nextEnableTime = millis() + 5000; // reset charging enable time
        CONSOLE.print("CHARGER DISCONNECTED chgV=");
        CONSOLE.print(chargingVoltage);
        CONSOLE.print(" batV=");
        CONSOLE.println(batteryVoltage);
      }
    }
    timeMinutes = (millis() - chargingStartTime) / 1000 / 60;
    if (underVoltage()) {
      CONSOLE.print("SWITCHING OFF (undervoltage) batV=");
      CONSOLE.print(batteryVoltage);
      CONSOLE.print("<");
      CONSOLE.println(batSwitchOffIfBelow);
      buzzer.sound(SND_OVERCURRENT, true);
      if (switchOffAllowedUndervoltage) batteryDriver.keepPowerOn(false);
    } else if ((millis() >= switchOffTime) || (switchOffByOperator)) {
      CONSOLE.println("SWITCHING OFF (idle timeout)");
      buzzer.sound(SND_OVERCURRENT, true);
      if ((switchOffAllowedIdle) || (switchOffByOperator)) batteryDriver.keepPowerOn(false);
    } else batteryDriver.keepPowerOn(true);

    // battery voltage slope
    float w = 0.999;
    batteryVoltageSlope = w * batteryVoltageSlope + (1 - w) * (batteryVoltage - batteryVoltageLast) * 60.0 / 5.0; // 5s => 1min
    batteryVoltageLast = batteryVoltage;

    // Check for significant voltage difference
    // if the charging voltage is significantly lower than the battery voltage, we have a bad contact
    // this can happen if the charger is not connected properly or the contacts are dirty
    // we will not charge the battery in this case
    badChargerContactState = false;
    if (millis() >= nextSlopeTime) {
      nextSlopeTime = millis() + 60000; // 1 minute
      //badChargerContactState = false;
      if (chargerConnectedState) {
        if (!chargingCompleted) {
          //if (chargingVoltBatteryVoltDiff < CHG_VOLT_DIFF && chargingCurrent < CHG_CURRENT) {
          if (batteryVoltage < batFullVoltage && chargingCurrent < CHG_CURRENT) {
            //possibly we could change the charger connected state aswell
            enableCharging(false); //open the relais for a true charging contact measurement
            // check for slope
            //if (batteryVoltageSlope < 0){
            badChargerContactState = true;
            CONSOLE.print("CHARGER BAD CONTACT chgV=");
            CONSOLE.print(chargingVoltage);
            CONSOLE.print(" chargingCurrent=");
            CONSOLE.print(chargingCurrent);
            CONSOLE.print(" batV=");
            CONSOLE.print(batteryVoltage);
            CONSOLE.print(" diffV=");
            CONSOLE.print(chargingVoltBatteryVoltDiff);
            CONSOLE.print(" slope(v/min)=");
            CONSOLE.println(batteryVoltageSlope);
          }
        }
      }
      if (abs(batteryVoltageSlope) < BAT_FULL_SLOPE) {
        batteryVoltageSlopeLowCounter = min(10, batteryVoltageSlopeLowCounter + 1);
      } else {
        batteryVoltageSlopeLowCounter = 0; //max(0, batteryVoltageSlopeLowCounter - 1);
      }
    }
  }

  if (millis() > nextEnableTime) {
    nextEnableTime = millis() + 5000;

    //this is very important: mower doesnt have the physical hardware to detect if it is connected or not when the relais is closed! We will "see" the battery voltage on the charger input.
    //so in my understanding "chargingCompleted = ((chargingCurrent <= batFullCurrent) || (batteryVoltage >= batFullVoltage));" seems odd but is necessary to force a relais open and then measure
    //if the charger is actually connected or not! We should make 2 separate checks for a truly full battery or a bad contact/not engaged charger.
    if (chargerConnectedState) {
      // charger in connected state and relais closed
      if (chargingEnabled) {
       
        // True bat full status, enabling longer timeout for next charge session if configured
        if (chargingCompletedDelay > 5) { // chargingCompleted check first after 6 * 5000ms = 30sec.
          chargingCompleted = ((chargingCurrent <= batFullCurrent) && (batteryVoltage >= batFullVoltage)); //charging is finished when voltage is high AND current is low, but we need this to force a relais open and then measure if the charger is actually connected or not!
        } else {
          chargingCompletedDelay++;
        }
        if (chargingCompleted) {
          // stop charging
          nextEnableTime = millis() + 1000 * enableChargingTimeout; // check charging current again in BAT_CHARGE_TIMEOUT minutes
          chargingCompleted = true;
          if (!CHG_NEVER_DISCONNECT) enableCharging(false);
        }

        /* // Check for bad charger contact and force a open relais to read the voltage of mower contacts
        if (chargingCompletedDelay > 5) { // chargingCompleted check first after 6 * 5000ms = 30sec.
          chargingCompleted = ((chargingCurrent <= batFullCurrent) || (batteryVoltage >= batFullVoltage)); //charging is finished when voltage is high AND current is low, but we need this to force a relais open and then measure if the charger is actually connected or not!
        }

        if (chargingCompleted) {
          // stop charging
          nextEnableTime = millis() + 1000 * BAT_CHARGE_TIMEOUT; // check charging current again in BAT_CHARGE_TIMEOUT minutes
          chargingCompleted = true;
          if (!CHG_NEVER_DISCONNECT) enableCharging(false);
        } */
        
      } else {
        // start charging
        if (!badChargerContactState) { //keep relais off if badCharger was detected
          enableCharging(true);
          chargingStartTime = millis();
        }
      }
    } else {
      // reset to avoid direct undocking after docking
      chargingCompleted = false;
      chargingCompletedDelay = 0; // reset chargingCompleteted delay counter
    }
  }

  reEnableTime = (nextEnableTime - millis()) / 60000; // (minutes) public for ChargeOp or other functions

  if (DEBUG_BATTERY) {
    if (millis() >= nextPrintTime) {
      nextPrintTime = millis() + DEBUG_OUTPUT_TIME;
      CONSOLE.print("chgComp=");
      CONSOLE.print(chargingCompleted);
      CONSOLE.print(" chgConn=");
      CONSOLE.print(chargerConnected());
      CONSOLE.print(" chgEn=");
      CONSOLE.print(chargingEnabled);
      CONSOLE.print(" chgTime=");
      CONSOLE.print(timeMinutes);
      CONSOLE.print(" badChgConnS=");
      CONSOLE.print(badChargerContactState);
      CONSOLE.print(" chg: ");
      CONSOLE.print(chargingVoltage);
      CONSOLE.print(" V  ");
      CONSOLE.print(chargingCurrent);
      CONSOLE.print(" A ");
      CONSOLE.print(" chgPower=");
      CONSOLE.print(chargingPower);
      CONSOLE.print(" W ");
      CONSOLE.print(" diff=");
      CONSOLE.print(chargingVoltage - batteryVoltage);
      CONSOLE.print(" V  ");
      CONSOLE.print(" | bat: ");
      CONSOLE.print(batteryVoltage);
      CONSOLE.print(" V  ");
      CONSOLE.print(" batslope: ");
      CONSOLE.print(batteryVoltageSlope);
      CONSOLE.print(" V  ");
      CONSOLE.println();
    }
  }
}
