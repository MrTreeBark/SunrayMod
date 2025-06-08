// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

String WaitOp::name(){
    return "Wait";
}

void WaitOp::begin(){
    waitStartTime = millis();
    CONSOLE.println("WaitOp::begin - INFO: waiting!");
    buzzer.sound(SND_WAIT, true);          
    motor.setLinearAngularSpeed(0,0, false); 
}


void WaitOp::end(){
}

void WaitOp::run(){
    battery.resetIdle();
    
    if (millis() > waitStartTime + waitTime){
        if (waitTime == MOWSPINUPTIME) {
            motor.waitSpinUp = false;
            CONSOLE.println("WaitOp::run - INFO: waiting for mowmotor done!");
        }
        buzzer.sound(SND_READY, true);
        changeOp(*nextOp, false);
    }     
}


