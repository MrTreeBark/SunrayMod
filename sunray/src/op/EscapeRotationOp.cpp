// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../map.h"
#include "../../config.h"


String EscapeRotationOp::name(){
    return "EscapeRotation";
}

void EscapeRotationOp::begin(){
    // obstacle avoidance
    driveReverseStopTime = millis() + (ESCAPE_FORWARD_WAY/OBSTACLEAVOIDANCESPEED*1000);
    resetMotion();
}


void EscapeRotationOp::end(){
}


void EscapeRotationOp::run(){
    battery.resetIdle();

    motor.setLinearAngularSpeed(OBSTACLEAVOIDANCESPEED,0,false);				
																				
	if ((DISABLE_MOW_MOTOR_AT_OBSTACLE) && (motor.switchedOn)) {	//MrTree
      CONSOLE.println("EscapeRotationOp:: switch OFF mowmotor");			//MrTree
	  motor.setMowState(false);  																	  	
	}  																                                   																					
    if (millis() > driveReverseStopTime){
        CONSOLE.println("driveReverseStopTime");
        //motor.setLinearAngularSpeed(0,0,false);
        //motor.stopImmediately(false); 
        driveReverseStopTime = 0;
        if (detectLift()) {
            CONSOLE.println("error: lift sensor!");
            stateSensor = SENS_LIFT;
            changeOp(errorOp);
            return;
        }
        if (maps.isDocking()){
            CONSOLE.println("continue docking");
            // continue without obstacles
            changeOp(*nextOp, false);    // continue current operation
        } else {
            CONSOLE.println("continue operation with modified obstacle");
            maps.addObstaclePosition();              
            //Point pt;
            //if (!maps.findObstacleSafeMowPoint(pt)){
            //    changeOp(dockOp); // dock if no more (valid) mowing points
            //} else changeOp(*nextOp);    // continue current operation
            changeOp(*nextOp, false);    // continue current operation
        }
    }
    if (OBSTACLE_CHAINING) {
        if (!robotShouldWait() && !detectObstacle() && !detectObstacleRotation()){
            //if (ESCAPE_LAWN) detectLawn(); //MrTree                              
            //
        }
    }
}



void EscapeRotationOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeRotationOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}


