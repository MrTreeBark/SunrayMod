// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"
#include "../../StateEstimator.h"
#include "../../config.h" //MrTree

String EscapeForwardOp::name(){
    return "EscapeForward";
}

void EscapeForwardOp::begin(){
    // rotate stuck avoidance
    if (escapeForwardCounter == 0)	escapeForwardStartTime = millis();   					                    // set triggered time on entrance if escapeForwardCounter successfully resets	
	escapeForwardCounter++;                                                                                     //MrTree iterate counter
    if (escapeForwardStartTime + 10000 < millis() && escapeForwardCounter < 5) escapeForwardCounter = 0;		//reset counter if escapeForward succeded without too many triggers in given time
    driveForwardStopTime = millis() + (ESCAPE_FORWARD_WAY/OBSTACLEAVOIDANCESPEED*1000); 	                    //MrTree just add a constant time to compensate offsets?
    resetMotion();
}


void EscapeForwardOp::end(){
}


void EscapeForwardOp::run(){
    battery.resetIdle();
	if (CHANGE_OBSTACLE_ROTATION){
        if (escapeForwardCounter == 3) {
	        CONSOLE.println("EscapeForwardOp:: too many retries for escapeForward op, assuming obstacle in front: changeOp-->escapeReverseOp");	      
            changeOp(escapeReverseOp, true);
            return;	
	    } 
        if (escapeForwardCounter == 4) {
		    CONSOLE.println("EscapeForwardOp:: Skipping to nextOp");          //MrTree dont´t know how how to solve opchain going to escapereverse and then directly to mowop :( ..   )
            changeOp(*nextOp);
		    return;
	    }
        if (escapeForwardCounter > 4) {
		    CONSOLE.println("EscapeForwardOp:: Error: too many retries in configured time");
		    stateSensor = SENS_OBSTACLE;            
            changeOp(errorOp);
		    escapeForwardCounter = 0;
		    return;
	    }
    }										
    if (millis() > driveForwardStopTime){
        CONSOLE.println("driveForwardStopTime");
        motor.setLinearAngularSpeed(0,0,false);
        driveForwardStopTime = 0;
        CONSOLE.println("adding modified obstacle");
        maps.addObstaclePosition();  
        changeOp(*nextOp);    // continue current operation              
    } else {
        motor.setLinearAngularSpeed(OBSTACLEAVOIDANCESPEED,0,false);	    //MrTree						
        if ((DISABLE_MOW_MOTOR_AT_OBSTACLE) && (motor.switchedOn)) {	//MrTree
            CONSOLE.println("EscapeForwardOp:: Switch Off mow");		//MrTree
	        motor.setMowState(false);  																	  	
	    }  																							 
    }
    if (OBSTACLE_CHAINING) {
        if (!robotShouldWait() && !detectObstacle() && !detectObstacleRotation()){
            //if (ESCAPE_LAWN) detectLawn(); //MrTree                              
            //
        }
    }
}

void EscapeForwardOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeForwardOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}
