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


String EscapeLawnOp::name(){
    return "EscapeLawn";
}

void EscapeLawnOp::begin(){
	
	//reverse from high grass																								//would be better to use .maps and reverse to a set gpspoint.........
    if (escapeLawnCounter == 0)	escapeLawnStartTime = millis();   															//set triggered time on entrance if escapeLawnCounter successfully resets	
	escapeLawnCounter++;																									//iterate counter		
	if (((escapeLawnStartTime + ESCAPELAWNTIMER) < millis()) && (escapeLawnCounter <= MAXRETRY)) escapeLawnCounter = 0;		//reset counter if ESCAPELAWNTIME succeded without too many triggers of EscapeLawn
	driveReverseStopTime = millis() + (escapeLawnDistance/ESCAPELAWNSPEED*1000);     										//MrTree just take a deadtime of 500ms to compensate deadtimes.. workaround...
    escapeLawnWaitTime	= 0;
	gpsObstacleNotAllowedTime = millis() + NO_GPS_OBSTACLE_DEADTIME;
	gpsObstacleNotAllowed = true;
	CONSOLE.print("escapeLawnCounter = ");
	CONSOLE.println(escapeLawnCounter);	
	CONSOLE.print("escapeLawnDistance = ");
    CONSOLE.println(escapeLawnDistance);
	CONSOLE.print("driveReverseStopTime = ");
    CONSOLE.println((escapeLawnDistance/ESCAPELAWNSPEED*1000));
	resetMotion();
	//if (!motor.switchedOn) {
	//	  CONSOLE.println("EscapeLawnOp:: Mow motor was switched off and is switched On again!");
	//	  motor.setMowState(true);	
	//}
}


void EscapeLawnOp::end(){
}


void EscapeLawnOp::run(){
    battery.resetIdle();		 																																																	
	if (escapeLawnCounter > MAXRETRY) {
		if (MAXRETRYOBSTACLE && OBSTACLE_AVOIDANCE){
		  CONSOLE.println("EscapeLawnOp:: too many retries in configured time (ESCAPELAWNTIMER), triggering Obstacleavoidance");
		  escapeLawnCounter = 0;
		  escapeFinished = true;	      
          changeOp(escapeReverseOp, false);
		  return;
		} else {
		  CONSOLE.println("EscapeLawnOp:: Error: too many retries in configured time (ESCAPELAWNTIMER)");
		  stateSensor = SENS_OBSTACLE;            
          escapeLawnCounter = 0;
		  escapeFinished = true;
		  changeOp(errorOp);
		  return;
		}
	} else {
		//if (!motor.switchedOn) {
		  //CONSOLE.println("EscapeLawnOp::run Mow motor was switched off and is switched On again!");
		  //motor.setMowState(true);	
		//}
		if (!motor.switchedOn) {
		  CONSOLE.println("EscapeLawnOp::run Mow motor was switched off canceling Operation!");
		   //motor.setMowState(true);
		   escapeFinished = true; 							//significant change?
		   changeOp(*nextOp, false);						// continue current operation
		   return;	  	
		}
       //if (millis() < motor.motorMowSpinUpTime + MOWSPINUPTIME) {
         //if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
	     //motor.setLinearAngularSpeed(0,0,false);
		 //CONSOLE.println("EscapeLawnOp::run trying to wait for mowmotor");
		 //driveReverseStopTime = millis() + (escapeLawnDistance/ESCAPELAWNSPEED*1000);
       //}
	
	   if (millis() > driveReverseStopTime){
			if (driveReverseStopTime != 0) CONSOLE.println("EscapeLawnOp: driveReverseStopTime");
			motor.setLinearAngularSpeed(0,0,false);	
			if ((ESCAPELAWNWAITTIME > 0) && (escapeLawnCounter > 1 ) && (driveReverseStopTime != 0)){
				escapeLawnWaitTime = millis()+ESCAPELAWNWAITTIME;
				CONSOLE.println("EscapeLawnOp: Waiting for mow motor to recover!");
			}
			driveReverseStopTime = 0;
			if (detectLift()) {
				CONSOLE.println("error: lift sensor!");
				stateSensor = SENS_LIFT;
				changeOp(errorOp);
				return;
			} else {
				if (millis() > escapeLawnWaitTime) {
				  CONSOLE.println("EscapeLawnOp: high lawn, continue operation without virtual obstacle");  //MrTree
				  CONSOLE.println("EscapeLawnOp: triggering retryslow!");
				  escapeFinished = true;
				  gpsObstacleNotAllowed = true;
				  gpsObstacleNotAllowedTime = millis() + NO_GPS_OBSTACLE_DEADTIME;
				  motor.motorMowStallFlag = false; //MrTree reset flag if triggered by rpm stall
				  if (CHANGE_SPEED_SET) motor.retryslow = true;
				  motor.retrySlowTime = millis()+RETRYSLOWTIME; 	//trigger slow retry, set keepSlowTime 
				  changeOp(*nextOp, false);						// continue current operation
				}
			}
		} else {
			motor.setLinearAngularSpeed(-ESCAPELAWNSPEED,0,false);
		}
    }
}



void EscapeLawnOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeLawnOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}


