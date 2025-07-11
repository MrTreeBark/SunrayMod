// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "LineTracker.h"
#include "robot.h"
#include "StateEstimator.h"
#include "helper.h"
#include "pid.h"
#include "src/op/op.h"
#include "Stats.h"
#include "events.h"

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;

float targetDist = 0;     //MrTree
float lastTargetDist = 0; //MrTree
float pathLength = 0;

float setSpeed = 0.5; //externally controlled (app) linear speed (m/s)
float CurrSpeed = 0;  //actual used speed from motor.linearSpeedSet
float CurrRot = 0;  //actualk rotation from mow.motor
float linear = 0;
float angular = 0;

float x_old = 0;
float y_old = 0;
float x_new = 0;
float y_new = 0;
Point lastPoint;
Point target;
Point lastTarget;

//Point targetTracker;
//Point lastTargetTracker;

bool mow = false;
bool trackslow_allowed = false;
bool straight = false;
bool transition = false;
bool shouldRotate = false;                      //MrTree
bool shouldRotatel = false;                     //MrTree
bool angleToTargetFits = false;
bool angleToTargetPrecise = true;
bool langleToTargetFits = false;
bool targetReached = false;
bool stateKidnapped = false;
bool dockTimer = false;                         //MrTree
bool unDockTimer = false;
bool oneTrigger = false;                        //MrTree
bool printmotoroverload = false;
float trackerDiffDelta = 0;
float targetDelta = 0;
float distToPath = 0;
float lineDist = 0;

unsigned long reachedPointBeforeDockTime = 0;   //MrTree
bool allowDockRotation = true;                //MrTree: disable rotation on last dockingpoint

void purePursuitTracker() {
  if (DEBUG_TRACKER) {
    CONSOLE.println("---");
    CONSOLE.println("Purepursuit tracker, data...");
  }
  const float lookaheadSpeedFactor = 8; //(val) for given speed, lookAhead point will be multiplied with this factor. If 1, lookAhead point will equal to speed if not constrained
  const float maxLookahead = MOTOR_MAX_SPEED * lookaheadSpeedFactor; // 4.0; //(m) 
  const float minLookahead = 2; //(m)
  const float turnGain = 2.5; //this should be calculated from rotation ramp max to get maxrotation on 180° or 90° heading error
  const float maxTurnSpeed = ROTATION_RAMP_MAX * DEG_TO_RAD;
  const float minTurnSpeed = ROTATION_RAMP_MIN * DEG_TO_RAD;
  const float angleStopThreshold = TARGETFITS_ANGLE * DEG_TO_RAD;
  const float anglePrecise = ANGLEPRECISE * DEG_TO_RAD;
  const float minSpeed = MOTOR_MIN_SPEED;
  //const float stopDistance = 0.2;
  const float wheelbaseVirtual = 1 ;//WHEEL_BASE_CM / 100.0;

  float headingError = trackerDiffDelta;
  float distToTarget = targetDist;
  float lastDistToTarget = lastTargetDist;
  float speed = linear;
  float turn = 0;
  float rampTurn = 0;

  if (DEBUG_TRACKER) {
    CONSOLE.print("speed input: "); CONSOLE.println(speed);
  }

//rotation only if too far
/*   if (fabs(headingError) > angleStopThreshold) {
    speed = 0;
    
    if (DEBUG_TRACKER) {
      CONSOLE.println("rotation mode active");
      CONSOLE.print("abs heading error: "); CONSOLE.println(abs(headingError) * RAD_TO_DEG);
    }

    rampTurn = safeMap(fabs(headingError), anglePrecise, angleStopThreshold, minTurnSpeed, maxTurnSpeed);
    rampTurn = constrain(rampTurn, minTurnSpeed, maxTurnSpeed);
        
    turn = rampTurn * ((headingError < 0) ? -1.0f : 1.0f);
    if (DEBUG_TRACKER) {
      CONSOLE.print("final turn: "); CONSOLE.println(turn);
    }
    angular = turn;
    linear = speed;
    return;
  } */

  // --- dynamic Lookahead 
  float lookahead = speed * lookaheadSpeedFactor;
  if (DEBUG_TRACKER) {
    CONSOLE.print("dynamic lookahead: "); CONSOLE.println(lookahead);
  }
  lookahead = constrain(lookahead, minLookahead, maxLookahead);
  if (DEBUG_TRACKER) {
    CONSOLE.print("lookahead constrained: "); CONSOLE.println(lookahead);
  }
  // --- linedirection of path
  float dx = target.x() - lastTarget.x();
  float dy = target.y() - lastTarget.y();
  if (DEBUG_TRACKER) {
    CONSOLE.print("linedirection calc ---  dx: "); CONSOLE.print(dx); CONSOLE.print(" dy: "); CONSOLE.println(dy);
  }
  float lineLen = sqrtf(dx * dx + dy * dy); //can use pathLength here
  if (DEBUG_TRACKER) {
    CONSOLE.print("calced linelen: "); CONSOLE.println(lineLen);
  }
  if (!isfinite(dx) || !isfinite(dy) || !isfinite(lineLen)) {
    if (DEBUG_TRACKER) CONSOLE.println("no valid line coordinates - tracking stopped");
    linear = 0;
    angular = 0;
    return;
  }

  if (lineLen < 1.0) {
    if (DEBUG_TRACKER) CONSOLE.println("WARNING: line too short or undefined - using angle to target for direction");
    dx = cos(targetDelta);
    dy = sin(targetDelta);
    lineLen = 1.0f;
  }

  float ux = dx / lineLen;
  float uy = dy / lineLen;
  if (DEBUG_TRACKER) {
    CONSOLE.print("line direction ux/uy: "); CONSOLE.print(ux); CONSOLE.print(" / "); CONSOLE.println(uy);
  }
  // --- project robot on line
  float px = stateX - lastTarget.x();
  float py = stateY - lastTarget.y();
  if (DEBUG_TRACKER) {
    CONSOLE.print("px: "); CONSOLE.print(px); CONSOLE.print(" py: "); CONSOLE.println(py);
  }
  float projLen = px * ux + py * uy;
  if (DEBUG_TRACKER) {
    CONSOLE.print("projected length on path: "); CONSOLE.println(projLen);
  }
  // --- Lookahead-Punkt auf Linie ---
  float lx = lastTarget.x() + (projLen + lookahead) * ux;
  float ly = lastTarget.y() + (projLen + lookahead) * uy;
  if (DEBUG_TRACKER) {
    CONSOLE.print("lookahead point: lx / ly = "); CONSOLE.print(lx); CONSOLE.print(" / "); CONSOLE.println(ly);
  }
  float angleToLookahead = pointsAngle(stateX, stateY, lx, ly);
  if (DEBUG_TRACKER) {
    CONSOLE.print("angleToLookahead: "); CONSOLE.println(angleToLookahead * RAD_TO_DEG);
  }
  float angleDiff = scalePI(angleToLookahead - stateDelta);
  if (DEBUG_TRACKER) {
    CONSOLE.print("angleDiff robotState to lookaheadPoint: "); CONSOLE.println(angleDiff * RAD_TO_DEG);
  }


  // --- Automatisches Rückwärtsfahren bei überschießen ---
  //lookahead begrenzen -------- ist hier falsch und wird nicht genutzt
  float remainingDist = max(0.0, lineLen - projLen);
  lookahead = min(lookahead, remainingDist + 0.5); // Maximal bis kurz hinter das Ziel
  lookahead = constrain(lookahead, minLookahead, maxLookahead);
  bool overshoot = false;

  // Ziel erreicht?
  const float targetTolerance = 0.3; // z.B. 30cm
  if (projLen >= (lineLen - targetTolerance)) {
    if (DEBUG_TRACKER) {
      CONSOLE.println("target reached / Segment finished!");
    }
    // Hier: Nächsten Pfadpunkt setzen oder Modus wechseln
    //return;
  }
  
  //overshoot option 1
  if (distToTarget > lastDistToTarget + 0.1 && (fabs(angleDiff) > 90.0 * DEG_TO_RAD)) {
    //overshoot = true;
    if (DEBUG_TRACKER) {
      CONSOLE.println("overshoot 1: distToTarget > lastDistToTarget!");
    }
  }

  //overshoot option 2
  // Richtung Vektor Ziel -> Start:
  // 1. Richtung des Segments (Endpunkt - Startpunkt)
  float seg_dx = target.x() - lastTarget.x();
  float seg_dy = target.y() - lastTarget.y();
  // 2. Vektor Endpunkt -> Roboter
  float toRobot_dx = stateX - target.x();
  float toRobot_dy = stateY - target.y();
  float dot = seg_dx * toRobot_dx + seg_dy * toRobot_dy;
  if (dot > 0 && (fabs(angleDiff) > 90.0 * DEG_TO_RAD)) {
    //overshoot = true;
    if (DEBUG_TRACKER) {
      CONSOLE.println("overshoot2: vector indicates overshoot! ");
      CONSOLE.print("dotVetor: "); CONSOLE.println(dot);
    }
  }

  if (overshoot) {
    speed *= -1;
    angleDiff = scalePI(angleDiff + PI);
    if (DEBUG_TRACKER) {
      CONSOLE.println("Reverse Mode acitve!");
      CONSOLE.print(" angleDiff: "); CONSOLE.println(angleDiff * RAD_TO_DEG);
    }
  }

  //purePusrsuit steering
  float sinDiff = sin(angleDiff);
  if (DEBUG_TRACKER) {
    CONSOLE.print("sin(angleDiff): "); CONSOLE.println(sinDiff);
  }
  turn = turnGain * sinDiff / wheelbaseVirtual;
  if (DEBUG_TRACKER) {
    CONSOLE.print("Unconstrained turn: "); CONSOLE.println(turn);
  }

  //if (abs(turn) < minTurnSpeed/2) turn = 0;         // deadzone around zero
  turn = constrain(turn, -maxTurnSpeed, maxTurnSpeed);
  

  if (DEBUG_TRACKER) {
    CONSOLE.print("Turn rate: "); CONSOLE.println(turn);
  }
/*   //slowdown on headingError
  if (fabs(headingError) > 5.0 * DEG_TO_RAD) {
    float reduction = 1.0f - (fabs(headingError) / (PI / 2.0f));
    if (DEBUG_TRACKER) {
      CONSOLE.print("speed reduction factor: "); CONSOLE.println(reduction);
    }
    speed *= constrain(reduction, 0.1f, 1.0f);
    if (DEBUG_TRACKER) {
      CONSOLE.print("Speed reduced to: "); CONSOLE.println(speed);
    }

  } */

  //slow down if error to target is too far and growing
  if (fabs(angleDiff) > anglePrecise) {
    float reduction = 1.0f;
    // map speed
    reduction = safeMap(fabs(angleDiff), anglePrecise, angleStopThreshold, 1.0f, 0.0f);
    reduction = constrain(reduction, 0.0f, 1.0f);
    if (DEBUG_TRACKER) {
      CONSOLE.print("speed factor (linear): ");
      CONSOLE.println(reduction);
    } 
    speed *= reduction;
    if (DEBUG_TRACKER) {
      CONSOLE.print("Speed mapped to: ");
      CONSOLE.println(speed);
    }
  }


/*   //ensure minspeed
  if (fabs(speed) < minSpeed) {
    if (DEBUG_TRACKER) {
      CONSOLE.print("Speed too low, correcting to minSpeed: "); CONSOLE.println(minSpeed);
    }
    speed = minSpeed;
  } */

  //consider general maps direction
  if (maps.trackReverse) {
    speed *= -1.0f;
    turn *= -1.0f;
    if (DEBUG_TRACKER) CONSOLE.println("Reverse mode active");
  }

  //output linear, angular
  linear = speed;
  angular = turn;
  if (DEBUG_TRACKER) {
    CONSOLE.print("Final linear: "); CONSOLE.println(linear);
    CONSOLE.print("Final angular: "); CONSOLE.println(angular);
  }
}

void hybridTracker() {
  
}
  

bool AngleToTargetFits() {
  // allow rotations only near last or next waypoint or if too far away from path
  if (targetDist < 0.3 || lastTargetDist < 0.3 || fabs(distToPath) > 1.0 ) {
    angleToTargetFits = ((fabs(trackerDiffDelta) * 180.0 / PI )<= TRANSITION_ANGLE);              //MrTree we have more than TRANSITION_ANGLE difference to point, else linetracker stanley angular faktor p will sort things out
  } else {
	// while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    angleToTargetFits = ((fabs(trackerDiffDelta) * 180.0 / PI) < TARGETFITS_ANGLE);       
  }

  if ((!angleToTargetFits || !angleToTargetPrecise)) angleToTargetFits = false;   //MrTree added !dockTimer to prevent the jumping gps point to cause linear=0 because of !angleToTargetFits, added !angleToTargetPrecise 
  if (dockTimer || unDockTimer) angleToTargetFits = true;
  return angleToTargetFits;  
}

void rotateToTarget() {
  // angular control (if angle to far away, rotate to next waypoint)
    if (!angleToTargetFits) angleToTargetPrecise = false;
    linear = 0; //MrTree while turning from >= 20/45 deg difference, linear is set to 0... still decelerating or accelerating on stepin/out
    if (ROTATION_RAMP) {
      //angular = lerp(ROTATION_RAMP_MIN / 180.0 * PI, ROTATION_RAMP_MAX / 180.0 * PI, fabs(trackerDiffDelta));
      //CONSOLE.print("lerp angular:  ");CONSOLE.println(angular*180/PI);
      angular = fabs(trackerDiffDelta) + ROTATION_RAMP_MIN / 180.0 * PI;
      //CONSOLE.print("raw angular +5 :  ");CONSOLE.println(angular*180/PI);
      
      angular = constrain(angular, ROTATION_RAMP_MIN / 180.0 * PI, ROTATION_RAMP_MAX / 180.0 * PI);
      //CONSOLE.print("constrain angular:  ");CONSOLE.println(angular*180/PI);
    } else {
      if (fabs(trackerDiffDelta) * 180.0 / PI >= ANGLEDIFF1) angular = ROTATETOTARGETSPEED1 / 180.0 * PI;   //MrTree set angular to fast defined in config.h
      if (fabs(trackerDiffDelta) * 180.0 / PI < ANGLEDIFF1) angular = ROTATETOTARGETSPEED2  / 180.0 * PI;    //MrTree slow down turning when near desired angle     
      if (fabs(trackerDiffDelta) * 180.0 / PI <= ANGLEDIFF2) angular = ROTATETOTARGETSPEED3 / 180.0 * PI;    //MrTree slow down turning even more when almost at desired angle     
    }
    if (trackerDiffDelta < 0) {     //MrTree set rotation direction and do not keep it :)
      angular *= -1;
    }                     
    if (fabs(trackerDiffDelta) * 180.0 / PI < ANGLEPRECISE){
      angular = 0;
      resetStateEstimation();
      if (CurrRot == 0) angleToTargetPrecise = true;                          //MrTree Step out of everything when angle is precise... and we stopped rotating 
    }
    //add option to disable and start rotating even if still moving
    if (fabs(CurrSpeed) > 0.0) angular = 0;                //MrTree reset angular if current speed is over given value (still deccelerating)
    // test for alfred wheels slipping
    // we need an imu feedback of possible wheel slipping
    float angularTest = 0;
    angularTest = turnInPlaceControl(angular);
    //CONSOLE.print("angular | angularTest  ");CONSOLE.print(angular);CONSOLE.print(" | ");CONSOLE.println(angularTest);
}

float turnInPlaceControl(float angularIn) {
  const int MAX_SPEED = 100;
  const int MIN_SPEED = 30;
  const float SLOWDOWN_FACTOR = 0.8;
  const float RECOVERY_RATE = 5;
  const float THRESHOLD = 0.5;  // Verhältnis: IMU/Soll-Drehrate
  float angularOut = 0;
  float imuZ = stateDeltaIMU;
  float desiredZ = fabs(angularIn);

  if (desiredZ < 0.01) desiredZ = 0.01; // Schutz gegen Division durch 0

  float ratio = imuZ / desiredZ;

  if (ratio < THRESHOLD) {
    // Verdacht auf Schlupf oder Blockade
    angularIn = max(int(angularIn * SLOWDOWN_FACTOR), MIN_SPEED);
  } else {
    // Normalverhalten – wieder hochregeln
    angularIn = min(angularIn + int(RECOVERY_RATE), MAX_SPEED);
  }
  angularOut = angularIn;
  // Drehen basierend auf Vorzeichen der Soll-Drehrate
  //int direction = (getDesiredYawRate() >= 0) ? 1 : -1;
  return(angularOut);
}

void stanleyTracker() {
   
  //Stanley parameters
  static float k;
  static float p;
  const float maxRot = DEG_TO_RAD * 30;

  //do not use agressive stanley if floatsittuaion
  if (gps.solution == SOL_FLOAT || gps.solution == SOL_INVALID) {
    stanleyTrackingNormalK = STANLEY_FLOAT_K_NORMAL; 
    stanleyTrackingNormalP = STANLEY_FLOAT_P_NORMAL;
    stanleyTrackingSlowK = STANLEY_FLOAT_K_SLOW;
    stanleyTrackingSlowP = STANLEY_FLOAT_P_SLOW;
  } else {
    stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL; 
    stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;
    stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
    stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;
  }
  //FIXME
  if (MAP_STANLEY_CONTROL) { //this is going to be made not an option, but a must
    //Mapping of Stanley Control Parameters in relation to actual Setpoint value of speed
    //Values need to be multiplied, because map() function does not work well with small range decimals
    // linarSpeedSet needed as absolut value for mapping

    k = map(fabs(CurrSpeed) * 1000, MOTOR_MIN_SPEED * 1000, MOTOR_MAX_SPEED * 1000, stanleyTrackingSlowK * 1000, stanleyTrackingNormalK * 1000); //MOTOR_MIN_SPEED and MOTOR_MAX_SPEED from config.h
    p = map(fabs(CurrSpeed) * 1000, MOTOR_MIN_SPEED * 1000, MOTOR_MAX_SPEED * 1000, stanleyTrackingSlowP * 1000, stanleyTrackingNormalP * 1000); //MOTOR_MIN_SPEED and MOTOR_MAX_SPEED from config.h
    k = k / 1000;
    p = p / 1000;
    k = max(stanleyTrackingSlowK, min(stanleyTrackingNormalK, k));  // limitation for value if out of range
    p = max(stanleyTrackingSlowP, min(stanleyTrackingNormalP, p));  // limitation for value if out of range
  } else {
    k = stanleyTrackingNormalK;                                     // STANLEY_CONTROL_K_NORMAL;
    p = stanleyTrackingNormalP;                                     // STANLEY_CONTROL_P_NORMAL;
    if (maps.trackSlow && trackslow_allowed) {     //this will create bugs, because you never now when this slowparameter in docking will be activated
      k = stanleyTrackingSlowK;                                     //STANLEY_CONTROL_K_SLOW;         
      p = stanleyTrackingSlowP;                                     //STANLEY_CONTROL_P_SLOW;
    }
  }
                                                                                                                            //MrTree
  angular =  p * trackerDiffDelta + atan2(k * lateralError, (0.001 + fabs(CurrSpeed)));       //MrTree, use actual speed correct for path errors
  // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
  //angular = max(-PI/6, min(PI/6, angular)); //MrTree still used here because of gpsfix jumps that would lead to an extreme rotation speed or singulatity when reaching a point
  angular = max(-maxRot, min(maxRot, angular)); //MrTree still used here because of gpsfix jumps that would lead to an extreme rotation speed or singulatity when reaching a point
  //if (!maps.trackReverse && motor.linearCurrSet < 0) angular *= -1;   // it happens that mower is reversing without going to a map point (obstacle escapes) but trying to get straight to the next point angle (transition angle), for this case angular needs to be reversed
  //After all, we want to use stanley for transition angles as well, too have a smooth operation between points without coming to a complete stop
  //For that we will scale down the actual linear speed set dependent to the angle difference to the NEXT targetpoint, we need also to deactivate distance ramp for nextangletotargetfits = true
  //so distanceramp is basically just a function for !angletotargetfits and can be moved into this function permanent without possibility to activate or deactivate? This would also eliminate the need in distanceramp to try and compensate small angles to not come to a stop
  //which is now anyway forced by angletotargetfits and has no effect.

  //linear = linearSpeedState() * angletonexttarget
}

void linearSpeedState(){
  bool distRampActive = false;
  unsigned int chosenIndexlDebugOutput = 0;
  bool allFalse = false;
  const int aLen = 10;                                           //array length of linearSpeed[]
  const String linearSpeedNames[aLen] = {                       //strings for message output accordingly to state
                                    "FLOATSPEED",
                                    "NEARWAYPOINTSPEED",
                                    "SONARSPEED",
                                    "OVERLOADSPEED",
                                    "KEEPSLOWSPEED",
                                    "RETRYSLOWSPEED",
                                    "TRACKSLOWSPEED",
                                    "DOCK_NO_ROTATION_SPEED",
                                    "DOCKPATHSPEED",
                                    "DOCKSPEED"
                                  };
  const float linearSpeed[aLen] = {
                                    FLOATSPEED,
                                    NEARWAYPOINTSPEED,
                                    SONARSPEED,
                                    OVERLOADSPEED,
                                    KEEPSLOWSPEED,
                                    RETRYSLOWSPEED,
                                    TRACKSLOWSPEED,
                                    DOCK_NO_ROTATION_SPEED,
                                    DOCKPATHSPEED,
                                    DOCKSPEED
                                  };
  static bool linearBool[aLen];     //helper array to choose lowest value from sensor/mower states
  static int chosenIndex;           //helper to know what speed is used
  static int chosenIndexl;          //helper to compare a index change and trigger a meassage in console
  int speedIndex = 0;               //used for linearBool and linearSpeed array index 
  trackslow_allowed = true;

  /*
  // in case of docking or undocking - check if trackslow is allowed
  if ( maps.isUndocking() || maps.isDocking() ) {
    static float dockX = 0;
    static float dockY = 0;
    static float dockDelta = 0;
    maps.getDockingPos(dockX, dockY, dockDelta);
    // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
    if (distance(dockX, dockY, stateX, stateY) > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
      trackslow_allowed = false;
    }
  }
  */
  if (OVERRIDE_MOW_SPEED) {           //FIXME!!!!
    linear = MOWSPEED;
    setSpeed = MOWSPEED;
  } else {
    linear = setSpeed;                //always compare speeds against desired setSpeed 
  }
  //which states can be true in runtime?
  linearBool[0] = ((gps.solution == SOL_FLOAT) && (stateLocalizationMode == LOC_GPS));                  // [0] FLOATSPEED                                        
  linearBool[1] = (targetDist < NEARWAYPOINTDISTANCE || lastTargetDist < NEARWAYPOINTDISTANCE);         // [1] NEARWAYPOINTSPEED
  linearBool[2] = (sonar.nearObstacle() || bumperDriver.nearObstacle() || lidarBumper.nearObstacle());  // [2] SONARSPEED == NEAROBSTACLESPEED ==LIDARSPEED       
  linearBool[3] = (motor.motorLeftOverload || motor.motorRightOverload || motor.motorMowOverload);      // [3] OVERLOADSPEED
  linearBool[4] = (motor.keepslow);                                                                     // [4] KEEPSLOWSPEED
  linearBool[5] = (motor.retryslow);                                                                    // [5] RETRYSLOWSPEED
  linearBool[6] = (maps.trackSlow && trackslow_allowed);                                                // [6] TRACKSLOWSPEED
  linearBool[7] = (dockTimer || unDockTimer);                                                           // [7] DOCK_NO_ROTATION_SPEED
  linearBool[8] = (maps.isAtDockPath());                                                                // [8] DOCKPATHSPEED
  linearBool[9] = (maps.isGoingToDockPath());                                                           // [8] DOCKSPEED
  
  //disable near way point speed if we use the distance ramp
  if (DISTANCE_RAMP) linearBool[1] = false;

  //choose the lowest speed of the true states set before in the bool array
  for(speedIndex = 0; speedIndex < aLen; speedIndex ++){
    if (linearBool[speedIndex] == true){
      if (linearSpeed[speedIndex] < linear){
        linear = linearSpeed[speedIndex];
        chosenIndex = speedIndex;
      }
    } else {
      allFalse = true;
    }
  }

  //trigger a message if speed changes
  if (chosenIndex != chosenIndexl){
    if (chosenIndex == 3) {
      Logger.event(EVT_MOTOR_OVERLOAD_REDUCE_SPEED);
      CONSOLE.println("motor overload detected: reducing linear speed");
    }
    CONSOLE.print("Linetracker.cpp - linearSpeedState(): ");
    CONSOLE.print(linearSpeedNames[chosenIndex]);
    CONSOLE.print(" = ");
    CONSOLE.print(linearSpeed[chosenIndex]);
    CONSOLE.println(" m/s");
    chosenIndexlDebugOutput = chosenIndex;
  }

  //consider the distance ramp wih the chosen speed if we are approaching or leaving a waypoint
  if (DISTANCE_RAMP) {
    if (targetDist < 2 * NEARWAYPOINTDISTANCE || lastTargetDist < 2 * NEARWAYPOINTDISTANCE) { //start computing before reaching point distance (not neccessary)
      linear = distanceRamp(linear);
      distRampActive = true;
    } else {
      distRampActive = false;
    }
  }
  
  // if there is no active slowdown from setSpeed, we will have setSpeed as goal
  if (stateLocalizationMode == LOC_APRIL_TAG){
    if (!stateAprilTagFound){
      linear = 0; // wait until april-tag found 
      angular = 0; 
    } else {
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      //linear = 0; // wait until april-tag found 
      //angular = 0; 
    }
  }

  if (stateLocalizationMode == LOC_REFLECTOR_TAG){
    if (!stateReflectorTagFound){
      linear = 0; // wait until reflector-tag found 
      angular = 0; 
    } else {
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      float maxAngular = 0.015;  // 0.02
      float maxLinear = 0.05;      
      angular =  max(min(1.0 * trackerDiffDelta, maxAngular), -maxAngular);
      angular =  max(min(angular, maxAngular), -maxAngular);      
      linear = 0.05;      
      //if (maps.trackReverse) linear = -0.05;   // reverse line tracking needs negative speed           
    }
  }
  
  if (stateLocalizationMode == LOC_GUIDANCE_SHEET){
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      angular = 0;
  }

  if (linear != 0){
    if (linear > MOTOR_MAX_SPEED) linear = MOTOR_MAX_SPEED;
    if (linear < MOTOR_MIN_SPEED) linear = MOTOR_MIN_SPEED;
  }

  if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed  --> THIS NEEDS TO GO SOMEWHERE ELSE?

  if (DEBUG_SPEEDS) {
    CONSOLE.println("SPEED DEBUG START  --------------------------->");
    CONSOLE.println("");
    CONSOLE.println("                          def  |  state");
      CONSOLE.print("               SETSPEED: ");CONSOLE.print(setSpeed);            CONSOLE.print("  -  ");      CONSOLE.println(allFalse);
      CONSOLE.print("             FLOATSPEED: ");CONSOLE.print(linearSpeed[0]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[0]);
      CONSOLE.print("      NEARWAYPOINTSPEED: ");CONSOLE.print(linearSpeed[1]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[1]);
      CONSOLE.print("             SONARSPEED: ");CONSOLE.print(linearSpeed[2]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[2]);
      CONSOLE.print("          OVERLOADSPEED: ");CONSOLE.print(linearSpeed[3]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[3]);
      CONSOLE.print("          KEEPSLOWSPEED: ");CONSOLE.print(linearSpeed[4]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[4]);
      CONSOLE.print("         RETRYSLOWSPEED: ");CONSOLE.print(linearSpeed[5]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[5]);
      CONSOLE.print("         TRACKSLOWSPEED: ");CONSOLE.print(linearSpeed[6]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[6]);
      CONSOLE.print(" DOCK_NO_ROTATION_SPEED: ");CONSOLE.print(linearSpeed[7]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[7]);
      CONSOLE.print("          DOCKPATHSPEED: ");CONSOLE.print(linearSpeed[8]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[8]);
      CONSOLE.print("              DOCKSPEED: ");CONSOLE.print(linearSpeed[9]);      CONSOLE.print("  -  ");      CONSOLE.println(linearBool[9]);
      CONSOLE.println("");
      CONSOLE.print("      speedstate BEFORE: ");CONSOLE.println(linearSpeedNames[chosenIndexlDebugOutput]);
      if (!allFalse) {
      CONSOLE.print("               USED NOW: ");CONSOLE.print(linearSpeedNames[chosenIndexl]);CONSOLE.print("  -  linear in linetracker: ");CONSOLE.println(linear);
      } else { 
      CONSOLE.print("               USED NOW: ");CONSOLE.print("SETSPEED");CONSOLE.print("  -  linear in linetracker: ");CONSOLE.println(linear);
      }
      CONSOLE.print("    DistanceRampActive?: ");CONSOLE.println(distRampActive);
      CONSOLE.print("       setSpeed active?: ");CONSOLE.println(allFalse);
      CONSOLE.println("");
    CONSOLE.println("SPEED DEBUG END    <---------------------------");
  }

  chosenIndexl = chosenIndex;
}

/* float distanceRamp(float linear){
    float maxSpeed = linear*1000;
    float minSpeed = DISTANCE_RAMP_MINSPEED * 1000;
    float maxDist = (linear * NEARWAYPOINTDISTANCE /setSpeed) * 1000;     //if we are going slow for example because of float, the ramp will kick in when mower is nearer to point
    float minDist = 0;                                                    //TARGET_REACHED_TOLERANCE*1000;
    float actDist = 0;
    float rampSpeed = 0;
    static bool wasStraight;

    if (targetDist <= lastTargetDist) {                                  //need to decide what ramp, leaving or aproaching? --> approaching
      maxDist += maxSpeed;                                               //add an speed dependent offset to target distance when approaching, because mower comes with high speed that causes a timing issue
      actDist = targetDist;
      if (straight) {
        minSpeed = TRANSITION_SPEED * 1000; //maxSpeed * 0.5;                           //if we don´t need to rotate, do not decellarate too much
        transition = true;
      }
      wasStraight = straight;
    } else {
      if (wasStraight) {
        minSpeed = TRANSITION_SPEED * 1000; //maxSpeed * 0.5;
        transition = true;
      }
      actDist = lastTargetDist;
      //actDist += 0.05; //add an offset      
    }

    actDist *= 1000;

    if (targetDist + lastTargetDist < maxDist) { //points are not far away from each other
      actDist *= 2;                              //multiply the actDist to trick the map function (hurryup because we wont reach full speed anyway)
    }

    rampSpeed = map(actDist, minDist, maxDist, minSpeed, maxSpeed);
    rampSpeed = constrain(rampSpeed, minSpeed, maxSpeed);
    rampSpeed /= 1000;
    //CONSOLE.print(straight); CONSOLE.print(" "); CONSOLE.println(rampSpeed);
    return rampSpeed;
} */

float distanceRamp(float linear){
    if (lastTargetDist > NEARWAYPOINTDISTANCE && targetDist > NEARWAYPOINTDISTANCE) return linear;  //leave, nothing to do

    float maxSpeed = linear;                                        //maxSpeed is the actual linetracker linear set 
    float minSpeed = DISTANCE_RAMP_MINSPEED;
    float maxDist = NEARWAYPOINTDISTANCE;                           //if we are going slow for example because of float, the ramp will kick in when mower is nearer to point
    float minDist = TARGET_REACHED_TOLERANCE;                      
    float actDist = 0;
    float rampSpeed = 0;
    static bool wasStraight;

    if (targetDist <= lastTargetDist) {                             //need to decide what ramp, leaving or aproaching? --> approaching
      maxDist += maxSpeed;                                        //add an speed dependent offset to target distance when approaching, because mower comes with high speed that causes a timing issue
      actDist = targetDist;
      if (straight) {
        minSpeed = TRANSITION_SPEED;                                //if we don´t need to rotate and want todo a point transition, do not decellarate too much and use TRANSITION_SPEED
        transition = true;
      }
      wasStraight = straight;
    } else {                                                        // --> leaving
      if (wasStraight) {
        minSpeed = TRANSITION_SPEED;
        transition = true;
      }
      actDist = lastTargetDist;     
    }

    rampSpeed = safeMap(actDist, minDist, maxDist, minSpeed, maxSpeed);
    rampSpeed = constrain(rampSpeed, minSpeed, maxSpeed);
    
    //CONSOLE.print(straight); CONSOLE.print(" "); CONSOLE.println(rampSpeed);
    return rampSpeed;
}

void gpsConditions() {
  // check some pre-conditions that can make linear+angular speed zero
  if ((stateLocalizationMode == LOC_GPS) && (fixTimeout != 0)) {
    if (millis() > lastFixTime + fixTimeout * 1000.0) {
      activeOp->onGpsFixTimeout();
    }
  }
  //CONSOLE.print(maps.shouldGpsReboot); CONSOLE.print(" <- shouldreboot | isatgpsrebootpoint -> "); CONSOLE.print(maps.isAtGpsRebootPoint()); CONSOLE.print(" | dockPointIdx: ");CONSOLE.println(maps.dockPointsIdx);
  if (DOCK_GPS_REBOOT) {
    if (maps.shouldGpsReboot && maps.isAtGpsRebootPoint()){
      activeOp->onDockGpsReboot();
    }
  }

  // gps-jump/false fix check
  if (KIDNAP_DETECT) {
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;

    if ( maps.isUndocking() || maps.isDocking() ) {
      float dockX = 0;
      float dockY = 0;
      float dockDelta = 0;
      maps.getDockingPos(dockX, dockY, dockDelta);
      float dist = distance(dockX, dockY, stateX, stateY);
      // check if current distance to docking station is below
      // KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK to trigger KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK
      if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK) {
        allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
      }
    }

    if ((stateLocalizationMode == LOC_GPS) && (fabs(distToPath) > allowedPathTolerance)){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped){
        stateKidnapped = true;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(distToPath);
        activeOp->onKidnapped(stateKidnapped);
      }

    } else {
      if (stateKidnapped) {
        stateKidnapped = false;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(distToPath);
        activeOp->onKidnapped(stateKidnapped);        
      }
    }
  }
}

void noDockRotation() {
  if (DOCK_NO_ROTATION) {
    if (maps.wayMode != WAY_DOCK) return;
    if ((maps.isTargetingLastDockPoint() && !maps.isUndocking())){        //MrTree step in algorithm if allowDockRotation (computed in maps.cpp) is false and mower is not undocking
      
      if (!dockTimer){                                                  //set helper bool to start a timer and print info once
        reachedPointBeforeDockTime = millis();                          //start a timer when going to last dockpoint
        dockTimer = true;                                               //enables following code
        CONSOLE.println("allowDockRotation = false, timer to successfully dock startet. angular = 0, turning not allowed");
      }

      if (dockTimer){

        //resetLinearMotionMeasurement();                                         //need to test if this is still neccessary
        if (lastTargetDist > DOCK_NO_ROTATION_DISTANCE){                          //testing easier approach for DOCK_NO_ROTATION setup
          angular = 0;
          linear = DOCK_NO_ROTATION_SPEED;
          targetReached = false;
          if (!buzzer.isPlaying()) buzzer.sound(SND_ERROR, true);                  
        }

        if (millis() > reachedPointBeforeDockTime+DOCK_NO_ROTATION_TIMER){      //check the time until mower has to reach the charger and triger obstacle if not reached
          CONSOLE.println("noDockRotation(): not docked in given time, triggering maps.retryDocking!");
          dockTimer = false;
          triggerObstacle();     
        } 
      }
    } else {
        dockTimer = false;     
    }
    return;
  }
  return;
}

void noUnDockRotation(){
  if (DOCK_NO_ROTATION) {
    if (maps.wayMode != WAY_DOCK) return;
    
    if (maps.isBetweenLastAndNextToLastDockPoint() && maps.isUndocking()){
      if (!unDockTimer){                                                  //set helper bool to start a timer and print info once
        reachedPointBeforeDockTime = millis();                          //start a timer when going to last dockpoint
        unDockTimer = true;                                               //enables following code
        CONSOLE.println("noUnDockRotation(): timer to successfully undock startet. angular = 0, turning not allowed");
      }

      if (unDockTimer){                                                     
        angular = 0;
        linear = -DOCK_NO_ROTATION_SPEED;
        if (!buzzer.isPlaying()) buzzer.sound(SND_ERROR, true);                  
        if (millis() > reachedPointBeforeDockTime+DOCK_NO_ROTATION_TIMER){      //check the time until mower has to reach the charger and triger obstacle if not reached
          CONSOLE.println("noUnDockRotation(): reversed for given Time, triggering Wait before further retreating to reboot gps point!");
          unDockTimer = false;
          maps.dockPointsIdx--;
          //targetReached = true;
          //waitOp.waitTime = 15000;
          triggerWaitCommand(15000);    
        } 
      }
    } else {
        unDockTimer = false;     
    }
    return;
  }
  return;  
}

void checkMowAllowed() {
  mow = false;                                //changed to false
  if (MOW_START_AT_WAYMOW &! oneTrigger) {                                                             
    if (maps.wayMode == WAY_MOW) {            //do not activate mow until there is a first waymow 
      mow = true;                             //this will only work directly after undocking and way free, the first time it is in waymow, mow will be true like before until finishing     
      oneTrigger = true;
    }                                              
  } else {
    if (maps.wayMode == WAY_MOW || WAY_EXCLUSION || WAY_PERIMETER){
      mow = true;
    }
  }

  if (stateOp == OP_DOCK ) {
    mow = false;
    oneTrigger = false;
  }
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl) {
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;
  
  // Check valid path on entrance, get next point instantly
/*   if (target.x() == lastTarget.x() && target.y() == lastTarget.y()) {
    CONSOLE.println("Linetracker WARNING: target equals lastTarget – no path");
    linear = 0;
    angular = 0;
    targetReached = true;
    activeOp->onTargetReached();
    straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false, stateX, stateY)) {
      // finish
      activeOp->onNoFurtherWaypoints();
    }
    return;
  } */

  CurrSpeed = motor.linearSpeedSet;           //get the speed from motor.linearSpeedSet (with ramping)
  CurrRot = motor.angularSpeedSet;            //get the turn rate from motor.angulatSpeedSet (with ramping)
  transition = false;
  linear = 0;                                 //no motion
  angular = 0;                                //no motion
  
  targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());  //prepare HeadingError to next targetpoint
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);     //flip the prepeared HeadingError if reversing
  targetDelta = scalePIangles(targetDelta, stateDelta);               //scale the HeadingError named targetDelta
  trackerDiffDelta = distancePI(stateDelta, targetDelta);             //sort the shortes shortest rotation direction
  
/*   if (!isfinite(targetDelta)) {
    CONSOLE.println("Ungültiger targetDelta");
    targetDelta = 0;
  }
  if (!isfinite(trackerDiffDelta)) {
    CONSOLE.println("Ungültiger trackerDiffDelta");
    trackerDiffDelta = 0;
  }
  if (!isfinite(targetDist)) {
    CONSOLE.println("Ungültiger targetDist");
    targetDist = 0;
  } */
  
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());  //orthogonal distance to line, the lateral robot position from the tracked Path, using infinite line so there will be no float exception
  distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());            //orthogonal distance to line
  targetDist = maps.distanceToTargetPoint(stateX, stateY);                                                      //the robot target distance
  lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);                                              //the robot distance to the last targetpoint
  
  targetReached = (targetDist < TARGET_REACHED_TOLERANCE);                                                      //the switch if the target is reached, linetracker triggers new target point from map.cpp
  pathLength = maps.distanceToTargetPoint(lastTarget.x(), lastTarget.y());                                      //the absolute distance from mower to the last target point
 const int mode = CONTROLLER_MODE; 
 switch (mode) {
    case 1:
        if (!AngleToTargetFits()) { 
          rotateToTarget();
        } else {
          linearSpeedState();  //compares the linear Speed to use according to configured mower state (maybe this should be first in line)
          stanleyTracker();    //track the path
        }
      break;
    case 2:
        linearSpeedState();
        purePursuitTracker();
      break;
    case 3:
        CONSOLE.println("NOT IMPLEMENTED");
      break;
    default:
      CONSOLE.println("tracker mode unknown!");
      break;
  }
  
/*   if (!AngleToTargetFits()) { 
    rotateToTarget();
  } else {
    linearSpeedState();  //compares the linear Speed to use according to configured mower state (maybe this should be first in line)
    stanleyTracker();    //track the path
  } */

  gpsConditions();      //check for gps conditions to eg. trigger obstacle or fixtimeout (shouldn´t that be in mowop???)
  noDockRotation();     //disable angular for dock/undock situations
  noUnDockRotation();   //disable angular for dock/undock situations
  checkMowAllowed();    //check mow switched on/off

  if (runControl) {

    shouldRotate = robotShouldRotate();

    if (DEBUG_LINETRACKER) {
      // ouput target point change
      x_new = target.x();
      y_new = target.y();
      if (x_old != x_new || y_old != y_new) {
        CONSOLE.print("LineTracker.cpp targetPoint  x = ");
        CONSOLE.print(x_new);
        CONSOLE.print(" y = ");
        CONSOLE.println(y_new);
        x_old = x_new;
        y_old = y_new;
      }
      // output rotate state change
      if (shouldRotate != shouldRotatel) {
        CONSOLE.print("Linetracker.cpp ShouldRotate = ");
        CONSOLE.println(shouldRotate);
        shouldRotatel = shouldRotate;
      }
      // output tracking data permanently
      CONSOLE.println("DEBUG_LINETRACKER START -->");
      CONSOLE.print("    angleToTargetFits: ");CONSOLE.println(angleToTargetFits);
      CONSOLE.print("              angular: ");CONSOLE.println(angular * 180.0 / PI);
      CONSOLE.print("     trackerDiffDelta: ");CONSOLE.println(trackerDiffDelta * 180 / PI);
      CONSOLE.print("        distToPath --> ");CONSOLE.print(distToPath);CONSOLE.print(" | ");CONSOLE.print(targetDist);CONSOLE.println(" <-- targetDist");
      CONSOLE.print("motor.angularSpeedSet: ");CONSOLE.println(motor.angularSpeedSet * 180 / PI);
      CONSOLE.print("       shouldRotate(): ");CONSOLE.println(robotShouldRotate());
      CONSOLE.println("<-- DEBUG_LINETRACKER END");
    }
  
    if (detectLift()){ // in any case, turn off mower motor if lifted  
      mow = false;  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
      linear = 0;
      angular = 0; 
    }

    if (mow != motor.switchedOn && motor.enableMowMotor){
      CONSOLE.print("Linetracker.cpp changes mow status: ");
      CONSOLE.println(mow);
      motor.setMowState(mow); 
    }
    motor.setLinearAngularSpeed(linear, angular, true);    
  }

  if (targetReached) {
    activeOp->onTargetReached();
    straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false, stateX, stateY)) {
      // finish
      activeOp->onNoFurtherWaypoints();
    }
  }
}
