// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include <Arduino.h>
#include "StateEstimator.h"
#include "src/op/op.h"

#include "config.h"
#include "robot.h"
#include "Stats.h"
#include "helper.h"
#include "i2c.h"
#include "events.h"


LocalizationMode stateLocalizationMode = LOC_GPS;

float stateX = 0;  // position-east (m)
float stateY = 0;  // position-north (m)
float stateDelta = 0;  // direction (rad)
float stateHeading = 0; // direction (deg)
float stateRoll = 0;
float statePitch = 0;
float stateDeltaGPS = 0;
float stateDeltaIMU = 0;
float imuRawYaw_sc = 0; //PIscaled raw yaw from imuDriver.yaw
float stateGroundSpeed = 0; // m/s

bool stateAprilTagFound = false;
float stateXAprilTag = 0; // camera-position in april-tag frame
float stateYAprilTag = 0;  
float stateDeltaAprilTag = 0; 

bool stateReflectorTagFound = false;
bool stateReflectorTagOutsideFound = false;
bool stateReflectorUndockCompleted = false;
float stateXReflectorTag = 0; // camera-position in reflector-tag frame
float stateYReflectorTag = 0;  
float stateXReflectorTagLast = 0;
float stateDeltaReflectorTag = 0; 

float lastPosN = 0;
float lastPosE = 0;
float lastPosDelta = 0;

float stateDeltaLast = 0;
float stateDeltaSpeed = 0;
//float stateDeltaSpeedLP = 0;
float stateDeltaSpeedIMU = 0;
float stateDeltaSpeedWheels = 0;
float diffIMUWheelYawSpeed = 0;
//float diffIMUWheelYawSpeedLP = 0;

bool gpsJump = false;
bool resetLastPos = true;

float lastIMUYaw = 0; 
float lateralError = 0; // lateral error
float rollChange = 0;
float pitchChange = 0;
bool imuIsCalibrating = false;
int imuCalibrationSeconds = 0;
unsigned long nextImuCalibrationSecond = 0;
unsigned long nextDumpTime = 0;
unsigned long timeLastState = 0;

unsigned long solutionTime = 0;
unsigned long lastSolutionTime = 0;
unsigned long solutionTimeDelta = 0;

const unsigned short bufLen = ROBOT_CONTROL_CYCLE/2;  //seems to be excactly the control cycle time for sync
float ringBuffer[bufLen] = {0};
unsigned short bufInd = 0;

LowPassFilter imuLpfRoll(0.01);
LowPassFilter imuLpfPitch(0.01);
LowPassFilter imuLpfYaw(0.01);
LowPassFilter imuLpfRollChange(0.01);
LowPassFilter imuLpfPitchChange(0.01);
LowPassFilter imuLpfYawChange(0.01);
LowPassFilter imuLpfStateDeltaSpeed(0.01);
LowPassFilter wheelsLpfDeltaSpeed(0.05);
LowPassFilter stateLpfDeltaSpeed(0.1);

/* LowPassFilter imuLpRoll(0.9);
LowPassFilter imuLpPitch(0.9);
LowPassFilter imuLpYaw(0.9);
LowPassFilter imuLpRollChange(0.9);
LowPassFilter imuLpPitchChange(0.9);
LowPassFilter imuLpYawChange(0.9); */

// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide#using-the-mpu-9250-dmp-arduino-library
// start IMU sensor and calibrate
bool startIMU(bool forceIMU){    
  // detect IMU
  uint8_t data = 0;
  int counter = 0;  
  while ((forceIMU) || (counter < 1)){          
     imuDriver.detect();
     if (imuDriver.imuFound){
       break;
     }
     I2Creset();  
     Wire.begin();    
     #ifdef I2C_SPEED
       Wire.setClock(I2C_SPEED);   
     #endif
     counter++;
     if (counter > 5){    
       // no I2C recovery possible - this should not happen (I2C module error)
       CONSOLE.println("ERROR IMU not found");
       //stateSensor = SENS_IMU_TIMEOUT;
       activeOp->onImuError();
       //setOperation(OP_ERROR);      
       //buzzer.sound(SND_STUCK, true);            
       return false;
     }
     watchdogReset();          
  }  
  if (!imuDriver.imuFound) {
    Logger.event(EVT_ERROR_IMU_NOT_CONNECTED);
    return false;
  }  
  counter = 0;  
  while (true){    
    if (imuDriver.begin()) break;
    CONSOLE.print("Unable to communicate with IMU.");
    CONSOLE.print("Check connections, and try again.");
    CONSOLE.println();
    delay(1000);    
    counter++;
    if (counter > 5){
      //stateSensor = SENS_IMU_TIMEOUT;
      activeOp->onImuError();
      Logger.event(EVT_ERROR_IMU_NOT_CONNECTED);    
      //setOperation(OP_ERROR);      
      //buzzer.sound(SND_STUCK, true);            
      return false;
    }
    watchdogReset();       
  }              
  imuIsCalibrating = true;   
  nextImuCalibrationSecond = millis() + 1000;
  imuCalibrationSeconds = 0;
  return true;
}


void dumpImuTilt(){
  if (millis() < nextDumpTime) return;
  nextDumpTime = millis() + 10000;
  CONSOLE.print("IMU tilt: ");
  CONSOLE.print("ypr=");
  CONSOLE.print(imuRawYaw_sc/PI*180.0);
  CONSOLE.print(",");
  CONSOLE.print(imuDriver.pitch/PI*180.0);
  CONSOLE.print(",");
  CONSOLE.print(imuDriver.roll/PI*180.0);
  CONSOLE.print(" rollChange=");
  CONSOLE.print(rollChange/PI*180.0);
  CONSOLE.print(" pitchChange=");
  CONSOLE.println(pitchChange/PI*180.0);
}

// read IMU sensor (and restart if required)
// I2C recovery: It can be minutes or hours, then there's an I2C error (probably due an spike on the 
// SCL/SDA lines) and the I2C bus on the pcb1.3 (and the arduino library) hangs and communication is delayed. 
// We check if the communication is significantly (10ms instead of 1ms) delayed, if so we restart the I2C 
// bus (by clocking out any garbage on the I2C bus) and then restarting the IMU module.
// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide/using-the-mpu-9250-dmp-arduino-library
/*void readIMU(){
  if (!imuDriver.imuFound) return;
  // Check for new data in the FIFO
  unsigned long startTime = millis();
  bool avail = (imuDriver.isDataAvail());
  if (!avail) CONSOLE.println("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOODAAAAAAAAAAAAAAAAATAAAAAAAAAAAAAAAAAAA");
  // check time for I2C access : if too long, there's an I2C issue and we need to restart I2C bus...
  unsigned long duration = millis() - startTime;    
  if (avail) imuDataTimeout = millis() + 10000; // reset IMU data timeout, if IMU data available
  //CONSOLE.print("duration:");
  //CONSOLE.println(duration);  
  if ((duration > 60) || (millis() > imuDataTimeout)) {
    if (millis() > imuDataTimeout){
      CONSOLE.print("ERROR IMU data timeout: ");
      CONSOLE.print(millis()-imuDataTimeout);
      CONSOLE.println(" (check RTC battery if problem persists)");  
    } else {
      CONSOLE.print("ERROR IMU timeout: ");
      CONSOLE.print(duration);     
      CONSOLE.println(" (check RTC battery if problem persists)");          
    }
    stateSensor = SENS_IMU_TIMEOUT;
    motor.stopImmediately(true);    
    statImuRecoveries++;            
    if (!startIMU(true)){ // restart I2C bus
      return;
    }    
    return;
  } 
  
  if (avail) {        
    //CONSOLE.println("fifoAvailable");
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    #ifdef ENABLE_TILT_DETECTION
      rollChange += (imuDriver.roll-stateRoll);
      pitchChange += (imuDriver.pitch-statePitch);               
      rollChange = 0.95 * rollChange;
      pitchChange = 0.95 * pitchChange;
      statePitch = imuDriver.pitch;
      stateRoll = imuDriver.roll;
      CONSOLE.print("fabs(stateRoll)  ");        
      CONSOLE.print(fabs(stateRoll));
      CONSOLE.print(",  ");
      CONSOLE.print("fabs(statePitch)  ");
      CONSOLE.println(fabs(statePitch));
      CONSOLE.print("rollChange/PI*180.0  ");        
      CONSOLE.print(rollChange/PI*180.0);
      CONSOLE.print(",  ");
      CONSOLE.print("pitchChange/PI*180.0  ");
      CONSOLE.println(pitchChange/PI*180.0);
      //if ( (fabs(scalePI(stateRoll)) > 60.0/180.0*PI) || (fabs(scalePI(statePitch)) > 100.0/180.0*PI)
      //      || (fabs(rollChange) > 30.0/180.0*PI) || (fabs(pitchChange) > 60.0/180.0*PI)   )  {
      if ( (fabs(scalePI(stateRoll)) > 30.0/180.0*PI) || (fabs(scalePI(statePitch)) > 100.0/180.0*PI)) {
        dumpImuTilt();
        activeOp->onImuTilt();
        //stateSensor = SENS_IMU_TILT;
        //setOperation(OP_ERROR);
      }           
    #endif
    motor.robotPitch = scalePI(imuDriver.pitch);
    imuDriver.yaw = scalePI(imuDriver.yaw);
    //CONSOLE.println(imuDriver.yaw / PI * 180.0);
    lastIMUYaw = scalePI(lastIMUYaw);
    lastIMUYaw = scalePIangles(lastIMUYaw, imuDriver.yaw);
    stateDeltaIMU = -scalePI ( distancePI(imuDriver.yaw, lastIMUYaw) );  
    //CONSOLE.print(imuDriver.yaw);
    //CONSOLE.print(",");
    //CONSOLE.print(stateDeltaIMU/PI*180.0);
    //CONSOLE.println();
    lastIMUYaw = imuDriver.yaw;      
    imuDataTimeout = millis() + 10000;         
  } else {  //use the old data

  }     
}*/
//TODO: just use one function run here, read imu and then compute all states directly with fresh sensor data. GPS is slow, IMU is fast. No problems so. A Lawnrobot without IMU should not exist in post 2020...
void readIMU(){
  if (!imuDriver.imuFound) return;
  // Check for new data in the FIFO
  if (!imuDriver.isDataAvail()) {
    //mpu_reset_fifo; // causes cyclic shit still
    //CONSOLE.println("INFO: NO IMU DATA");
    return;
  }

  ////////////////////////////////////////////
  //setup vals raw
  static float imuRawRoll = 0;
  static float imuRawYaw = 0;
  static float imuRawPitch = 0;
  static float imuRawRollLast = 0;
  static float imuRawYawLast = 0;
  static float imuRawPitchLast = 0;
  static float imuRawRollChange = 0; 
  static float imuRawYawChange = 0;
  static float imuRawPitchChange = 0;    
  //setup vals filter
  static float imuLpRoll = 0;
  static float imuLpYaw = 0;
  static float imuLpPitch = 0;
  static float imuLpRollLast = 0;
  static float imuLpYawLast = 0;
  static float imuLpPitchLast = 0;
  static float imuLpRollChange = 0;
  static float imuLpYawChange = 0;
  static float imuLpPitchChange = 0;
  //scaled vars
  //static float imuRawYaw_sc = 0; //made extern
  static float imuRawYawLast_sc = 0; //PI scaled yaw (rad/s)

  //get raw vals
  imuRawRoll = imuDriver.roll;
  imuRawYaw = imuDriver.yaw;
  imuRawPitch = imuDriver.pitch;
  
  //do raw calculations
  imuRawRollChange = imuRawRoll - imuRawRollLast; 
  imuRawYawChange = imuRawYaw - imuRawYawLast;
  imuRawPitchChange = imuRawPitch - imuRawPitchLast;

  //Filter all imuRaw values
  imuLpRoll = imuLpfRoll(imuRawRoll);
  imuLpYaw = imuLpfYaw(imuRawYaw);
  imuLpPitch = imuLpfPitch(imuRawPitch);
  
  //do filtered calculations (extra: again with raw, so that filter can be adjusted seperately for state changes)
  imuLpRollChange = imuLpfRollChange(imuRawRollChange);
  imuLpYawChange = imuLpfYawChange(imuRawRollChange);
  imuLpPitchChange = imuLpfPitchChange(imuRawPitchChange);
  //imuLpRollChange = imuLpRoll - imuLpRollLast;
  //imuLpYawChange = imuLpYaw - imuLpYawLast;
  //imuLpPitchChange = imuLpPitch - imuLpPitchLast;
  
  //piscale values
  imuRawYaw_sc = scalePI(imuRawYaw);
  imuRawYawLast_sc = scalePI(imuRawYawLast);
  imuRawYawLast_sc = scalePIangles(imuRawYawLast_sc, imuRawYaw_sc);
  stateDeltaIMU = -scalePI(distancePI(imuRawYaw_sc, imuRawYawLast_sc));
  imuRawYawLast = imuRawYaw;

  //remember for change calculations
  imuRawRollLast = imuRawRoll;
  imuRawYawLast = imuRawYaw;
  imuRawPitchLast = imuRawPitch;

  imuLpRollLast = imuLpRoll;
  imuLpYawLast = imuLpYaw;
  imuLpPitch = imuLpPitch;
  
  imuRawYawLast_sc = imuRawYaw_sc;
  
  //give vars to globals (FIXME)

  motor.robotPitch = scalePI(imuLpPitch);   //give motor the pitch
  
/*imuDriver.yaw = scalePI(imuDriver.yaw);   
  lastIMUYaw = scalePI(lastIMUYaw);        
  lastIMUYaw = scalePIangles(lastIMUYaw, imuDriver.yaw); 
  stateDeltaIMU = -scalePI ( distancePI(imuDriver.yaw, lastIMUYaw) );  
  lastIMUYaw = imuDriver.yaw;       */
  imuDataTimeout = millis() + 10000;  

  #ifdef ENABLE_TILT_DETECTION    // this needs to be adapted to cycletime

  // Alarm if spikes
  if (imuRawRollChange > 50.0 / 180.0 * PI) {
      CONSOLE.print("stateEstimator.cpp - IMU: WARNING! imuRawRollChange, delta over 50 deg/ite --> unplausible! IMU TILT ignored. imuRawRollChange: ");
      CONSOLE.println(imuRawRollChange);
  } else if (imuRawPitchChange > 50.0 / 180.0 * PI) {
      CONSOLE.print("stateEstimator.cpp - IMU: WARNING! imuRawPitchChange, delta over 50 deg/ite --> unplausible! IMU TILT ignored. imuRawPitchChange: ");
      CONSOLE.println(imuRawPitchChange);
  } else if (
      (fabs(scalePI(imuRawRoll)) > 45.0 / 180.0 * PI) || 
      (fabs(scalePI(imuRawPitch)) > 45.0 / 180.0 * PI) ||
      (fabs(imuRawRollChange) > 20.0 / 180.0 * PI) || 
      (fabs(imuRawPitchChange) > 20.0 / 180.0 * PI)) 
  {
      // dumpImuTilt();  // optional aktivieren
      CONSOLE.println("Statestimator change activeOP -> onImuTilt");

      // >>>> Aktualisierte Statusanzeige <<<<
      CONSOLE.print("     imuRawRoll:       ");
      CONSOLE.print(fabs(scalePI(imuRawRoll)) * 180.0 / PI);
      CONSOLE.println("°   > 45°");

      CONSOLE.print("     imuRawRollChange: ");
      CONSOLE.print(fabs(imuRawRollChange) * 180.0 / PI);
      CONSOLE.println("°   > 20°");

      CONSOLE.print("     imuRawPitch:      ");
      CONSOLE.print(fabs(scalePI(imuRawPitch)) * 180.0 / PI);
      CONSOLE.println("°   > 45°");

      CONSOLE.print("     imuRawPitchChange:");
      CONSOLE.print(fabs(imuRawPitchChange) * 180.0 / PI);
      CONSOLE.println("°   > 20°");

      // >>>> Triggerausgabe <<<<
      if (fabs(scalePI(imuRawRoll)) > 45.0 / 180.0 * PI) {
          CONSOLE.print("TRIGGERED: imuRawRoll > 45° (");
          CONSOLE.print(fabs(scalePI(imuRawRoll)) * 180.0 / PI);
          CONSOLE.println("°)");
      }
      if (fabs(scalePI(imuRawPitch)) > 45.0 / 180.0 * PI) {
          CONSOLE.print("TRIGGERED: imuRawPitch > 45° (");
          CONSOLE.print(fabs(scalePI(imuRawPitch)) * 180.0 / PI);
          CONSOLE.println("°)");
      }
      if (fabs(imuRawRollChange) > 20.0 / 180.0 * PI) {
          CONSOLE.print("TRIGGERED: imuRawRollChange > 20° (");
          CONSOLE.print(fabs(imuRawRollChange) * 180.0 / PI);
          CONSOLE.println("°)");
      }
      if (fabs(imuRawPitchChange) > 20.0 / 180.0 * PI) {
          CONSOLE.print("TRIGGERED: imuRawPitchChange > 20° (");
          CONSOLE.print(fabs(imuRawPitchChange) * 180.0 / PI);
          CONSOLE.println("°)");
      }

      activeOp->onImuTilt();
  }
  #endif           
}

void resetImuTimeout(){
  imuDataTimeout = millis() + 10000;  
}

// compute robot state (x,y,delta)
// uses complementary filter ( https://gunjanpatel.wordpress.com/2016/07/07/complementary-filter-design/ )
// to fusion GPS heading (long-term) and IMU heading (short-term)
// with IMU: heading (stateDelta) is computed by gyro (stateDeltaIMU)
// without IMU: heading (stateDelta) is computed by odometry (deltaOdometry)
void computeRobotState(){
  static unsigned long lastControlTime = 0;
  unsigned long controlTime = millis();
  unsigned long deltaControlTime = (controlTime - lastControlTime);  // in Sekunden
  lastControlTime = controlTime;

  if (deltaControlTime < robot_control_cycle - 10 || deltaControlTime > robot_control_cycle + 10) {
    CONSOLE.print("WARNING stateEstimator: unmatched cycletime --> "); 
    CONSOLE.print(deltaControlTime);
    CONSOLE.print("  robot_control_cycle = ");
    CONSOLE.print(robot_control_cycle);
    CONSOLE.println(" +- 10ms");
  }

  float deltaTime;  
  stateLocalizationMode = LOC_GPS;
  bool useGPSposition = true; // use GPS position?
  bool useGPSdelta = true; // correct yaw with gps delta estimation?
  bool useImuAbsoluteYaw = false; // use IMU yaw absolute value?

  // ------- lidar localization --------------------------
  #ifdef GPS_LIDAR
    useGPSdelta = false;
    useImuAbsoluteYaw = true;
  #endif      
  
  // ------- sideways guidance sheets ---------------------
  #ifdef DOCK_GUIDANCE_SHEET // use guidance sheet for docking/undocking?
    if ( maps.isBetweenLastAndNextToLastDockPoint() ){
      stateLocalizationMode = LOC_GUIDANCE_SHEET;
      useGPSposition = false;
      useGPSdelta = false;
      useImuAbsoluteYaw = false;
    }
  #endif

  // ------- vision (april-tag) --------------------------
  #ifdef DOCK_APRIL_TAG  // use april-tag for docking/undocking?
    if (maps.isBetweenLastAndNextToLastDockPoint() ){
      stateLocalizationMode = LOC_APRIL_TAG;
      useGPSposition = false;
      useGPSdelta = false;
      useImuAbsoluteYaw = false;
      if (stateAprilTagFound){  
        float robotX = stateXAprilTag; // robot-in-april-tag-frame (x towards outside tag, y left, z up)
        float robotY = stateYAprilTag;
        float robotDelta = scalePI(stateDeltaAprilTag);    
        /*CONSOLE.print("APRIL TAG found: ");      
        CONSOLE.print(robotX);
        CONSOLE.print(",");
        CONSOLE.print(robotY);
        CONSOLE.print(",");    
        CONSOLE.println(robotDelta/3.1415*180.0);*/        
        float dockX;
        float dockY;
        float dockDelta;
        if (maps.getDockingPos(dockX, dockY, dockDelta)){
          // transform robot-in-april-tag-frame into world frame
          float worldX = dockX + robotX * cos(dockDelta+3.1415) - robotY * sin(dockDelta+3.1415);
          float worldY = dockY + robotX * sin(dockDelta+3.1415) + robotY * cos(dockDelta+3.1415);            
          stateX = worldX;
          stateY = worldY;
          stateDelta = scalePI(robotDelta + dockDelta);
          if (DOCK_FRONT_SIDE) stateDelta = scalePI(stateDelta + 3.1415);
        }
      }
    }
  #endif

  // map dockpoints setup:                                  |===============================
  //               GPS waypoint         GPS waypoint    outside tag     
  //                 x----------------------x---------------------------------------------O (last dockpoint)
  //                                      outside                inside tag visible      inside tag
  //                                      tag visible       |===============================          
  //                                                           
  // localization:              GPS                     LiDAR          
  
  #ifdef DOCK_REFLECTOR_TAG  // use reflector-tag for docking/undocking?
    if (maps.isBetweenLastAndNextToLastDockPoint()) {
      if (maps.shouldDock) {
        stateReflectorTagOutsideFound = false;        
        stateReflectorUndockCompleted = false;
      }
      if ((stateReflectorTagOutsideFound) && (stateXReflectorTag > 0.5)) stateReflectorUndockCompleted = true;
      if (!stateReflectorUndockCompleted) { 
        stateLocalizationMode = LOC_REFLECTOR_TAG;
        useGPSposition = false;
        useGPSdelta = false;
        useImuAbsoluteYaw = false;
        if (stateReflectorTagFound){  
          // don't use stateXReflectorTag as we don't know which tag was detected   
          float robotX = stateXReflectorTag;  // robot-in-reflector-tag-frame (x towards outside tag, y left, z up)      
          if ((stateXReflectorTag > 0) && (stateXReflectorTagLast < 0)){
            if (!maps.shouldDock){
              stateReflectorTagOutsideFound = true;
            } 
          }        
          stateXReflectorTagLast = stateXReflectorTag;
          float robotY = stateYReflectorTag;
          float robotDelta = scalePI(stateDeltaReflectorTag);    
          /*CONSOLE.print("REFLECTOR TAG found: ");      
          CONSOLE.print(robotX);
          CONSOLE.print(",");
          CONSOLE.print(robotY);
          CONSOLE.print(",");    
          CONSOLE.println(robotDelta/3.1415*180.0);*/        
          float dockX;
          float dockY;
          float dockDelta;
          //int dockPointsIdx = maps.dockPoints.numPoints-1; 
          int dockPointsIdx = maps.dockPointsIdx;
          if (maps.getDockingPos(dockX, dockY, dockDelta, dockPointsIdx)){
            // transform robot-in-reflector-tag-frame into world frame
            robotX = 0.2;
            if (!maps.shouldDock) robotX = -0.2;  
            if (robotX < 0) {
              // flip robot at marker
              //robotX *= -1;
              //robotY *= -1;
            }
            float worldX = dockX + robotX * cos(dockDelta+3.1415) - robotY * sin(dockDelta+3.1415);
            float worldY = dockY + robotX * sin(dockDelta+3.1415) + robotY * cos(dockDelta+3.1415);            
            stateX = worldX;
            stateY = worldY;
            stateDelta = scalePI(robotDelta + dockDelta);
            if (DOCK_FRONT_SIDE) stateDelta = scalePI(stateDelta + 3.1415);
          }
        }
      }
    }
  #endif

  // ---------- odometry ticks ---------------------------
  int leftDelta = motor.ticksLeft;
  int rightDelta = motor.ticksRight;    
    
  float distLeft = ((float)leftDelta) / ((float)motor.ticksPerCm);
  float distRight = ((float)rightDelta) / ((float)motor.ticksPerCm);
  
  if ( (abs(distLeft) > 10.0) ||  (abs(distRight) > 10.0) ) {
    CONSOLE.print("computeRobotState WARN: distOdometry too large - distLeft=");
    CONSOLE.print(distLeft);
    CONSOLE.print(", distRight=");
    CONSOLE.println(distRight);
    distLeft = 0;
    distRight = 0;
  }  
  
  float distOdometry = (distLeft + distRight) / 2.0;  
  float deltaOdometry = -(distLeft - distRight) / motor.wheelBaseCm;    
  //CONSOLE.print("deltaOdometry: ");CONSOLE.println(deltaOdometry);
  // ---------- GPS relative/absolute position source -----------------
  float posN = 0;
  float posE = 0;

  deltaTime = (millis() - timeLastState)/1000.0; //this is in seconds
  /*
  float lp01 = 0.99;//1 -0.1*deltaTime; //Ultraslow longpass
  float lp1 = 0.9;//1 - 1*deltaTime;   
  float lp2 = 0.8;// - 2*deltaTime;
  float lp3 = 0.7;// - 3*deltaTime;
  float lp4 = 0.6;// - 4*deltaTime;   //with iterationtime of 20ms (50Hz), this will be about 0.92 for LP filter */
 
  if (absolutePosSource){
    relativeLL(absolutePosSourceLat, absolutePosSourceLon, gps.lat, gps.lon, posN, posE);    
  } else {
    posN = gps.relPosN;  
    posE = gps.relPosE;     
  }   
  
  if (fabs(motor.linearSpeedSet) < MOTOR_MIN_SPEED/2){       //0.001    //this may cause gps not updating when moving manually
    resetLastPos = true;
  }
  
  if (gps.solutionAvail && (gps.solution == SOL_FIXED || gps.solution == SOL_FLOAT)){
    gps.solutionAvail = false;
    lastSolutionTime = solutionTime;
    solutionTime = millis();
    solutionTimeDelta = solutionTime - lastSolutionTime;
    //CONSOLE.print("SolutionTimeDelta: ");
    //CONSOLE.println(solutionTimeDelta);        
    
    //stateGroundSpeed = lp2 * stateGroundSpeed + (1 - lp2) * gps.groundSpeed; //MrTree.. not sure why this is put into lowpassfilter, i am sure ublox already does all to give accurate groundspeed.... maybe just by habit
    stateGroundSpeed = gps.groundSpeed;         //0.7 * stateGroundSpeed + 0.3 * gps.groundSpeed;    
    
    float distGPS = sqrt( sq(posN-lastPosN)+sq(posE-lastPosE) );
    if ((distGPS > 0.3) || (resetLastPos)){
      if ((distGPS > 0.3) && (solutionTimeDelta < 350)) {  //consider the last available soulution time, pathfinder will raise solutionTimeDelta up to 1000ms
        gpsJump = true;
        statGPSJumps++;
        CONSOLE.print("GPS jump: ");
        CONSOLE.println(distGPS);
      }
      resetLastPos = false;
      lastPosN = posN;
      lastPosE = posE;
      lastPosDelta = stateDelta;
    } else if (distGPS > 0.1) {                                                                                         //if GPS moves
      float diffLastPosDelta = distancePI(stateDelta, lastPosDelta);                                                    //pi distance between actual and last angle
      if (fabs(diffLastPosDelta) /PI * 180.0 < 10){  // robot sensors indicate it is not turning                        //go on only if mower isnt rotating much
        if ( (fabs(motor.linearSpeedSet) > 0) && (fabs(motor.angularSpeedSet) /PI *180.0 < 45) ) {                      //go on only if mower shall move linear and is under certain rotationcommand....??
          stateDeltaGPS = scalePI(atan2(posN-lastPosN, posE-lastPosE));    
          if (motor.linearSpeedSet < 0) stateDeltaGPS = scalePI(stateDeltaGPS + PI);                                    // consider if driving reverse
          //stateDeltaGPS = scalePI(2*PI-gps.heading+PI/2);
          float diffDelta = distancePI(stateDelta, stateDeltaGPS);                 
          if (useGPSdelta){
            if (    ((gps.solution == SOL_FIXED) && (maps.useGPSfixForDeltaEstimation ))
                || ((gps.solution == SOL_FLOAT) && (maps.useGPSfloatForDeltaEstimation)) )
            {   // allows planner to use float solution?         
              if (fabs(diffDelta/PI*180) > 45){ // IMU-based heading too far away => use GPS heading
                stateDelta = stateDeltaGPS;
                stateDeltaIMU = 0;
              } else {
                // delta fusion (complementary filter, see above comment)
                stateDeltaGPS = scalePIangles(stateDeltaGPS, stateDelta);
                stateDelta = scalePI(fusionPI(0.9, stateDelta, stateDeltaGPS));               
              }            
            }
          }
        }
      }
      lastPosN = posN;
      lastPosE = posE;
      //stateHeading = (stateDelta - PI/2) * 180/PI;
      lastPosDelta = stateDelta;
    } 
    
    if (useGPSposition){
      if (gps.solution == SOL_FIXED) {
        // fix
        lastFixTime = millis();
        if (maps.useGPSfixForPosEstimation) {
          stateX = posE;
          stateY = posN;
        }        
      } else {
        // float
        if (maps.useGPSfloatForPosEstimation){ // allows planner to use float solution?
          stateX = posE;
          stateY = posN;              
        }
      }
    }
  } 

  
  /*
  // for testing lidar marker-based docking without GPS  
  #ifdef DOCK_REFLECTOR_TAG
    static bool initializedXYDelta = false;
    if (useGPSposition){
      if (!initializedXYDelta){
        CONSOLE.println("initializedXYDelta");
        stateX = 0;
        stateY = 0;
        stateDelta = 0;      
        initializedXYDelta = true;
      }
    }
  #endif
  */
  

  // odometry distances
  stateX += distOdometry/100.0 * cos(stateDelta);
  stateY += distOdometry/100.0 * sin(stateDelta);        
  if (stateOp == OP_MOW) statMowDistanceTraveled += distOdometry/100.0;
  
  //direction angle
  if ((imuDriver.imuFound) && (maps.useIMU)) {
    // IMU available and should be used by planner        
    if (useImuAbsoluteYaw){
      stateDelta = imuRawYaw_sc; //use fast lp here?
    } else {
      stateDelta = scalePI(stateDelta + stateDeltaIMU );          
    }     
  } else {
    // odometry
    stateDelta = scalePI(stateDelta + deltaOdometry);
  }
  //CONSOLE.print("stateDelta: ");CONSOLE.println(stateDelta);
  stateHeading = (stateDelta - PI/2) * 180/PI;
  
  //angular speed of IMU
  if (imuDriver.imuFound){
    CONSOLE.print("stateDeltaIMU: ");CONSOLE.println(stateDeltaIMU);

    //stateDeltaSpeedIMU = lp1 * stateDeltaSpeedIMU + (1 - lp1) * stateDeltaIMU / deltaTime; //0.99 * stateDeltaSpeedIMU + 0.01 * stateDeltaIMU / deltaTime; // IMU yaw rotation speed (20ms timestep)  
    stateDeltaSpeedIMU = imuLpfStateDeltaSpeed(stateDeltaIMU/deltaTime);
    CONSOLE.print("stateDeltaIMURaw: ");CONSOLE.println(stateDeltaIMU/deltaTime/PI*180);
    CONSOLE.print("stateDeltaIMU: ");CONSOLE.println(stateDeltaIMU/PI*180.0);
    CONSOLE.print("stateDeltaSpeedIMU: ");CONSOLE.println(stateDeltaSpeedIMU/PI*180.0);
  }

  /*
  //wheels dont seem to sync to imu, imu is late. try to sync imu and wheels rotation signal with a ringbuffer
  ringBuffer[bufInd] = deltaOdometry / deltaTime;       //fill buffer
  bufInd++;                                             //iterate
  if (bufInd == bufLen) bufInd = 0;                     //check overflow
  float stateDeltaSpeedWheelsSync = ringBuffer[bufInd]; //replace stateDeltaSpeedWheels with oldest element --> alway the next element that would come for index in next iteration
  */
  
  //angular Speed of wheels
  stateDeltaSpeedWheels = wheelsLpfDeltaSpeed(deltaOdometry/deltaTime);
  //stateDeltaSpeedWheels = lp2 * stateDeltaSpeedWheels + (1 - lp2) * deltaOdometry / deltaTime;

  //CONSOLE.println(stateDelta / PI * 180.0);
  stateDeltaIMU = 0;

  // compute yaw rotation speed (delta speed)
  // stateDelta can be calculated from odometrie or imu
  stateDeltaSpeed = (stateDelta - stateDeltaLast) / deltaTime;  // 20ms timestep
  
  //this is the wheels speed
  stateDeltaSpeed = stateLpfDeltaSpeed(stateDeltaSpeed);
  //stateDeltaSpeedLP = lp1 * stateDeltaSpeedLP + (1 - lp1) * fabs(stateDeltaSpeed); //stateDeltaSpeedLP * 0.95 + fabs(stateDeltaSpeed) * 0.05;     
  
  stateDeltaLast = stateDelta;

  if (imuDriver.imuFound) {
    // compute difference between IMU yaw rotation speed and wheels yaw rotation speed
    
    diffIMUWheelYawSpeed = stateDeltaSpeedIMU - stateDeltaSpeedWheels; //already 2 filtered values
    //diffIMUWheelYawSpeedLP = lp1 * diffIMUWheelYawSpeedLP + (1 - lp1) * fabs(diffIMUWheelYawSpeed);  //MrTree changed from diffIMUWheelYawSpeedLP = diffIMUWheelYawSpeedLP * 0.95 + fabs(diffIMUWheelYawSpeed) * 0.05;
  }

  // invalid position => reset to zero
  if ( (abs(stateX) > 10000) || (abs(stateY) > 10000) ){
    stateX = 0;
    stateY = 0;
  }

  /*if (DEBUG_STATE_ESTIMATOR) {
    CONSOLE.print("             deltaTime: ");CONSOLE.println(deltaTime);
    //CONSOLE.print("                imuyaw: ");CONSOLE.print(imuRawYaw_sc);                     CONSOLE.print("       statedeltayaw_IMU: ");CONSOLE.println(stateDeltaIMU);
    CONSOLE.print("            stateDelta: ");CONSOLE.print(stateDelta/180*PI);                 CONSOLE.print("           stateDeltaGps: ");CONSOLE.println(stateDeltaGPS);
    CONSOLE.print("        linearSpeedSet: ");CONSOLE.print(motor.linearSpeedSet);              CONSOLE.print("        stateGroundSpeed: ");CONSOLE.println(stateGroundSpeed);
    CONSOLE.print("       angularSpeedSet: ");CONSOLE.print(motor.angularSpeedSet/PI*180.0);    CONSOLE.print("         stateDeltaSpeed: ");CONSOLE.println(stateDeltaSpeed/PI*180); 
    CONSOLE.print(" stateDeltaSpeedWheels --> ");CONSOLE.print(stateDeltaSpeedWheels/PI*180.0);CONSOLE.print(" | ");CONSOLE.print(stateDeltaSpeedIMU/PI*180.0);CONSOLE.println(" <-- stateDeltaSpeedIMU");
    CONSOLE.print("  diffIMUWheelYawSpeed: ");CONSOLE.print(diffIMUWheelYawSpeed/PI*180.0);     CONSOLE.print("     stateDeltaSpeed_IMU: ");CONSOLE.println(stateDeltaSpeed/PI*180);
    CONSOLE.print("  diffIMUWheelYawSpeed: ");CONSOLE.println(diffIMUWheelYawSpeed/PI*180.0);
    CONSOLE.print("                      stateDeltaSpeedWheel/stateDeltaSpeedIMU: ");CONSOLE.println(stateDeltaSpeedWheels/(stateDeltaSpeedIMU + 0.00001));
    
  }*/

  timeLastState = millis();

  
}
