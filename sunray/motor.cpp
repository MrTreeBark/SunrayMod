// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "motor.h"
#include "config.h"
#include "helper.h"
#include "robot.h"
#include "Arduino.h"
#include "rcmodel.h"
#include <math.h>

/* //RPM Lowpassfilter
LowPassFilter lpfMotorLeftRpm(0.001);
LowPassFilter lpfMotorRightRpm(0.001);
LowPassFilter lpfMotorMowRpm(0.001); 
LowPassFilter lpfMotorMowRpmFast(0.0035);       //fast for adaptive speed

//Current Lowpassfilter
LowPassFilter lpfMotorLeftSense(0.001);
LowPassFilter lpfMotorRightSense(0.001);
LowPassFilter lpfMotorMowSense(0.001); 
LowPassFilter lpfMotorMowSenseFast(0.0035);   //fast for adaptive speed

//Pwm Lowpassfilter

LowPassFilter lpfMotorLeftPwm(0.0001);    //0.0001
LowPassFilter lpfMotorRightPwm(0.0001);
LowPassFilter lpfMotorMowPwm(0.0001);  */

void Motor::begin() {


  speedUpTrig = false;
  linearCurrSet = 0;
  waitSpinUp = false;
  motorMowRpmCheck = false;
  motorMowStallFlag = false;
  motorMowRpmError = false;
  motorMowSpunUp = false;
	pwmMax = MOTOR_PID_LIMIT;
  switchedOn = false;
  mowPowerMax = MOWPOWERMAX;
  mowPowerMin = MOWPOWERMIN;
  mowPwm = MOW_PWM_NORMAL;
  mowRpm = MOW_RPM_NORMAL;
  mowHeightMillimeter = 50;
  ticksPerRevolution = TICKS_PER_REVOLUTION;
  mowticksPerRevolution = MOTOR_MOW_TICKS_PER_REVOLUTION;
	wheelBaseCm = WHEEL_BASE_CM;    // wheel-to-wheel distance (cm) 36
  wheelDiameter = WHEEL_DIAMETER; // wheel diameter (mm)
  ticksPerCm = ((float)ticksPerRevolution) / (((float)wheelDiameter)/10.0) / PI;    // computes encoder ticks per cm (do not change)  

  ticksLeft = 0;
  ticksRight = 0;
  ticksMow = 0;

  motorLeftPID.Kp       = MOTOR_PID_KP;
  motorLeftPID.Ki       = MOTOR_PID_KI;
  motorLeftPID.Kd       = MOTOR_PID_KD;
  motorLeftPID.reset(); 
  motorRightPID.Kp       = MOTOR_PID_KP;
  motorRightPID.Ki       = MOTOR_PID_KI;
  motorRightPID.Kd       = MOTOR_PID_KD;
  motorRightPID.reset();
  motorMowPID.Kp       = MOWMOTOR_PID_KP;
  motorMowPID.Ki       = MOWMOTOR_PID_KI;
  motorMowPID.Kd       = MOWMOTOR_PID_KD;
  motorMowPID.reset();     	              

  motorLeftLpf.Tf = MOTOR_PID_LP;
  motorLeftLpf.reset();
  motorRightLpf.Tf = MOTOR_PID_LP;  
  motorRightLpf.reset();
  motorMowLpf.Tf = MOWMOTOR_PID_LP;
  motorMowLpf.reset();
  
  //RPM Lowpassfilter
  lpfMotorLeftRpm.Tf = 2;
  lpfMotorRightRpm.Tf = 2;
  lpfMotorMowRpm.Tf = 3; 
  lpfMotorMowRpmFast.Tf = 0.2;       //fast for adaptive speed
  lpfMotorLeftRpm.reset();
  lpfMotorRightRpm.reset();
  lpfMotorMowRpm.reset();
  lpfMotorMowRpmFast.reset();
  //Current Lowpassfilter
  lpfMotorLeftSense.Tf = 1;
  lpfMotorRightSense.Tf = 1;
  lpfMotorMowSense.Tf = 1; 
  lpfMotorMowSenseFast.Tf = 0.1;   //fast for adaptive speed
  lpfMotorLeftSense.reset();
  lpfMotorRightSense.reset();
  lpfMotorMowSense.reset();
  lpfMotorMowSenseFast.reset();
  //Pwm Lowpassfilter

  lpfMotorLeftPwm.Tf = 3;
  lpfMotorRightPwm.Tf = 3;
  lpfMotorMowPwm.Tf = 3;
  lpfMotorLeftPwm.reset();
  lpfMotorRightPwm.reset();
  lpfMotorMowPwm.reset();




  robotPitch = 0;
  #ifdef MOTOR_DRIVER_BRUSHLESS
    motorLeftSwapDir = true;
  #else
    motorLeftSwapDir = false;  
  #endif
  motorRightSwapDir = false;
  
  // apply optional custom motor direction swapping 
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    motorLeftSwapDir = !motorLeftSwapDir;
  #endif
  #ifdef MOTOR_RIGHT_SWAP_DIRECTION
    motorRightSwapDir = !motorRightSwapDir;
  #endif

  motorError = false;
  recoverMotorFault = false;
  recoverMotorFaultCounter = 0;
  nextRecoverMotorFaultTime = 0;
  enableMowMotor = ENABLE_MOW_MOTOR; //Default: true
  tractionMotorsEnabled = true;
  
  motorLeftOverload = false;
  motorRightOverload = false;
  motorMowOverload = false;
  
  odometryError = false;
  
  motorLeftSense = 0;
  motorRightSense = 0;
  motorMowSense = 0;
  motorLeftSenseLP = 0;
  motorRightSenseLP = 0;
  motorMowSenseLP = 0;
  motorsSenseLP = 0;

  activateLinearSpeedRamp = USE_LINEAR_SPEED_RAMP;
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  SpeedFactor = 1;
  motorLeftRpmSet = 0;
  motorRightRpmSet = 0;
  motorMowRpmSet = 0;
  mowPowerAct = 0;
  mowPowerActLP = 0;
  motorMowPwmSet = 0;
  motorMowPowerMax = 35;
  motorMowForwardSet = true;
  toggleMowDir = MOW_TOGGLE_DIR;

  lp005 = 0;
  lp01 = 0;
  lp1 = 0;
  lp2 = 0;
  lp3 = 0;
  lp4 = 0;

  deltaControlTime = 0;
  lastMowStallCheckTime = 0;
  nextOutputTime = 0;
  motorLeftTicks = 0;  
  motorRightTicks = 0;
  motorMowTicks = 0;
  motorLeftTicksZero = 0;
  motorRightTicksZero = 0;
  motorMowTicksZero = 0;
  motorLeftPwm = 0;    
  motorRightPwm = 0; 
  motorMowPwm = 0;
  motorLeftPwmLP = 0;
  motorRightPwmLP = 0;   
  motorMowPwmLP = 0;
  
  motorLeftRpm = 0;
  motorRightRpm = 0;
  motorMowRpm = 0;  
  motorLeftRpmLast = 0;
  motorRightRpmLast = 0;
  motorLeftRpmLP = 0;
  motorRightRpmLP = 0;
  motorMowRpmLP = 0;
  motorMowRpmLPFast = 0;
  
  setLinearAngularSpeedTimeoutActive = false;  
  setLinearAngularSpeedTimeout = 0;
  motorMowSpinUpTime = 0;

  mowRPM_RC = 0;
  mowPWM_RC = 0;
  
  motorRecoveryState = false;

  retryslow = false;
  keepslow = false;
  y_before = 100;
  keepslow_y = 100;
  
  motorMowStall = false;
  motorMowStallDuration = 0;

  drvfixtimer = DRVFIXTIMER;
  drvfixreset = false;
  drvfixcounter = 0;
}

int pwmCap(int val, int min, int max){
  if (val >= 0) {
    if (val < min) return 0;
    if (val > max) return max;
    return val;
  }
  if (val <= 0) {
    if (val > -min) return 0;
    if (val < -max) return -max;
    return val;
  }
  return 0;
}

void Motor::setMowPwm( int val ){
  CONSOLE.print("Motor::setMowPwm = ");
  CONSOLE.println(val);
  mowPwm = MOW_PWM_NORMAL;
}

bool Motor::waitMowMotor() {
  if (millis() < motor.motorMowSpinUpTime + MOWSPINUPTIME) {
    // wait until mowing motor is running or stopping
    if (!waitSpinUp) CONSOLE.println("Motor::waitMowMotor() wait for mowmotor....");  //one time message
    if (!buzzer.isPlaying()) buzzer.sound(SND_MOWSTART, true);
    
    waitSpinUp = true;
    motorMowSpunUp = false;
    return true;
  } //else waitSpinUp = false;
  if (waitSpinUp && switchedOn) { //Wait finished after switch on
    CONSOLE.println("Motor::waitMowMotor() finished switching on....");  //one time message
    waitSpinUp = false;
    //motorMowSpunUp = true;
    return false;
  }
  if (waitSpinUp && !switchedOn) { //Wait finished after switch off
    CONSOLE.println("Motor::waitMowMotor() finished switching off....");  //one time message
    waitSpinUp = false;
    motorMowSpunUp = false;
    return false;
  }
  return false;
}

void Motor::setMowHeightMillimeter( int val )
{
  CONSOLE.print("Motor::setMowHeightMillimeter ");
  CONSOLE.println(val);
  mowHeightMillimeter = val;
  motorDriver.setMowHeight(mowHeightMillimeter);
}

void Motor::speedPWM ( int pwmLeft, int pwmRight, int pwmMow )
{ 
  //Correct Motor Direction
  if (motorLeftSwapDir) pwmLeft *= -1;
  if (motorRightSwapDir) pwmRight *= -1;

  #ifdef MOTOR_MOW_SWAP_DIRECTION
    pwmMow *= -1;
  #endif
  
  //CONSOLE.println(pwmMow);
  
  /*
  // ensure pwm is lower than Max                      
  pwmLeft = min(pwmMax, max(-pwmMax, pwmLeft));
  pwmRight = min(pwmMax, max(-pwmMax, pwmRight));  
  pwmMow = min(pwmMax, max(-pwmMax, pwmMow)); 
  */
  bool releaseBrakes = false;  
  if (releaseBrakesWhenZero){
    if ((pwmLeft == 0) && (pwmRight == 0)){
      if (millis() > motorReleaseBrakesTime) releaseBrakes = true;
    } else {
      motorReleaseBrakesTime = millis() + 2000;
    }
  }
  motorDriver.setMotorPwm(pwmLeft, pwmRight, pwmMow, releaseBrakes);
}

// linear: m/s
// angular: rad/s
// -------unicycle model equations----------
//      L: wheel-to-wheel distance
//     VR: right speed (m/s)
//     VL: left speed  (m/s)
//  omega: rotation speed (rad/s)
//      V     = (VR + VL) / 2       =>  VR = V + omega * L/2
//      omega = (VR - VL) / L       =>  VL = V - omega * L/2
void Motor::setLinearAngularSpeed(float linear, float angular, bool useLinearRamp){  

  if (abs(linear) < MOTOR_MIN_SPEED * 0.5 && abs(linear) != 0) {
    linear = 0;     //illegal creeeping input, reset to zero
    CONSOLE.println("WARNING: Motor::setLinearAngularSpeed - illegal creeping input for linear speed! --> MOTOR_MINSPEED / 2 --> linear clamped to ZERO");
  }

  if (angular == 0) resetAngularMotionMeasurement();        // global reset
  if (linear == 0) resetLinearMotionMeasurement();          // global reset
  
  if (angular && linear != 0) {
    setLinearAngularSpeedTimeout = millis() + 1500;         // Changed to 2500 from 1000, this possibly causes the stop and go if mower is controlled manually over WiFi 
    setLinearAngularSpeedTimeoutActive = true;
  }

  float linearDelta = linear - linearSpeedSet;              // need a delta for triggering ramp
  if (linear > 0) linear = linear * adaptiveSpeed();        // adaptive speed for mowing operation only interferes when in forward drive
  linearCurrSet = linear;                                   // safe linear in a global before messing around with it, this is the "new" value, linearSpeedSet is the "old" value

  if (activateLinearSpeedRamp && useLinearRamp) {                             //this is a speed ramp for changes in speed during operation, to smooth transitions a little bit. needs to be quick
    float maxAcc = LINEAR_ACCEL;   // max Acceleration [mm/s²]
    float maxDec = LINEAR_DECEL;   // max Deceleration [mm/s²]          
    float dt = deltaControlTime / 1000.0; // deltaControlTime in seconds for accel ramp
    float maxStep = (linearDelta > 0) ? maxAcc * dt : maxDec * dt; // direction, accel or decel? compute maxStep

  if (fabs(linearDelta) > maxStep) {
    linear += (linearDelta > 0 ? 1 : -1) * maxStep; // use acc/dec ramp
  } else {
    linear = linearCurrSet; // no ramp needed
  }
    //we want to cut the ramp if linear is zero to improve pointprecision, this is a trick and it works!
    if (linearCurrSet == 0 && fabs(linear) < 2 * MOTOR_MIN_SPEED) linear = 0;
  }
 
  //linearSpeedSet = linearCurrSet; // if no ramp
  linearSpeedSet = linear; // if no ramp or a cut ramp or dec/acc ramp

  //Global linear Speedlimit 
  if (GLOBALSPEEDLIMIT) {                                                     //MrTree
    if (linearSpeedSet > 0){
      linearSpeedSet = constrain(linearSpeedSet, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    }
    if (linearSpeedSet < 0){
      linearSpeedSet = constrain(linearSpeedSet, -MOTOR_MAX_SPEED, -MOTOR_MIN_SPEED);
    }
  }

  angularSpeedSet = angular;
  float rspeed = linearSpeedSet + angularSpeedSet * (wheelBaseCm /100.0 /2);          
  float lspeed = linearSpeedSet - angularSpeedSet * (wheelBaseCm /100.0 /2);

  // RPM = V / (2*PI*r) * 60
  //CONSOLE.print("motorRightRpmSet: "); CONSOLE.println(motorRightRpmSet);
  //CONSOLE.print("motorLeftRpmSet: "); CONSOLE.println(motorLeftRpmSet);
  
  motorRightRpmSet =  rspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
  motorLeftRpmSet = lspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;
} 

void Motor::enableTractionMotors(bool enable){
  if (enable == tractionMotorsEnabled) return;
  if (enable)
    CONSOLE.println("traction motors enabled");
  else 
    CONSOLE.println("traction motors disabled");
  tractionMotorsEnabled = enable;
}

void Motor::setReleaseBrakesWhenZero(bool release){
  if (release == releaseBrakesWhenZero) return;
  if (release){
    motorReleaseBrakesTime = millis() + 2000;
    CONSOLE.println("traction motors will release brakes when zero (may only work on owlPlatform)");
  } else { 
    CONSOLE.println("traction motors will not release brakes when zero");
  }
  releaseBrakesWhenZero = release;
}

void Motor::setMowState(bool switchOn) {
  if (!USE_MOW_RPM_SET) {
    handleMowPwmMode(switchOn);
  } else {
    handleMowRpmMode(switchOn);
  }
}

void Motor::handleMowPwmMode(bool switchOn) {
  if (enableMowMotor && switchOn) {
    if (abs(motorMowPwmSet) > 0) {
      CONSOLE.println("Motor::setMowState PWM - Motor already switched on!");
      return;
    }

    switchedOn = true;
    mowPwm = MOW_PWM_NORMAL;
    motorMowSpinUpTime = millis();

    if (!buzzer.isPlaying()) buzzer.sound(SND_MOWSTART, true);

    if (toggleMowDir) {
      motorMowForwardSet = !motorMowForwardSet;
      motorMowPwmSet = motorMowForwardSet ? mowPwm : -mowPwm;
    } else {
      motorMowPwmSet = mowPwm;
    }

    CONSOLE.print("Motor::setMowState ");
    CONSOLE.print(switchOn);
    CONSOLE.print(" PWM: ");
    CONSOLE.println(motorMowPwmSet);

  } else if (switchedOn) {
    motorMowSpinUpTime = millis();
    motorMowPwmSet = 0;
    motorMowRpmSet = 0;
    switchedOn = false;
    motorMowSpunUp = false;
    CONSOLE.println("Motor::setMowState PWM OFF");

  } else {
    CONSOLE.println("Motor::setMowState PWM - Motor already switched off!");
  }
}

void Motor::handleMowRpmMode(bool switchOn) {
  if (enableMowMotor && switchOn) {
    if (abs(motorMowRpmSet) > 0) {
      CONSOLE.println("Motor::setMowState RPM - Motor already switched on!");
      return;
    }

    switchedOn = true;
    motorMowSpinUpTime = millis();

    if (!buzzer.isPlaying()) buzzer.sound(SND_MOWSTART, true);

    if (toggleMowDir) {
      motorMowForwardSet = !motorMowForwardSet;
      motorMowRpmSet = motorMowForwardSet ? mowRpm : -mowRpm;
    } else {
      motorMowRpmSet = mowRpm;
    }

    CONSOLE.print("Motor::setMowState ");
    CONSOLE.print(switchOn);
    CONSOLE.print(" RPM: ");
    CONSOLE.println(motorMowRpmSet);

  } else if (switchedOn) {
    CONSOLE.println("Motor::setMowState RPM OFF");
    motorMowSpinUpTime = millis();
    motorMowRpmSet = 0;
    switchedOn = false;
    motorMowSpunUp = false;

  } else {
    CONSOLE.println("Motor::setMowState RPM - Motor already switched off!");
  }
}

void Motor::stopImmediately(bool includeMowerMotor){
  CONSOLE.println("motor.cpp - STOP IMMEDIATELY");
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorLeftRpmSet = 0; 
  motorRightRpmSet = 0;   
  motorLeftPwm = 0;
  motorRightPwm = 0;
   
  if (includeMowerMotor) {
    #ifdef DRV_SERIAL_ROBOT
      motorMowPwm = 0;//MrTree
      motorMowRpm = 0;//MrTree
      //switchedOn = false;
    #endif
    //switchedOn = false;
    motorMowSpunUp = false;  
    motorMowPwmSet = 0;
    //motorMowPwm = 0;//MrTree
    motorMowRpmSet = 0;//MrTree
    //motorMowRpm = 0;//MrTree
    setMowState(false);    
  }
  setLinearAngularSpeed(0, 0, false);  
  speedPWM(motorLeftPwm, motorRightPwm, motorMowPwm);
  // reset PID
  motorLeftPID.reset();
  motorRightPID.reset();
  motorMowPID.reset();//MrTree
  motorLeftLpf.reset();
  motorRightLpf.reset();
  motorMowLpf.reset();
  // reset unread encoder ticks
  ticksLeft=0;
  ticksRight=0;
  ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);        
}

void Motor::run() {

  static unsigned long lastControlTime = 0;
  unsigned long controlTime = millis();
  deltaControlTime = (controlTime - lastControlTime);  // in Sekunden
  lastControlTime = controlTime;

  if (DEBUG_TIMING) {
    if (deltaControlTime < robot_control_cycle - 10 || deltaControlTime > robot_control_cycle + 10) {
      CONSOLE.print("WARNING motor control: unmatched cycletime --> "); 
      CONSOLE.print(deltaControlTime);
      CONSOLE.print("  robot_control_cycle = ");
      CONSOLE.print(robot_control_cycle);
      CONSOLE.println(" +- 10ms");
    }
  }

  //if (deltaControlTime < 15) return;


  //Change to stopimmediately!
  if (setLinearAngularSpeedTimeoutActive){
    if (millis() > setLinearAngularSpeedTimeout){
      //CONSOLE.println("Motor::run - LinearAngularSpeedTimeout");
      setLinearAngularSpeedTimeoutActive = false;
      setLinearAngularSpeed(0, 0);
      linearCurrSet = 0;
      linearSpeedSet = 0;
      motorLeftRpmSet = 0;
      motorRightRpmSet = 0;
    }
  }
    
  sense();
  changeSpeedSet();             //MrTree
  checkMotorMowStall();         //MrTree
  drvfix();
  
  //CONSOLE.print("mowSpinUp: ");
  //CONSOLE.println(motor.waitSpinUp);
  // if motor driver indicates a fault signal, try a recovery   
  // if motor driver uses too much current, try a recovery     
  // if there is some error (odometry, too low current, rpm fault), try a recovery 
  if (!recoverMotorFault) {
    bool someFault = ( (checkFault()) || (checkCurrentTooHighError()) || (checkMowRpmFault()) 
                         || (checkOdometryError()) || (checkCurrentTooLowError()) );
    if (someFault){
      stopImmediately(true);
      recoverMotorFault = true;
      nextRecoverMotorFaultTime = millis() + 1000;                  
      motorRecoveryState = true;
    } 
  } 

  // try to recover from a motor driver fault signal by resetting the motor driver fault
  // if it fails, indicate a motor error to the robot control (so it can try an obstacle avoidance)  
  if (nextRecoverMotorFaultTime != 0){
    if (millis() > nextRecoverMotorFaultTime){
      if (recoverMotorFault){
        nextRecoverMotorFaultTime = millis() + 10000;
        recoverMotorFaultCounter++;                                               
        CONSOLE.print("motor fault recover counter ");
        CONSOLE.println(recoverMotorFaultCounter);
        //stopImmediately(true);
        motorDriver.resetMotorFaults();
        recoverMotorFault = false;  
        if (recoverMotorFaultCounter >= 6){ // too many successive motor faults
          //stopImmediately(true);
          CONSOLE.println("ERROR: motor recovery failed");
          recoverMotorFaultCounter = 0;
          motorError = true;
        }
      } else {
        CONSOLE.println("resetting recoverMotorFaultCounter");
        recoverMotorFaultCounter = 0;
        nextRecoverMotorFaultTime = 0;
        motorRecoveryState = false;
      }        
    }
  }
  
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  
  if (motorLeftPwm < 0) ticksLeft *= -1;
  if (motorRightPwm < 0) ticksRight *= -1;
  if (motorMowPwm < 0) ticksMow *= -1;
  
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;

 /*  CONSOLE.print("leftTicks: ");
  CONSOLE.print(ticksLeft);
  CONSOLE.print("  rightTicks: ");
  CONSOLE.print(ticksRight);
  CONSOLE.print("   mowTicks: ");
  CONSOLE.println(ticksMow);

  CONSOLE.print("   mowMowTicks: ");
  CONSOLE.println(motorMowTicks);*/

  // calculate speed via tick count
  // 2000 ticksPerRevolution: @ 30 rpm  => 0.5 rps => 1000 ticksPerSec
  // 20 ticksPerRevolution: @ 30 rpm => 0.5 rps => 10 ticksPerSec
  if (deltaControlTime > 0) {

    motorLeftRpm = 60.0 * ( (float)ticksLeft / (float)ticksPerRevolution ) * (1000 / (float)deltaControlTime);
    motorRightRpm = 60.0 * ( (float)ticksRight / (float)ticksPerRevolution ) * (1000 / (float)deltaControlTime);
    motorMowRpm = 60.0 * ( (float)ticksMow / (float)mowticksPerRevolution ) * (1000 / (float)deltaControlTime);
  
    if (isfinite(motorLeftRpm)) motorLeftRpmLP = lpfMotorLeftRpm(motorLeftRpm);
    if (isfinite(motorRightRpm)) motorRightRpmLP = lpfMotorRightRpm(motorRightRpm);
    if (isfinite(motorMowRpm)) {
      motorMowRpmLP = lpfMotorMowRpm(motorMowRpm); 
      motorMowRpmLPFast = lpfMotorMowRpmFast(motorMowRpm);
    } 
  }
  
  //CONSOLE.print("   mowRpm: ");CONSOLE.print(motorMowRpm);CONSOLE.print("   mowRpmLP: ");CONSOLE.print(motorMowRpmLP);CONSOLE.print("   mowRpmLPFast: ");CONSOLE.println(motorMowRpmLPFast);

  if (ticksLeft == 0) {
    motorLeftTicksZero++;
    if (motorLeftTicksZero > 50) {
      motorLeftRpm = 0;
      motorLeftRpmLP = 0;
      motorLeftTicksZero = 0;
    } 
  } else motorLeftTicksZero = 0;

  if (ticksMow == 0) {
    motorMowTicksZero++;
    if (motorMowTicksZero > 50) {
      motorMowRpm = 0;
      motorMowRpmLP = 0;
      motorMowTicksZero = 0;
    } 
  } else motorLeftTicksZero = 0;

  if (ticksRight == 0) {
    motorRightTicksZero++;
    if (motorRightTicksZero > 50) {
      motorRightRpm = 0;
      motorRightRpmLP = 0;
      motorRightTicksZero = 0;
    }
  } else motorRightTicksZero = 0;
  
  // speed controller
  
  control();

  motorLeftRpmLast = motorLeftRpm;
  motorRightRpmLast = motorRightRpm;

  //DEBUG OUTPUT


  if (DEBUG_MOTORCONTROL && millis() > nextOutputTime){
    nextOutputTime = millis() + DEBUG_MOTOR_CONTROL_TIME;
    CONSOLE.println("     motor.cpp --------------------------------> ");
    CONSOLE.println(" ");
    CONSOLE.print("                   PWMCurr (l,r,m) = ");  CONSOLE.print(motorLeftPwm);  CONSOLE.print(", ");  CONSOLE.print(motorRightPwm);    CONSOLE.print(", ");CONSOLE.println(motorMowPwm);
    CONSOLE.print("               motor*PWMLP (l,r,m) = ");CONSOLE.print(motorLeftPwmLP);  CONSOLE.print(", ");  CONSOLE.print(motorRightPwmLP);  CONSOLE.print(", ");CONSOLE.println(motorMowPwmLP);
    CONSOLE.print("             motor*SenseLP (l,r,m) = ");CONSOLE.print(motorLeftSenseLP);CONSOLE.print(", ");  CONSOLE.print(motorRightSenseLP);CONSOLE.print(", ");CONSOLE.println(motorMowSenseLP);
    CONSOLE.println("     <------------------------------------------ ");
  }
}  

// check if motor current too high
bool Motor::checkCurrentTooHighError(){
  bool motorLeftFault = (motorLeftSenseLP > MOTOR_FAULT_CURRENT);
  bool motorRightFault = (motorRightSenseLP > MOTOR_FAULT_CURRENT);
  bool motorMowFault = (motorMowSenseLP > MOW_FAULT_CURRENT);
  if (motorLeftFault || motorRightFault || motorMowFault){
    CONSOLE.print("ERROR motor current too high: ");
    CONSOLE.print("  current=");
    CONSOLE.print(motorLeftSenseLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSenseLP);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSenseLP);
    return true;
  } 
  return false; 
}

// check if motor current too low
bool Motor::checkCurrentTooLowError(){

  if  (    ( (abs(motorMowPwm) > 100) && (abs(motorMowPwmLP) > 100) && (motorMowSenseLP < MOW_TOO_LOW_CURRENT)) 
        ||  ( (abs(motorLeftPwm) > 100) && (abs(motorLeftPwmLP) > 100) && (motorLeftSenseLP < MOTOR_TOO_LOW_CURRENT))    
        ||  ( (abs(motorRightPwm) > 100) && (abs(motorRightPwmLP) > 100) && (motorRightSenseLP < MOTOR_TOO_LOW_CURRENT))  ){        
    // at least one motor is not consuming current      
    // first try recovery, then indicate a motor error to the robot control (so it can try an obstacle avoidance)    
    CONSOLE.print("ERROR: motor current too low: PWM (left,right,mow)=");
    CONSOLE.print(motorLeftPwm);
    CONSOLE.print(",");
    CONSOLE.print(motorRightPwm);
    CONSOLE.print(",");
    CONSOLE.print(motorMowPwm);
    CONSOLE.print("  motor*PWMLP (left,right,mow)=");
    CONSOLE.print(motorLeftPwmLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightPwmLP);
    CONSOLE.print(",");
    CONSOLE.print(motorMowPwmLP);
    CONSOLE.print("  average current amps (left,right,mow)=");
    CONSOLE.print(motorLeftSenseLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSenseLP);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSenseLP);
    return true;
  }
  return false;
}

// check motor driver (signal) faults
bool Motor::checkFault(){
  bool fault = false;
  bool leftFault = false;
  bool rightFault = false;
  bool mowFault = false;
  if (ENABLE_FAULT_DETECTION){    
    motorDriver.getMotorFaults(leftFault, rightFault, mowFault);
  }
  if (leftFault) {
    CONSOLE.println("Error: motor driver left signaled fault");
    fault = true;
  }
  if  (rightFault) {
    CONSOLE.println("Error: motor driver right signaled fault"); 
    fault = true;
  }
  if (mowFault) {
    CONSOLE.println("Error: motor driver mow signaled fault");
    fault = true;
  }
  return fault;
}

// check odometry errors
bool Motor::checkOdometryError(){
  if (ENABLE_ODOMETRY_ERROR_DETECTION){
    if  (   ( (abs(motorLeftPwm) > 100) && (abs(motorLeftPwmLP) > 100) && (abs(motorLeftRpmLP) < 0.001))    
        ||  ( (abs(motorRightPwm) > 100) && (abs(motorRightPwmLP) > 100) && (abs(motorRightRpmLP) < 0.001))  )
    {               
      // odometry error
      CONSOLE.print("ERROR: odometry error - rpm too low (left, right)=");
      CONSOLE.print(motorLeftRpmLP);
      CONSOLE.print(",");
      CONSOLE.println(motorRightRpmLP);     
      return true;        
    }
  }
  return false;
}

// check motor overload
void Motor::checkOverload(){
  motorLeftOverload = (motorLeftSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorRightOverload = (motorRightSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorMowOverload = (motorMowSenseLP > MOW_OVERLOAD_CURRENT);

/*CONSOLE.print("motorLeftSenseLP    ");
  CONSOLE.print(motorLeftSenseLP);
  CONSOLE.print("  motorRightSenseLP   ");
  CONSOLE.println(motorRightSenseLP);*/

  if (motorLeftOverload || motorRightOverload || motorMowOverload){                   //MrTree
    if (motorOverloadDuration == 0){
      CONSOLE.print("WARNING motor overload (average current too high) - duration=");
      CONSOLE.print(motorOverloadDuration);
      CONSOLE.print("  avg current amps (left,right,mow)=");
      CONSOLE.print(motorLeftSenseLP);
      CONSOLE.print(",");
      CONSOLE.print(motorRightSenseLP);
      CONSOLE.print(",");
      CONSOLE.println(motorMowSenseLP);
    }
    motorOverloadDuration += 20;     
  } else {
    motorOverloadDuration = 0;
  }
}

// Adaptive_Speed: ramp mowingspeed with mow motor rpm or mow motor power, handles also 3 speed stages (retryslow, keepslow, normal)         
float Motor::adaptiveSpeed(){
  if (ADAPTIVE_SPEED){
    //returns
    if (!switchedOn) {
      return 1;
    }

    //prepare variables
    float x = 0;
    float x1 = 0;
    float x2 = 0;                                                    
    float y = 0;
    float y1 = 0;
    float y2 = 0;

    if (MOWPOWERMAX_AUTO) mowPowerMax = motorMowPowerMax;

    if (ADAPTIVE_SPEED_MODE == 1) {
      x = mowPowerAct * 1000;
      x1 = mowPowerMin * 1000;
      x2 = mowPowerMax * 1000;
      if (x2 < x1) x2 = motorMowPowerMax * 1000;                                       //safety first, if misconfigured use auto track

      if (ADAPTIVE_SPEED_USE_MINSPEED) y1 = ADAPTIVE_SPEED_MINSPEED/linearCurrSet*100; //static linear goal
      else y1 = MOTOR_MIN_SPEED/linearCurrSet*100;                                     //static linear goal
      y2 = 100; 
      x = ((x2 - x1) * sqrt(x / x2)); //quadratic slowdowm
      
    } 

    if (ADAPTIVE_SPEED_MODE == 2) {
      x = abs(motorMowRpmLPFast) * 1000;
      x1 = (abs(motorMowRpmSet) * (MOW_RPMtr_SLOW+5)/100) * 1000; //add some offset to trigger slow, because we dont want to trigger slowstate but become slow before that happens
      x2 = (abs(motorMowRpmSet) - MOW_RPM_DEADZONE) * 1000;         
      y1 = 100;
      if (ADAPTIVE_SPEED_USE_MINSPEED) y2 = ADAPTIVE_SPEED_MINSPEED/linearCurrSet*100; //linearSpeedSet
      else y2 = MOTOR_MIN_SPEED/linearCurrSet*100; //linearSpeedSet
      if (TEST_WAIT_BEFORE_REVERSE && (abs(motorMowRpmLPFast) > (abs(motorMowRpmSet) * (MOW_RPMtr_STALL)/100)) && (abs(motorMowRpmLPFast) < (abs(motorMowRpmSet) * (MOW_RPMtr_STALL+MOW_RPMtr_WAITZONE)/100))){
        //waitOp.waitTime = 3000;
        triggerWaitCommand(3000); 
      }
  
    }
    y = map(x, x1, x2, y2, y1);
    
    //CONSOLE.print("y= "); CONSOLE.print(y);CONSOLE.print("   x= "); CONSOLE.print(x); CONSOLE.print(" x1= ");CONSOLE.print(x1); CONSOLE.print(" x2= ");CONSOLE.print(x2);CONSOLE.print(" y2= ");CONSOLE.print(y2);CONSOLE.print(" y1= ");CONSOLE.println(y1);                             
    y = constrain(y, MOTOR_MIN_SPEED/linearCurrSet*100, 100);     //limit val
    
    if (y > y_before) y = y_before + 5*deltaControlTime; //0.2;//y = 0.995 * y_before + 0.005 * y; //if speed was reduced use a ramp for getting faster only (smoothing)
    //if ((speedUpTrig)&&(linear >= linearCurrSet)) linear = 0.998 * linearSpeedSet + 0.002 * linear; //Speed up slow if triggered from adaptive speed function
    y_before = y;
    SpeedFactor = y/100.0;                                        //used for linear modifier as: MOTOR_MIN_SPEED/setSpeed*100 < Speedfactor <= 1

    return SpeedFactor;

  } else {                                                        //MrTree adaptive speed is not activated
    return 1;
  }
}  

void Motor::changeSpeedSet(){

  if (!CHANGE_SPEED_SET) return;
  if (!switchedOn || (millis() < motor.motorMowSpinUpTime + MOWSPINUPTIME)) {
      keepslow = false;
      retryslow = false;
      keepSlowTime = 0;
      retrySlowTime = 0;
      speedUpTrig = false;
      return;
    }

  //prepare variables
  float slowtrig = 0;
  static bool triggerFlag;
  //float retrytrig = 0;
  float controlval = 0;
  int mownormal = 0;
  int mowslow = 0;
  int mowretry = 0;
  int mowset = 0;

  if (USE_MOW_RPM_SET){
    slowtrig = MOW_RPM_NORMAL * MOW_RPMtr_SLOW/100;
    controlval = abs(motorMowRpmLPFast);
    mownormal = MOW_RPM_NORMAL;
    mowslow = MOW_RPM_SLOW;
    mowretry = MOW_RPM_RETRY;
    mowset = motorMowRpmSet;
    triggerFlag = controlval < slowtrig;  //trigger by rpm percentage
  } else {
    if (MOWPOWERMAX_AUTO) mowPowerMax = motorMowPowerMax;
      else slowtrig = mowPowerMax * MOW_POWERtr_SLOW / 100.0;
    controlval = mowPowerAct;
    mownormal = MOW_PWM_NORMAL;
    mowslow = MOW_PWM_SLOW;
    mowretry = MOW_PWM_RETRY;
    mowset = motorMowPwmSet;
    triggerFlag = controlval > slowtrig;                //trigger with power higher trigger setpoint
  }

  if (keepslow && retryslow) keepslow = false;             //reset keepslow because retryslow is prior
  
  if (triggerFlag){                              //trigger and set timer once, 
    if ((!keepslow) && (!retryslow)){                      //only if not already trigged
      CONSOLE.println("Adaptive_Speed: Keeping or retrying slow!");
      CONSOLE.print("           controlVal = ");CONSOLE.println(controlval);
      CONSOLE.print("             slowtrig = ");CONSOLE.println(slowtrig);
      
      keepslow = true;                                          //enable keepslow state
      
      if (abs(mowset) != mowslow){                              //set the keepslow rpm
        if (USE_MOW_RPM_SET){
          CONSOLE.println("Adaptive_Speed: Using MOW_RPM_SLOW");
          if (motorMowRpmSet > 0) motorMowRpmSet = mowslow;                    //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing?
          if (motorMowRpmSet < 0) motorMowRpmSet = -mowslow;                   //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing?    
        } else {
          CONSOLE.println("Adaptive_Speed: Using MOW_PWM_SLOW");
          if (motorMowPwmSet > 0) motorMowPwmSet = mowslow;                    //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing?
          if (motorMowPwmSet < 0) motorMowPwmSet = -mowslow;                   //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing? 
        }                                                   
      }
    }                                                                  //step out of trigger condition
    if (keepslow) keepSlowTime = millis()+KEEPSLOWTIME;                //set or refresh keepslowtimer
    if (retryslow) retrySlowTime = millis()+RETRYSLOWTIME;             //if we already are in retryslow condition, we refresh the timer of retry also                               
  }                                          
  
  if (retryslow){                                                 //retryslow is triggered by the end of escapelawnop                       
    if (abs(mowset) != mowretry) {
      if (USE_MOW_RPM_SET){
        CONSOLE.println("Adaptive_Speed: Using MOW_RPM_RETRY");
        if (motorMowRpmSet > 0) motorMowRpmSet = mowretry; 
        if (motorMowRpmSet < 0) motorMowRpmSet = -mowretry;
      } else {
        CONSOLE.println("Adaptive_Speed: Using MOW_PWM_RETRY");
        if (motorMowPwmSet > 0) motorMowPwmSet = mowretry; 
        if (motorMowPwmSet < 0) motorMowPwmSet = -mowretry; 
      }                      
    }            
    if (millis() > retrySlowTime) {
      CONSOLE.println("Adaptive_Speed: Retryslow done! Going to Keepslow!");
      retryslow = false;
      keepslow = true;
      keepSlowTime = millis()+KEEPSLOWTIME;
    }
  }

  if (keepslow){                                                //keepslow is triggered in this function, retryslow in escapelawnOp   
    if (millis() > keepSlowTime) {
      CONSOLE.println("Adaptive_Speed: Keepslow done!");
      CONSOLE.println("Adaptive_Speed: Speeding up!");
      keepslow = false;                                         //reset keepslow if no rpm stall or high mowmotorpower     
      speedUpTrig = true;
    }
  }

  if (speedUpTrig){
    if (millis() > keepSlowTime + KEEPSLOWTIME) {
      CONSOLE.println("Adaptive_Speed: Speeding up done!");
      if (USE_MOW_RPM_SET) 
        CONSOLE.println("Adaptive_Speed: Using MOW_RPM_NORMAL");
      else 
        CONSOLE.println("Adaptive_Speed: Using MOW_PWM_NORMAL");
      if (USE_MOW_RPM_SET){
        if (motorMowRpmSet > 0) motorMowRpmSet = mownormal;
        if (motorMowRpmSet < 0) motorMowRpmSet = -mownormal;
      } else {
        if (motorMowPwmSet > 0) motorMowPwmSet = mownormal;
        if (motorMowPwmSet < 0) motorMowPwmSet = -mownormal;
      }
      speedUpTrig = false;
    }
  }
}

// check mow motor RPM stalls                                                          
void Motor::checkMotorMowStall(){ 
  if (ESCAPE_LAWN && switchedOn && !RC_Mode && !robotShouldWait()) {
    static unsigned long lastStalltime = 0;
    unsigned long deltaStallTime = (millis() - lastMowStallCheckTime);
    lastMowStallCheckTime = millis();
    
    if (millis() < lastStalltime + BUMPER_DEADTIME || bumper.obstacle()) return;

    //initial Message when mowmotor is turned on
    if (ESCAPE_LAWN_MODE == 1){
      if (!motorMowSpunUp){
        motorMowSpunUp = true;    
        CONSOLE.println("checkMotorMowStall: Using mow motor power for triggering escape lawn.");
        if (MOWPOWERMAX_AUTO)    
          CONSOLE.println("checkMotorMowStall: MOWPOWERMAX_AUTO is enabled. Using max calculated Mowpower from batVoltage and mowMotor amps during operation.");
      }
      
    }

    //initial Message when mowmotor is turned on
    if (ESCAPE_LAWN_MODE == 2) {  //MrTree in RPM mode we put out some messages with rpm data to use for configuration and information 
      if (!motorMowSpunUp){
        //if (!USE_MOW_RPM_SET) motorMowRpmSet = motorMowRpmLPFast;
        if ((abs(motorMowRpmLPFast) < abs(motorMowRpmSet)-150) || (abs(motorMowRpmLPFast) > abs(motorMowRpmSet)+150)){
          motorMowSpunUp = true;    
          CONSOLE.println("checkMotorMowStall: WARNING mow motor did not spun up to RPM set, ignore small values.");
          CONSOLE.println("Increase SPINUPTIME or consider mow motor not reaching or overspinning specific RPM set by more than +- 150 RPM.");
          CONSOLE.print("DATA: SPINUPTIME (ms), driverPWM, mowRPM, mowRPMSet, RPM difference: ");
          CONSOLE.print(MOWSPINUPTIME);
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowPwm));
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowRpmLPFast));
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowRpmSet));
          CONSOLE.print(", ");
          CONSOLE.println(abs(motorMowRpmSet)-abs(motorMowRpmLPFast));
        } else {
          motorMowSpunUp = true;
          CONSOLE.println("checkMotorMowStall: Mow motor Spun up!");
          CONSOLE.print("DATA: SPINUPTIME (ms), driverPWM, mowRPM, mowRPMSet: ");
          CONSOLE.print(MOWSPINUPTIME);
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowPwm));
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowRpmLPFast));
          CONSOLE.print(", ");
          CONSOLE.println(abs(motorMowRpmSet));
        }
      } 
    }

    //checking the stall condition
    if (USE_MOW_RPM_SET) {
        motorMowStall = (abs(motorMowRpmLPFast) < (abs(MOW_RPM_NORMAL)*MOW_RPMtr_STALL/100));  //LPFast? // changed motorMowRpmSet to mowRpm
      } else {
        if (MOWPOWERMAX_AUTO) motorMowStall = (mowPowerAct > motorMowPowerMax * MOW_POWERtr_STALL/100);
        else motorMowStall = (mowPowerAct > mowPowerMax * MOW_POWERtr_STALL/100);
      }
    
    //output of stallcondition, counting time and output when condition is over
    if ((motorMowStall) ){
      lastStalltime = millis();
      if (motorMowStallDuration == 0)CONSOLE.println("checkMotorMowStall:: mow motor stalled!");
      motorMowStallFlag = true;
      motorMowStallDuration += deltaStallTime;     
    } else {
      if (motorMowStallDuration != 0){
        motorMowStallFlag = false;         
        CONSOLE.print("checkMotorMowStall: WARNING mow motor stalled for duration(ms) >= ");
        CONSOLE.println(motorMowStallDuration);
        if (ESCAPE_LAWN_MODE == 2){
          CONSOLE.print("Data: Trigger RPM, Mow RPM: ");
          CONSOLE.print((abs(MOW_RPM_NORMAL)*MOW_RPMtr_STALL/100));
          CONSOLE.print(", ");
          CONSOLE.println(abs(motorMowRpmSet));
        }     
        if (ESCAPE_LAWN_MODE == 1){
          CONSOLE.print("Data: POWERtrigger, MowPower: ");
          CONSOLE.print(motorMowPowerMax*MOW_POWERtr_STALL/100);
          CONSOLE.print(", ");
          CONSOLE.println(mowPowerAct);
        }      
      }
      motorMowStallDuration = 0;
    }
  
    if (DEBUG_MOTOR_MOWSTALL && millis() > nextOutputTime) {
      nextOutputTime = millis() + DEBUG_MOTOR_CONTROL_TIME;
      CONSOLE.println("     motor.cpp / checkMowMotorStall -----------> ");
      CONSOLE.println(" ");
      CONSOLE.print("            motorMowStall: ");CONSOLE.print(motorMowStall);                      CONSOLE.print(" bool,            mowPowerAct: ");CONSOLE.print(mowPowerAct);                                           CONSOLE.print(" Watt,   motorMowPowerMax: ");CONSOLE.print(motorMowPowerMax);CONSOLE.println(" Watt");
      CONSOLE.print("          mowStallTrigger: ");CONSOLE.print(mowPowerMax * MOW_POWERtr_STALL/100);CONSOLE.print(" Watt,   mowStallTrigger_AUTO: ");CONSOLE.print(mowPowerAct > motorMowPowerMax * MOW_POWERtr_STALL/100);CONSOLE.print(" bool,       AUTO enabled: ");CONSOLE.print(MOWPOWERMAX_AUTO);CONSOLE.println("   bool");
      CONSOLE.println("");
      CONSOLE.println("     <------------------------------------------ ");
    }

    return;
  }
  motorMowStall = false;
  return;
}

void Motor::drvfix(){
  if (DRV8308_FIX) {
    if (drvfixreset) {
      drvfixcounter++;
      if (drvfixcounter >= DRVFIXITERATIONS) {
        speedPWM(0, 0, 0);
        drvfixreset = false;
        drvfixcounter = 0;
      }
    }
    if ((millis() >= drvfixtimer) && (battery.chargerConnected()) && (motorLeftPwm == 0) && (motorRightPwm == 0) && (motorMowPwm == 0)){
      drvfixtimer = millis() + DRVFIXTIMER;
      speedPWM(PWM_GEAR, PWM_GEAR, PWM_MOW);
      drvfixreset = true;
    }  
    return;
  } else {
    return;
  }
}

bool Motor::checkMowRpmFault(){
  if (ENABLE_RPM_FAULT_DETECTION){
    if  ( (abs(motorMowPwm) > 100) && (abs(motorMowPwmLP) > 100) && (abs(motorMowRpmLP) < 10.0)) {        
      CONSOLE.print("ERROR: mow motor, average rpm too low: pwm=");
      CONSOLE.print(motorMowPwm);
      CONSOLE.print("  pwmLP=");      
      CONSOLE.print(motorMowPwmLP);      
      CONSOLE.print("  rpmLP=");
      CONSOLE.print(motorMowRpmLP);
      CONSOLE.println("  (NOTE: choose ENABLE_RPM_FAULT_DETECTION=false in config.h, if your mowing motor has no rpm sensor!)");
      return true;
    }
  }  
  return false;
}

// measure motor currents
void Motor::sense(){
  motorDriver.getMotorCurrent(motorLeftSense, motorRightSense, motorMowSense);
  
  motorLeftSenseLP = lpfMotorLeftSense(motorLeftSense);
  motorRightSenseLP = lpfMotorRightSense(motorRightSense);
  motorMowSenseLP = lpfMotorMowSense(motorMowSense);

 /*  motorLeftSenseLP = lp2 * motorLeftSenseLP + (1.0-lp2) * motorLeftSense;
  motorRightSenseLP = lp2 * motorRightSenseLP + (1.0-lp2) * motorRightSense;
  motorMowSenseLP = lp2 * motorMowSenseLP + (1.0-lp2) * motorMowSense;  */

  motorsSenseLP = motorRightSenseLP + motorLeftSenseLP + motorMowSenseLP;
  
  motorLeftPwmLP = lpfMotorLeftPwm(motorLeftPwm);
  motorRightPwmLP = lpfMotorRightPwm(motorRightPwm);
  motorMowPwmLP = lpfMotorMowPwm(motorMowPwm);
/* 
  CONSOLE.print("motorLeftPwmLP: ");CONSOLE.println(motorLeftPwmLP);
  CONSOLE.print("motorRightPwmLP: ");CONSOLE.println(motorRightPwmLP);
  CONSOLE.print("motorMowPwmLP: ");CONSOLE.println(motorMowPwmLP); */

  /* motorLeftPwmLP = lp01 * motorLeftPwmLP + (1.0-lp01) * ((float)motorLeftPwm);  
  motorRightPwmLP = lp01 * motorRightPwmLP + (1.0-lp01) * ((float)motorRightPwm);
  motorMowPwmLP = lp01 * motorMowPwmLP + (1.0-lp01) * ((float)motorMowPwm);  */
 
  // compute normalized current (normalized to 1g gravity)
  //float leftAcc = (motorLeftRpm - motorLeftRpmLast) / deltaControlTime;
  //float rightAcc = (motorRightRpm - motorRightRpmLast) / deltaControlTime;
  float cosPitch = cos(robotPitch); 
	float pitchfactor;
  float robotMass = 1.0;
	// left wheel friction
	if (  ((motorLeftPwm >= 0) && (robotPitch <= 0)) || ((motorLeftPwm < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch; // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch;  // increase by angle
	motorLeftSenseLPNorm = abs(motorLeftSenseLP) * robotMass * pitchfactor;  
	// right wheel friction
	if (  ((motorRightPwm >= 0) && (robotPitch <= 0)) || ((motorRightPwm < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch;  // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch; // increase by angle
  motorRightSenseLPNorm = abs(motorRightSenseLP) * robotMass * pitchfactor; 

  //////////////////////////////// MrTree need more data
  mowPowerAct = battery.batteryVoltage * motorMowSense;             //actual mow motor Power
  mowPowerActLP = battery.batteryVoltage * motorMowSenseLP;
  motorLeftPowerAct = battery.batteryVoltage * motorLeftSenseLP;        //actual left motor power
  motorRightPowerAct = battery.batteryVoltage * motorRightSenseLP;      //actual right motor power
  motorLeftPowerMax = max(motorLeftPowerMax, motorLeftPowerAct);      //save left motor max power during mower activated
  motorRightPowerMax = max(motorRightPowerMax, motorRightPowerAct);   //save right motor max power during mower activated
  motorMowPowerMax = max(motorMowPowerMax, mowPowerActLP);              //save mow motor max power during mower activated
  ///////////////////////////////
  checkOverload();  
}


void Motor::control(){

 /*  #ifdef DIRECT_DRIVE

  const float pwmPerRpmLeft  = DIRECT_DRIVE_PWM_TO_RPM;
  const float pwmPerRpmRight = DIRECT_DRIVE_PWM_TO_RPM;

  static int motorLeftPwmTarget = 0;
  static int motorRightPwmTarget = 0;

  // Ziel-PWM berechnen
  motorLeftPwmTarget  = motorLeftRpmSet  * pwmPerRpmLeft;
  motorRightPwmTarget = motorRightRpmSet * pwmPerRpmRight;

  // Sicherheitsabschaltung
    if (fabs(motorLeftRpmSet) < 0.01 && fabs(motorRightRpmSet) < 0.01) {
    motorLeftPwmTarget = 0;
    motorRightPwmTarget = 0;
  }

   // Rampenwerte
  const int rampUpStep   = 25;   // langsame Beschleunigung
  const int rampDownStep = 25;  // schnellere Abbremsung

  // Linkes Rad rampenlogik
  if (motorLeftPwm < motorLeftPwmTarget)
    motorLeftPwm = min(motorLeftPwm + rampUpStep, motorLeftPwmTarget);
  else
    motorLeftPwm = max(motorLeftPwm - rampDownStep, motorLeftPwmTarget); 

  // Rechtes Rad rampenlogik
  if (motorRightPwm < motorRightPwmTarget)
    motorRightPwm = min(motorRightPwm + rampUpStep, motorRightPwmTarget);
  else
    motorRightPwm = max(motorRightPwm - rampDownStep, motorRightPwmTarget);
 
  motorRightPwm = motorRightPwmTarget;
  motorLeftPwm = motorLeftPwmTarget;


// Begrenzung
  motorLeftPwm  = pwmCap(motorLeftPwm, 5, pwmMax);
  motorRightPwm = pwmCap(motorRightPwm, 5, pwmMax);



  // Mähmotor wie gehabt
  if (!USE_MOW_RPM_SET && motorMowPwmSet == 0){
    motorMowPwm = lp1 * motorMowPwm + (1 - lp1) * motorMowPwmSet;
    if (abs(motorMowPwm) < 20) motorMowPwm = 0;
  } else {
    motorMowPwm = motorMowPwmSet;
    if (abs(motorMowPwmSet) < 50) motorMowPwm = 0;
  }

  if (!tractionMotorsEnabled) {
    motorLeftPwm = motorRightPwm = 0;
  }

  speedPWM(motorLeftPwm, motorRightPwm, motorMowPwm);
  return;
  
  #else */

    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float Kp_m = 0;
    float Ki_m = 0;
    float Kd_m = 0;
    
    //referenz 20Hz, cap PID vals
    if (deltaControlTime > 50) {
      Kp = MOTOR_PID_KP;
      Ki = MOTOR_PID_KI;
      Kd = MOTOR_PID_KD;
      Kp_m = MOWMOTOR_PID_KP;
      Ki_m = MOWMOTOR_PID_KI;
      Kd_m = MOWMOTOR_PID_KD;
    } else {
      Kp = MOTOR_PID_KP / 50.0 * deltaControlTime;
      Ki = MOTOR_PID_KI / 50.0 * deltaControlTime;
      Kd = MOTOR_PID_KD / 50.0 * deltaControlTime;
      Kp_m = MOWMOTOR_PID_KP / 50.0 * deltaControlTime;
      Ki_m = MOWMOTOR_PID_KI / 50.0 * deltaControlTime;
      Kd_m = MOWMOTOR_PID_KD / 50.0 * deltaControlTime;
    }

    motorLeftPID.Kp       = Kp; //MOTOR_PID_KP;//50.0*robot_control_cycle;
    motorLeftPID.Ki       = Kd; //MOTOR_PID_KI;//50.0*robot_control_cycle;
    motorLeftPID.Kd       = Ki; //MOTOR_PID_KD;//50.0*robot_control_cycle;
    //motorLeftPID.reset(); 
    motorRightPID.Kp       = Kp; //MOTOR_PID_KP;//50.0*robot_control_cycle;
    motorRightPID.Ki       = Kd; //MOTOR_PID_KI;//50.0*robot_control_cycle;
    motorRightPID.Kd       = Ki; //MOTOR_PID_KD;//50.0*robot_control_cycle;
    //motorRightPID.reset();
    if (USE_MOW_RPM_SET) {
      motorMowPID.Kp       = Kp_m; //MOWMOTOR_PID_KP;//50.0*robot_control_cycle;
      motorMowPID.Ki       = Ki_m; //MOWMOTOR_PID_KI;//50.0*robot_control_cycle;
      motorMowPID.Kd       = Kd_m; //MOWMOTOR_PID_KD;//50.0*robot_control_cycle;
      //motorMowPID.reset();     	              
    }

    //Calculate PWM for left driving motor
    motorLeftPID.TaMax = 0.10; //100ms 
    //motorLeftPID.x = motorLeftLpf(motorLeftRpm);
    /* if (ALFRED) {
      if (ticksLeft == 0 || ticksLeft > 10) motorLeftPID.x = motorLeftRpmLP;
      else motorLeftPID.x = motorLeftRpmLP;
    } else motorLeftPID.x = motorLeftRpm; */
    motorLeftPID.x = motorLeftRpm;
    motorLeftPID.w = motorLeftRpmSet;
    motorLeftPID.y_min = -pwmMax;
    motorLeftPID.y_max = pwmMax;
    motorLeftPID.max_output = pwmMax;
    motorLeftPID.output_ramp = MOTOR_PID_RAMP;
    motorLeftPID.compute();
    //CONSOLE.print("motorLeftPID.y:    ");CONSOLE.println(motorLeftPID.y);
    motorLeftPwm = motorLeftPwm + motorLeftPID.y;
    //CONSOLE.print("motorLeftPwm:  ");CONSOLE.println(motorLeftPwm);  
    motorLeftPwm = pwmCap(motorLeftPwm,0,255);

    //Calculate PWM for right driving motor
    motorRightPID.TaMax = 0.15;
    //motorRightPID.x = motorRightLpf(motorRightRpm);
    /* if (ALFRED) {
      if (ticksRight == 0 || ticksRight > 10) motorRightPID.x = motorRightRpmLP; 
      else motorRightPID.x = motorRightRpmLP;
    } else motorRightPID.x = motorRightRpm; */
    motorRightPID.x = motorRightRpm; 
    motorRightPID.w = motorRightRpmSet;
    motorRightPID.y_min = -pwmMax;
    motorRightPID.y_max = pwmMax;
    motorRightPID.max_output = pwmMax;
    motorRightPID.output_ramp = MOTOR_PID_RAMP;
    motorRightPID.compute();
    motorRightPwm = motorRightPwm + motorRightPID.y;
    motorRightPwm = pwmCap(motorRightPwm,0,255);  // 0<-clampzero>min-max<-

    //Calculate PWM for mow motor
    if (USE_MOW_RPM_SET) {                                                               
      if ((mowRPM_RC != 0)&&(RC_Mode)) {
        if (motorMowRpmSet < 0)motorMowRpmSet = -mowRPM_RC; //MrTree
        if (motorMowRpmSet > 0)motorMowRpmSet = mowRPM_RC; //MrTree
      }
      
      float comp = MOWMOTOR_RPM_OFFSET;
      if (motorMowRpmSet < 0) comp = -comp;
      
      motorMowPID.TaMax = 0.15;
      //motorMowPID.x = motorMowLpf(motorMowRpm - comp) ;
      /* if (ALFRED) {
        if (ticksMow == 0 || ticksMow > 10) motorMowPID.x = motorMowRpmLP - comp;
        else motorMowPID.x = motorMowRpm - comp;
      } else motorMowPID.x = motorMowRpm - comp; */
      motorMowPID.x = motorMowRpmLPFast - comp;
      motorMowPID.w = motorMowRpmSet;
      motorMowPID.y_min = -pwmMax;
      motorMowPID.y_max = pwmMax;
      motorMowPID.max_output = pwmMax;
      motorMowPID.output_ramp = MOWMOTOR_PID_RAMP;
      motorMowPID.compute();
      motorMowPwm = motorMowPwm + motorMowPID.y;

      motorMowPwm = pwmCap(motorMowPwm,0,255);


      if (motorMowRpmSet == 0){ //we use simple low pass when stopping mow motor
        motorMowPwm = lp1 * motorMowPwm + (1 - lp1) * motorMowPwmSet;
        if (abs(motorMowRpmLPFast) < 400) motorMowPwm = 0;    
      }      
      if (!USE_MOW_RPM_SET && motorMowPwmSet == 0){ //we use simple low pass when stopping mow motor
        motorMowPwm = lp1 * motorMowPwm + (1 - lp1) * motorMowPwmSet;
        if (abs(motorMowPwm) < 20) motorMowPwm = 0;    
      }      
    } else {
      if (mowPWM_RC != 0 && RC_Mode) {
        if (motorMowPwmSet < 0)motorMowPwmSet = -mowPWM_RC;
        if (motorMowPwmSet > 0)motorMowPwmSet = mowPWM_RC;
      } 
      motorMowPwm = lp3 * motorMowPwm + (1 - lp3) * motorMowPwmSet;
      if (abs(motorMowPwmSet) < 50) motorMowPwm = 0;
    }
    speedPWM(motorLeftPwm, motorRightPwm, motorMowPwm);
  
  //#endif

  //set PWM for all motors
  if (!tractionMotorsEnabled){
    motorLeftPwm = motorRightPwm = 0;
  }

  //CONSOLE.print("motorLeftLpf(motorLeftRpm):  ");CONSOLE.println(valuesome);  
    
  static unsigned long nextOutputTime = 0;
  if (DEBUG_PID) {
    CONSOLE.print("motorTicks      l|r|m  -  ");CONSOLE.print(ticksLeft);       CONSOLE.print("    | ");CONSOLE.print(ticksRight); CONSOLE.print("    | ");CONSOLE.println(ticksMow);
  }
  if (DEBUG_PID && millis() > nextOutputTime){
    nextOutputTime = millis() + 1000;
    
    CONSOLE.print("motorRpmSet     l|r|m  -  ");CONSOLE.print(motorLeftRpmSet); CONSOLE.print(" | ");CONSOLE.print(motorRightRpmSet); CONSOLE.print(" | ");CONSOLE.println(motorMowRpmSet);
    CONSOLE.print("motorRpm        l|r|m  -  ");CONSOLE.print(motorLeftRpm);    CONSOLE.print(" | ");CONSOLE.print(motorRightRpm);    CONSOLE.print(" | ");CONSOLE.println(motorMowRpm);
    CONSOLE.print("motorRpmLP      l|r|m  -  ");CONSOLE.print(motorLeftRpmLP);  CONSOLE.print(" | ");CONSOLE.print(motorRightRpmLP);  CONSOLE.print(" | ");CONSOLE.println(motorMowRpmLP);
    CONSOLE.print("motorPwm        l|r|m  -  ");CONSOLE.print(motorLeftPwm);    CONSOLE.print(" | ");CONSOLE.print(motorRightPwm);    CONSOLE.print(" | ");CONSOLE.println(motorMowPwm);
    CONSOLE.print("motorPwmLP      l|r|m  -  ");CONSOLE.print(motorLeftPwmLP);  CONSOLE.print(" | ");CONSOLE.print(motorRightPwmLP);  CONSOLE.print(" | ");CONSOLE.println(motorMowPwmLP);
  }
  //CONSOLE.println();
  //CONSOLE.print("deltaTime ");CONSOLE.print(deltaControlTime);CONSOLE.print("   motorLeftPwm ");CONSOLE.print(motorLeftPwm);CONSOLE.print("   motorRightPwm ");CONSOLE.print(motorRightPwm);CONSOLE.print("   motorMowPwm ");CONSOLE.println(motorMowPwm);  
}

void Motor::dumpOdoTicks(int seconds){
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;
  CONSOLE.print("t=");
  CONSOLE.print(seconds);
  CONSOLE.print("  ticks Left=");
  CONSOLE.print(motorLeftTicks);  
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightTicks);             
  CONSOLE.print("  current Left=");
  CONSOLE.print(motorLeftSense);
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightSense);
  CONSOLE.println();               
}

void Motor::test(){
  CONSOLE.println("motor test - 10 revolutions");  
  unsigned long nextInfoTime = 0;
  int seconds = 0;
  int pwmLeft = 0;
  int pwmRight = 0;
  bool slowdown = true;
  unsigned long stopTicks = ticksPerRevolution * 10;
  unsigned long nextControlTime = 0;
  int testPwms[] = {200, 150, 100, 50};

  for (int i = 0; i <= 3; i++) {
    
    int pwmTarget = testPwms[i];
    motorLeftTicks = 0;  
    motorRightTicks = 0;
    pwmLeft = pwmRight = pwmTarget;
    slowdown = true;

    CONSOLE.print("Testcycle = ");
    CONSOLE.println(i+1);
    CONSOLE.print("Testing PWM = ");
    CONSOLE.println(pwmTarget);

    while (motorLeftTicks < stopTicks || motorRightTicks < stopTicks){

      if (millis() > nextControlTime){
        nextControlTime = millis() + 20;
        if ((slowdown) && ((motorLeftTicks + ticksPerRevolution  > stopTicks)||(motorRightTicks + ticksPerRevolution > stopTicks))){  //Letzte halbe drehung verlangsamen
          pwmLeft = pwmRight = 20;
          slowdown = false;
        }    
        if (millis() > nextInfoTime){      
          nextInfoTime = millis() + 1000;            
          dumpOdoTicks(seconds);
          seconds++;      
        }    
        if(motorLeftTicks >= stopTicks)
        {
          pwmLeft = 0;
        }  
        if(motorRightTicks >= stopTicks)
        {
          pwmRight = 0;      
        }
        
        speedPWM(pwmLeft, pwmRight, 0);
        sense();
        //delay(50);         
        watchdogReset();     
        robotDriver.run();
      }
    
    }
    delay(2000);
  }  
  speedPWM(0, 0, 0);
  CONSOLE.println("motor test done - please ignore any IMU/GPS errors");
}

void Motor::plot(){
  CONSOLE.println("motor plot (left,right,mow) - NOTE: Start Arduino IDE Tools->Serial Plotter (CTRL+SHIFT+L)");
  delay(5000);
  CONSOLE.println("pwmLeft,pwmRight,pwmMow,ticksLeft,ticksRight,ticksMow");
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  motorMowTicks = 0;
  int pwmLeft = 0;
  int pwmRight = 0; 
  int pwmMow = 0;
  int cycles = 0;
  int acceleration = 1;
  bool forward = true;
  unsigned long nextPlotTime = 0;
  unsigned long stopTime = millis() + 1 * 60 * 1000;
  unsigned long nextControlTime = 0;

  while (millis() < stopTime){   // 60 seconds...
    if (millis() > nextControlTime){
      nextControlTime = millis() + 20; 

      int ticksLeft=0;
      int ticksRight=0;
      int ticksMow=0;
      motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
      motorLeftTicks += ticksLeft;
      motorRightTicks += ticksRight;
      motorMowTicks += ticksMow;

      if (millis() > nextPlotTime){ 
        nextPlotTime = millis() + 100;
        CONSOLE.print(300+pwmLeft);
        CONSOLE.print(",");  
        CONSOLE.print(300+pwmRight);
        CONSOLE.print(",");
        CONSOLE.print(pwmMow);
        CONSOLE.print(",");        
        CONSOLE.print(300+motorLeftTicks);    
        CONSOLE.print(",");
        CONSOLE.print(300+motorRightTicks);
        CONSOLE.print(",");
        CONSOLE.print(motorMowTicks);        
        CONSOLE.println();
        motorLeftTicks = 0;
        motorRightTicks = 0;
        motorMowTicks = 0;      
      }

      speedPWM(pwmLeft, pwmRight, pwmMow);
      if (pwmLeft >= 255){
        forward = false;
        cycles++; 
      }      
      if (pwmLeft <= -255){
        forward = true;
        cycles++;               
      } 
      if ((cycles == 2) && (pwmLeft >= 0)) {
        if (acceleration == 1) acceleration = 20;
          else acceleration = 1;
        cycles = 0;
      }         
      if (forward){
        pwmLeft += acceleration;
        pwmRight += acceleration;
        pwmMow += acceleration;
      } else {
        pwmLeft -= acceleration;
        pwmRight -= acceleration;
        pwmMow -= acceleration;
      }
      pwmLeft = min(255, max(-255, pwmLeft));
      pwmRight = min(255, max(-255, pwmRight));          
      pwmMow = min(255, max(-255, pwmMow));                
    }  
    //sense();
    //delay(10);
    watchdogReset();     
    robotDriver.run(); 
  }
  speedPWM(0, 0, 0);
  CONSOLE.println("motor plot done - please ignore any IMU/GPS errors");
}
