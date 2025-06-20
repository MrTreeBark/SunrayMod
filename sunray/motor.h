// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "lowpass_filter.h"

// selected motor
enum MotorSelect { MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW };
typedef enum MotorSelect MotorSelect;

extern unsigned int robot_control_cycle;
int pwmCap(int val, int min, int max);

class Motor {
public:
  //Runtime control flags and states
  bool toggleMowDir;
  bool activateLinearSpeedRamp;
  bool waitSpinUp;
  bool motorMowStallFlag;
  bool speedUpTrig;
  bool switchedOn;
  bool motorMowRpmError;
  bool motorMowSpunUp;
  bool motorMowForwardSet;
  bool keepslow;
  bool retryslow;
  bool motorRecoveryState;
  bool motorLeftSwapDir;
  bool motorRightSwapDir;
  bool motorError;
  bool motorLeftOverload;
  bool motorRightOverload;
  bool motorMowOverload;
  bool motorMowStall;
  bool tractionMotorsEnabled;
  bool releaseBrakesWhenZero;
  bool enableMowMotor;
  bool odometryError;
  bool motorMowRpmCheck;

  //Kinematic parameters
  float robotPitch;              // Robot pitch (rad)
  float wheelBaseCm;             // Distance between wheels (cm)
  int wheelDiameter;             // Wheel diameter (mm)
  int ticksPerRevolution;        // Ticks per wheel revolution
  int mowticksPerRevolution;     // Ticks per mower revolution
  float ticksPerCm;              // Ticks per cm movement

  //PWM, RPM and Speed control
  int pwmMax;
  int mowPwm;
  int mowHeightMillimeter;
  int mowRPM_RC;
  int mowPWM_RC;
  float mowRpm;
  float mowPowerMax;
  float mowPowerMin;
  float currentFactor;
  float SpeedFactor;
  float y_before;
  float keepslow_y;

  //Setpoints
  float linearCurrSet;
  float linearSpeedSet;
  float angularSpeedSet;
  float motorMowRpmSet;
  float motorMowPwmSet;

  //Measured values
  float trackerDiffDelta = 0;

  float motorLeftPowerAct;
  float motorLeftPowerMax = 0;
  float motorRightPowerAct;
  float motorRightPowerMax = 0;
  float motorMowPowerMax;
  float mowMotorCurrentAverage;
  float mowPowerAct;
  float mowPowerActLP;
  float motorMowRpm;
  float motorMowRpmLP;
  float motorMowRpmLPFast;
  float motorMowPwmLP;
  float motorMowSense;
  float motorMowSenseLP;
  float motorLeftSense;
  float motorLeftSenseLP;
  float motorLeftSenseLPNorm;
  float motorRightSense;
  float motorRightSenseLP;
  float motorRightSenseLPNorm;
  float motorsSenseLP;
  int motorMowPwm;

  //Odometer ticks
  int ticksLeft;
  int ticksRight;
  int ticksMow;
  unsigned long motorLeftTicks;
  unsigned long motorRightTicks;
  unsigned long motorMowTicks;
  int motorLeftTicksZero;
  int motorRightTicksZero;
  int motorMowTicksZero;

  //Timers
  unsigned long motorOverloadDuration;
  unsigned long motorMowStallDuration;
  unsigned long nextOutputTime;
  unsigned long motorMowSpinUpTime;
  unsigned long keepSlowTime;
  unsigned long retrySlowTime;
  unsigned long motorReleaseBrakesTime;
  unsigned long deltaControlTime;
  unsigned long lastMowStallCheckTime;
  unsigned long drvfixtimer;
  bool drvfixreset;
  unsigned int drvfixcounter;
  bool recoverMotorFault;
  int recoverMotorFaultCounter;
  unsigned long nextRecoverMotorFaultTime;
  bool setLinearAngularSpeedTimeoutActive;
  unsigned long setLinearAngularSpeedTimeout;

  //PID controllers and filters classes
  PID motorLeftPID;
  PID motorRightPID;
  PID motorMowPID;
  LowPassFilter motorLeftLpf;   //PID
  LowPassFilter motorRightLpf;  //PID
  LowPassFilter motorMowLpf;    //PID

  //Motorleft
  LowPassFilter lpfMotorLeftPwm;
  LowPassFilter lpfMotorLeftRpm;
  LowPassFilter lpfMotorLeftSense;
  //Motorright
  LowPassFilter lpfMotorRightPwm;
  LowPassFilter lpfMotorRightRpm;
  LowPassFilter lpfMotorRightSense;
  //Motormow
  LowPassFilter lpfMotorMowPwm;
  LowPassFilter lpfMotorMowRpm;
  LowPassFilter lpfMotorMowRpmFast;
  LowPassFilter lpfMotorMowSense;
  LowPassFilter lpfMotorMowSenseFast;


  //Interface methods
  void begin();
  void run();
  void test();
  void plot();
  void sense();
  void control();
  void enableTractionMotors(bool enable);
  void setLinearAngularSpeed(float linear, float angular, bool useLinearRamp = true);
  void setMowState(bool switchOn);
  void setMowPwm(int val);
  void setMowHeightMillimeter(int val);
  void setReleaseBrakesWhenZero(bool release);
  bool waitMowMotor();
  void stopImmediately(bool includeMowerMotor);
  void dumpOdoTicks(int seconds);

protected:
  float lp005;
  float lp01;
  float lp1;
  float lp2;
  float lp3;
  float lp4;

  float motorLeftRpmSet;
  float motorRightRpmSet;
  float motorLeftRpm;
  float motorRightRpm;
  float motorLeftRpmLP;
  float motorRightRpmLP;
  float motorLeftRpmLast;
  float motorRightRpmLast;
  int   motorLeftPwm;
  int   motorRightPwm;
  float motorLeftPwmLP;
  float motorRightPwmLP;

  //Internal control functions
  void speedPWM(int pwmLeft, int pwmRight, int pwmMow);
  bool checkFault();
  void checkOverload();
  bool checkOdometryError();
  bool checkMowRpmFault();
  void drvfix();
  void checkMotorMowStall();
  float adaptiveSpeed();
  void changeSpeedSet();
  bool checkCurrentTooHighError();
  bool checkCurrentTooLowError();
  void handleMowPwmMode(bool switchOn);
  void handleMowRpmMode(bool switchOn);
};

#endif