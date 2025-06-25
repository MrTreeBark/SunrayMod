// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "MpuDriver.h"
#include "../../config.h"
#include "../../i2c.h"

MpuDriver::MpuDriver(){    
}

void MpuDriver::selectChip(){
}

void MpuDriver::detect(){
  // detect MPUxxxx  
  uint8_t data = 0;
  selectChip();
  I2CreadFrom(MPU_ADDR, 0x75, 1, &data, 1); // whoami register
  CONSOLE.print(F("MPU ID=0x"));
  CONSOLE.println(data, HEX);     
  #if defined MPU6050 || defined MPU9150       
    if (data == 0x68) {
        CONSOLE.println("MPU6050/9150 found");
        imuFound = true;
        return;
    } else if (data == 0x72) {
        CONSOLE.println("MPU6052 found");
        imuFound = true;
        return;
    }
  #endif
  #if defined MPU9250 
    if (data == 0x73) {
        CONSOLE.println("MPU9255 found");
        imuFound = true;
        return;
    } else if (data == 0x71) {
        CONSOLE.println("MPU9250 found");
        imuFound = true;
        return;
    }
  #endif
  imuFound = false;
  CONSOLE.println(F("MPU6050/9150/9250/9255 not found - Did you connect AD0 to 3.3v and choose it in config.h?"));          
}


bool MpuDriver::begin(){ 
    CONSOLE.println("using imu driver: MpuDriver");
    //selectChip();
    if (mpu.begin() != INV_SUCCESS){
        return false;
    }
    unsigned short fiforate = (1000/ROBOT_CONTROL_CYCLE); //Hz
    //mpu.setAccelFSR(2);	      
    mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT  // Enable 6-axis quat
               | DMP_FEATURE_GYRO_CAL // Use gyro calibration
               //|DMP_FEATURE_SEND_CAL_GYRO
               | DMP_FEATURE_SEND_RAW_ACCEL
              , fiforate); // Set DMP FIFO rate
    // DMP_FEATURE_LP_QUAT can also be used. It uses the 
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive    
    //mpu.dmpSetOrientation(orientationMatrix);
    //mpu.setLPF(98); //test against spikes in gyrodata
    return true;
}


void MpuDriver::run(){
}


bool MpuDriver::isDataAvail(){
    if (!imuFound) return false;

    const int fifoPacketSize = 22; // for 6X_LP_QUAT + RAW_ACCEL
    int bytes = mpu.fifoAvailable();

    // continue if a full packet is avail
    if (bytes < fifoPacketSize) return false;

    // buffer is filling --> reset
    if (bytes > fifoPacketSize * 2) {
        mpu.resetFifo();
        // warn flag, nothing here
        return false;
    }

    // get packet
    if (mpu.dmpUpdateFifo() != INV_SUCCESS) return false;

    // computeEulerAngles can be used -- after updating the
    // quaternion values -- to estimate roll, pitch, and yaw
    // toEulerianAngle(imu.calcQuat(imu.qw), imu.calcQuat(imu.qx), imu.calcQuat(imu.qy), imu.calcQuat(imu.qz), imu.roll, imu.pitch, imu.yaw);
    
    quatW = mpu.qw;
    quatX = mpu.qx;
    quatY = mpu.qy;
    quatZ = mpu.qz;
    mpu.computeEulerAngles(false);
    
    /*     
    CONSOLE.print("IMU ax, ay, az = ");
    CONSOLE.print(mpu.ax);
    CONSOLE.print(", ");
    CONSOLE.print(mpu.ay);
    CONSOLE.print(", ");
    CONSOLE.println(mpu.az);
    */
    
    roll = mpu.roll;
    pitch = mpu.pitch;
    yaw = mpu.yaw;

    ax = mpu.ax / 16384.0 ;    //x acceleration of mpu in g
    ay = mpu.ay / 16384.0 ;    //y acceleration of mpu in g
    az = mpu.az / 16384.0 ;    //z acceleration of mpu in g

    //mpu.computeCompassHeading();
    //mpu.calcAccel(X_AXIS);
    //mpu.calcAccel(Y_AXIS);
    //mpu.calcAccel(Z_AXIS);
    //heading = mpu.heading; //MrTree heading of mpu

    //if (bytes > 448) resetData(); //test
    return true;
}         
    
void MpuDriver::resetData(){
    mpu.resetFifo();
}



