#ifndef MPU_SENSOR_H_
#define MPU_SENSOR_H_

#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "MPU6050_6Axis_MotionApps20.h"

/*
 * Written by Edvinas Kilbauskas, 2014
 * You can contact me. Email: EdvinasKilbauskas@gmail.com
 * Github: http://github.com/EdvinasKilbauskas
 */

class MPUSensor
{
private:
  MPU6050 m_mpu;
  
  uint8_t m_mpuIntStatus;                 
  uint8_t m_devStatus;                     
  uint16_t m_packetSize;                   // estimated packet size  
  uint16_t m_fifoCount;                    // fifo buffer size   
  uint8_t m_fifoBuffer[64];                // fifo buffer 
  
  static volatile bool m_dataReady;
  
  Quaternion m_quaternion;                 // quaternion for mpu output
  VectorFloat m_gravity;                   // gravity vector for ypr
 
  float m_YPRLast[3];
  float m_YPR[3];  
  
  static void interruptDataReady()
  {
    m_dataReady = true;
  }
  
public:
  MPUSensor(){};
  
  void init()
  {
      Wire.begin();
      m_mpu.initialize();
      m_devStatus = m_mpu.dmpInitialize();
      if(m_devStatus == 0){
        m_mpu.setDMPEnabled(true);
        attachInterrupt(0, interruptDataReady, RISING);
        m_mpuIntStatus = m_mpu.getIntStatus();
        m_packetSize = m_mpu.dmpGetFIFOPacketSize();
     } 
  }

  void waitTillReady()
  {
    while(m_dataReady == false);
  }
  
  void log()
  {   
      //Serial.print("mpuIntStatus: "); Serial.print(m_mpuIntStatus);
      //Serial.print("\tfifo count: "); Serial.print(m_fifoCount);
      //Serial.print("\tready: "); Serial.print(m_dataReady);
      Serial.print("Roll(X): "); Serial.print(getRoll(),3);
      Serial.print("\tPitch(Y): "); Serial.print(getPitch(),3);
      Serial.print("\tYaw(Z): "); Serial.print(getYaw(),3);
  }

  void calculate()
  {
    m_dataReady = false;
    m_mpuIntStatus = m_mpu.getIntStatus();
    m_fifoCount = m_mpu.getFIFOCount();
    
    //if((m_mpuIntStatus & 0x10) || m_fifoCount >= 1024){ 
     // m_mpu.resetFIFO(); 
    /*}else */if(m_mpuIntStatus & 0x02){    
      /*while (m_fifoCount < m_packetSize)*/ m_fifoCount = m_mpu.getFIFOCount();
  
      m_mpu.getFIFOBytes(m_fifoBuffer, m_fifoCount);/*m_packetSize);*/
      
      m_fifoCount -= m_packetSize;
    
      m_mpu.dmpGetQuaternion(&m_quaternion, m_fifoBuffer);
      m_mpu.dmpGetGravity(&m_gravity, &m_quaternion);
      m_mpu.dmpGetYawPitchRoll(m_YPRLast, &m_quaternion, &m_gravity);
    }
    
    //if(fifoCount ==
    for(int i = 0; i < 3; i++)
          m_YPR[i] = m_YPRLast[i];
          
    if(m_fifoCount > 0)
         m_mpu.resetFIFO();

  }

  float getYaw(){
    return m_YPR[0];
  }

  float getPitch()
  {
    return m_YPR[1];
  }

  float getRoll(){
    return m_YPR[2];
  }
  
};

volatile bool MPUSensor::m_dataReady = true;

#endif


