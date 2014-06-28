/*
 * Written by Edvinas Kilbauskas, 2014
 * You can contact me. Email: EdvinasKilbauskas@gmail.com
 * Github: http://github.com/EdvinasKilbauskas
 */

//#include "MPUSensor.h"



/*void MPUSensor::init()
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

void MPUSensor::calculate()
{
    m_dataReady = false;
    m_mpuIntStatus = m_mpu.getIntStatus();
    m_fifoCount = m_mpu.getFIFOCount();
    
    if((m_mpuIntStatus & 0x10) || m_fifoCount >= 1024){ 
      m_mpu.resetFIFO(); 

    }else if(m_mpuIntStatus & 0x02){    
      while (m_fifoCount < m_packetSize) m_fifoCount = m_mpu.getFIFOCount();
  
      m_mpu.getFIFOBytes(m_fifoBuffer, m_packetSize);
      
      m_fifoCount -= m_packetSize;
    
      m_mpu.dmpGetQuaternion(&m_quaternion, m_fifoBuffer);
      m_mpu.dmpGetGravity(&m_gravity, &m_quaternion);
      m_mpu.dmpGetYawPitchRoll(m_YPRLast, &m_quaternion, &m_gravity);
    }
    
    for(int i = 0; i < 3; i++)
          m_YPR[i] = m_YPRLast[i];
}

float MPUSensor::getYaw()
{
  return m_YPR[0];
}

float MPUSensor::getPitch()
{
  return m_YPR[1];
}

float MPUSensor::getRoll()
{
  return m_YPR[2];
}

void MPUSensor::interruptDataReady()
{
  m_dataReady = true;
}*/


