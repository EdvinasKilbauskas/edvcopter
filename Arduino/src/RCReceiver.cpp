/*
 * Written by Edvinas Kilbauskas, 2014
 * You can contact me. Email: EdvinasKilbauskas@gmail.com
 * Github: http://github.com/EdvinasKilbauskas
 */

#include "RCReceiver.h"
#include "PinChangeInt.h"

int RCReceiver::m_channels[6] = {0,0,0,0,0,0};
int RCReceiver::m_channelsLast[6] = {0,0,0,0,0,0};
int RCReceiver::m_lastMicros[6] = {micros(),micros(),micros(),micros(),micros(),micros()};
bool RCReceiver::m_locked = false;

/*
  Initialized receiver. ch1:ch6 - channels pins on arduino, power - pin to send 5V
*/
void RCReceiver::init(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int power)
{
  pinMode(power, OUTPUT);
  digitalWrite(power, HIGH);
  
  // attach intterupt functions to pins on arduino. Function is going to be called everything the voltage changes.
  PCintPort::attachInterrupt(ch1, interruptCh1, CHANGE);
  PCintPort::attachInterrupt(ch2, interruptCh2, CHANGE);
  PCintPort::attachInterrupt(ch3, interruptCh3, CHANGE);
  PCintPort::attachInterrupt(ch4, interruptCh4, CHANGE);
  PCintPort::attachInterrupt(ch5, interruptCh5, CHANGE);
  PCintPort::attachInterrupt(ch6, interruptCh6, CHANGE);
}

//
float RCReceiver::getChannel(int index)
{
  float val = (m_channels[index-1]-RC_LOW)/(RC_HIGH-RC_LOW);
  return val == -1.0f? 0.5f : val;
}

void RCReceiver::log()
{
  //#ifdef DEBUG
  Serial.print("\t1ch: "); Serial.print(getChannel(1));
  Serial.print("\t2ch:"); Serial.print(getChannel(2));
  Serial.print("\t3ch: "); Serial.print(getChannel(3));
  Serial.print("\t4ch: "); Serial.print(getChannel(4));
  Serial.print("\t5ch: "); Serial.print(getChannel(5));
  Serial.print("\t6ch: "); Serial.print(getChannel(6));
  Serial.print('\t');
  //#endif
}

void RCReceiver::lock()
{
  m_locked = true;
}

void RCReceiver::unlock()
{
  m_locked = false;
}

// Because 
void RCReceiver::interruptCh1()
{
  int val = micros() - m_lastMicros[0]; // calculate the RC channel value by checking how much time passed since last voltage change in the RC pin.
  if(m_locked == false && val <= 2000 && val >= 0) m_channelsLast[0] = m_channels[0] = val;
    else m_channels[0] = m_channelsLast[0];
  m_lastMicros[0] = micros(); 
}

void RCReceiver::interruptCh2()
{
  int val = micros() - m_lastMicros[1];
  if(m_locked == false && val <= 2000 && val >= 0) m_channelsLast[1] = m_channels[1] = val;
    else m_channels[1] = m_channelsLast[1];
  m_lastMicros[1] = micros(); 
}

void RCReceiver::interruptCh3()
{
  int val = micros() - m_lastMicros[2];
  if(m_locked == false && val <= 2000 && val >= 0) m_channelsLast[2] = m_channels[2] = val;
    else m_channels[2] = m_channelsLast[2];
  m_lastMicros[2] = micros();
}

void RCReceiver::interruptCh4()
{
  int val = micros() - m_lastMicros[3];
  if(m_locked == false && val <= 2000 && val >= 0) m_channelsLast[3] = m_channels[3] = val;
    else m_channels[3] = m_channelsLast[3];
  m_lastMicros[3] = micros(); 
}
void RCReceiver::interruptCh5()
{
  int val = micros() - m_lastMicros[4];
  if(m_locked == false && val <= 2000 && val >= 0) m_channelsLast[4] = m_channels[4] = val;
    else m_channels[4] = m_channelsLast[4];
  m_lastMicros[4] = micros(); 
}

void RCReceiver::interruptCh6()
{
  int val = micros() - m_lastMicros[5];
  if(m_locked == false && val <= 2000 && val >= 0) m_channelsLast[5] = m_channels[5] = val;
    else m_channels[5] = m_channelsLast[5];
  m_lastMicros[5] = micros(); 
}



