/*
 * Written by Edvinas Kilbauskas, 2014
 * You can contact me. Email: EdvinasKilbauskas@gmail.com
 * Github: http://github.com/EdvinasKilbauskas
 */

#ifndef RC_RECEIVER
#define RC_RECEIVER

#define RC_LOW 1000.0f
#define RC_HIGH 2000.0f

class RCReceiver
{
private:
    static int m_channels[6];
    static int m_channelsLast[6];
    static int m_lastMicros[6];
    static bool m_locked;
    
    static void interruptCh1();
    static void interruptCh2();
    static void interruptCh3();
    static void interruptCh4();
    static void interruptCh5();
    static void interruptCh6();
public:
    RCReceiver(){};
    void init(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int power);
    void lock();
    void unlock();
    float getChannel(int index);
    void log();
};

#endif



