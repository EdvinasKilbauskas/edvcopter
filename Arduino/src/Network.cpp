#include "Network.h"


void serialSend16(uint16_t integer)
{
    Serial.write((const uint8_t*)&integer,2);
}

