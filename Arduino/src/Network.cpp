/*
 * Written by Edvinas Kilbauskas, 2014
 * You can contact me. Email: EdvinasKilbauskas@gmail.com
 * Github: http://github.com/EdvinasKilbauskas
 */

#include "Network.h"

void serialSend16(uint16_t integer)
{
    Serial.write((const uint8_t*)&integer,2);
}

