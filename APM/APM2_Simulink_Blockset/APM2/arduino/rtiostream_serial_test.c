/*
 * Copyright 2009-2010 The MathWorks, Inc.
 */

#include "rtiostream.h"
#include "WProgram.h"

void serial_simple_step(void);

void serial_simple_step(void)
{

#define MAX_PACKET_SIZE 50

    char data[MAX_PACKET_SIZE];

    static unsigned int packetSize=1;
    int i;

    if (packetSize <= MAX_PACKET_SIZE) {
        size_t sizeRecvd;
        unsigned int packetIdx=0;
        rtIOStreamRecv(0, data, (size_t) packetSize, &sizeRecvd);
        packetIdx=(unsigned int) sizeRecvd;
        while (packetIdx < packetSize) {
            rtIOStreamRecv(0, &data[packetIdx], (size_t) (packetSize-packetIdx), &sizeRecvd);
            packetIdx=packetIdx+(unsigned int) sizeRecvd;
        }
        for (i=0; i<packetSize; i++) {
            /* Expected return packet is received data + 1 */
            data[i]=data[i]+1; 
        }
        {
            size_t tmp;
            rtIOStreamSend( 0, data, packetSize, &tmp);
        }
        packetSize++;
    }
}


int main(void)
{
  unsigned long previous = 0;

  /* Initialize rtiostream channel */
  rtIOStreamOpen(0,NULL);

  while (true) {
    if (millis()-previous >= 1000) {
      previous = millis();
      serial_simple_step();
    }
  }

  return 0;
}
