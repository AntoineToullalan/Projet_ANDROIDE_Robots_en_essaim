/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

#ifndef _MIRF_H_
#define _MIRF_H_

#include <avr/io.h>
#include "variables.h"
#include "leds.h"
#include "sensors.h"
#include "utility.h"


// Mirf settings
#define mirf_CH         40
#define mirf_PAYLOAD    16
#define mirf_CONFIG     ( (1<<MASK_RX_DR) | (1<<EN_CRC) | (0<<CRCO) )

// Pin definitions for chip select and chip enabled of the MiRF module
#define CE  PB4
#define CSN PB0

// Definitions for selecting and enabling MiRF module
#define mirf_CSN_hi     PORTB |=  (1<<CSN);
#define mirf_CSN_lo     PORTB &= ~(1<<CSN);
#define mirf_CE_hi      PORTB |=  (1<<CE);
#define mirf_CE_lo      PORTB &= ~(1<<CE);

#ifdef __cplusplus
extern "C" {
#endif

// Public standard functions
void mirf_init();
void mirf_config();
void mirf_send(uint8_t * value, uint8_t len);
void mirf_set_RADDR(uint8_t * adr);
void mirf_set_TADDR(uint8_t * adr);
uint8_t mirf_data_ready();
void mirf_get_data(uint8_t * data);
uint8_t rx_fifo_is_empty();
void flush_rx_fifo();

void writeAckPayload(unsigned char *data, unsigned char size);
void flushTxFifo();
void handleRFCommands();
uint8_t readPayloadWidthFromTopFifo();
uint8_t readPayloadWidthFromPipe0();
void rfEnableDebugMode();
void rfDisableDebugMode();
void rfDebugSendData();
void rfDebugNextPacket();

// Public extended functions
void mirf_config_register(uint8_t reg, uint8_t value);
void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len);
void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* _MIRF_H_ */
