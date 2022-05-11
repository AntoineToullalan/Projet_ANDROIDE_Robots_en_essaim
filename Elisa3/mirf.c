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

#include "mirf.h"
#include "nRF24L01.h"
#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )


// Flag which denotes transmitting mode
volatile uint8_t PTX;

void mirf_init() 
// Initializes pins as interrupt to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
    // Define CSN and CE as Output and set them to default
    //DDRB |= ((1<<CSN)|(1<<CE));
    mirf_CE_hi;
    mirf_CSN_hi;

	mirf_config();
}


void mirf_config() 
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
{

	uint8_t temp[3];

	// power down
	mirf_config_register(CONFIG, 0x0D);

	// address width
	mirf_config_register(SETUP_AW, 0x01);

	// tx address
	temp[0] = (rfAddress>>8)&0xFF;
	temp[1] = rfAddress & 0xFF;
	temp[2] = 0x00;
	mirf_write_register(TX_ADDR, temp, 3);	

	// rx address => same as tx address for auto ack
	mirf_write_register(RX_ADDR_P0, temp, 3);

	// enable auto ack for pipe0
	mirf_config_register(EN_AA, 0x01);

	// enable pipe0
	mirf_config_register(EN_RXADDR, 0x01);

	// 500�s (+ 86�s on-air), 2 re-transmissions
	mirf_config_register(SETUP_RETR, 0x12);

    // select RF channel
    mirf_config_register(RF_CH,40);

	// RX payload size; it isn't needed because the dynamic payload length is activated for ACK+PAYLOAD feature
    mirf_config_register(RX_PW_P0, PAYLOAD_SIZE);

	// enable extra features
    mirf_CSN_lo;
    SPI_Write_Byte(NRF_ACTIVATE);
    SPI_Write_Byte(0x73);
    mirf_CSN_hi;
	
	// enable dynamic payload for pipe0
	mirf_config_register(NRF_DYNPD, 0x01);

	// enable payload with ACK and dynamic payload length
	mirf_config_register(NRF_FEATURE, 0x06);
		
	// power up; enable crc (2 bytes); prx; max_rt, tx_ds enabled
	mirf_config_register(CONFIG, 0x0F);	

    // Start receiver 
    //PTX = 0;        // Start in receiving mode
    //RX_POWERUP;     // Power up in receiving mode
    //mirf_CE_hi;     // Listening for pakets
}

void mirf_set_RADDR(uint8_t * adr) 
// Sets the receiving address
{
    mirf_CE_lo;
    mirf_write_register(RX_ADDR_P0,adr,5);
    mirf_CE_hi;
}

void mirf_set_TADDR(uint8_t * adr)
// Sets the transmitting address
{
	mirf_write_register(TX_ADDR, adr,5);
}

/*
#if defined(__AVR_ATmega8__)
SIGNAL(SIG_INTERRUPT0) 
#endif // __AVR_ATmega8__
#if defined(__AVR_ATmega168__)
SIGNAL(SIG_PIN_CHANGE2) 
#endif // __AVR_ATmega168__  
// Interrupt handler 
{
    uint8_t status;   
    // If still in transmitting mode then finish transmission
    if (PTX) {
    
        // Read MiRF status 
        mirf_CSN_lo;                                // Pull down chip select
        status = spi_fast_shift(NOP);               // Read status register
        mirf_CSN_hi;                                // Pull up chip select

        mirf_CE_lo;                             // Deactivate transreceiver
        RX_POWERUP;                             // Power up in receiving mode
        mirf_CE_hi;                             // Listening for pakets
        PTX = 0;                                // Set to receiving mode

        // Reset status register for further interaction
        mirf_config_register(STATUS,(1<<TX_DS)|(1<<MAX_RT)); // Reset status register
    }
}
*/

uint8_t mirf_data_ready() 
// Checks if data is available for reading
{
    if (PTX) return 0;
    uint8_t status;
    // Read MiRF status 
    mirf_CSN_lo;                                // Pull down chip select
    status = SPI_Write_Byte(NOP);               // Read status register
    mirf_CSN_hi;                                // Pull up chip select
    return status & (1<<RX_DR);

}

uint8_t rx_fifo_is_empty() {
	
	uint8_t fifo_status = 0;

	mirf_read_register(FIFO_STATUS, &fifo_status, 1);
	
	return (uint8_t)(fifo_status&0x01);
}

void flush_rx_fifo() {

    mirf_CSN_lo;
    SPI_Write_Byte(FLUSH_RX);
    mirf_CSN_hi;

}

void mirf_get_data(uint8_t * data) 
// Reads mirf_PAYLOAD bytes into data array
{
    mirf_CSN_lo;                               		// Pull down chip select
    SPI_Write_Byte( R_RX_PAYLOAD );            		// Send cmd to read rx payload
    SPI_ReadWrite_Block(data,data,PAYLOAD_SIZE); 	// Read payload
    mirf_CSN_hi;                               		// Pull up chip select
    mirf_config_register(STATUS,(1<<RX_DR));   		// Reset status register
}

void mirf_config_register(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
    mirf_CSN_lo;
    SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
    SPI_Write_Byte(value);
    mirf_CSN_hi;
}

void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
    mirf_CSN_lo;
    SPI_Write_Byte(R_REGISTER | (REGISTER_MASK & reg));
    SPI_ReadWrite_Block(value,value,len);
    mirf_CSN_hi;
}

void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len) 
// Writes an array of bytes into inte the MiRF registers.
{
    mirf_CSN_lo;
    SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
    SPI_Write_Block(value,len);
    mirf_CSN_hi;
}


void mirf_send(uint8_t * value, uint8_t len) 
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    while (PTX) {}                  // Wait until last paket is send

    mirf_CE_lo;

    PTX = 1;                        // Set to transmitter mode
    TX_POWERUP;                     // Power up
    
    mirf_CSN_lo;                    // Pull down chip select
    SPI_Write_Byte( FLUSH_TX );     // Write cmd to flush tx fifo
    mirf_CSN_hi;                    // Pull up chip select
    
    mirf_CSN_lo;                    // Pull down chip select
    SPI_Write_Byte( W_TX_PAYLOAD ); // Write cmd to write payload
    SPI_Write_Block(value,len);   // Write payload
    mirf_CSN_hi;                    // Pull up chip select
    
    mirf_CE_hi;                     // Start transmission
}

void writeAckPayload(unsigned char *data, unsigned char size) {

	unsigned char k = 0;

	flushTxFifo();

    mirf_CSN_lo;

	SPI_Write_Byte(NRF_W_ACK_PAYLOAD_P0);

	for(k=0; k<size; k++) {
		SPI_Write_Byte(data[k]);
	}	

    mirf_CSN_hi;


}

uint8_t readPayloadWidthFromTopFifo() {
	uint8_t pWidth = 0;

    mirf_CSN_lo;
    SPI_Write_Byte(NRF_R_RX_PL_WID);
	pWidth = SPI_Write_Byte(NOP); 	// not specified in the datasheet but the "NRF_R_RX_PL_WID" has a parameter,
									// we need to send a NOP to receive the actual payload size
    mirf_CSN_hi;
	
	return pWidth;
}

uint8_t readPayloadWidthFromPipe0() {
	uint8_t pWidth = 0;

	mirf_read_register(RX_PW_P0, &pWidth, 1);
	
	return pWidth;
}

void flushTxFifo() {

    mirf_CSN_lo;
    SPI_Write_Byte(FLUSH_TX);
    mirf_CSN_hi;

}

void handleRFCommands() {

	unsigned int i=0;
	//uint8_t pWidth = 0;
	//uint8_t pWidthP0 = 0;

	if(mirf_data_ready()) {

		//if(spiCommError) {
		//	usart0Transmit(0xFE,1);
		//	return;
		//}

		rfFlags |= 0x02;

		// clear irq status
		mirf_config_register(STATUS, 0x70);

		// Sometimes happens that even if the IRQ for data reception is raised then the actual data
		// aren't present in the fifo with consequent wrong data read and wrong behavior of the robot.
		// To avoid this situtation we add this check before actually reading from the fifo to be 
		// sure there are correct data to be read.
		// We don't know why the IRQ for data reception is raised, maybe is not correctly reset sometimes
		// or it is raised when it shouldn't...
		if(rx_fifo_is_empty()) {
			return;
		}

		/*		
		pWidth = readPayloadWidthFromTopFifo();
		if(pWidth != 13) {	// discard the data if the expected payload size isn't correct
			usart0Transmit(pWidth, 1);
			return;
		}
		if(pWidth > 32) {	// from the datasheet if the payload is > 32 then the fifo should be flushed and the packet discarded
			flush_rx_fifo();
			return;
		}
		//usart0Transmit(pWidth, 1);
		pWidthP0 = readPayloadWidthFromPipe0();
		//usart0Transmit(pWidthP0, 1);
		if(pWidthP0 != 13) {
			usart0Transmit(pWidthP0, 1);
		}
		*/

		mirf_get_data(rfData);
		flush_rx_fifo();

		//usartTransmit(rfData[0]);

		if(rfDebugMode==1) {

			writeAckPayload(ackPayload, 16);
			
		} else {

			//if((data[3]&0b00001000)==0b00001000) {	// check the 4th bit to sleep
			// it was noticed that some robots sometimes "think" to receive something and the data read are wrong,
			// this could lead to go to sleep involuntarily; in order to avoid this situation we define that the
			// sleep message should be completely zero, but the flag bit
			if(rfData[0]==0 && rfData[1]==0 && rfData[2]==0 && rfData[3]==0b00001000 && rfData[4]==0 && rfData[5]==0) {

				sleep(60);

			}

			if(calibrateOdomFlag==0) { 
				speedr = (rfData[4]&0x7F);	// cast the speed to be at most 127, thus the received speed are in the range 0..127 (usually 0..100),
				speedl = (rfData[5]&0x7F);	// the received speed is then shifted by 3 (x8) in order to have a speed more or less
											// in the same range of the measured speed that is 0..800.
											// In order to have greater resolution at lower speed we shift the speed only by 2 (x4),
											// this means that the range is more or less 0..400.


				if((rfData[4]&0x80)==0x80) {			// motor right forward
					pwm_right_desired = speedr; 		// speed received (0..127) is expressed in 1/5 of mm/s (0..635 mm/s)
				} else {								// backward
					pwm_right_desired = -(speedr);
				}

				if((rfData[5]&0x80)==0x80) {			// motor left forward
					pwm_left_desired = speedl;
				} else {								// backward
					pwm_left_desired = -(speedl);
				}

			}


			for(i=0; i<3; i++) {
				dataLED[i]=rfData[i]&0xFF;
			}
			pwm_red = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[0]&0xFF)/100;
			pwm_blue = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[1]&0xFF)/100;
			pwm_green = MAX_LEDS_PWM-MAX_LEDS_PWM*(dataLED[2]&0xFF)/100;
			updateRedLed(pwm_red);
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);


			if((rfData[3]&0b00000001)==0b00000001) {	// turn on back IR
				LED_IR1_LOW;
			} else {
				LED_IR1_HIGH;
			}

			if((rfData[3]&0b00000010)==0b00000010) {	// turn on front IRs
				LED_IR2_LOW;
			} else {
				LED_IR2_HIGH;
			}

			if((rfData[3]&0b00000100)==0b00000100) {	// check the 3rd bit to enable/disable the IR receiving
				irEnabled = 1;
			} else {
				irEnabled = 0;
			}

			if((rfData[3]&0b00010000)==0b00010000) {	// check the 5th bit to start calibration of all sensors
				calibrateSensors();
				resetOdometry();
			}

			if((rfData[3]&0b01000000)==0b01000000) {	// check the seventh bit to enable/disable obstacle avoidance
				obstacleAvoidanceEnabled = 1;
			} else {
				obstacleAvoidanceEnabled = 0;
			}

			if((rfData[3]&0b10000000)==0b10000000) {	// check the eight bit to enable/disable obstacle avoidance
				cliffAvoidanceEnabled = 1;
			} else {
				cliffAvoidanceEnabled = 0;
			}

			// handle small green leds
			#ifdef HW_REV_3_1			

				if(bit_is_set(rfData[6], 0) ) {
					GREEN_LED0_ON;
				} else {
					GREEN_LED0_OFF;
				}
				
				if(bit_is_set(rfData[6], 1) ) {
					GREEN_LED1_ON;
				} else {
					GREEN_LED1_OFF;
				}
				
				if(bit_is_set(rfData[6], 2) ) {
					GREEN_LED2_ON;
				} else {
					GREEN_LED2_OFF;
				}												

				if(bit_is_set(rfData[6], 3) ) {
					GREEN_LED3_ON;
				} else {
					GREEN_LED3_OFF;
				}

				if(bit_is_set(rfData[6], 4) ) {
					GREEN_LED4_ON;
				} else {
					GREEN_LED4_OFF;
				}

				if(bit_is_set(rfData[6], 5) ) {
					GREEN_LED5_ON;
				} else {
					GREEN_LED5_OFF;
				}

				if(bit_is_set(rfData[6], 6) ) {
					GREEN_LED6_ON;
				} else {
					GREEN_LED6_OFF;
				}

				if(bit_is_set(rfData[6], 7) ) {
					GREEN_LED7_ON;
				} else {
					GREEN_LED7_OFF;
				}

			#endif
		
			if(currentSelector == 8) {
				if(calibrateOdomFlag==0) {
					if((rfData[7]&0b00000001)==0b00000001) {
						calibrateSensors();
						proximityResult[8] = 1023;	// because the first time this value could be low after calibration
						proximityResult[11] = 1023;	// and in that case a false black line will be detected
						calibState = CALIBRATION_STATE_FIND_THRS_0;
						calibVelIndex = 1;
						calibrateOdomFlag = 1;
					}
				}
			}

			// read and handle the remaining bytes of the payload (at the moment not used)


			// write back the ack payload
			ackPayload[0] = packetId&0xFF;

			switch(packetId) {
				case 3:
					ackPayload[1] = proximityResult[0]&0xFF;
					ackPayload[2] = proximityResult[0]>>8;
					ackPayload[3] = proximityResult[1]&0xFF;
					ackPayload[4] = proximityResult[1]>>8;
					ackPayload[5] = proximityResult[2]&0xFF;
					ackPayload[6] = proximityResult[2]>>8;
					ackPayload[7] = proximityResult[3]&0xFF;
					ackPayload[8] = proximityResult[3]>>8;
					ackPayload[9] = proximityResult[5]&0xFF;
					ackPayload[10] = proximityResult[5]>>8;
					ackPayload[11] = proximityResult[6]&0xFF;
					ackPayload[12] = proximityResult[6]>>8;
					ackPayload[13] = proximityResult[7]&0xFF;
					ackPayload[14] = proximityResult[7]>>8;
					#ifdef HW_REV_3_1
						ackPayload[15] = CHARGE_ON | (BUTTON0 << 1) | (CHARGE_STAT << 2);
					#else
						ackPayload[15] = CHARGE_ON | (BUTTON0 << 1);
					#endif
					packetId = 4;
					break;

				case 4:
					ackPayload[1] = proximityResult[4]&0xFF;
					ackPayload[2] = proximityResult[4]>>8;
					ackPayload[3] = proximityResult[8]&0xFF;
					ackPayload[4] = proximityResult[8]>>8;
					ackPayload[5] = proximityResult[9]&0xFF;
					ackPayload[6] = proximityResult[9]>>8;
					ackPayload[7] = proximityResult[10]&0xFF;
					ackPayload[8] = proximityResult[10]>>8;
					ackPayload[9] = proximityResult[11]&0xFF;
					ackPayload[10] = proximityResult[11]>>8;
					ackPayload[11] = accX&0xFF;
					ackPayload[12] = accX>>8;
					ackPayload[13] = accY&0xFF;
					ackPayload[14] = accY>>8;
					ackPayload[15] = irCommand;
					packetId = 5;
					break;

				case 5:
					ackPayload[1] = proximityValue[0]&0xFF;
					ackPayload[2] = proximityValue[0]>>8;
					ackPayload[3] = proximityValue[2]&0xFF;
					ackPayload[4] = proximityValue[2]>>8;
					ackPayload[5] = proximityValue[4]&0xFF;
					ackPayload[6] = proximityValue[4]>>8;
					ackPayload[7] = proximityValue[6]&0xFF;
					ackPayload[8] = proximityValue[6]>>8;
					ackPayload[9] = proximityValue[10]&0xFF;
					ackPayload[10] = proximityValue[10]>>8;
					ackPayload[11] = proximityValue[12]&0xFF;
					ackPayload[12] = proximityValue[12]>>8;
					ackPayload[13] = proximityValue[14]&0xFF;
					ackPayload[14] = proximityValue[14]>>8;
					ackPayload[15] = currentSelector;
					packetId = 6;
					break;

				case 6:
					ackPayload[1] = proximityValue[8]&0xFF;
					ackPayload[2] = proximityValue[8]>>8;
					ackPayload[3] = proximityValue[16]&0xFF;
					ackPayload[4] = proximityValue[16]>>8;
					ackPayload[5] = proximityValue[18]&0xFF;
					ackPayload[6] = proximityValue[18]>>8;
					ackPayload[7] = proximityValue[20]&0xFF;
					ackPayload[8] = proximityValue[20]>>8;
					ackPayload[9] = proximityValue[22]&0xFF;
					ackPayload[10] = proximityValue[22]>>8;
					ackPayload[11] = accZ&0xFF;
					ackPayload[12] = accZ>>8;	
					ackPayload[13] = batteryLevel&0xFF;
					ackPayload[14] = batteryLevel>>8;
					ackPayload[15] = 0;
					packetId = 7;
					break;


				case 7:
					ackPayload[1] = ((signed long int)leftMotSteps)&0xFF;
					ackPayload[2] = ((signed long int)leftMotSteps)>>8;
					ackPayload[3] = ((signed long int)leftMotSteps)>>16;
					ackPayload[4] = ((signed long int)leftMotSteps)>>24;
					ackPayload[5] = ((signed long int)rightMotSteps)&0xFF;
					ackPayload[6] = ((signed long int)rightMotSteps)>>8;
					ackPayload[7] = ((signed long int)rightMotSteps)>>16;
					ackPayload[8] = ((signed long int)rightMotSteps)>>24;
					lastTheta = theta;
					ackPayload[9] = ((signed int)(lastTheta*573.0))&0xFF;	// radians to degrees => 573 = 1800/PI
					ackPayload[10] = ((signed int)(lastTheta*573.0))>>8;				
					ackPayload[11] = ((unsigned int)xPos)&0xFF;
					ackPayload[12] = ((unsigned int)xPos)>>8;
					ackPayload[13] = ((unsigned int)yPos)&0xFF;
					ackPayload[14] = ((unsigned int)yPos)>>8;
				
					//ackPayload[9] = ((unsigned int)(thetaOld*573.0))&0xFF;	// radians to degrees => 573 = 1800/PI
					//ackPayload[10] = ((unsigned int)(thetaOld*573.0))>>8;
					//ackPayload[11] = ((unsigned int)xPosOld)&0xFF;
					//ackPayload[12] = ((unsigned int)xPosOld)>>8;
					//ackPayload[13] = ((unsigned int)yPosOld)&0xFF;
					//ackPayload[14] = ((unsigned int)yPosOld)>>8;
					ackPayload[15] = 0;
					packetId = 3;
					break;

			}

			writeAckPayload(ackPayload, 16);

		}

		

	}

}

void rfEnableDebugMode() {
	rfDebugMode = 1;
	rfDebugCounter = 3;
}

void rfDisableDebugMode() {
	rfDebugMode = 0;
}

void rfDebugSendData() {
	ackPayload[0] = rfDebugCounter;
	while(rfData[0] != rfDebugCounter) {
		handleRFCommands();
	}
	ackPayload[0] = 0x00;
	if(rfDebugCounter < 255) {
		rfDebugCounter++;
	} else {
		rfDebugCounter = 3;
	}
}

void rfDebugNextPacket() {
	rfDebugCounter = 3;
}






