#ifndef __RF_COMM_H__
#define __RF_COMM_H__


/*
 * Author: Timo Birnschein (timo.birnschein@microforge.de)
 * Date: 2018/07/03
 * License: LGPL
 * Special thanks to Acton for making a remote so bad that it motivated me to
 * reverse engineer it and make my own. It was an interesting task and I 
 * definitely learned a lot in the process. Cheers.
 * 
 * Overview:
 * This is a bare bones and speciallized NRF24L01+ driver to communicate with
 * the Acton Blink QU4TRO and take control over it.
 * 
 * Note: The NRF driver is actualy not really a driver as it only replicates all
 * instructions the microprocessor of the remote was also sending to its NRF.
 * Therefore, please take this with a grain of salt.
 * 
 */

#include <stdint.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Tools.h"
#include "Hardware.h"

// define all basic addesses for operating the NRF24L01+
// Please refere to NRF24L01+ Datasheet for more info on these
#define R_REGISTER 			0x00 // 0b00000000	// last five bits define register address: 000AAAAA
#define W_REGISTER 			0x20 // 0b00100000	// last five bits define register address: 000AAAAA
#define R_RX_PAYLOAD 		0x61 // 0b01100001
#define W_TX_PAYLOAD 		0xA0 // 0b10100001
#define FLUSH_TX 			0xE1 // 0b11100001
#define FLUSH_RX			0xE2 // 0b11100010
#define REUSE_TX_PL			0xE3 // 0b11100011
#define R_RX_PL_WID			0x60 // 0b01100000
#define W_ACK_PAYLOAD		0xA8 // 0b10101000	// last three bits define PIPE address: 00000PPP
#define W_TX_PAYLOAD_NOACK	0xB0 // 0b10110000
#define NOP				    0xFF // 0b11111111

// define all RW registers that - for some reason - are considered a
// different category than the above
// Please refere to NRF24L01+ Datasheet for more info on these
#define CONFIG 				0x00
#define EN_AA 				0x01
#define EN_RXADDR 			0x02
#define SETUP_AW 			0x03
#define SETUP_RETR 			0x04
#define RF_CH 				0x05
#define RF_SETUP 			0x06
#define STATUS 				0x07
#define OBSERVE_TX 			0x08
#define RPD 				0x09
#define RX_ADDR_P0 			0x0A
#define RX_ADDR_P1 			0x0B
#define RX_ADDR_P2 			0x0C
#define RX_ADDR_P3 			0x0D
#define RX_ADDR_P4 			0x0E
#define RX_ADDR_P5 			0x0F
#define TX_ADDR 			0x10
#define RX_PW_P0 			0x11
#define RX_PW_P1 			0x12
#define RX_PW_P2 			0x13
#define RX_PW_P3 			0x14
#define RX_PW_P4 			0x15
#define RX_PW_P5 			0x16
#define FIFO_STATUS 	0x17
#define DYNPD 				0x1C
#define FEATURE 			0x1D

extern volatile uint8_t rtr_counter;  // MAX retry counter for counting how many times a message was tried to be sent or tried to be received
extern volatile uint8_t freqCounter;  // Counter that holds the current communication frequency number.
extern volatile uint8_t lastKnownFrequency;  // Counter that holds the current communication frequency number.
extern volatile uint8_t lostCounter;  // If counter > 2, the remote should be reinitialized completely. Chances are, SPI communication messed up NRF chip.

extern volatile uint8_t numHopsBeforeCheckingLastKnown; // Holds the number of frequency hops that may be taken before the previously working will be tested
extern volatile uint8_t numHopsTaken; // Holds the number of frequency hops that may be taken before the previously working will be tested

uint8_t write_ToAddress(uint8_t addr, uint8_t value);

uint8_t read_FromAddress(uint8_t addr);

uint8_t exec_command(uint8_t addr);

// Writes a number of bytes to the NRF
uint8_t write_BytesToAddress(uint8_t addr, uint8_t *values, uint8_t length);

// Reads a number of bytes from the NRF
uint8_t read_BytesFromAddress(uint8_t addr, uint8_t *values, uint8_t length);

// Pulses the CS line to let NRF know that data needs to be sent
void trigger_CS(void);

#endif // __QUATRO_H__

