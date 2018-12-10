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
 
#include "RF_Comm.h"

volatile uint8_t rtr_counter = 0;  // MAX retry counter for counting how many times a message was tried to be sent or tried to be received
volatile uint8_t freqCounter = 0;  // Counter that holds the current communication frequency number.
volatile uint8_t lastKnownFrequency = 0;  // Counter that holds the current communication frequency number.
volatile uint8_t lostCounter = 0;  // If counter > 2, the remote should be reinitialized completely. Chances are, SPI communication messed up NRF chip.

volatile uint8_t numHopsBeforeCheckingLastKnown = 3; // Holds the number of frequency hops that may be taken before the previously working will be tested
volatile uint8_t numHopsTaken = 0; // Holds the number of frequency hops that may be taken before the previously working will be tested

uint8_t write_ToAddress(uint8_t addr, uint8_t value)
{
  uint8_t status = 0x00;
  digitalWrite(CSN_PIN, LOW);
  status = SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(CSN_PIN, HIGH);
  
  delayMicroseconds(50);

  return status;
}

uint8_t read_FromAddress(uint8_t addr)
{
  uint8_t status = 0x00;
  digitalWrite(CSN_PIN, LOW);
  status = SPI.transfer(addr);
  status = SPI.transfer(0x00);
  digitalWrite(CSN_PIN, HIGH);
  
  delayMicroseconds(50);

  return status;
}


uint8_t exec_command(uint8_t addr)
{
  uint8_t status = 0x00;
  digitalWrite(CSN_PIN, LOW);
  status = SPI.transfer(addr);
  digitalWrite(CSN_PIN, HIGH);
  delayMicroseconds(50);
  
  return status;
}

// Writes a number of bytes to the NRF
uint8_t write_BytesToAddress(uint8_t addr, uint8_t *values, uint8_t length)
{
  uint8_t status = 0x00;
  digitalWrite(CSN_PIN, LOW);
  status = SPI.transfer(addr);
  for (uint8_t i = 0; i < length; i++)
  {
    status = SPI.transfer(values[i]);
  }
  digitalWrite(CSN_PIN, HIGH);
  delayMicroseconds(50);

  return status;
}

// Reads a number of bytes from the NRF
uint8_t read_BytesFromAddress(uint8_t addr, uint8_t *values, uint8_t length)
{
  uint8_t status = 0x00;
  digitalWrite(CSN_PIN, LOW);
  status = SPI.transfer(addr);
  for (uint8_t i = 0; i < length; i++)
  {
    values[i] = SPI.transfer(0xFF); // write as many fake bytes as we need to read values from the NRF
  }
  digitalWrite(CSN_PIN, HIGH);
  delayMicroseconds(50);

  return status;
}

// Pulses the CS line to let NRF know that data needs to be sent
void trigger_CS(void)
{
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(CS_PIN, LOW);
}

