/*
   Author: Timo Birnschein (timo.birnschein@microforge.de)
   Date: 2018/07/03
   License: LGPL
*/

#ifndef __PAIRING_STATEMACHINE_H__
#define __PAIRING_STATEMACHINE_H__

#include <stdint.h>
#include <EEPROM.h>
#include <SPI.h>

enum pairingStateMachine 
  {
    initPairing_1, 
    checkForACK_1, 
    pairingStage_2, 
    checkForACK_2, 
    setAsReceiver, 
    receiveNewAddress, 
    setupNewInitAfterPairing
  };
  
extern volatile pairingStateMachine pairingState;

extern volatile uint8_t pairingAttempts;  // Counts the number of pairing attempts until the system goes back into drive

uint8_t pairRemote(void);

#endif
