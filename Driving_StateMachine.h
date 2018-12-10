/*
   Author: Timo Birnschein (timo.birnschein@microforge.de)
   Date: 2018/07/03
   License: LGPL
*/

#ifndef __DRIVING_STATEMACHINE_H__
#define __DRIVING_STATEMACHINE_H__

enum mainStateMachine {pairingRemote, initRemote, driveRemote, lostRemote, idleRemote, offRemote};
enum driveStateMachine {flushTX, clearPendingIRQ, sendMessage, checkStatus, receiveMessage};

extern volatile mainStateMachine mainState;
extern volatile driveStateMachine driveState;

extern volatile bool messageType;      // There are two messages to be sent to the board. False sends first, true the second
extern volatile bool TX_DS_WasSetAlready_WaitFor_RX_DR;      // Extremely unusual state where ACK_PAYLOAD was lost and RX_DR never comes on

#endif // __DRIVING_STATEMACHINE_H__
