/*
   Author: Timo Birnschein (timo.birnschein@microforge.de)
   Date: 2018/07/03
   License: LGPL
*/

#include "Driving_StateMachine.h"

volatile mainStateMachine mainState;
volatile driveStateMachine driveState;
volatile performanceStateMachine performanceState;


volatile bool messageType;      // There are two messages to be sent to the board. False sends first, true the second
volatile bool TX_DS_WasSetAlready_WaitFor_RX_DR = false;      // Extremely unusual state where ACK_PAYLOAD was lost and RX_DR never comes on
