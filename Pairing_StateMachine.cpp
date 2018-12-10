/*
   Author: Timo Birnschein (timo.birnschein@microforge.de)
   Date: 2018/07/03
   License: LGPL
   
     WARNING:  The pairing is rather complicated and elaborate. Lots of states and a strikt timing constraints.
               If timing is not met, the board will leave the pairing attempt immediately and go back to start.
               Make sure you communicate fast enough and avoid all unnecessary delayes and RS232 communication.

     STEPS REQUIRED:
     - Write to CONFIG 0x20 0x0E: CRC, 2 bytes, Activate power
     - Set powerlevel to -12db power and 2mbits
     - Set initial channel to 2
     - Set RX/TX address to 0xE3 0xE3 0xE3 0xE3 0xE3
     - Clear all pending IRQs
     - Flush TX
     - Clear all pending IRQs again
     - Send binding message: 0xC0 0xFIRST_BYTE_OF_LAST_ADDRESS
     - Check if an ACK was received  by checking for TX_DS bit in STATUS register: 0x2E (TX_DS | RX_FIFO_EMPTY)
     - If no ACK was received MAX_RTR will turn on: 0x1E: (MAX_RTR | RX_FIFO_EMPTY)
       - Check STATUS again to make sure no ACK was received
       - Increment frequency hopper
       - Start over at the top with same pairing address: 0xE3 0xE3 0xE3 0xE3 0xE3
     - If ACK was received
       - Set RX/TX address to 0xFIRST_BYTE_OF_LAST_ADDRESS (0xFF - 0xFIRST_BYTE_OF_LAST_ADDRESS) 0xE3 0xE3 0xE3
       - Flush TX buffer
       - Set powerlevel to -6db power and 2mbits (slightly more power but this time we're reasonably sure we're communicating with the correct receiver)
       - Clear all pending IRQs
       - Clear all pending IRQs again
       - Send binding message again on new address: 0xC0 0xFIRST_BYTE_OF_LAST_ADDRESS
       - Check if an ACK was received  by checking for TX_DS bit in STATUS register: 0x2E (TX_DS | RX_FIFO_EMPTY)
         - If no ACK was received MAX_RTR will turn on: 0x1E: (MAX_RTR | RX_FIFO_EMPTY)
           - Check STATUS again to make sure no ACK was received
           - Increment frequency hopper
           - Start over at the top with same pairing address: 0xE3 0xE3 0xE3 0xE3 0xE3
         - If ACK was received
           - Reset NRF to be a receiver: Write to CONFIG 0x20 0x0F: CRC, 2 bytes, Activate power, PRIM_RX
           - Flush RX Buffer
           - Clear all pending IRQ
           - Set powerlevel to 0db power and 2mbits
           - Check STATUS repeatedly until 0x40 is present: RX_DR IRQ was fired!
           - Retrieve message from the FIFO and store new address bytes from received message: 0x00 0xXX 0xXX
     - If all was successful, restart as usual using new address
  */

#include "Pairing_StateMachine.h"
#include "RF_Comm.h"
#include "Driving_StateMachine.h"

volatile pairingStateMachine pairingState;

volatile uint8_t pairingAttempts = 0;  // Counts the number of pairing attempts until the system goes back into drive

uint8_t pairRemote()
{
  static uint8_t status = 0x00;   // used for general storage of the STATUS
  static uint8_t status1 = 0x00;  // used when two STATUS values need to be compared
  static uint8_t status2 = 0x00;  // used when two STATUS values need to be compared


  //Serial.println("P Pairing Remote");
  digitalWrite(lostLED_PIN, LOW);
  digitalWrite(3, HIGH); // Set debug pin

  // UNUSED AT THE MOMENT TODO! If pairing needs to be left, restart remote. Button is hard to reach anyways.
  if (pairingAttempts > 100)
  {
    // Pairing could not be performed successfully! Go back into drive
    pairingState = setupNewInitAfterPairing;
    return 0; // Return and let top state know to break
  }

  if (freqCounter > 15) // Sanity check on the frequency counter
    freqCounter = 0 ;

  // The frequency counter must be initialized somewhere else before reaching this
  switch (pairingState) // STATE CONFIRMED WORKING
  {
    case initPairing_1: // send out pairing command on standard pairing address 5 x 0xE3
      //Serial.println("P   initPairing_1");
      set_batteryState(1);
      status = write_ToAddress(W_REGISTER | CONFIG, 0x0E); // Enable CRC, 2 byte CRC, Power Up
      status = write_ToAddress(W_REGISTER | RF_SETUP, 0x0B); // set RF_Config to low power and 2 mBits
      status = write_ToAddress(W_REGISTER | RF_CH, frequencies[freqCounter++]); // Set operating frequency to 2: 2.402 Ghz
      status = write_BytesToAddress(W_REGISTER | TX_ADDR, quatroPairingAddress1, 5); // Set PAIRING TC address to board
      status = write_BytesToAddress(W_REGISTER | RX_ADDR_P0, quatroPairingAddress1, 5); // Set PAIRING RX address from board
      status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
      status = exec_command(FLUSH_TX); // clean out the TX buffer - should return 0x0E
      status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
      // Change pairing message to reflect the first byte of the last known address

      quatroPairingMessage[1] = quatroAddress[0];
      status = write_BytesToAddress(W_TX_PAYLOAD, quatroPairingMessage, 2); // Send message 1 to the transmitter
      trigger_CS(); // Trigger the transmitter

      rtr_counter = 0; // Reset RTR counter so that we can cycle through checking for ACK
      pairingState = checkForACK_1;
      break;

    case checkForACK_1:
      set_batteryState(2);
      //Serial.println("P   checkForACK_1");
      status1 = exec_command(STATUS); // Read what's going on in the status - did we successfully send the message or timed out?
      status2 = exec_command(NOP);    // Read a second time using the NOP command which should also return the same value

      if (status1 == status2)
      {
        if (status1 == 0x0E) // We have not yet received any ACK from the board. We will wait.
        {
          //Serial.println("P     0x0E");
          set_batteryState(3);
          rtr_counter++;
          //Serial.println(rtr_counter);
          if (rtr_counter > 10)
          {
            rtr_counter = 0; // Reset the counter
            // we have reached the maximum number of retries and should concider this channel unused
            // Try again with a new frequncy which was already incremented in the init state
            // It is expected that this state will never be reached. If we did anyways, there is something wrong and we should restart pairing
            pairingState = initPairing_1;
          }
          //delay(1);
        }
        else if (status1 & 0x10) // MAX_RTR was set. We send the same message again
        {
          //Serial.print("P     ");
          //Serial.println(status1);
          set_batteryState(4);
          rtr_counter++;
          //Serial.println(rtr_counter);

          status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
          //delay(1);
          status = exec_command(FLUSH_TX); // clean out the TX buffer - should return 0x0E
          //delay(1);
          status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
          //delay(1);
          // Change pairing message to reflect the first byte of the last known address

          quatroPairingMessage[1] = quatroAddress[0];
          status = write_BytesToAddress(W_TX_PAYLOAD, quatroPairingMessage, 2); // Send message 1 to the transmitter
          trigger_CS();
          //delay(1);

          if (rtr_counter > 1)
          {
            rtr_counter = 0; // Reset the counter
            // we have reached MAX_RTR maximum number of retries and need to start over with a new frequency!
            pairingState = initPairing_1;
          }
        }
        else if (status1 & 0x20) // Data was successfully sent and we can continue to the next state pairingStage_2
        {
          //Serial.print("P     ");
          //Serial.println(status1);
          set_batteryState(5);
          // The Pairing message was sent, ACK was received and we can continue
          pairingState = pairingStage_2;
        }
      }
      break;

    case pairingStage_2:  // Switch to negociated pairing address and send same pairing message again
      //Serial.println("P   pairingStage_2");
      set_batteryState(6);

      // Change pairing message to reflect the first byte of the last known address
      quatroPairingAddress2[0] = quatroAddress[0];
      quatroPairingAddress2[1] = 0xFF - quatroAddress[0];

      status = write_BytesToAddress(W_REGISTER | TX_ADDR, quatroPairingAddress2, 5); // Set PAIRING TC address to board
      status = write_BytesToAddress(W_REGISTER | RX_ADDR_P0, quatroPairingAddress2, 5); // Set PAIRING RX address from board
      status = exec_command(FLUSH_TX); // clean out the TX buffer - should return 0x0E
      status = write_ToAddress(W_REGISTER | RF_SETUP, 0x0D); // set RF_Config to medium power and 2 mBits
      status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
      status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E

      // Change pairing message to reflect the first byte of the last known address
      quatroPairingMessage[1] = quatroAddress[0];

      status = write_BytesToAddress(W_TX_PAYLOAD, quatroPairingMessage, 2); // Send message 1 to the transmitter
      trigger_CS();

      pairingState = checkForACK_2;

      break;

    case checkForACK_2: // If everything goes well, we receive the ACK from the board
      //Serial.println("P   checkForACK_2");
      set_batteryState(7);

      status1 = exec_command(STATUS); // Read what's going on in the status - did we successfully send the message or timed out?
      status2 = exec_command(NOP);    // Read a second time using the NOP command which should also return the same value

      if (status1 == status2)
      {
        if (status1 == 0x0E) // We have not yet received any ACK from the board. We will wait.
        {
          //Serial.println("P     0x0E");
          set_batteryState(10);
          rtr_counter++;
          //Serial.println(rtr_counter);
          if (rtr_counter > 10)
          {
            rtr_counter = 0; // Reset the counter
            // we have reached the maximum number of retries and should concider this channel unused
            // Try again with a new frequncy which was already incremented in the init state
            // It is expected that this state will never be reached. If we did anyways, there is something wrong and we should restart pairing
            pairingState = initPairing_1;
          }
        }
        else if (status1 & 0x10) // MAX_RTR was set and RX FIFO is empty
        {
          //Serial.print("P     ");
          //Serial.println(status1);
          set_batteryState(8);
          rtr_counter++;
          //Serial.println(rtr_counter);

          status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
          status = exec_command(FLUSH_TX); // clean out the TX buffer - should return 0x0E
          status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E

          // Change pairing message to reflect the first byte of the last known address
          quatroPairingMessage[1] = quatroAddress[0];
          status = write_BytesToAddress(W_TX_PAYLOAD, quatroPairingMessage, 2); // Send message 1 to the transmitter
          trigger_CS();

          if (rtr_counter > 1)
          {
            rtr_counter = 0; // Reset the counter
            // we have reached MAX_RTR maximum number of retries and need to start over with a new frequency!
            pairingState = initPairing_1;
          }
        }
        else if (status1 & 0x20) // Data was successfully sent and we can continue to the next state pairingStage_2
        {
          //Serial.print("P     ");
          //Serial.println(status1);
          set_batteryState(9);
          // The Pairing message was sent, ACK was received and we can continue
          pairingState = setAsReceiver;
        }
      }
      break;

    case setAsReceiver: // Switch roles using the same address to make sure it's really the board and listen
      //Serial.println("P   setAsReceiver");
      set_batteryState(10);

      status = write_ToAddress(W_REGISTER | CONFIG, 0x0F); // Enable CRC, 2 byte CRC, Power Up, PRX receive mode!
      status = exec_command(FLUSH_RX); // clean out the RX buffer - should return 0x2E at this stage
      status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
      status = write_ToAddress(W_REGISTER | RF_SETUP, 0x0F); // set RF_Config to high power and 2 mBits
      digitalWrite(CS_PIN, HIGH); // MUST BE HIGH IN ORDER TO RECEIVE IN PRX MODE!!!  <--- this pin has cost me 4h of my life

      pairingState = receiveNewAddress;
      break;

    case receiveNewAddress: // Listen on channel and wait for welcome message with two new address bytes
      digitalWrite(3, LOW); // Set debug pin
      //Serial.println("P   receiveNewAddress");
      set_batteryState(11);

      status1 = exec_command(STATUS); // Read what's going on in the status - did we successfully send the message or timed out?
      status2 = exec_command(NOP);    // Read a second time using the NOP command which should also return the same value

      if (status1 == status2)
      {
        if (status1 == 0x0E) // We have not yet received anything from the board. We will wait.
        {
          //Serial.println("P     0x0E");
          set_batteryState(12);
          rtr_counter++;
          //Serial.println(rtr_counter);
          if (rtr_counter > 100)
          {
            rtr_counter = 0; // Reset the counter
            // we have reached the maximum number of retries and should concider this channel unused
            // Try again with a new frequncy which was already incremented in the init state
            // It is expected that this state will never be reached. If we did anyways, there is something wrong and we should restart pairing
            pairingState = initPairing_1;
            digitalWrite(CS_PIN, LOW); // RESET CS PIN BECAUSE WE WILL GO BACK TO TRANSMIT MODE!!!  <--- this pin has cost me 4h of my life
          }
        }
        else if (status1 & 0x40) // RX_DR was set and RX FIFO contains data in PIPE 0 ---- SUCCESS!!!
        {
          //Serial.print("P     ");
          //Serial.println(status1);
          set_batteryState(13);
          // Get new address from board and store it in the EEProm so that we can use it later after power up
          status = read_BytesFromAddress(R_RX_PAYLOAD, quatroPairingReturnMessage, 3);
          // Take byte 1 and 2 as new address
          quatroAddress[0] = quatroPairingReturnMessage[1];
          quatroAddress[1] = quatroPairingReturnMessage[2];

          //Serial.println("******************** !!!!SUCCESSFULLY PAIRED!!!! ********************");
          //Serial.println("P     Store new address into EEPROM...");
          // Store the newly received address into EEPROM
          EEPROM.write(0, quatroAddress[0]);
          EEPROM.write(1, quatroAddress[1]);
          EEPROM.write(2, quatroAddress[2]);
          EEPROM.write(3, quatroAddress[3]);
          EEPROM.write(4, quatroAddress[4]);

          // Also store that this remote was already initialized
          //Serial.println("P     Set paired remote flag...");
          EEPROM.write(7, 0x5A);

          digitalWrite(CS_PIN, LOW); // RESET CS PIN BECAUSE WE WILL GO BACK TO TRANSMIT MODE!!!  <--- this pin has cost me 4h of my live

          for (int i = 0; i < 10; i++)
          {
            tone(6, 400, 100);
            delay(100);
            set_batteryState(0xFF);
            tone(6, 800, 100);
            delay(100);
            set_batteryState(0x00);
          }

          pairingState = setupNewInitAfterPairing;
          break;
        }
        else // something very much off happened and we should restart pairing
        {
          //Serial.println("P     Leaving pairing receiveNewAddress to restart. Something happend...");
          //Serial.print("P     ");
          //Serial.println(status);
          set_batteryState(14);

          // Something went wrong, we received a status we didn't anticipate
          freqCounter = 0; // Fresh start. We'll find the right frequency fast
          mainState = initRemote;
          driveState = flushTX;
          pairingState = initPairing_1;
          digitalWrite(CS_PIN, LOW); // RESET CS PIN BECAUSE WE WILL GO BACK TO TRANSMIT MODE!!!  <--- this pin has cost me 4h of my live
          digitalWrite(lostLED_PIN, HIGH);
        }
      }
      break;
    case setupNewInitAfterPairing:
      //Serial.println("P     Resetting controller and leaving pairing states after successful pairing");
      // Initialize state machines
      freqCounter = 0;
      mainState = initRemote;
      driveState = flushTX;
      pairingState = initPairing_1;
      set_batteryState(0xFF);
      messageType = false;

      init_remote(); // Read paired address from the EEPROM

      digitalWrite(3, LOW); // Set debug pin low
      break;
  }
}

