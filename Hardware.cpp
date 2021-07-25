/*
   Author: Timo Birnschein (timo.birnschein@microforge.de)
   Date: 2018/07/03
   License: LGPL
*/

#include "Hardware.h"

volatile uint8_t frequencies[16] = {0x02, 0x19, 0x30, 0x47, 0x05, 0x1C, 0x33, 0x4A, 0x08, 0x1F, 0x36, 0x4D, 0x0B, 0x22, 0x39, 0x50};
volatile uint8_t quatroAddress[5] = {0x51, 0xFC, 0xC9, 0xC9, 0xC9};
volatile uint8_t quatroPairingAddress1[5] = {0xE3, 0xE3, 0xE3, 0xE3, 0xE3};
volatile uint8_t quatroPairingAddress2[5] = {0xE3, 0xE3, 0xE3, 0xE3, 0xE3};
volatile uint8_t quatroMessage_1[5] = {0x00, 0x80, 0x00, 0x00, 0x00};
volatile uint8_t quatroMessage_2[5] = {0x80, 0x00, 0x00, 0x00, 0x00};
volatile uint8_t quatroPairingMessage[2] = {0xC0, 0x00};
volatile uint8_t quatroPairingReturnMessage[3] = {0x00, 0x00, 0x00};
volatile uint8_t quatroReturnMessage[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

volatile uint8_t boardBatteryState = 0;
volatile float remoteBatteryVoltage = 0; // Stores the current remote battery voltage to allow turning off the throttle when battery reaches critical voltage. DCDC shuts off at 2.3V

volatile bool remoteBatteryLevelCritical = false; // If true, no more throttle will be allowed, only brakes!

void init_remote(void)
{
  // Check if remote was successfully paired before. If yes, startup as normal.
  // If not paired successfully or pairing button was just pressed, goto pairing.
  
  if (EEPROM.read(7) == 0x5A) // we already successfully paired the remote
  {
    quatroAddress[0]= EEPROM.read(0);
    quatroAddress[1]= EEPROM.read(1);
    quatroAddress[2]= EEPROM.read(2);
    quatroAddress[3]= EEPROM.read(3);
    quatroAddress[4]= EEPROM.read(4);
  }
  return;
}


void set_batteryState(uint8_t value)
{
  static uint32_t oldMillis = 0; // This is good for 1,193h of battery status blinking when remote left on - not that any remote battery would last that long

  if (millis() - oldMillis > 750)
  {
    digitalWrite(battLED1_PIN, (value & 0x08) ? LOW : HIGH); // Battery status < 100%
    digitalWrite(battLED2_PIN, (value & 0x04) ? LOW : HIGH); // Battery status < 75%
    digitalWrite(battLED3_PIN, (value & 0x02) ? LOW : HIGH); // Battery status < 50%
    digitalWrite(battLED4_PIN, (value & 0x01) ? LOW : HIGH); // Battery status < 25%
    
    oldMillis = millis();
  }
  else if (millis() - oldMillis > 20)
  {
    digitalWrite(battLED1_PIN, LOW); // Battery status < 100%
    digitalWrite(battLED2_PIN, LOW); // Battery status < 75%
    digitalWrite(battLED3_PIN, LOW); // Battery status < 50%
    digitalWrite(battLED4_PIN, LOW); // Battery status < 25%
  }
}

// Rescale input values to a range from -1 to 1
float rescaleADCThrottleValue(uint16_t input, uint16_t valueMIN, uint16_t valueMAX, uint16_t valueCenter)
{
  if (input < valueCenter) // e.g. input == 360 -> accelerating
  {
    uint16_t diff = valueCenter - valueMIN; // ~163
    float retVal =  (float)(1.0f / (float)diff) * (float)((float)input - (float)valueCenter);
    if (retVal < -1.0f)
      retVal = -1.0f;
    return retVal;
  }
  else if (input > valueCenter) // e.g. input == 691 -> braking
  {
    uint16_t diff = valueMAX - valueCenter; // ~168
    float retVal =  (float)(1.0f / (float)diff) * (float)((float)input - (float)valueCenter);
    if (retVal > 1.0f)
      retVal = 1.0f;
    return retVal;
  }
  else
  {
    return 0.0f;
  }
  return 0;
}

/* UNTESTED
 *  Must be a value between -1 and 1
 */
float exponentialCurve(float inputValue, float expoFactor)
{
  return ( (1 - expoFactor) * (inputValue * inputValue * inputValue) ) + (expoFactor * inputValue);
}

// This function implements a digital filter to smoothen out user jitter on the throttle potentiometer
float pt1_damper (float input, float dampingFactor, float integralPart, float &lastDampedValue, float &lastIntegralPart)
{
  lastDampedValue = lastDampedValue + ((input - lastDampedValue) * dampingFactor);
  
  if (lastDampedValue < input)
    lastIntegralPart = lastIntegralPart + integralPart;
  else if (lastDampedValue > input)
    lastIntegralPart = lastIntegralPart - integralPart;
  
  lastDampedValue = lastDampedValue + lastIntegralPart;
  
  return lastDampedValue;
}

// This function implements a digital filter to smoothen out user jitter on the throttle potentiometer -- UNTESTED
float pt1_damper (float input, float dampingFactor, float &lastDampedValue)
{
  lastDampedValue = lastDampedValue + ((input - lastDampedValue) * dampingFactor);
  return lastDampedValue;
}

// Removes the deadzone from an existing motor controller if programmed in
float deadzoneCompensationAndRescale(float input, float posDeadZone, float negDeadZone, uint8_t boardMin, uint8_t boardMax, uint8_t boardDeadzoneMin, uint8_t boardDeadzoneMax, uint8_t boardCenter)
{
  uint8_t retVal = 0;
  uint8_t centerToMin = (boardCenter - boardDeadzoneMin) - boardMin;
  uint8_t centerToMax = boardMax - (boardCenter + boardDeadzoneMax);
  
  if (input > (posDeadZone)) // posDeadZone in percent / 100
  { // braking
    retVal = (boardCenter + boardDeadzoneMax) + (centerToMax * input);
    if (retVal < boardCenter)
      retVal = boardCenter;
  }
  else if (input < (negDeadZone)) // negDeadZone in percent / 100
  { // accelerating
    retVal = (boardCenter - boardDeadzoneMin) - (centerToMin * -input);
    if (retVal > boardCenter)
      retVal = boardCenter;
  }
  else
  {
    retVal = boardCenter;
  }
  
  return retVal;
}

