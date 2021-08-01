/*
   Author: Timo Birnschein (timo.birnschein@microforge.de)
   Date: 2018/07/03
   License: LGPL
*/

#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include <stdint.h>
#include <EEPROM.h>
#include <SPI.h>

#define CS_PIN              7
#define CSN_PIN             8
#define IRQ_PIN             2

#define battLED1_PIN        14
#define battLED2_PIN        15
#define battLED3_PIN        16
#define battLED4_PIN        17
#define lostLED_PIN         18

#define mode_PIN            5
#define pairing_PIN         10
/* 
 * TODO: Fine the correct pin for the battery voltage. 
 * Used to measure current battery voltage and refuse throttle when low.
 * Only brakes available when battery is low to prevent sudden cutoff of all functions when remote batteries are low.
 */
#define batteryVoltage_PIN  A0  

extern volatile uint8_t frequencies[16];
extern volatile uint8_t quatroAddress[5];
extern volatile uint8_t quatroPairingAddress1[5];
extern volatile uint8_t quatroPairingAddress2[5];
extern volatile uint8_t quatroMessage_1[5];
extern volatile uint8_t quatroMessage_2[5];
extern volatile uint8_t quatroPairingMessage[2];
extern volatile uint8_t quatroReturnMessage[8];
extern volatile uint8_t quatroPairingReturnMessage[3];
extern volatile uint8_t boardBatteryState;
extern volatile float   remoteBatteryVoltage;
extern volatile bool    remoteBatteryLevelCritical;

void init_remote(void);

void set_batteryState(uint8_t value);

void set_remoteBatteryAlarm();

float rescaleADCThrottleValue(uint16_t input, uint16_t valueMIN, uint16_t valueMAX, uint16_t valueCenter);

float exponentialCurve(float inputValue, float expoFactor);

float pt1_damper (float input, float dampingFactor, float integralPart, float &lastDampedValue, float &lastIntegralPart);

float pt1_damper (float input, float dampingFactor, float &lastDampedValue);

float deadzoneCompensationAndRescale(float input, float posDeadZone, float negDeadZone, uint8_t boardMin, uint8_t boardMax, uint8_t boardDeadzoneMin, uint8_t boardDeadzoneMax, uint8_t boardCenter);


#endif
