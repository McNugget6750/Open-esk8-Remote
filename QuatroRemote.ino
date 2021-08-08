/*
   Author: Timo Birnschein (timo.birnschein@microforge.de)
   Date: 2021/07/24
   License: LGPL
   Special thanks to Acton for making a remote so bad that it motivated me to
   reverse engineer it and make my own. It was an interesting task and I
   definitely learned a lot in the process. Cheers.

   Overview:
   This is a bare bones and speciallized NRF24L01+ driver to communicate with
   the Acton Blink QU4TRO and take control over it.
   This serves multiple functions that are important to me as of now:
   - Reduce the amount of deadzone in the controller.
   - Extend the throw of the throttle wheel
   - Use the entire travel to from 0 to 100% (or at least 90%)
   - Introduce a hysteresis to allow some wobble on the throttle wheel
   - Introduce a user definable ramp for different ride modes that does only
     change the acceleration and max speed but not the boards response to
     uneven surfaces and gravel
   - User definable exponential function for smooth cruising and fast acceleration
   - Behavior for quick responses for fast accel and quick strong brakes based
     on how fast and how much the throttle was moved.
*/

// TODO: Change all filters to use time based filtering instead of ticks

#include <stdint.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include "RF_Comm.h"
#include "Hardware.h"
#include "Pairing_StateMachine.h"
#include "Driving_StateMachine.h"


enum calibrationStateMachine
{
  findMIN,
  findMAX,
  findCenter
};

calibrationStateMachine throttleCalibration;
// Pairing button definition
int pairingButtonState = HIGH;               // the current reading from the input pin
int lastPairingButtonState = HIGH;    // the previous reading from the input pin
int modeButtonState = HIGH;               // the current reading from the input pin
int lastModeButtonState = HIGH;    // the previous reading from the input pin
unsigned long lastPairingButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long pairingButtonDebounceDelay = 50;    // the debounce time; increase if the output flickers

unsigned long lastModeButtonDebounceTime = 0;  // the last time the output pin was toggled
unsigned long modeButtonDebounceDelay = 50;    // the debounce time; increase if the output flickers

unsigned long lastTimeSinceStatusReport = 0;

// Throttle calibration values
uint16_t minThrottleADC = 1024;
uint16_t centerThrottleADC = 512;
uint16_t maxThrottleADC = 0;
uint8_t throttleHysteresis = 20;
uint16_t throttleCenterSettleTime = 1000;
uint8_t throttleCenterNoise = 4;
bool throttleCalibrated = true;

// Throttle Output filtering
float lastDampedValue = 0;
float dampingFactor = 0.05;
float dampingFactorDriveModeModifier = 10;
float integralPart = 0.00; // LEAVE AT ZERO! NOT STABLE!
float lastIntegralPart = 0;

// ACTON BLINK QU4TRO related values
float deadzone = 0.03;
float posDeadZone = 0.05; // Brake deadzone - 0.05 is still to high
float negDeadZone = -0.05;
float deadzoneHysteresis = 0.01; // when leaving the deadzone, subtract this until back inside
float expoFactor = 1.0;
uint8_t boardDeadzoneMin = 4;
uint8_t boardDeadzoneMax = -4; // Brake deadzone removal - 2 is still too high. I set it to -4 to get the complete range for brakes and then go from there.
// TODO: All deadzone removal values should be > 0 to make sense to the user and start at 0 when set.
uint8_t boardCenter = 128;
uint8_t boardMin = 32; // Value at which the board produces max acceleration torque
uint8_t boardMax = 230; // Value at which the board produces max brake torque - TODO GUESS BETTER!

// Speed estimation model for less lag in cruise and beginner mode:
float skateboardSpeed = 0.0; // value between 0 and 1 representing standstill to full throttle
float lastSkateboardSpeed = 0.0; // value from last iteration of velocity model
float skateboardSpeedAccelDampening = 0.004; // Factor describing acceleration behavior
float skateboardSpeedCoastDecelIterator = 0.00017; // When coasting, using this simple integrator to reduce velocity
float skateboardSpeedBrakeDecelFactor = 0.001; // When braking, using this factor to reduce velocity

void setup() {
  // initialize serial communications at 115200 bps:
  Serial.begin(115200);
  lastTimeSinceStatusReport = millis();
  
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);  // Debug Pin to measure total time of pairing
  pinMode(CS_PIN, OUTPUT);  // CS is the chip send trigger of the NRF
  pinMode(CSN_PIN, OUTPUT); // CSN is chip select of the NRF
  pinMode(IRQ_PIN, INPUT);  // IRQ is a signal from the NRF and is active low
  digitalWrite(CS_PIN, LOW); // CS is a trigger to send data and is active high
  digitalWrite(CSN_PIN, HIGH); // CS is a trigger to send data and is active high

  pinMode(battLED1_PIN, OUTPUT);  // Battery status < 25%
  pinMode(battLED2_PIN, OUTPUT);  // Battery status < 50%
  pinMode(battLED3_PIN, OUTPUT);  // Battery status < 75%
  pinMode(battLED4_PIN, OUTPUT);  // Battery status < 100%
  pinMode(lostLED_PIN, OUTPUT);  // Connection to board lost

  pinMode(mode_PIN, INPUT_PULLUP);  // Switch for changing modes
  pinMode(pairing_PIN, INPUT_PULLUP);  // Button for repairing the remote to the board


  digitalWrite(lostLED_PIN, HIGH); // Connection to board lost - start search

  // when pin IRQ_PIN goes low, call the IRQ function -- This is actually unused.
  // The NRF chip gets polled all the time at the moment
  //attachInterrupt(digitalPinToInterrupt(IRQ_PIN), receivedIRQ, FALLING);

  // initialize SPI:
  SPI.begin();
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Initialize state machines
  freqCounter = 0;
  mainState = initRemote;
  driveState = sendMessage;
  pairingState = initPairing_1;
  performanceState = sport;
  velocityState = coast;
  set_batteryState(0xFF);
  messageType = false;

  init_remote(); // Read paired address from the EEPROM

  digitalWrite(3, LOW); // Set debug pin low

  tone(6, 440, 100);
  delay(100);
  tone(6, 600, 100);
  delay(100);
  tone(6, 800, 100);
  delay(100);
  tone(6, 1000, 100);
  delay(100);

  /* Throttle Calibration
     Check if the throttle was pulled back (value larger) and is too far away from a possible center to be considered center.
     User wants to recalibrate the throttle .
     1. Store max ADC value
     2. Refresh min value as long as ADC shrinks (user pushes throttle forward
     3. When min hit, user will go back to center. After hysteresis threshold was past, store min value
     4. Wait until throttle was still for 2 seconds
     5. Store center
     6. Save all values to EEPROM
  */

  //if ((analogRead(A7) > 600))// && (digitalRead(mode_PIN) == LOW))
  if ((analogRead(A7) > 600) && (digitalRead(mode_PIN) == LOW))
  {
    Serial.println("Starting Throttle Calibration");
    throttleCalibration = findMAX;
    throttleCalibrated = false;
  }
  else // read old data from EEPROM
  {
    Serial.print("Reading throttle values from EEPROM...");
    // store all values to EEPROM
    maxThrottleADC = EEPROM.read(10);
    maxThrottleADC |= EEPROM.read(11) << 8;
    minThrottleADC = EEPROM.read(12);
    minThrottleADC |= EEPROM.read(13) << 8;
    centerThrottleADC = EEPROM.read(14);
    centerThrottleADC |= EEPROM.read(15) << 8;
    Serial.println("...[DONE]");
    Serial.print("maxThrottleADC: ");
    Serial.println(maxThrottleADC);
    Serial.print("minThrottleADC: ");
    Serial.println(minThrottleADC);
    Serial.print("centerThrottleADC: ");
    Serial.println(centerThrottleADC);
  }

  while (throttleCalibrated == false)
  {
    switch (throttleCalibration)
    {
      case findMAX:
        /*
            Update max value until throttle is moved forward again (getting smaller).
            If value < max - hysteresis goto findMIN
        */
        if (analogRead(A7) > maxThrottleADC)
          maxThrottleADC = analogRead(A7);

        if (analogRead(A7) < maxThrottleADC - throttleHysteresis)
        {
          Serial.print("Found MAX Value at: ");
          Serial.println(maxThrottleADC);
          tone(6, 200, 100);
          delay(100);
          throttleCalibration = findMIN;
        }

        break;
      case findMIN:
        /*
           Update min value until throttle is moved all the way forward.
           If value > min + hysteresis goto findCenter
        */
        if (analogRead(A7) < minThrottleADC)
          minThrottleADC = analogRead(A7);

        if (analogRead(A7) > minThrottleADC + throttleHysteresis)
        {
          Serial.print("Found MIN Value at: ");
          Serial.println(minThrottleADC);
          tone(6, 1000, 100);
          delay(100);
          throttleCalibration = findCenter;
        }

        break;
      case findCenter:
        /*
           Center should be roughly in the middle but it can have an offset that needs to be taken into account.
           Release throttle, observe value until still within tolerance (4 ADC values).
           Wait for 1 seconds and set center if still the same (within 4 ADCs)
        */
        uint32_t centerSettledFor = millis();
        uint16_t lastThrottleValue = analogRead(A7);
        bool bSettled = false;
        while (millis() - centerSettledFor < throttleCenterSettleTime)
        {
          /*
             if lastValue > currentValue - centerNoise && lastValue < currentValue + centerNoise
          */
          uint16_t currentValue = analogRead(A7);
          if ((lastThrottleValue > currentValue - throttleCenterNoise) && (lastThrottleValue < currentValue + throttleCenterNoise))
          {
            if (bSettled == false)
            {
              centerSettledFor = millis();
              bSettled = true;
            }
            centerThrottleADC = currentValue;
          }
          else
          {
            bSettled = false;
            centerSettledFor = millis();
          }

          lastThrottleValue = analogRead(A7);
        }
        // finished finding center

        Serial.print("Found center Value at: ");
        Serial.println(centerThrottleADC);
        tone(6, 440, 100);
        delay(100);
        tone(6, 440, 100);
        delay(100);

        Serial.print("Writing values to EEPROM...");
        // store all values to EEPROM
        EEPROM.write(10, (uint8_t)maxThrottleADC);
        EEPROM.write(11, (uint8_t)(maxThrottleADC >> 8));
        EEPROM.write(12, (uint8_t)minThrottleADC);
        EEPROM.write(13, (uint8_t)(minThrottleADC >> 8));
        EEPROM.write(14, (uint8_t)centerThrottleADC);
        EEPROM.write(15, (uint8_t)(centerThrottleADC >> 8));
        Serial.println("...[DONE]");

        throttleCalibrated = true;

        break;
    } // end switch case
  } // end throttleCalibration While

  /*while (1)
    {
    //Serial.println(analogRead(A0));
    //Serial.println(analogRead(A1));
    //Serial.println(analogRead(A2));
    //Serial.println(analogRead(A3));
    //Serial.println(analogRead(A4));
    Serial.print(5.0f/1024.0f*(float)analogRead(A5));
    //Serial.println(analogRead(A6));
    //Serial.println(analogRead(A7));
    Serial.println("V ");
    delay(100);
    }*/
}

void loop() {
  if (mainState != pairingRemote)
    delay(5); // This is supposed to be the only delay in the loop that is needed permanently. Gives both board and remote time to process data within their respective NRF24L01

  //Serial.println("Main Loop");

  // Check remote battery level:
  // ELSE is not required as one spike into low battery warning is enough to stop acceleration from functioning until after next restart. It can only be set once.
  
  remoteBatteryVoltage = 5.0f / 1024.0f * (float)analogRead(A5);
  if (remoteBatteryVoltage < 2.1f)
  {
    remoteBatteryLevelCritical = true; // Once set, it can't be reset until the remote is power cycled!
    tone(6, 2000, 100);
    //delay(100);
    //tone(6, 1000, 100);
    //delay(100);
    set_remoteBatteryAlarm();
    Serial.println("Remote control battery level critical!!");    
    //delay(10);
  }

  // Report battery voltage every 5 seconds
  if (millis() - lastTimeSinceStatusReport > 5000)
  {
    lastTimeSinceStatusReport = millis();
    Serial.print("Battery Voltage: ");
    Serial.println(remoteBatteryVoltage);
  }

  // Check for input:
  // read the state of the switch into a local variable:
  int reading = digitalRead(pairing_PIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastPairingButtonState) {
    // reset the debouncing timer
    lastPairingButtonDebounceTime = millis();
  }

  if ((millis() - lastPairingButtonDebounceTime) > pairingButtonDebounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != pairingButtonState) {
      pairingButtonState = reading;

      // Go straight into pairing state if the new button state is LOW
      if (pairingButtonState == LOW)
      {
        freqCounter = 0;
        mainState = pairingRemote;
        driveState = sendMessage;
        pairingState = initPairing_1;
        set_batteryState(0xFF);
      }
    }
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastPairingButtonState = reading;

 // Check for input:
  // read the state of the switch into a local variable:
  reading = digitalRead(mode_PIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastModeButtonState) {
    // reset the debouncing timer
    lastModeButtonDebounceTime = millis();
  }

  if ((millis() - lastModeButtonDebounceTime) > modeButtonDebounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != modeButtonState) {
      if (reading == LOW)
      {
        //Serial.println("Mode key was pressed");
      }
      if (reading == HIGH)
      {
        
        digitalWrite(battLED1_PIN, LOW); // Battery status < 100%
        digitalWrite(battLED2_PIN, LOW); // Battery status < 75%
        digitalWrite(battLED3_PIN, LOW); // Battery status < 50%
        digitalWrite(battLED4_PIN, LOW); // Battery status < 25%
        switch (performanceState)
        {
          case sport:
            Serial.println("Switching to cruise mode!");
            performanceState = cruiser;
            tone(6, 400, 250);
            digitalWrite(battLED2_PIN, LOW); // Battery status < 75%
            digitalWrite(battLED3_PIN, LOW); // Battery status < 50%
            delay(100);
            tone(6, 600, 250);
            digitalWrite(battLED2_PIN, HIGH); // Battery status < 75%
            digitalWrite(battLED3_PIN, HIGH); // Battery status < 50%
            delay(100);
            tone(6, 600, 250);
            digitalWrite(battLED2_PIN, LOW); // Battery status < 75%
            digitalWrite(battLED3_PIN, LOW); // Battery status < 50%
            delay(100);
            tone(6, 400, 250);
            digitalWrite(battLED2_PIN, HIGH); // Battery status < 75%
            digitalWrite(battLED3_PIN, HIGH); // Battery status < 50%
            delay(100);
            break;
          case cruiser:
            Serial.println("Switching to beginner mode!");
            performanceState = beginner;
            tone(6, 1000, 100);
            digitalWrite(battLED1_PIN, HIGH); // Battery status < 100%
            delay(100);
            tone(6, 800, 100);
            digitalWrite(battLED2_PIN, HIGH); // Battery status < 75%
            digitalWrite(battLED1_PIN, LOW); // Battery status < 100%
            delay(100);
            tone(6, 600, 100);
            digitalWrite(battLED3_PIN, HIGH); // Battery status < 50%
            digitalWrite(battLED2_PIN, LOW); // Battery status < 75%
            delay(100);
            tone(6, 440, 100);
            digitalWrite(battLED4_PIN, HIGH); // Battery status < 25%
            digitalWrite(battLED3_PIN, LOW); // Battery status < 50%
            delay(100);
            break;
          case beginner:
            Serial.println("Switching to sport mode!");
            performanceState = sport;
            tone(6, 440, 100);
            digitalWrite(battLED4_PIN, HIGH); // Battery status < 25%
            delay(100);
            tone(6, 600, 100);
            digitalWrite(battLED3_PIN, HIGH); // Battery status < 50%
            digitalWrite(battLED4_PIN, LOW); // Battery status < 25%
            delay(100);
            tone(6, 800, 100);
            digitalWrite(battLED2_PIN, HIGH); // Battery status < 75%
            digitalWrite(battLED3_PIN, LOW); // Battery status < 50%
            delay(100);
            tone(6, 1000, 100);
            digitalWrite(battLED1_PIN, HIGH); // Battery status < 100%
            digitalWrite(battLED2_PIN, LOW); // Battery status < 75%
            delay(100);
            break;
        }
        digitalWrite(battLED1_PIN, LOW); // Battery status < 100%
        digitalWrite(battLED2_PIN, LOW); // Battery status < 75%
        digitalWrite(battLED3_PIN, LOW); // Battery status < 50%
        digitalWrite(battLED4_PIN, LOW); // Battery status < 25%
      }
      modeButtonState = reading;
    }
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastModeButtonState = reading;

  /* Main state machine for pairing, init, drive, connection lost
     If the remote was never paired, it enforces pairing.
     If it was paired before, it will attempt to connect using the stored
     addresses in the flash eeprom.
     If the connection was successful, it will go into drive.
     If not, it will go into connection lost, searching through the frequencies
     the board uses.
     If connection lost or no motion is detected for timeout time,
     the controller will beep to get the user to turn it off.
  */
  // Return value of each SPI transaction. Every first transaction will return the status register
  uint8_t status = 0x00;   // used for general storage of the STATUS
  uint8_t status1 = 0x00;  // used when two STATUS values need to be compared
  uint8_t status2 = 0x00;  // used when two STATUS values need to be compared


  /******************************************************************************************************************/
  /*******************   READ AND PROCESS ALL USER INPUT    *********************************************************/
  /******************************************************************************************************************/
  /*
     1. Read ADC
     2. Scale positive and negative values to float -1 and +1 with 0 in the center no matter if input values are asymmetrical
     3. Calculate EXPO
     4. Filter output for positive values and negative values separately
         - Letting go of the throttle, smoothly transitions to 0 torque
         - However, transitioning from throttle to brake and vice versa is instantaneous
     5. Rescale to values of motor controller
        Remove DeadZone (if not included in preveous step)
     6. Send value to board
  */

  // 1. Read ACD
  uint16_t throttleValue = analogRead(A7);
  //Serial.println(throttleValue);
  // 2. calculate a value between -1 and +1
  float throttle = rescaleADCThrottleValue(throttleValue, minThrottleADC, maxThrottleADC, centerThrottleADC);
  //Serial.println(throttle);
  // 3. Calculate Expo
  throttle = exponentialCurve(throttle, expoFactor); // currently set to 1.0 which doesn't do anything. Turns out the remote already feels good without expo.
  //Serial.println(throttle);
  //Serial.println(lastDampedValue);
  
  // 4. Filter
  // TODO
  // 5. Modes
  switch(performanceState)
  {
    case sport:
      expoFactor = 1.0;
      break;
      
    case cruiser:
      if (throttle < -0.04) // accelerate
      {
        // If we braked and want to accellerate, it must be instant:
        if (lastDampedValue > 0)
          dampingFactorDriveModeModifier = 0.5; // Quick transition to braking
        else
          dampingFactorDriveModeModifier = 10; // smooth acceleration
        expoFactor = 1;
        throttle = throttle * 0.62;
        if (velocityState == coast || velocityState == brake) // To reduce acceleration latency, we use the velocity model data to set the new current throttle value
        {
          //lastDampedValue = lastSkateboardSpeed;
          velocityState = accel;
        }
        if (throttle > lastSkateboardSpeed)
        {
          lastDampedValue = throttle;
          if (throttle < -0.62)
            throttle = -0.62;
        }
        throttle = pt1_damper (throttle, dampingFactor / dampingFactorDriveModeModifier, integralPart, lastDampedValue, lastIntegralPart);
      }
      else if (throttle > 0.04) // brake
      {
        // If we accellerated and want to brake, it must be instant:
        if (lastDampedValue < 0)
          dampingFactorDriveModeModifier = 0.5; // Quick transition to acceleration 
        else
          dampingFactorDriveModeModifier = 2.5; // Smooth braking
        expoFactor = 0.8;
        throttle = throttle * 1.0;
        throttle = pt1_damper (throttle, dampingFactor / dampingFactorDriveModeModifier, integralPart, lastDampedValue, lastIntegralPart);
        velocityState = brake;
      }
      else
      {
        dampingFactorDriveModeModifier = 2; // Smooth throttle release
        expoFactor = 1;
        throttle = throttle * 1.0;
        throttle = pt1_damper (throttle, dampingFactor / dampingFactorDriveModeModifier, integralPart, lastDampedValue, lastIntegralPart);
        velocityState = coast;
      }
      break;
      
    case beginner:
      if (throttle < -0.04) // accelerate
      {
        // If we braked and want to accellerate, it must be instant:
        if (lastDampedValue > 0)
          dampingFactorDriveModeModifier = 0.5; // Quick transition to acceleration 
        else
          dampingFactorDriveModeModifier = 3; // Smooth braking
        expoFactor = 1;
        throttle = throttle * 0.35;
        if (velocityState == coast || velocityState == brake) // To reduce acceleration latency, we use the velocity model data to set the new current throttle value
        {
          //lastDampedValue = lastSkateboardSpeed;
          velocityState = accel;
        }
        if (throttle > lastSkateboardSpeed)
        {
          lastDampedValue = throttle;
          if (throttle < -0.35)
            throttle = -0.35;
        }
        throttle = pt1_damper (throttle, dampingFactor / dampingFactorDriveModeModifier, integralPart, lastDampedValue, lastIntegralPart);
      }
      else if (throttle > 0.04) // brake
      {
        // If we accellerated and want to brake, it must be instant:
        if (lastDampedValue < 0)
          dampingFactorDriveModeModifier = 0.2; // Quick transition to braking 
        else
          dampingFactorDriveModeModifier = 3; // Smooth braking
        expoFactor = 1;
        throttle = throttle * 0.65;
        throttle = pt1_damper (throttle, dampingFactor / dampingFactorDriveModeModifier, integralPart, lastDampedValue, lastIntegralPart);
        velocityState = brake;
      }
      else
      {
        dampingFactorDriveModeModifier = 10; // Very smooth throttle release
        expoFactor = 1;
        throttle = throttle * 1.0;
        throttle = pt1_damper (throttle, dampingFactor / dampingFactorDriveModeModifier, integralPart, lastDampedValue, lastIntegralPart);
        velocityState = coast;
      }
      break;
  }

// Estimating the skareboard speed
  if (skateboardSpeed >= throttle)
  {
    skateboardSpeed = pt1_damper(throttle, skateboardSpeedAccelDampening, lastSkateboardSpeed);
  }
  else if(throttle > 0.04)
  {
    skateboardSpeed += skateboardSpeedBrakeDecelFactor * throttle;
    lastSkateboardSpeed = skateboardSpeed;
  }
  else
  {
    skateboardSpeed += skateboardSpeedCoastDecelIterator;
    lastSkateboardSpeed = skateboardSpeed;
  }

  if (skateboardSpeed < -1)
    skateboardSpeed = -1;
  else if (skateboardSpeed > 0)
    skateboardSpeed = 0;
    
// If desired, print the actual estimated miles per hour to serial
  //Serial.print(22.0 * -skateboardSpeed); Serial.println(" MPH");
  //Serial.print(throttle);
  //Serial.print("   ");
  //Serial.println(lastSkateboardSpeed);
  
  // 6. Rescale
  //    Remove Deadzone
  throttleValue = deadzoneCompensationAndRescale(throttle,
                  posDeadZone,
                  negDeadZone,
                  boardMin,
                  boardMax,
                  boardDeadzoneMin,
                  boardDeadzoneMax,
                  boardCenter);

  //Serial.print(22.0 * -skateboardSpeed); Serial.println(" MPH");
  // Todo: If the battery is empty, we don't just want to cut power as that might be extremely dangerous!
  //       In that case, we want to ramp down the maximum available power within 10 seconds or so until idle is reached.
  // 6. Send value to board

  // Check for critical battery voltage
  if (remoteBatteryLevelCritical)
    quatroMessage_1[1] = (throttleValue >= boardCenter) ? throttleValue : 128; // If throttlevalue < boardCenter (braking) do it. Otherwise, no acceleration
  else
    quatroMessage_1[1] = throttleValue; // Remote Battery is fine, just throttle through
  //Serial.println(quatroMessage_1[1]);


  /******************************************************************************************************************/
  /*******************   CALCULATE ACCELERATION CURVES    ***********************************************************/
  /******************************************************************************************************************/

  /*
     All of the following should run in a timed / scheduled environment. 100hz or 25hz should be sufficient.
     Defined timing is predictable and can easily be adjusted and tuned. Specifically, when the throttle value
     doesn't change, less communication needs to happen and hence, energy can be saved.

     Configurable values:
       Deadzone for acceleration AND brakes separately
       Expo value of acceleration curve AND brake curve separately
       Sliding hysteresis window size and rate of change
       Acceleration curve for acceleratio AND brakes separately
     I also had the idea of a faster response when the user input varies greatly from the current value and rate of change.
     Panic brake is one of these items. If the user decides I NEED BRAKES NOW!! he should get brakes immediately.
     How can this be implemented properly without loosing predictability of the controller?
  */

  switch (mainState)
  {
    case pairingRemote:
      if (pairRemote())
      {
        break;
      }
      else
      {
        break;
      }
      break;

    /******************************************************************************************************************/
    /*******************   REMOTE OPERATION AND INITIALIZATION    *****************************************************/
    /******************************************************************************************************************/
    case initRemote:
      //Serial.println("  Init Remote");
      digitalWrite(lostLED_PIN, HIGH);

      if (freqCounter > 15) // Sanity check on the frequency counter
        freqCounter = 0 ;

      lostCounter = 0;  // Reset lostCounter because we are going to reinitialize the NRF chip

      // Set all registers according to SPI trace. Ignote STATUS register from board at this stage.
      status = write_ToAddress(W_REGISTER | CONFIG, 0x0E); // Enable CRC, 2 byte CRC, Power Up
      status = write_ToAddress(W_REGISTER | EN_AA, 0x01); // Enable Auto ACK for PIPE 0
      status = write_ToAddress(W_REGISTER | EN_RXADDR, 0x01); // Enable data PIPE 0
      status = write_ToAddress(W_REGISTER | SETUP_AW, 0x03); // Set RX/TX address length to 5 bytes
      status = write_ToAddress(W_REGISTER | SETUP_RETR, 0x22); // Set retransmit delay to 750 micro seconds and set 2 retransmits
      status = write_ToAddress(W_REGISTER | RF_SETUP, 0x0F); // set RF_Config to high power and 2 mBits
      status = write_ToAddress(W_REGISTER | DYNPD, 0x01); // Enable dynamic payload length on PIPE 0
      status = write_ToAddress(W_REGISTER | FEATURE, 0x06); // Enable dynamic payload, enable payload ACK
      status = write_ToAddress(W_REGISTER | RX_PW_P0, 0x03); // Set number of bytes in RX payload to 3 bytes
      status = write_ToAddress(W_REGISTER | RF_CH, frequencies[freqCounter]); // Set operating frequency to 2: 2.402 Ghz
      status = write_BytesToAddress(W_REGISTER | TX_ADDR, quatroAddress, 5); // Set TC address to board
      status = write_BytesToAddress(W_REGISTER | RX_ADDR_P0, quatroAddress, 5); // Set RX address from board

      mainState = driveRemote;
      driveState = sendMessage;
      pairingState = initPairing_1;
      break;

    case driveRemote:
      //Serial.println("  Drive Remote");
      digitalWrite(lostLED_PIN, LOW);
      // flush TX buffer
      // Clear out all pending IRQs
      // Send message 1 to board
      // Check if a message was received back from board
      // Send message 2 to board
      // Check if a message was received back from board
      // Repeat until connection lost (MAX_RTR was fired)

      switch (driveState)
      {
        case sendMessage:
          //Serial.println("    Send Message");

          status = exec_command(FLUSH_TX); // clean out the TX buffer - should return 0x0E
          status = exec_command(FLUSH_RX); // clean out the RX buffer because we only receive, after we send - should return 0x0E
          status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
          if (!messageType)
          {
            status = write_BytesToAddress(W_TX_PAYLOAD, quatroMessage_1, 5); // Send message 1 to the transmitter
            trigger_CS();
          }
          else
          {
            status = write_BytesToAddress(W_TX_PAYLOAD, quatroMessage_2, 5); // Send message 1 to the transmitter
            trigger_CS();
          }

          messageType = !messageType;
          driveState = checkStatus;
          break;

        case checkStatus:
          //Serial.println("    Check Status");
          status1 = exec_command(STATUS); // Read what's going on in the status - did we receive the message or timed out?
          status2 = exec_command(NOP); // Read a second time using the NOP command which should also return the same value

          if (status1 == status2) // both status byte reads are the same, therefore it didn't change in two subsequent reads
          {
            if (status1 & 0x60) // RX_DR as well as TX_DS are true: received data and sent data
            {
              //tone(6, 3000, 10);
              //Serial.println(status1, HEX);
              rtr_counter = 0; // Everything went well. Reset retry counter
              driveState = receiveMessage;
              break;
            }
            if (status1 & 0x40) // 0b0100 0000 = RX_DR is true, TX_DS is false, MAX_RTR is false
            {
              tone(6, 2000, 10);
              //Serial.println(status1, HEX);
              /*
                 Just read whatever is there and restart as normal.
                 Can't ever happen unless another remote sends data on the same channel with the same address.
                 Has no influence on remote and can be ignored.
              */
              driveState = receiveMessage;
            }
            if (status1 & 0x20) // 0b0010 0000 = TX_DS is true, RX_DR is false
            {
              tone(6, 1000, 10);
              //Serial.println(status1, HEX);
              /*
                 We enter this state when the message was sent, an ACK was received but there was no data received as ACK_PAYLOAD.
                 RX_DR is not set even though TX_DS is set!
                 This is a potentially dangerous state, as the TX buffer is empty and the "lost" message cannot be resent.
                 Also, since an ACK was received, the MAX_RTR flag is never set. The state machine would stop here.
                 If the payload was lost, we should to send the message again after we waited once more.
              */
              if (TX_DS_WasSetAlready_WaitFor_RX_DR == false)
              {
                TX_DS_WasSetAlready_WaitFor_RX_DR = true;
                // Data wasn't received, yet (no ack PAYLOAD received). Just wait a little more and check again until MAX_RTR or RX_DR is set.
                driveState = checkStatus; // This was flushTX before and that seems to make no sense at all
              }
              else
              {
                TX_DS_WasSetAlready_WaitFor_RX_DR = false;
                driveState = sendMessage; // We bail! Never received an ACK_PAYLOAD following the ACK. We need to send again.
              }
            }
            else if (status1 & 0x10) // MAX_RTR was set - this happens quite often!
            {
              //tone(6, 800, 20);
              //delay(20);
              //Serial.print("P     A PACKAGE COULD NOT BE TRANSMITTED TO THE BOARD: 0x");
              //Serial.println(status1, HEX);
              rtr_counter++;
              //Serial.println(rtr_counter);

              if (rtr_counter > 10)
              {
                // we have reached MAX_RTR maximum number of retries reached
                mainState = lostRemote;
              }
              else
              {
                status = read_FromAddress(R_REGISTER | FIFO_STATUS); // 0b0000 0010 -> RX_FIFO_FULL ! That seems to be the issue. Nothing can go in or out.
                //Serial.println(status, HEX);
                if (status & 0x10) // worst case TX_FIFO_EMPTY: The data was lost! Send message again immediately!
                {
                  mainState = driveRemote;
                  driveState = sendMessage;
                }
                else
                {
                  status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
                  trigger_CS(); // Send last message again!
                }
              }
            }
            else // Whatever else happens, go back to check status. Since we captured all other IRQ sources above, this is obsolete and only serves reliability
            {
              //tone(6, 200, 20);
              //delay(20);
              //Serial.print("P     EVERYTHING OTHER THAN 0x60 or 0x10: 0x");
              //Serial.println(status1, HEX);
              
              // Report status every 5 seconds
              if (millis() - lastTimeSinceStatusReport > 5000)
              {
                Serial.println("FATAL: Remote in critical state! Re-pair with skateboard, recalibrate throttle!");
              }

              driveState = checkStatus;
            }
          }
          break;

        case receiveMessage:
          //Serial.println("    Receive Message");
          status = read_BytesFromAddress(R_RX_PAYLOAD, quatroReturnMessage, 8); // Read message back and please don't ask me why it was 8 bytes in the remote since only one byte actually contains any data
          boardBatteryState = quatroReturnMessage[0] & 0x0F;

          // Only display the board battery state as long as it's still relevant. It's not relevant if the remote is empty (TODO: That needs to be discussed!)
          if (!remoteBatteryLevelCritical)
            set_batteryState(boardBatteryState);

          digitalWrite(lostLED_PIN, LOW); // Connection to the board was successfully established
          //digitalWrite(lostLED_PIN, (quatroReturnMessage[0] & 0x10) == true ? LOW : HIGH); // Connection to the board was successfully established

          lostCounter = 0; // We can safely reset the lostCounter here, as the connection was definitely established in between losts.

          /*
             This section makes sure, we always jump back to the last known good frequency every once in a while during a frequency search.
             This can significantly reduce the time it takes to regain the connection.
             One exception is, the last known frequency was slightly higher than the new actually good frequency. That might cause a longer search of about 1 second.
             However, this should only happen upon board restart because the board (server) is the only peer to ever change the channel. The client always follows.
          */
          if ((numHopsTaken != 0) && (lastKnownFrequency != freqCounter))
          {
            lastKnownFrequency = freqCounter;
            Serial.print("Good Freq: ");
            Serial.println(lastKnownFrequency);
          }
          else
          {
            freqCounter = lastKnownFrequency;
          }
          driveState = sendMessage;
          break;
      }
      break;

    case lostRemote:
      // Report status every 5 seconds
      if (millis() - lastTimeSinceStatusReport > 5000)
      {
        Serial.println("  Connection to skateboard lost. Searching known frequencies using current pairing key...");
      }
      digitalWrite(lostLED_PIN, HIGH);
//      tone(6, 500, 20);
      //delay(20);

      set_batteryState(0xFF);

      rtr_counter = 0;

      // Check if we should check the last known frequency again before continuing the search for the right channel
      if (numHopsTaken < numHopsBeforeCheckingLastKnown)
      {
        numHopsTaken++;
        freqCounter++;
      }

      if (freqCounter > 15)
      {
        /*
            This is the case where we searched all frequencies already and did not receive anything back from the board
            This is therfore pretty bad and we need to consider the possibility that the data and the ack was lost,
            or we are in an intermediate state where the data has left the TX register and a simple trigger on CS is unsufficient to regain connection.
        */
        freqCounter = 0;
        lostCounter++;  // Increment lostCounter as we have passed all frequencies twice and have found no connection

        if (lostCounter < 2)
        {
          //Serial.println("Lost It");
          mainState = driveRemote;
          driveState = sendMessage;
        }
        else
        {
          mainState = initRemote;
        }
      }
      else // This is the normal case. Hop to another frequency and trigger CS to try and send the data again
      {
        if (numHopsTaken >= numHopsBeforeCheckingLastKnown)
        {
          numHopsTaken = 0;
          freqCounter--; // reduce frequency counter by one, otherwise we will never hit all of them but skip instead
          //Serial.println(lastKnownFrequency);
          status = write_ToAddress(W_REGISTER | RF_CH, frequencies[lastKnownFrequency]); // Set operating frequency to the last known working
        }
        else
        {
          //Serial.println(freqCounter);
          status = write_ToAddress(W_REGISTER | RF_CH, frequencies[freqCounter]); // Set operating frequency to the next possible frequency
        }

        status = read_FromAddress(R_REGISTER | FIFO_STATUS);
        //Serial.println(status, HEX);
        if (status & 0x10) // worst case: The data was lost!
        {
          mainState = driveRemote;
          driveState = sendMessage;
        }
        else
        {
          status = write_ToAddress(W_REGISTER | STATUS, 0x70); // Clear out all pending IRQs - We will ignore all - should return 0x0E
          trigger_CS(); // Trigger sending the still existing message in the TX FIFO again. It will remain there until PRX confirmed receipt with an ACK
          mainState = driveRemote;
          driveState = checkStatus;
        }
      }

      break;

    case idleRemote:
      //Serial.println("  Idle Remote");
      digitalWrite(lostLED_PIN, LOW);
      break;

    case offRemote:
      //Serial.println("  Off Remote");
      digitalWrite(lostLED_PIN, LOW);
      break;
  }
  if (millis() - lastTimeSinceStatusReport > 5000)
  {
    lastTimeSinceStatusReport = millis();
  }
}


//IRQ routine that fires when data was sent, data was received, or max retries was reached
void receivedIRQ(void)
{
  // The IRQ pin is not actually used in this remote.
  // It's there, but we're polling STATUS all the time anyways so it makes no sense to bother with this.

  digitalWrite(lostLED_PIN, !digitalRead(lostLED_PIN));
  return;
}


