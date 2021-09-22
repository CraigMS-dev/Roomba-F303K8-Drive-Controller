#include <motorClass.h>
#include <Arduino.h>
#include "utilityFuncs.h"

//#define PURE_RPM


tachoWheel::tachoWheel(int pinA, int pinB, int pinA_offset, int pinB_offset){
    _PinA = pinA;
	_PinB = pinB;
    _PinAOffset = pinA_offset;
	_PinBOffset = pinB_offset;

	pinMode(_PinA, INPUT);      // sets pin A as input
	pinMode(_PinB, INPUT);      // sets pin B as input

	_ASet = digitalRead(_PinA);   // read the input pin
	_BSet = digitalRead(_PinB);   // read the input pin

	_Position = 0;
}

#ifdef ENCODER_TACHO
    void tachoWheel::encoderTickA(){}
    void tachoWheel::encoderTickB(){}
    void tachoWheel::encoderTick()
    {
        PeriodBetweenPulses   = micros() - LastTimeWeMeasured;
        LastTimeWeMeasured    = micros();

        if(PulseCounter >= AmountOfReadings){  // If counter for amount of readings reach the set limit:
        
            PeriodAverage  = PeriodSum / AmountOfReadings;
            PulseCounter   = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
            PeriodSum      = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

            int RemapedAmountOfReadingsL = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
                                        // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
                                        // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
                                        // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
                                        // 4th and 5th values are the amount of readings range.
            RemapedAmountOfReadingsL = constrain(RemapedAmountOfReadingsL, 1, 5);  // Constrain the value so it doesn't go below or above the limits.
            AmountOfReadings = RemapedAmountOfReadingsL;  // Set amount of readings as the remaped value.
        
        }else{
            currentState         = (((GPIOA->IDR >> _PinAOffset) & 1) << 1) | ((GPIOA->IDR >> _PinBOffset) & 1);
            increment            = QEM[lastState][currentState];
            
            if(increment > 0){
                direction = 1;
            }else if(increment < 0){
                direction = -1;
            }else{
                direction = 0;
            }

            PulseCounter++;  // Increase the counter for amount of readings by 1.
            wheelSpeedDistanceL += increment;
            
            lastState = currentState;

            fired = 1;

            PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
            lastState = currentState;
        }//*/
    }
    /*void tachoWheel::encoderTickB()
    {
        _PinA = ((GPIOA->IDR >> _PinAOffset) & 1);
        _PinB = ((GPIOA->IDR >> _PinBOffset) & 1);
        currentState = (_PinA << 1) | _PinB;

        increment = QEM[currentState][lastState];

        PulseCounter += increment;
        wheelSpeedDistanceL += increment;

        //_ASet = _PinA == HIGH; // Check encoder sensor A // Access the Digital Input Register directly - MUCH faster than digitalRead()
        //increment = (_ASet == _BSet) ? +1 : -1;
        //_Position += increment; // Adjust counter + if A leads B

        

        lastState = currentState;
    }*/

    void tachoWheel::sendInfo()
    {
        unsigned long milliSecs = millis();

        //LNew = ((GPIOA->IDR >> _PinAOffset) & 1) * 2 + ((GPIOA->IDR >> _PinBOffset) & 1);
        //increment = QEM[LOld * 4 + LNew];

        calcVelocity();
        Serial.print(average);
        //Serial.println();
    }

    void tachoWheel::calcVelocity(){
        // The following is going to store the two values that might change in the middle of the cycle.
        // We are going to do math and functions with those values and they can create glitches if they change in the
        // middle of the cycle.
        LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
        CurrentMicros = micros();  // Store the micros() in a variable.

        // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
        // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
        // LastTimeCycleMeasure I set it as the CurrentMicros.
        // The need of fixing this is that we later use this information to see if pulses stopped.
        if(CurrentMicros < LastTimeCycleMeasure){
        LastTimeCycleMeasure = CurrentMicros;
        }

        // Calculate the frequency:
        FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.

        // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
        if(PeriodBetweenPulses > ZeroTimeout          - ZeroDebouncingExtra || 
        CurrentMicros       - LastTimeCycleMeasure > ZeroTimeout         - ZeroDebouncingExtra){
            // If the pulses are too far apart that we reached the timeout for zero:
            FrequencyRaw = 0;  // Set frequency as 0.
            ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.

        } else {
            ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
        }

        FrequencyReal = FrequencyRaw / 10000;  // Get frequency without decimals.
                                        // This is not used to calculate RPM but we remove the decimals just in case
                                        // you want to print it.

        // Calculate the RPM:
        RPM = FrequencyRaw / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                        // 60 seconds to get minutes.
        RPM = RPM / 10000;  // Remove the decimals.

        // Smoothing RPM:
        total = total - readings[readIndex];  // Advance to the next position in the array.
        readings[readIndex] = RPM;            // Takes the value that we are going to smooth.
        total = total + readings[readIndex];  // Add the reading to the total.
        readIndex = readIndex + 1;            // Advance to the next position in the array.

        if(readIndex >= numReadings){ // If we're at the end of the array:
        readIndex = 0;  // Reset array index.
        }

        // Calculate the average:
        average = total / numReadings;  // The average value it's the smoothed result.
    }
#endif

#ifdef ENCODER_TACHO_OLD
    void tachoWheel::encoderTickA(){
        PeriodBetweenPulses   = micros() - LastTimeWeMeasured;
        LastTimeWeMeasured    = micros();

        

        //increment = (_ASet != _BSet) ? +1 : -1;
        //_Position += increment; // Adjust counter + if A leads B

        fired = 1;
        
        if(PulseCounter >= AmountOfReadings){  // If counter for amount of readings reach the set limit:
        
            PeriodAverage  = PeriodSum / AmountOfReadings;
            PulseCounter   = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
            PeriodSum      = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

            int RemapedAmountOfReadingsL = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
                                        // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
                                        // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
                                        // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
                                        // 4th and 5th values are the amount of readings range.
            RemapedAmountOfReadingsL = constrain(RemapedAmountOfReadingsL, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
            AmountOfReadings = RemapedAmountOfReadingsL;  // Set amount of readings as the remaped value.
        
        }else{
            LOld = LNew;
            
            //_ASet = ((GPIOA->IDR >> _PinAOffset) & 1) == HIGH; // Check encoder sensor A // Access the Digital Input Register directly - MUCH faster than digitalRead()
            //increment = (_ASet != _BSet) ? +1 : -1;
            //_Position += increment; // Adjust counter + if A leads B

            LNew = ((GPIOA->IDR >> _PinAOffset) & 1) * 2 + ((GPIOA->IDR >> _PinBOffset) & 1);
            increment = QEM[LOld * 4 + LNew];

            PulseCounter += increment;
            wheelSpeedDistanceL += increment;

            if(increment > 0){
                direction = 1;
            }else if(increment < 0){
                direction = -1;
            }else{
                direction = 0;
            }
            
            //PulseCounter++;  // Increase the counter for amount of readings by 1.
            PeriodSum += PeriodBetweenPulses;  // Add the periods so later we can average.
        }//*/
    }

    void tachoWheel::encoderTickB(){
        //time_between_pulses   = micros() - LastTimeWeMeasured;
        //LastTimeWeMeasured    = micros();
        
        //increment = (_ASet == _BSet) ? +1 : -1;
        //_Position += increment; // Adjust counter + if A leads B
        
        /*fired = 1;
        
        if(PulseCounter >= AmountOfReadings){  // If counter for amount of readings reach the set limit:
        
            PeriodAverage  = PeriodSum / AmountOfReadings;
            PulseCounter   = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
            PeriodSum      = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

            int RemapedAmountOfReadingsL = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
                                        // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
                                        // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
                                        // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
                                        // 4th and 5th values are the amount of readings range.
            RemapedAmountOfReadingsL = constrain(RemapedAmountOfReadingsL, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
            AmountOfReadings = RemapedAmountOfReadingsL;  // Set amount of readings as the remaped value.
        
        }else{
            LOld = LNew;
            
            //_ASet = ((GPIOA->IDR >> _PinBOffset) & 1) == HIGH; // Check encoder sensor A // Access the Digital Input Register directly - MUCH faster than digitalRead()
            //increment = (_ASet == _BSet) ? +1 : -1;
            //_Position += increment; // Adjust counter + if A leads B
            LNew = ((GPIOA->IDR >> 0) & 1) * 2 + ((GPIOA->IDR >> 1) & 1);
            increment = QEM[LOld * 4 + LNew];

            if(increment > 0){
                direction = 1;
            }else if(increment < 0){
                direction = -1;
            }else{
                direction = 0;
            }
            
            PulseCounter += increment;
            wheelSpeedDistanceL += increment;

            //PulseCounter++;  // Increase the counter for amount of readings by 1.
            PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
        }*/
    }
    void tachoWheel::sendInfo()
    {
        bool a = ((GPIOA->IDR >> _PinAOffset) & 1);
        bool b = ((GPIOA->IDR >> _PinBOffset) & 1);
        unsigned long milliSecs = millis();

        //LNew = ((GPIOA->IDR >> _PinAOffset) & 1) * 2 + ((GPIOA->IDR >> _PinBOffset) & 1);
        //increment = QEM[LOld * 4 + LNew];

        //calcVelocity();
        //Serial.print(increment);
        //Serial.print("\t");
        //Serial.print(wheelSpeedDistanceL);
        //Serial.print("\t");
        //Serial.print(direction);
        //Serial.print("\t");
        //Serial.print(average);
        //Serial.print("\t");
        Serial.println();
    }
    void tachoWheel::calcVelocity(){
        // The following is going to store the two values that might change in the middle of the cycle.
        // We are going to do math and functions with those values and they can create glitches if they change in the
        // middle of the cycle.
        LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
        CurrentMicros = micros();  // Store the micros() in a variable.

        // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
        // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
        // LastTimeCycleMeasure I set it as the CurrentMicros.
        // The need of fixing this is that we later use this information to see if pulses stopped.
        if(CurrentMicros < LastTimeCycleMeasure){
        LastTimeCycleMeasure = CurrentMicros;
        }

        // Calculate the frequency:
        FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.

        // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
        if(PeriodBetweenPulses > ZeroTimeout          - ZeroDebouncingExtra || 
        CurrentMicros       - LastTimeCycleMeasure > ZeroTimeout         - ZeroDebouncingExtra){
            // If the pulses are too far apart that we reached the timeout for zero:
            FrequencyRaw = 0;  // Set frequency as 0.
            ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.

        } else {
            ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
        }

        FrequencyReal = FrequencyRaw / 10000;  // Get frequency without decimals.
                                        // This is not used to calculate RPM but we remove the decimals just in case
                                        // you want to print it.

        // Calculate the RPM:
        RPM = FrequencyRaw / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                        // 60 seconds to get minutes.
        RPM = RPM / 10000;  // Remove the decimals.

        // Smoothing RPM:
        total = total - readings[readIndex];  // Advance to the next position in the array.
        readings[readIndex] = RPM;            // Takes the value that we are going to smooth.
        total = total + readings[readIndex];  // Add the reading to the total.
        readIndex = readIndex + 1;            // Advance to the next position in the array.

        if(readIndex >= numReadings){ // If we're at the end of the array:
        readIndex = 0;  // Reset array index.
        }

        // Calculate the average:
        average = total / numReadings;  // The average value it's the smoothed result.
    }

#endif

#ifdef PURE_RPM
    void tachoWheel::encoderTick(){
        //uint8_t p1val = ((GPIOA->IDR >> 0) & 1);
		//uint8_t p2val = ((GPIOA->IDR >> 1) & 1);
    }
    
void tachoWheel::encoderTickLeft(){
    PeriodBetweenPulses   = micros() - LastTimeWeMeasured;
    LastTimeWeMeasured    = micros();

    fired = 1;
    
    if(PulseCounter >= AmountOfReadings){  // If counter for amount of readings reach the set limit:
    
        PeriodAverage  = PeriodSum / AmountOfReadings;
        PulseCounter   = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
        PeriodSum      = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

        int RemapedAmountOfReadingsL = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
                                    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
                                    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
                                    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
                                    // 4th and 5th values are the amount of readings range.
        RemapedAmountOfReadingsL = constrain(RemapedAmountOfReadingsL, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
        AmountOfReadings = RemapedAmountOfReadingsL;  // Set amount of readings as the remaped value.
    
    }else{
        LOld = LNew;
        // Access the Digital Input Register directly - MUCH faster than digitalRead()
        
        // Convert binary input to decimal value 
        // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
        LNew = ((GPIOA->IDR >> 0) & 1) * 2 + ((GPIOA->IDR >> 1) & 1);
        increment = QEM[LOld * 4 + LNew];
        if(increment > 0){
            direction = 1;
        }else if(increment < 0){
            direction = -1;
        }else{
            direction = 0;
        }
        
        PulseCounter += increment;
        wheelSpeedDistanceL += increment;

        //PulseCounter++;  // Increase the counter for amount of readings by 1.
        PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
    }
}

void tachoWheel::encoderTick(){
    PeriodBetweenPulses   = micros() - LastTimeWeMeasured;
    LastTimeWeMeasured    = micros();

    if(PulseCounter >= AmountOfReadings){  // If counter for amount of readings reach the set limit:
    
        PeriodAverage  = PeriodSum / AmountOfReadings;
        PulseCounter   = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
        PeriodSum      = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

        int RemapedAmountOfReadingsL = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
                                    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
                                    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
                                    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
                                    // 4th and 5th values are the amount of readings range.
        RemapedAmountOfReadingsL = constrain(RemapedAmountOfReadingsL, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
        AmountOfReadings = RemapedAmountOfReadingsL;  // Set amount of readings as the remaped value.
    
    }else{
        PulseCounter++;  // Increase the counter for amount of readings by 1.
        PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
    }
}

void tachoWheel::calcVelocity(){
    // The following is going to store the two values that might change in the middle of the cycle.
    // We are going to do math and functions with those values and they can create glitches if they change in the
    // middle of the cycle.
    LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
    CurrentMicros = micros();  // Store the micros() in a variable.

    // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
    // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
    // LastTimeCycleMeasure I set it as the CurrentMicros.
    // The need of fixing this is that we later use this information to see if pulses stopped.
    if(CurrentMicros < LastTimeCycleMeasure){
    LastTimeCycleMeasure = CurrentMicros;
    }

    // Calculate the frequency:
    FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.

    // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
    if(PeriodBetweenPulses > ZeroTimeout          - ZeroDebouncingExtra || 
    CurrentMicros       - LastTimeCycleMeasure > ZeroTimeout         - ZeroDebouncingExtra){
        // If the pulses are too far apart that we reached the timeout for zero:
        FrequencyRaw = 0;  // Set frequency as 0.
        ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.

    } else {
        ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
    }

    FrequencyReal = FrequencyRaw / 10000;  // Get frequency without decimals.
                                    // This is not used to calculate RPM but we remove the decimals just in case
                                    // you want to print it.

    // Calculate the RPM:
    RPM = FrequencyRaw / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                    // 60 seconds to get minutes.
    RPM = RPM / 10000;  // Remove the decimals.

    // Smoothing RPM:
    total = total - readings[readIndex];  // Advance to the next position in the array.
    readings[readIndex] = RPM;            // Takes the value that we are going to smooth.
    total = total + readings[readIndex];  // Add the reading to the total.
    readIndex = readIndex + 1;            // Advance to the next position in the array.

    if(readIndex >= numReadings){ // If we're at the end of the array:
    readIndex = 0;  // Reset array index.
    }

    // Calculate the average:
    average = total / numReadings;  // The average value it's the smoothed result.
}

#endif