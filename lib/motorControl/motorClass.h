#include <Arduino.h>

#define ENCODER_TACHO

class tachoWheel{
    public:
        tachoWheel(int pinA, int pinB, int pinA_offset, int pinB_offset);
        
        /*************
        * ENCODER VARS
        **************/
        int _PinA, _PinB;
        int _PinAOffset, _PinBOffset;
	    bool _ASet, _BSet;
	    long _Position;
        unsigned char lastState    = 0;
        unsigned char currentState = (((GPIOA->IDR >> _PinAOffset) & 1) << 1) | ((GPIOA->IDR >> _PinBOffset) & 1);
        
        enum class Direction {
            STILL = 0,
            CLOCKWISE = 1,
            COUNTERCLOCKWISE = -1
        };

        /***********************
        * DIRECTION AND VELOCITY
        ************************/
        Direction getDirection();

        unsigned long   time_between_pulses;
        long int        getPosition(){return _Position;};
        unsigned long   getVelocity(){return average;}
        
        void            encoderTickA();
        void            encoderTickB();
        
        unsigned long   getRPM();
        void            calcVelocity();
        void            encoderTick();
        void            encoderTickLeft();
        void            encoderTickRight();

                    

        void            sendInfo();
        void            sendInfo2();
        

        const uint16_t          PulsesPerRevolution = 1633; // Encoder pulses for 1 full rotation (Quadrature)
        const unsigned long     ZeroTimeout = 100000;       // If the period between pulses is too high, or even if the pulses stopped, then we would get stuck showing the
                                                            // last value instead of a 0. Because of this we are going to set a limit for the maximum period allowed.
                                                            // If the period is above this value, the RPM will show as 0.
                                                            // The higher the set value, the longer lag/delay will have to sense that pulses stopped, but it will allow readings
                                                            // at very low RPM.
                                                            // Setting a low value is going to allow the detection of stop situations faster, but it will prevent having low RPM readings.
                                                            // The unit is in microseconds.
        const byte              numReadings = 10;            // Number of samples for smoothing. The higher, the more smoothing, but it's going to
                                                            // react slower to changes. 1 = no smoothing. Default: 2.
        volatile unsigned long  LastTimeWeMeasured;         // Time at the last pulse
        volatile unsigned long  PeriodBetweenPulses = ZeroTimeout+1000; // Stores the period between pulses in microseconds.
                                                                        // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
        volatile unsigned long  PeriodAverage = ZeroTimeout+1000;       // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.

        unsigned long   CurrentMicros = micros(); // Micros at the current cycle
        long   FrequencyRaw;             // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
        long   FrequencyReal;            // Frequency without decimals.
        unsigned long   PeriodSum;                // Stores the summation of all the periods to do the average.
        long   RPM;                      // Raw RPM without any processing.
        unsigned long   LastTimeCycleMeasure = LastTimeWeMeasured; // Stores the last time we measure a pulse in that cycle.
                                            // We need a variable with a value that is not going to be affected by the interrupt
                                            // because we are going to do math and functions that are going to mess up if the values
                                            // changes in the middle of the cycle.
        int    PulseCounter = 1;         // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.
        unsigned int    AmountOfReadings = 1;     // Stores the last time we measure a pulse in that cycle.
                                                    // We need a variable with a value that is not going to be affected by the interrupt
                                                    // because we are going to do math and functions that are going to mess up if the values
                                                    // changes in the middle of the cycle.
        unsigned int    ZeroDebouncingExtra;      // Stores the extra value added to the ZeroTimeout to debounce it.

        // Variables for smoothing tachometer:
        long readings[10];  // The input. // [numReadings]
        long readIndex;  // The index of the current reading.
        long total;  // The running total.
        long average;  // The RPM value after applying the smoothing.

        volatile long encoderPosL = 0;
        volatile long encoderPosR = 0;

        long wheelSpeedDistanceCurrent = 0; // RPM
        long wheelSpeedDistanceOld = 0; // RPM
        long wheelSpeedDistanceDif = 0; // RPM
        float lastKnownPosL = 0;
        float lastKnownPosR = 0;

        bool fired = 0;
        /*int QEM [16] = {0,-1, 1, 2,
                        1, 0, 2,-1,
                       -1, 2, 0, 1,
                        2, 1,-1, 0}; */
        int QEM[4][4] = {{ 0,-1, 1, 2 },
                         { 1, 0, 2,-1 },
                         {-1, 2, 0, 1 },
                         { 2, 1,-1, 0 }}; 
        int LOld = 0; // Previous encoder value left motor
        int ROld = 0; // Previous encoder value right motor
        int LNew = 0; // New encoder value left motor
        int RNew = 0; // New encoder value right motor
        int increment = 0;
        int8_t direction; // 0 = clockwise, 1 = counter clockwise
        
};