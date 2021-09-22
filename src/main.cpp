#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <ICM_20948.h> // Sparkfun ICM_20948 IMU module
#include <PID_v1.h>
#include "utilityFuncs.h" 
#include "motorClass.h"			// #ifdef ENCODER_TACHO

// Hardware Timer check
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

/* Define Motor Driver Pins */
#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

/* Define encoder pins */
#define encoderLA A0 //3
#define encoderLB A1 //2
#define encoderRA A2 //3
#define encoderRB A3 //2
#define encoderLA_OFFSET 0 // ((GPIOA->IDR >> encoderLA_OFFSET) & 1)
#define encoderLB_OFFSET 1 // ((GPIOA->IDR >> encoderLB_OFFSET) & 1)
#define encoderRA_OFFSET 2 // ((GPIOA->IDR >> encoderLA_OFFSET) & 1)
#define encoderRB_OFFSET 3 // ((GPIOA->IDR >> encoderLB_OFFSET) & 1)

/* Strip lines per inch */
#define stripLPI 150.0

/* IMU definitions */

#define SPI_PORT SPI // 13 // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10	 // Which pin you connect CS to. Used only when "USE_SPI" is defined

// Configure SPI for IMU
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object

const float fullRev = 1632.67;		// Ticks per drive-shaft revolution (different per motor/gearbox)
const float wheelCirc = 3.14 * 0.8; // Circumference in metres

// PID Configuration parameters
// Specify the links and initial tuning parameters
struct motorPID
{
	volatile long encoderPos = 0; // Current encoder position since the last clearance
	double speedTotal = 0;		  // Sum of speed measurements
	double speedError = 0;		  // Difference between
	double speedError_pre = 0;	  // Error
	double speedErrorSum = 0;	  // Sum of errors
	double speed = 0;			  // Calculated wheel speed in RPM
	double speedDesired = 0;	  // PID Set Point
	double speedPWM = 0;		  // RPM
	double kp = 0;				  //0.02; // Proportional coefficient
	double ki = 0;				  //16.0; // Integral coefficient
	double kd = 0;				  //0.01; // Derivative coefficient

	float lastKnownPos = 0; // Last Known Position
	float ticks_per_millisecond = 0;

	long wheelSpeedDistance = 0; // RPM
} motorL_PID, motorR_PID;

int desR = 0;
int desL = 0;

int directionR = 0; // 1 = forward, 0 = backwards
int directionL = 0;

// System states/modes
enum sysModes
{
	IDLE,
	TEST_IMU,
	TEST_TURN,
	TEST_DRIVE,
	TEST_DRIVE_SPEED,
	TEST_TACHO,
	ACTIVE,
	STOPPED
};
int sysMode = IDLE; // Current system state

//******************************//
//******* MOTOR SETUP **********//
//******************************//
const int offsetA = 1; //  Set offset values to adjust motor direction if necessary. Values: 1 or -1
const int offsetB = 1;

Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Left Motor
Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Right Motor

//******************************//
//******* PID SETUP ************//
//******************************//
PID pidLeft(&motorL_PID.speed, &motorL_PID.speedPWM, &motorL_PID.speedDesired, motorL_PID.kp, motorL_PID.ki, motorL_PID.kd, P_ON_M, DIRECT);
PID pidRight(&motorR_PID.speed, &motorR_PID.speedPWM, &motorR_PID.speedDesired, motorR_PID.kp, motorR_PID.ki, motorR_PID.kd, P_ON_M, DIRECT);

//******************************//
//******* TIMER SETUP **********//
//******************************//
TIM_TypeDef *Instance1 = TIM2; // Creates an instance of hardware Timer 2
HardwareTimer *Timer = new HardwareTimer(Instance1);

//******************************//
//**** TACHOMETER SETUP ********//
//******************************//
tachoWheel tachoL_o(encoderLA, encoderLB, encoderLA_OFFSET, encoderLB_OFFSET); // Right Tachometer Object (Will be integrated into the motor class)
tachoWheel tachoR_o(encoderRB, encoderRA, encoderRB_OFFSET, encoderRA_OFFSET); // Right Tachometer Object (Will be integrated into the motor class)

//******************************//
//******** ITERATORS ***********//
//******************************//
int iter = 1;		// Loop counter
int iter_coeff = 1; // Multiplier for the loop counter - Changes to negative to provide a triangular iterator
int counter = 0;

//******************************//
//****** SERIAL CONFIG *********//
//******************************//
char payload[100];			 // Incoming serial payload
char modeSelect;			 // Mode select variable - Populated by the first character of the incoming packet
bool stringComplete = false; // Serial string completion

// Speed Calc Callback
void speedCalc_callback(void)
{
	#ifdef ENCODER_TACHO
		tachoL_o.calcVelocity();
		tachoR_o.calcVelocity();
		//motorL_PID.speed = tachoL_o.getVelocity();
		//motorR_PID.speed = tachoR_o.getVelocity();
	#endif
}

volatile long encoderPosL = 0;
long wheelSpeedDistanceL = 0; // RPM
// Flags for the encoder triggers - Mainly for debugging
bool Lfired = 0;
bool change_fired = 0;

// Encoder tracking flags
bool running = 1;
bool Lrun = 1;

// Quadrature Encoder matrix - 2s shouldn't ever appear Sauce: https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
//int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
//int LOld = 0; // Previous encoder value left motor
//int LNew = 0; // New encoder value left motor

	// Hall encoder ISRs. Called once for each sensor on pin-change (quadrature)
	void encoderLeft_callback(void)  { tachoL_o.encoderTick();  }
	void encoderRight_callback(void) { tachoR_o.encoderTick();  }

// Print the formatted IMU variables
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
	float aval = abs(val);
	if (val < 0)
		Serial.print("-");
	else
		Serial.print(" ");

	for (uint8_t indi = 0; indi < leading; indi++)
	{
		uint32_t tenpow = 0;
		if (indi < (leading - 1))
			tenpow = 1;

		for (uint8_t c = 0; c < (leading - 1 - indi); c++)
			tenpow *= 10;

		if (aval < tenpow)
			Serial.print("0");
		else
			break;
	}

	if (val < 0)
		Serial.print(-val, decimals);
	else
		Serial.print(val, decimals);
}

// Print the scaled and cleaned IMU data
void printScaledAGMT(ICM_20948_SPI *sensor)
{
	//Serial.print("Scaled. Acc (mg) [ ");
	//printFormattedFloat(sensor->accX(), 5, 2);
	//Serial.print("\t");
	//printFormattedFloat(sensor->accY(), 5, 2);
	//Serial.print("\t");
	printFormattedFloat(sensor->accZ(), 5, 2);
	Serial.print("\t");
	//printFormattedFloat(sensor->gyrX(), 5, 2);
	//Serial.print("\t");
	//printFormattedFloat(sensor->gyrY(), 5, 2);
	//Serial.print("\t");
	printFormattedFloat(sensor->gyrZ(), 5, 2);
	Serial.print("\t");
	//printFormattedFloat(sensor->magX(), 5, 2);
	//Serial.print("\t");
	//printFormattedFloat(sensor->magY(), 5, 2);
	//Serial.print("\t");
	printFormattedFloat(sensor->magZ(), 5, 2);
	//Serial.print("\t");
	//printFormattedFloat(sensor->temp(), 5, 2);
	Serial.println();
}

void setup()
{
	Serial.begin(115200);

	while (!Serial){}

	// Configure Encoder Pins
	Serial.print("Configuring pins and attaching interrupts...");

		// Attach hardware interrupts to encoder pins
		attachInterrupt(encoderLA, encoderLeft_callback, CHANGE);
		attachInterrupt(encoderLB, encoderLeft_callback, CHANGE);
		attachInterrupt(encoderRA, encoderRight_callback, CHANGE);
		attachInterrupt(encoderRB, encoderRight_callback, CHANGE);
	
	// Halt both motors
	brake(motorL, motorR);
	Serial.println("Done - Brakes applied");

	delay(50);
	//Serial.print("Initialising PID controllers... ");
	pidLeft.SetMode(AUTOMATIC); //start calculation.
	pidLeft.SetOutputLimits(-250, 250);
	pidLeft.SetSampleTime(20);

	pidRight.SetMode(AUTOMATIC); //start calculation.
	pidRight.SetOutputLimits(-250, 250);
	pidRight.SetSampleTime(20);
	//Serial.println("Done");

	//Serial.print("Setting PID characteristics... ");
	pidLeft.SetTunings(0.8, 11.0, 0.1); // kP, kI, kD
	pidRight.SetTunings(0.8, 11.0, 0.1);
	//Serial.println("Done");

	//Serial.println("Initializing IMU... ");
	myICM.begin(CS_PIN, SPI_PORT);

	bool initialized = false;
	while (!initialized)
	{
		//Serial.print(F("Initialization of the sensor returned: "));
		//Serial.println(myICM.statusString());

		if (myICM.status != ICM_20948_Stat_Ok)
		{
			//Serial.println("Trying again...");
			delay(500);
		}
		else
		{
			initialized = true;
			//Serial.println("IMU initialized");
		}
	}

	//Serial.print("Configuring Timer... ");
	delay(500);
	noInterrupts();
	Timer->setOverflow(60, HERTZ_FORMAT); // Read the tachometers 60 times per second
	Timer->attachInterrupt(speedCalc_callback);
	
	interrupts();

	Timer->resume();
	//Serial.println("Done - Timer Active");
	//Serial.print("s");
	
	Serial.println("System Ready...");
	
	//while ((char)Serial.read() != 's') { }
	//Serial.print("r");
	sysMode = TEST_TACHO;
}

int stall = 50; // A delay value for the top of the testing triangle
int idx = 0;	// Serial buffer index

// This is called immediately before every iteration of loop() to process any serial packets
void serialEvent()
{
	while (Serial.available())
	{
		// get the new byte:
		char inChar = (char)Serial.read();

		// Save the first character (mode select)
		if (idx == 0)
		{
			modeSelect = inChar;
		}
		else
		{
			// Add remaining chars to the payload
			payload[idx - 1] = inChar;
		}
		
		// Check for null terminator
		if (inChar == '\n')
		{
			stringComplete = true;
			idx = 0;
		}
		else 
		{
			idx++;
		}
	}
}

void loop()
{
	// If a full packet has been captured at the beginning of the loop, process the result
	if (stringComplete)
	{
		if (modeSelect == 'A')
		{
			Serial.println("Reset to Idle");
			sysMode = IDLE;
		}
		else if (modeSelect == 'B')
		{
			Serial.println("IMU Test");
			sysMode = TEST_IMU;
		}
		else if (modeSelect == 'C')
		{
			Serial.println("Test Drive\n");
			Serial.print("Speed (payload): ");
			printString(payload);
			sysMode = TEST_DRIVE_SPEED;
		}
		else if (modeSelect == 'D')
		{
			Serial.println("Test Drive Sequence\n");
			sysMode = TEST_DRIVE;
		}
		else if (modeSelect == 'T')
		{
			Serial.println("Test Turn Sequence\n");
			Serial.print("Angle (payload): ");
			printString(payload);
			
			sysMode = TEST_DRIVE_SPEED;
			sysMode = TEST_TURN;
		}
		else if (modeSelect == 'S')
		{
			Serial.println("Test Tachometry\n");
			sysMode = TEST_TACHO;
		}
		else if (modeSelect == 'X')
		{
			Serial.println("System Halted");
			sysMode = STOPPED;
		}
		stringComplete = false;
	}

	if (sysMode == IDLE)
	{
		motorR_PID.speedDesired = 0;
		motorL_PID.speedDesired = 0;
		motorL.drive(0); // Output
		motorR.drive(0); // Output
	}
	else if (sysMode == TEST_IMU)
	{
		//if (iter++ < 5000)
		//{
			if (myICM.dataReady())
			{
				myICM.getAGMT();		 // The values are only updated when you call 'getAGMT'
										 //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
				printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
			}
		//}
		//else
		//{
		//	sysMode = IDLE;
		//	iter = 0;
		//}
		//*/
	}
	else if (sysMode == TEST_DRIVE_SPEED)
	{
		if (payload > 0)
		{
			motorL_PID.speedDesired = atof(payload);
			motorR_PID.speedDesired = atof(payload);

			if(pidLeft.Compute()){
				//Serial.println("#########################");
				//Serial.println("Left PID Compute Failed!!");
				//Serial.println("#########################");
				motorL.drive(motorL_PID.speedPWM); // Output
			};

			if(pidRight.Compute()){
				//Serial.println("##########################");
				//Serial.println("Right PID Compute Failed!!");
				//Serial.println("##########################");
				motorR.drive(motorR_PID.speedPWM); // Output
			};

			// Print information on the serial monitor
			//Serial.print(iter); // Tachometer
			//Serial.print("\t");
			Serial.print(motorL_PID.speedDesired); // Tachometer
			Serial.print("\t");
			Serial.print(motorR_PID.speedDesired); // Tachometer
			Serial.print("\t");
			//Serial.print(motorL_PID.speedPWM); // Tachometer
			//Serial.print("\t");
			//Serial.print(motorR_PID.speedPWM); // Tachometer
			//Serial.print("\t");
			Serial.print(motorL_PID.speed); // Tachometer
			Serial.print("\t");
			Serial.print(motorR_PID.speed); // Tachometer
			Serial.println();
		}
		else
		{
			Serial.println("Brake both motors");
			motorL.drive(0); // Output
			//motorR.drive(0); // Output
			delay(20);
			brake(motorL, motorR);
		}
	}
	else if (sysMode == TEST_TURN)
	{
		//if (payload > 0){
			motorL_PID.speedDesired = atof(payload);
			motorR_PID.speedDesired = atof(payload);

			pidLeft.Compute();
			pidRight.Compute();

			// TODO NOT WORKING IN REVERSE
			motorL.drive(motorL_PID.speedPWM); // Output
			motorR.drive(motorR_PID.speedPWM); // Output

			// Print information on the serial monitor
			Serial.print(motorL_PID.speedDesired); // Tachometer
			Serial.print("\t");
			Serial.print(motorL_PID.speed); // Tachometer
			Serial.print("\t");
			Serial.print(motorR_PID.speed); // Tachometer
			Serial.print("\t");
			Serial.println();
		//}
		//else
		//{
			//Serial.println("Brake both motors");
			//motorL.drive(0); // Output
			//motorR.drive(0); // Output
			//delay(20);
			//brake(motorL, motorR);
		//}
	}
	else if (sysMode == TEST_DRIVE)
	{
		if(iter >= 200)
		{
			iter = 0;
		}
		else if(iter >= 100)
		{
			desL = 25;
			desR = 25;
			directionR = -1;
			directionL = 1;
		}
		else if(iter >= 0)
		{
			directionR = 1;
			directionL = -1;
			desL = 25;
			desR = 25;
		}
		iter++;

		motorL_PID.speedDesired = desL;
		motorR_PID.speedDesired = desR;

		pidLeft.Compute();
		pidRight.Compute();

		motorL.drive(motorL_PID.speedPWM*directionL); // Output
		motorR.drive(motorR_PID.speedPWM*directionR); // Output

		// Print information on the serial monitor
		//Serial.print(iter); // Tachometer
		//Serial.print("\t");
		Serial.print(motorL_PID.speedDesired*directionL); // Tachometer
		Serial.print("\t");
		Serial.print(motorR_PID.speedDesired*directionL); // Tachometer
		Serial.print("\t");
		//Serial.print(motorL_PID.speedPWM); // Tachometer
		//Serial.print("\t");
		//Serial.print(motorR_PID.speedPWM); // Tachometer
		//Serial.print("\t");
		Serial.print(motorL_PID.speed*directionL); // Tachometer
		Serial.print("\t");
		Serial.print(motorR_PID.speed*directionL); // Tachometer
		Serial.println();
		//*/
	}
	else if (sysMode == TEST_TACHO)
	{
		//if(tachoL_o.fired){
			tachoR_o.sendInfo();
			Serial.print("\t");
			tachoL_o.sendInfo();
			//tachoL_o.fired = 0;
			//tachoR_o.fired = 0;
			Serial.println();
		//}
		delay(30);
	}
	else if (sysMode == STOPPED)
	{
		motorR_PID.speedDesired = 0;
		motorL_PID.speedDesired = 0;
		motorL.drive(0); // Output
		motorR.drive(0); // Output
		brake(motorL, motorR);
	}
	delay(10); // 30 was pretty high. Changed to 10 for smoother communication and operation
}