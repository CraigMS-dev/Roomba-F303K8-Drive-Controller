#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <ICM_20948.h> // Sparkfun ICM_20948 IMU module
#include <PID_v1.h>
#include "motorClass.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>

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

/* Define encoder pins */
#define encoderLA A0 //3
#define encoderLB A1 //2
#define encoderRA A2 //3
#define encoderRB A3 //2

/* Strip lines per inch */
#define stripLPI 150.0

/* IMU definitions */
#define IMU_STATE 1
#define SPI_PORT SPI // 13 // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10	 // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

// Configure SPI for IMU
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object

const float fullRev = 1632.67;		// Ticks per drive-shaft revolution (different per motor/gearbox)
const float wheelCirc = 3.14 * 0.8; // Circumference in metres

// PID Configuration parameters
// Specify the links and initial tuning parameters
/*struct imuPID
{
	float q0;
	float q1;
	float q2;
	float q3;

	float yawTotal = 0;
	int   yawTotalSamples = 0;

	float yawTarget = 0;
	float yawActual = 0;
	float yawError = 0;
	float yawErrorOld = 0;
	float yawErrorChange;
	float yawErrorSlope = 0;
	float yawErrorArea = 0;
	
	double kp = 0;				  //0.02; // Proportional coefficient
	double kd = 0;				  //0.01; // Derivative coefficient
	double ki = 0;				  //16.0; // Integral coefficient
	
	int oldTime = 0;
	int newTime = 0;
	int dt;

}turnPID;*/

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
	double kd = 0;				  //0.01; // Derivative coefficient
	double ki = 0;				  //16.0; // Integral coefficient

	float lastKnownPos = 0; // Last Known Position
	float ticks_per_millisecond = 0;

	long wheelSpeedDistance = 0; // RPM
} motorL_PID, motorR_PID;

// System states/modes
enum sysModes
{
	IDLE,
	TEST_IMU,
	TEST_DRIVE,
	TEST_DRIVE_SPEED,
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

TIM_TypeDef *Instance2 = TIM3; // Creates an instance of hardware Timer 2
HardwareTimer *Timer2 = new HardwareTimer(Instance2);

//******************************//
//**** TACHOMETER SETUP ********//
//******************************//
tachoWheel tachoR_o; // Right Tachometer Object (Will be integrated into the motor class)
tachoWheel tachoL_o; // Right Tachometer Object (Will be integrated into the motor class)

//******************************//
//******** IMU SETUP ***********//
//******************************//
uint32_t timestamp;
//Adafruit_NXPSensorFusion filter; // slowest
Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset

//******************************//
//******** ITERATORS ***********//
//******************************//
int iter = 1;		// Loop counter
int iter_coeff = 1; // Multiplier for the loop counter - Changes to negative to provide a triangular iterator
int counter_1 = 0;

float timer_1 = 0;

//******************************//
//****** SERIAL CONFIG *********//
//******************************//
char payload[100];			 // Incoming serial payload
char modeSelect;			 // Mode select variable - Populated by the first character of the incoming packet
bool stringComplete = false; // Serial string completion

// Direction Callback - Calculate direction with IMU
//void dirCalc_callback(void){
	//timer_1 = millis();
	/*if (myICM.dataReady()){
		myICM.getAGMT();
		//turnPID.yawTotal += myICM.magX();
		turnPID.yawTotal += myICM.magZ();
		turnPID.yawTotalSamples++;
		
		if(turnPID.yawTotalSamples >= 5){
			turnPID.yawActual = turnPID.yawTotal;// / turnPID.yawTotalSamples;
			turnPID.yawTotal = 0;
			turnPID.yawTotalSamples = 0;
		}
	}//*/
	//Serial.println(millis());
//}

// Speed Calc Callback
void speedCalc_callback(void)
{
	tachoL_o.calcVelocity();
	tachoR_o.calcVelocity();
	motorL_PID.speed = tachoL_o.getVelocity();
	motorR_PID.speed = tachoR_o.getVelocity();
}

// Hall encoder ISRs. Called once for each sensor on pin-change (quadrature)
void encoderLeft_callback(void) { tachoL_o.encoderTick(); }
void encoderRight_callback(void) { tachoR_o.encoderTick(); }

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    Serial.print("-");
  }
  else
  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      Serial.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    Serial.print(-val, decimals);
  }
  else
  {
    Serial.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_SPI *sensor)
{
  printFormattedFloat(sensor->accX(), 5, 2); // 
  Serial.print("\t");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->gyrX(), 5, 2); // Gs
  Serial.print("\t");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.println();
}

void setup()
{
	Serial.begin(115200);

	while (!Serial) { }

	// Configure Encoder Pins
	//Serial.print("Configuring pins and attaching interrupts... ");
	pinMode(encoderLA, INPUT);
	pinMode(encoderLB, INPUT);
	pinMode(encoderRA, INPUT);
	pinMode(encoderRB, INPUT);
	digitalWrite(encoderLA, HIGH);
	digitalWrite(encoderLB, HIGH);
	digitalWrite(encoderRA, HIGH);
	digitalWrite(encoderRB, HIGH);

	// Attach hardware interrupts to encoder pins
	attachInterrupt(encoderLA, encoderLeft_callback, CHANGE);
	attachInterrupt(encoderLB, encoderLeft_callback, CHANGE);
	attachInterrupt(encoderRA, encoderRight_callback, CHANGE);
	attachInterrupt(encoderRB, encoderRight_callback, CHANGE);

	// Halt both motors
	brake(motorL, motorR);
	//Serial.println("Done - Brakes applied");

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
	pidRight.SetTunings(0.05, 18.0, 0.01);
	//Serial.println("Done");

	//Serial.println("Initializing IMU... ");
	SPI_PORT.begin();
	
	bool initialized = false;
	while (!initialized)
	{
		myICM.begin(CS_PIN, SPI_PORT);

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
			//turnPID.newTime = millis();
			//Serial.println("IMU initialized");
		}
	}

	//Serial.print("Configuring Timer... ");
	delay(500);
	noInterrupts();
	Timer->setOverflow(60, HERTZ_FORMAT); // Read the tachometers 60 times per second
	Timer->attachInterrupt(speedCalc_callback);

	//Timer2->setOverflow(120, HERTZ_FORMAT); // Read the tachometers 60 times per second
	//Timer2->attachInterrupt(dirCalc_callback);
	interrupts();

	Timer->resume();
	//Timer2->resume();
	//Serial.println("Done - Timer Active");
	
	timestamp = millis();

	Serial.println("System Ready!");

	//while ((char)Serial.read() != 's') { }
	sysMode = TEST_IMU;
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
	if(IMU_STATE)

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
			//printString(payload); // Removed for space
			sysMode = TEST_DRIVE_SPEED;
		}
		else if (modeSelect == 'D')
		{
			Serial.println("Test Drive Sequence\n");
			sysMode = TEST_DRIVE;
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
		if (myICM.dataReady())
		{
			float roll, pitch, heading;
  			float gx, gy, gz;
			static uint8_t counter = 0;

			 if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
				return;
			}
			timestamp = millis();

			myICM.getAGMT();
			filter.update(
				myICM.gyrX(), myICM.gyrY(), myICM.gyrZ(),
				myICM.accX(), myICM.accY(), myICM.accZ(),
				myICM.magX(), myICM.magY(), myICM.magZ());
			
			#if defined(AHRS_DEBUG_OUTPUT)
			Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
			#endif
			// only print the calculated output once in a while
			/*if (counter++ <= PRINT_EVERY_N_UPDATES) {
				return;
			}*/
			// reset the counter
			counter = 0;

			// print the heading, pitch and roll
			roll 	= filter.getRoll();
			pitch 	= filter.getPitch();
			heading = filter.getYaw();
			//Serial.print("Orientation: ");
			//Serial.print(heading);	Serial.print("\t");
			//Serial.print(pitch);	Serial.print("\t");
			//Serial.println(roll);

			Serial.print("y");
			Serial.print(heading);
			Serial.print("yp");
			Serial.print(pitch);
			Serial.print("pr");
			Serial.print(roll);
			Serial.println("r");

			// y168.8099yp12.7914pr-11.8401r
		}
		else
		{
			Serial.println("Waiting for data");
			delay(500);
		}//*/
	}
	else if (sysMode == TEST_DRIVE_SPEED)
	{
		if (payload > 0)
		{
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
		}
		else
		{
			motorL.drive(0); // Output
			motorR.drive(0); // Output
			delay(20);
			brake(motorL, motorR);
		}
	}
	else if (sysMode == TEST_DRIVE)
	{
		// Used for the triangle test
		//motorL_PID.speedDesired = map(iter, 0, 600, 0, 140);
		//motorR_PID.speedDesired = map(iter, 0, 600, 0, 140);

		pidLeft.Compute();
		pidRight.Compute();

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
		//*/
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