#include "Arduino.h"
#include "Encoder.h"
#include "Motor.h"


Motor::Motor(int whichMotor, int encoderPin1, int encoderPin2, long T = 50) :
	motorEncoder {encoderPin1, encoderPin2 }
{
	nummerMotor = whichMotor;
	timingMotor = millis();
	stopped = true;
	tSample = T;
	isIncreased = true;
	pinMode(13, OUTPUT);
	pinMode(12, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(11, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(8, OUTPUT);
}

bool Motor::countMotor()
{
	bool isCount = false; // to see if we have changed the speed
						  //    if (nummerMotor == 1)
						  //      positionNew = motorEncoder1.read();
						  //
						  //    if (nummerMotor == 2)
						  //      positionNew = motorEncoder2.read();
	positionNew = motorEncoder.read();

	if (positionNew != positionOld)
	{
		stopped = false;
		timingOld = millis();
		positionOld = positionNew;
		++count;
		if (count % (4 * 48) == 0) // Actualize the speed each time the motor axis makes 4 round
		{
			speedWheel = speedometer();
			isCount = true;
		}
	}
	else if ((millis() - timingOld)>(tSample*1.1))
	{
		rpm = 0;
		speedWheel = 0;
		stopped = true;
		isCount = true;
	}

	// Actuate the PID
	if (mode == 1)
	{
		if (millis() > (timingMotor + tSample))
			// To set the sampling time
		{
			//Serial.println(tSample);
			// the PID will set the new voltage and the new direction
			controllerPid(motorSpeed);
			timingMotor = millis();
		}
		// If the PID didn't set a new voltage, the old one is sent
	}
	//Serial.println(voltage);
	if (voltage != voltageOld)
	{
		motorRun(static_cast<int>(voltage), motorDirection);
	}


	return isCount;
}

double Motor::speedometer()
{
	long timeNew = millis();
	double countNew = positionNew;
	double deltaTime = timeNew - timeOld;
	double speed = static_cast<double>(countNew - countOld) / 2249 / static_cast<double>(deltaTime) * 1000 * 3.14159 * 6;
	timeOld = timeNew;
	countOld = countNew;
	return speed;
}

void Motor::motorRun(int v, int desiredDirection = 1)
{
	// Desired direction: 1=forward, -1=backward
	if (nummerMotor == 1)
	{
		// A
		if (desiredDirection == 1)
			digitalWrite(12, LOW);
		else if (desiredDirection == -1)
			digitalWrite(12, HIGH);
		if (desiredDirection != 0)
		{
			digitalWrite(9, LOW); // Release the brake
			analogWrite(3, v);
		}
	}
	else if (nummerMotor == 2)
	{
		// B
		if (desiredDirection == -1)
			digitalWrite(13, LOW);

		else if (desiredDirection == 1)
			digitalWrite(13, HIGH);
		if (desiredDirection != 0)
		{
			digitalWrite(8, LOW);
			analogWrite(11, v);
		}
	}
}


void Motor::motorStop()
{
	motorSpeed = 0;
	motorDirection = 0;
	voltage = 0;
	if (nummerMotor == 1)
	{
		// A
		digitalWrite(9, HIGH); // Put the brake on
		analogWrite(3, 0);
	}
	else if (nummerMotor == 2)
	{
		// B
		digitalWrite(8, HIGH);
		analogWrite(11, 0);
	}
}


void Motor::controllerPid(double desiredSpeed)
{
	if (desiredSpeed > 0)
		motorDirection = 1;
	else if (desiredSpeed < 0)
		motorDirection = -1;

	if (desiredSpeed != 0)
	{
		int maxVoltage = 255;
		int minVoltage = 30;
		int triggerVoltage = 100;
		/*double kp = 0.07;
		double ki = 0.025;
		double kd = 0.08;*/

		double kp = 0.07;
		double ki = 0.025;
		double kd = 0.08;


		double error = motorDirection*(desiredSpeed - getSpeed());
		voltage = (kp*error) + voltageOld + ki*errorOld + (kd*(error - errorOld));
		errorOld = error;
		// voltage += pid;

		// Anti wind-up to avoid that the error cummulate when there is saturation
		if (voltage > maxVoltage)
		{
			//Serial.println("Max voltage reached");
			voltage = maxVoltage;
			error = (voltage - voltageOld - errorOld*(ki - kd)) / (kp + kd);
		}
		if (voltage < minVoltage)
		{
			//Serial.println("Min voltage reached");
			voltage = minVoltage;
			error = (voltage - voltageOld - errorOld*(ki - kd)) / (kp + kd);
		}
		if (stopped && isIncreased)
			// To counter the friction forces when the motor is stopped
		{
			//Serial.println("Trigger needed");
			voltage = triggerVoltage;
			error = (voltage - voltageOld - errorOld*(ki - kd)) / (kp + kd);
		}
		voltageOld = voltage;
	}
}

void Motor::setSpeed(double desiredSpeed)
{
	mode = 1;
	if (desiredSpeed >= 0)
		motorDirection = 1;

	else if (desiredSpeed<0)
		motorDirection = -1;

	motorSpeed = desiredSpeed;
}

void Motor::setVoltage(double v, int desiredDirection)
{
	mode = 0;
	voltageOld = voltage;
	voltage = v;
	motorDirection = desiredDirection;
}

void Motor::setVoltage(double v)
{
	mode = 0;
	if (v < 0) setVoltage(-v, -1);
	else setVoltage(v, 1);
}