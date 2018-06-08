#include "Arduino.h"
#include "Encoder.h"
#include "Motor.h"
#include "NewPing.h"
//#include "TimerThree.h"
#include "Robot.h"




Robot::Robot(int T = 50, int *statePtr = NULL, double xPos = 0,
	double yPos = 0, double omegaPos = 0, int base = 23,
	int diameter = 9)
	// Default position = 0
	// Default dimension : wheelDiemater = 9cm, wheelBase = 23
{
	x = xPos;
	y = yPos;
	omega = omegaPos;
	wheelBase = base;
	wheelDiameter = diameter;
	timing = millis();
	timingSonar = timing;
	tSample = T;
	motorLeft.setSamplingTime(tSample);
	motorRight.setSamplingTime(tSample);
	speedOld = 0;
	mechanism.setStatePtr(statePtr);
}



void Robot::countRobot()
{
	mechanism.loop();
	// Should be executed at each itteration of the loop function
	bool isCount1 = motorLeft.countMotor();
	bool isCount2 = motorRight.countMotor();
	if (isCount1 || isCount2)
	{
		double speedRight = motorRight.getSpeed();
		double speedLeft = motorLeft.getSpeed();
		speed = (speedRight + speedLeft) / 2;
		angularSpeed = (speedRight - speedLeft)* wheelDiameter / 2 / wheelBase;
		localisation(); // To actuate the position of the robot
	}
	//actuateSonar();
}


void Robot::goStraight(double desiredSpeed)
{
	motorLeft.setSpeed(desiredSpeed);
	motorRight.setSpeed(desiredSpeed);
}

void Robot::stop()
{
	motorLeft.motorStop();
	motorRight.motorStop();
}

void Robot::localisation()
{
	double roundLNew = motorLeft.readEncoder();
	double roundRNew = motorRight.readEncoder();
	double Dl = static_cast<double>(roundLNew - roundLOld) / 2249 * 3.14159 * wheelDiameter;
	double Dr = static_cast<double>(roundRNew - roundROld) / 2249 * 3.14159 * wheelDiameter;
	roundLOld = roundLNew;
	roundROld = roundRNew;
	double Dc = (Dl + Dr) / 2;
	dForward += Dc;
	x += Dc * cos(omega);
	y += Dc * sin(omega);
	omega = omega + (Dr - Dl) / wheelBase;  // 23cm = wheel basis
	omega = atan2(sin(omega), cos(omega));
	// In order to limit the angle between -pi and pi
}



bool Robot::goTo(double xDesired, double yDesired)
{
	double omegaDesired = 0;
	double minimumSpeed = 10;
	double gainRho = 0.3;
	double gainAlpha = 0.8;

	bool isReached = false;
	double rho = sqrt(sq(xDesired - x) + sq(yDesired - y));
	if (rho / sqrt(sq(xDesired) + sq(yDesired)) <= 0.05 &&
		(xDesired != 0 || yDesired != 0))
		return isReached = true;
	if (rho <= 5 && xDesired == 0 && yDesired == 0)
		return isReached = true;

	// can't be true if it didn't enter in the if condition
	if (millis() > timing + tSample)
	{
		isReached = true; // Default value
		omegaDesired = atan2(yDesired - y, xDesired - x);
		double alpha = omegaDesired - omega;
		if (alpha > 3.14) {
			alpha = alpha - 2 * 3.14;
		}

		if (alpha < -3.14) {
			alpha = alpha + 2 * 3.14;
		}
		isReached = false; // We didn't reach the point yet
		double speedDesired = gainRho*rho*cos(alpha);
		double angularSpeedDesired = gainRho*sin(alpha)*cos(alpha) + gainAlpha*alpha;
		if (speedDesired > 50)
		{
			speedDesired = 50;
		}
		if (speedDesired < minimumSpeed)
		{
			speedDesired = minimumSpeed;
		}

		if (speedDesired > speedOld)
		{
			motorLeft.increase(true);
			motorRight.increase(true);
			// We want to accelerate the engines
		}
		else if (speedDesired <= speedOld)
		{
			motorLeft.increase(false);
			motorRight.increase(false);
		}
		double speedRight = (2 * speedDesired + angularSpeedDesired * wheelBase) / 2;//wheelDiameter;
		double speedLeft = (2 * speedDesired - angularSpeedDesired * wheelBase) / 2;//wheelDiameter;
		motorLeft.setSpeed(speedLeft);
		motorRight.setSpeed(speedRight);
		timing = millis();
	}
	return isReached;
}

bool Robot::getAngle(double omegaDesired)
{
	bool isReached = false;
	// can't be true if it didn't enter in the if condition
	if (millis() > timing + tSample)
	{
		double maximumSpeed = 7;
		isReached = true; // Default value
		double gainAlpha = 0.1;
		double alpha = omegaDesired - omega;
		if ((abs(alpha / omegaDesired)) > 0.02)
		{
			isReached = false; // We didn't reach the point yet
			double angularSpeedDesired = gainAlpha*alpha;

			double speedRight = (angularSpeedDesired * wheelBase) / 2;//wheelDiameter;
			double speedLeft = (-angularSpeedDesired * wheelBase) / 2;//wheelDiameter;
			if (abs(speedRight) > abs(speedRightOld))
				motorRight.increase(true);
			else if (abs(speedRight) <= abs(speedRightOld))
				motorRight.increase(false);
			if (abs(speedLeft) > abs(speedLeftOld))
				motorLeft.increase(true);
			else if (abs(speedLeft) <= abs(speedLeftOld))
				motorLeft.increase(false);

			if (abs(speedRight) < maximumSpeed && speedRight>0)
				speedRight = maximumSpeed;
			if (abs(speedLeft) < maximumSpeed && speedLeft>0)
				speedLeft = maximumSpeed;
			speedLeftOld = speedLeft;
			speedRightOld = speedRight;

			if (speedLeft >= 0)
				motorLeft.setSpeed(speedLeft);
			if (speedRight >= 0)
				motorRight.setSpeed((speedRight>0)*speedRight);

			timing = millis();
		}
	}
	return isReached;
}



void Robot::printSonar(long distance[SONAR_NUM])
{
	for (int i = 0; i < SONAR_NUM; i++)
	{
		Serial.print("Sensor ");
		Serial.print(i);
		Serial.print(": ");
		Serial.print(distance[i]);
		Serial.print("  ");
	}
	Serial.println();
}


//////   Overloaded function for printSonar   //////


void Robot::printSonar(double distance[SONAR_NUM])
{
	for (int i = 0; i < SONAR_NUM; i++)
	{
		Serial.print("Sensor ");
		Serial.print(i);
		Serial.print(": ");
		Serial.print(distance[i]);
		Serial.print("  ");
	}
	Serial.println();
}


//////////    Follow a wall on the left    /////////////

void Robot::newFollowWallLeft(double distance[SONAR_NUM],
	double desiredDistance, double vBase = 100.0)
{
	motorLeft.setMode(0);
	motorRight.setMode(0);

	double kp = 0.025;
	double kd = 55.0;
	double ki = 0.0;

	double vLim = vBase * 0.9;
	//bool isSucceed = actuateSonar();
	double actualDistance = 0.0;
	double deltaV = 0;
	bool dismiss = false;

	double error = 0;

	if (distance[1] == 0)
	{
		if (distance[0] == 0) dismiss = true;
		else actualDistance = distance[0];
	}
	else if (distance[0] == 0)
	{
		actualDistance = distance[1];
	}
	else actualDistance = min(distance[0], distance[1]);

	//printSonar(distance);
	//Serial.println(actualDistance);
	if (!dismiss)
	{
		error = desiredDistance - actualDistance;
		//Serial.println(kd*(error-errorOld));
		errorSum += error;
		//Serial.println(ki*errorSum);

		deltaV = kp * error + ki * errorSum
			+ kd * (error - errorOld);

		//Serial.println(deltaV);

		if (deltaV > vLim) {
			deltaV = vLim;
			errorSum -= error;
			error = (deltaV - ki * errorSum
				+ kd*errorOld) / (kp + kd);
			errorSum += error;
		}
		else if (deltaV < -vLim) {
			deltaV = -vLim;
			errorSum -= error;
			error = (deltaV - ki * errorSum
				+ kd*errorOld) / (kp + kd);
			errorSum += error;
		}
		deltaVOld = deltaV;
		errorOld = error;

		motorLeft.setVoltage(108.0 + deltaV, 1);
		//motorLeft.setVoltage(108.0, 1);
		motorRight.setVoltage(102.0 - deltaV, 1);
		//motorRight.setVoltage(99.0, 1);
		//Serial.println(deltaV);
		/*Serial.print("  V Left: ");
		Serial.print(vBase + deltaV);
		Serial.print("  V Right: ");
		Serial.println(vBase - deltaV);*/
		errorOld = error;
	}
}



void Robot::newFollowWallLeft(double actualDistance,
	double desiredDistance, double vBase = 100.0, int a = 0)
{
	// Overloaded function --> to be able to work on the 
	// parameters freely


	/*double kp = 600.0;
	double kd = 1200.0;
	double ki = 1.0;*/

	/*double kp = 400.0;
	double kd = 2000.0;
	double ki = 0.0;*/

	double kp = 300.0;
	double kd = 1000.0;
	double ki = 30.0;

	double vLim = vBase * 1.2;
	//bool isSucceed = actuateSonar();
	
	double deltaV = 0;
	bool dismiss = false;

	double error = 0;



	//printSonar(distance);
	//Serial.println(actualDistance);
	if (!dismiss)
	{
		error = desiredDistance - actualDistance;
		//Serial.println(kd*(error-errorOld));
		errorSum += error;
		//Serial.println(ki*errorSum);

		deltaV = kp * error + ki * errorSum
			+ kd * (error - errorOld);

		//Serial.println(deltaV);

		if (deltaV > vLim) {
			deltaV = vLim;
			errorSum -= error;
			error = (deltaV - ki * errorSum
				+ kd*errorOld) / (kp + kd);
			errorSum += error;
		}
		else if (deltaV < -vLim) {
			deltaV = -vLim;
			errorSum -= error;
			error = (deltaV - ki * errorSum
				+ kd*errorOld) / (kp + kd);
			errorSum += error;
		}
		if (errorOld == 0) errorOld = error;
		/*Serial.print(kp*error);
		Serial.print(' ');
		Serial.print(kd*(error-errorOld));
		Serial.print(' ');
		Serial.print(ki*errorSum);
		Serial.print(' ');
		Serial.print(deltaV);*/

		deltaVOld = deltaV;
		errorOld = error;

		
		//motorLeft.setVoltage(108.0 + deltaV);
		////analogWrite(3, 108.0 + deltaV);

		//motorRight.setVoltage(102.0 - deltaV);
		//analogWrite(11, 102.0 - deltaV);
		setDeltaV(deltaV);
		//Serial.println(deltaV);
		/*Serial.print("  V Left: ");
		Serial.print(vBase + deltaV);
		Serial.print("  V Right: ");
		Serial.println(vBase - deltaV);*/
		errorOld = error;
	}
}

void Robot::setDeltaV(const float &deltaV)
{
	motorLeft.setVoltage(100.0*1.1 + deltaV);
	motorRight.setVoltage(90.0*1.1 - deltaV);
}

void Robot::setOmega(double angle)
{
	float factor = 0.25;
	omega = omega * factor + (1-factor) * angle;
}


//////    Turn 90 degrees on the right     ///////
bool Robot::turnRight90(double omegaInit)
{
	if ((omegaInit - omega) < 1.02*(3.141592/2))
	{
		motorLeft.setVoltage(150, 1);
		motorRight.setVoltage(0, -1);
		return false;
	}
	else
	{
		motorLeft.motorStop();
		motorRight.motorStop();
		return true;
	}
}


/////////    Turn to    /////////////
bool Robot::turnTo(double omegaInit, double deltaOmega)
{
	int direction = 0;
	if ((deltaOmega) >= 0)
		direction = -1;
	else
		direction = 1;
	if (abs(omega - omegaInit) < 0.92*abs(deltaOmega))
	{
		motorLeft.setVoltage(150, direction);
		motorRight.setVoltage(0, 1);
		return false;
	}
	else
	{
		motorLeft.motorStop();
		motorRight.motorStop();
		return true;
	}
}


///////////////////////////////////////



///////     Sweep     ///////////
bool Robot::sweep()
{
	bool isReached;
	switch (sweepState)
	{
	case 1:
		isReached = turnTo(0.0, -0.3);
		if (isReached) ++sweepState;
		break;
	case 2:
		isReached = turnTo(-0.3, 0.6);
		if (isReached) ++sweepState;
		break;
	case 3:
		isReached = turnTo(0.3, -0.3);
		if (isReached)
		{
			sweepState = 1;
			return true;
		}
	}
	return false;
}



void Robot::interrupted()
{
	CylinderRecuperation::s_isInterrupted = true;
	mechanism.interrupted();
}

void Robot::empty()
{
	mechanism.openDoor();
}