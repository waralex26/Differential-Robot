#include "Arduino.h"
#include "Encoder.h"
#include "Motor.h"
#include "Sonar.h"
//#include "TimerThree.h"
#include "Robot.h"


Robot::Robot(int T = 50, double xPos = 0, double yPos = 0,
	double omegaPos = 0, int base = 23, int diameter = 6)
	: sonarR(30, 31), sonarL(22, 23), sonarF(24, 25),
	sonarFL(26, 27), sonarFR(28, 29)
	// Default position = 0
	// Default dimension : wheelDiemater = 6cm, wheelBase = 23
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
}

void Robot::countRobot()
{
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
	actuateSonar();
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
	double Dl = static_cast<double>(roundLNew - roundLOld) / 2249 * 3.14159 * 6;
	double Dr = static_cast<double>(roundRNew - roundROld) / 2249 * 3.14159 * 6;
	roundLOld = roundLNew;
	roundROld = roundRNew;
	double Dc = (Dl + Dr) / 2;

	x = x + Dc * cos(omega);
	y = y + Dc * sin(omega);
	omega = omega + (Dr - Dl) / wheelBase;  // 23cm = wheel basis
	omega = atan2(sin(omega), cos(omega));
	// In order to limit the angle between -pi and pi

}

void Robot::localisationSonar()
{
	x = x + sonarF.getDelta()*cos(omega);
	y = y + sonarR.getDelta()*cos(omega);
	omega = atan2(sonarR.getDelta(), sonarF.getDelta());
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
		//    Serial.print(omegaDesired);
		//    Serial.print("   ");
		//    Serial.print(omega);
		//    Serial.print("   ");
		if (alpha > 3.14) {
			alpha = alpha - 2 * 3.14;
		}

		if (alpha < -3.14) {
			alpha = alpha + 2 * 3.14;
		}
		//    Serial.println(alpha);
		//Serial.println(rho/sqrt(sq(xDesired) + sq(yDesired)));
		isReached = false; // We didn't reach the point yet
		double speedDesired = gainRho*rho*cos(alpha);
		double angularSpeedDesired = gainRho*sin(alpha)*cos(alpha) + gainAlpha*alpha;
		//Serial.println(speedDesired);
		if (speedDesired > 50)
		{
			speedDesired = 50;
		}
		if (speedDesired < minimumSpeed)
		{
			//Serial.println("Minimum speed reached");
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
		//Serial.println(rho/sqrt(sq(xDesired) + sq(yDesired)));
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

void Robot::actuateSonar()
{
	if (millis() > (timingSonar + tSonar))
	{
		sonarF.computeDistance();
		sonarR.computeDistance();
		sonarL.computeDistance();
		sonarFR.computeDistance();
		sonarFL.computeDistance();
		timingSonar = millis();
		newSonar = true;
	}
}

void Robot::printSonar()
{
	Serial.print("sonarF: ");
	Serial.println(sonarF.getDistance());
	Serial.print("sonarL: ");
	Serial.println(sonarL.getDistance());
	Serial.print("sonarR: ");
	Serial.println(sonarR.getDistance());
	Serial.print("sonarFL: ");
	Serial.println(sonarFL.getDistance());
	Serial.print("sonarFR: ");
	Serial.println(sonarFR.getDistance());
}

void Robot::followWallLeft(double desiredDistance, double vBase = 160.0)
{
	motorLeft.setMode(0);
	motorRight.setMode(0);
	double FL = 0.6;
	double L = 1.0 - FL;

	desiredDistance = L * desiredDistance + FL * desiredDistance / cos(3.1415 / 4);

	double kp = 2.0;
	double ki = 0.0;
	double kd = 0.0;

	double vLim = 80.0;
	if (newSonar == true)
	{
		double distanceLWall = sonarL.getDistance();
		double distanceFLWall = sonarFL.getDistance();
		Serial.print(distanceLWall);
		Serial.print(" ");
		Serial.print(distanceFLWall);
		Serial.print(" ");
		double distanceWall = FL * distanceFLWall + L * distanceLWall;
		Serial.print(distanceWall);
		Serial.print(" ");
		double error = desiredDistance - distanceWall;
		double vPid = kp * error + (ki!=0)*vPidOld + ki * errorOld + kd * (error - errorOld);
		Serial.print(vPid);
		Serial.print(" ");
		if (vPid > vLim)
		{
			vPid = vLim;
			error = (vPid - vPidOld - ki * errorOld - kd * (error - errorOld)) / kp;
		}
		else if (vPid < -vLim)
		{
			vPid = -vLim;
			error = (vPid - vPidOld + ki * errorOld - kd * (error - errorOld)) / kp;
			//Serial.print(error);
			//Serial.print(" ");
		}
		Serial.println(vPid);
		vPidOld = vPid;
		errorOld = error;

		double vLeftWheel = vBase + vPid;
		double vRightWheel = vBase - vPid;
		motorLeft.setVoltage(static_cast<int>(vLeftWheel),1);
		motorRight.setVoltage(static_cast<int>(vRightWheel),1);
		newSonar = false;
	}
	
}