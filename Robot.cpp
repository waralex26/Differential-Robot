#include "Arduino.h"
#include "Encoder.h"
#include "Motor.h"
#include "Sonar.h"
#include "NewPing.h"
//#include "TimerThree.h"
#include "Robot.h"


Robot::Robot(int T = 50, double xPos = 0, double yPos = 0,
	double omegaPos = 0, int base = 23, int diameter = 9)
	: sonarR(30, 31), sonarL(22, 23), sonarF(24, 25),
	sonarFL(26, 27), sonarFR(28, 29)
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

	x = x + Dc * cos(omega);
	y = y + Dc * sin(omega);
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

bool Robot::actuateSonar()
{
	if (millis() > (timingSonar + tSonar))
	{
		timingSonar += tSonar;
		if (sonarTurn == 1)
		{
			distanceR = sonarR.ping_cm();
			sonarTurn = 2;
			if (distanceR == 0)
				return false;
			else
				return true;
		}
		else if (sonarTurn == 2)
		{
			distanceFR = sonarFR.ping_cm();
			sonarTurn = 1;
			newSonar = true;
			if (distanceFR == 0)
				return false;
			else
				return true;
		}
	}
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
	//Serial.print("sonarF: ");
	//Serial.println(sonarF.getDistance());
	//Serial.print("sonarL: ");
	//Serial.println(sonarL.getDistance());
	//Serial.print("sonarR: ");
	//Serial.println(sonarR.getDistance());
	//Serial.print("sonarFL: ");
	//Serial.println(sonarFL.getDistance());
	//Serial.print("sonarFR: ");
	//Serial.println(sonarFR.getDistance());
}

void Robot::newFollowWallRight(long distance[SONAR_NUM],
	double desiredDistance, double vBase = 100.0)
{
	motorLeft.setMode(0);
	motorRight.setMode(0);

	double kp = 0.5;
	double kd = 2.0;

	double vLim = vBase / 4;
	//bool isSucceed = actuateSonar();
	long actualDistance = 0;
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

	printSonar(distance);
	if (!dismiss)
	{
		error = desiredDistance - actualDistance;

		deltaV = kp * error + kd * (error - errorOld);

		if (deltaV > vLim) deltaV = vLim;
		else if (deltaV < -vLim) deltaV = -vLim;

		motorLeft.setVoltage(vBase - deltaV, 1);
		motorRight.setVoltage(vBase + deltaV, 1);
		Serial.print("  V Left: ");
		Serial.print(vBase - deltaV);
		Serial.print("  V Right: ");
		Serial.println(vBase + deltaV);
		errorOld = error;
	}
}


void Robot::followWallRight(double desiredDistance, double vBase = 160.0)
{
	motorLeft.setMode(0);
	motorRight.setMode(0);

	double kp = 1.0;
	double ki = 0.0;
	double kd = 0.0;

	double vLim = vBase/2;
	if (millis() > (timingSonar + tSonar))
	{
		newSonar = true;
		timingSonar = millis();
	}
	bool dismiss = false;
	if (newSonar == true)
	{
		if (sonarTurn == 1)
		{
			distanceLWall = sonarR.ping_cm();
			sonarTurn = 2;
		}
		else if (sonarTurn == 2)
		{
			distanceFLWall = sonarFR.ping_cm();
			sonarTurn = 1;
		}
		if (distanceLWall == 0)
		{
			if (distanceFLWall == 0)
			{
				dismiss = true;
			}
			else
			{
				distanceLWall = distanceFLWall;
			}
		}
		if (distanceFLWall == 0)
		{
			distanceFLWall = distanceLWall;
		}
		if (!dismiss)
		{
			Serial.print(distanceLWall);
			Serial.print(" ");
			Serial.print(distanceFLWall);
			Serial.print(" ");
			//if (abs(distanceLWall - distanceFLWall) > desiredDistance)
			//{
			//	if (distanceLWall < distanceFLWall)
			//		distanceFLWall = distanceLWall;
			//	else
			//		distanceLWall = distanceFLWall;
			//}
			double distanceWall = min(distanceFLWall, distanceLWall);
			Serial.print(distanceWall);
			Serial.print(" ");
			double error = desiredDistance - distanceWall;
			//Serial.print(error);
			//Serial.print(" ");
			double vPid = kp * error + (ki != 0)*vPidOld + ki * errorOld + kd * (error - errorOld);
			Serial.print(vPid);
			Serial.print(" ");
			if (vPid > vLim)
			{
				vPid = vLim;
				error = (vPid - (ki != 0) * vPidOld - ki * errorOld - kd * (error - errorOld)) / kp;
			}
			else if (vPid < -vLim)
			{
				vPid = -vLim;
				error = (vPid - (ki != 0) * vPidOld + ki * errorOld - kd * (error - errorOld)) / kp;
			}
			Serial.println(vPid);
			vPidOld = vPid;
			errorOld = error;

			double vLeftWheel = vBase - vPid;
			double vRightWheel = vBase + vPid;
			motorLeft.setVoltage(static_cast<int>(vLeftWheel), 1);
			motorRight.setVoltage(static_cast<int>(vRightWheel), 1);
			newSonar = false;
		}
	}
	
}


void Robot::followWallLeftNewPing(double desiredDistance, int actualDistance,
	double vBase = 100.0)
{
	motorLeft.setMode(0);
	motorRight.setMode(0);

	double kp = 2.0;
	double ki = 0;
	double kd = 0;

	double vLim = 60.0;


	double error = desiredDistance - actualDistance;
	Serial.print(error);
	Serial.print(" ");
	double vPid = kp * error + (ki != 0)*vPidOld + ki * errorOld + kd * (error - errorOld);
	Serial.print(vPid);
	Serial.print(" ");
	if (vPid > vLim)
	{
		vPid = vLim;
		error = (vPid - (ki != 0) * vPidOld - ki * errorOld - kd * (error - errorOld)) / kp;
	}
	else if (vPid < -vLim)
	{
		vPid = -vLim;
		error = (vPid - (ki != 0) * vPidOld + ki * errorOld - kd * (error - errorOld)) / kp;
	}
	Serial.println(vPid);
	vPidOld = vPid;
	errorOld = error;

	double vLeftWheel = vBase + vPid;
	double vRightWheel = vBase - vPid;
	motorLeft.setVoltage(static_cast<int>(vLeftWheel), 1);
	motorRight.setVoltage(static_cast<int>(vRightWheel), 1);
}