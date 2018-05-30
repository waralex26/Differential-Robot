#ifndef ROBOT_H
#define ROBOT_H

#include "Encoder.h"
#include "Motor.h"
#include "CylinderRecuperation.h"
#include "NewPing.h"

#define SONAR_NUM 3
class Robot {
	Motor motorLeft{ 1, 18, 19 };
	// The sampling time will be set later on
	Motor motorRight{ 2, 20, 21 };
	double speed;
	double angularSpeed;
	double x;
	double y;
	double omega = 0;
	long roundROld;
	long roundLOld;
	int wheelBase;
	int wheelDiameter;
	long timing;
	int tSample;
	double speedOld;
	double speedLeftOld = 0;
	double speedRightOld = 0;
	int sonarTurn = 1;
	long timingSonar;
	int tSonar = 100;
	double errorOld = 0;
	bool newSonar = false;
	CylinderRecuperation mechanism;
	double deltaVOld = 0;
	double errorSum = 0;
	double dForward = 0;
	int sweepState = 1;

public:
	Robot(int T = 50, double xPos = 0, double yPos = 0,
		double omegaPos = 0, int base = 23, int diameter = 9);
	void countRobot();
	void goStraight(double desiredSpeed);
	void stop();
	double getSpeed() { return speed; }
	void localisation();
	double getX() { return x; }
	double getY() { return y; }
	double getOmega() { return omega; }
	void setOmega(double angle) { omega = angle; }
	double getDForward() { return dForward; }
	bool goTo(double xDesired, double yDesired);
	bool getAngle(double omegaDesired);
	void printSonar(long distance[SONAR_NUM]);
	void printSonar(double distance[SONAR_NUM]);
	void newFollowWallLeft(double distance[SONAR_NUM], 
		double desiredDistance, double vBase = 100.0);
	void newFollowWallLeft(double actualDistance,
		double desiredDistance, double vBase = 100.0,
		int a = 0);
	void setDeltaV(const float &deltaV);
	bool turnRight90(double omegaInit);
	bool turnTo(double omegaInit, double omegaFinal);
	bool sweep();
	void initialize() { mechanism.initialize(5, 6); }
	void interrupted();
};

#endif
