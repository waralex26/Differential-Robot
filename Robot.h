#ifndef ROBOT_H
#define ROBOT_H


class Robot {
	Motor motorLeft{ 1, 18, 19 };
	// The sampling time will be set later on
	Motor motorRight{ 2, 20, 21 };
	double speed;
	double angularSpeed;
	double x;
	double y;
	double omega;
	long roundROld;
	long roundLOld;
	int wheelBase;
	int wheelDiameter;
	long timing;
	int tSample;
	double speedOld;
	double speedLeftOld = 0;
	double speedRightOld = 0;
	Sonar sonarR;
	Sonar sonarL;
	Sonar sonarF;
	Sonar sonarFL;
	Sonar sonarFR;
	int sonarTurn = 1;
	long timingSonar;
	int tSonar = 100;
	double errorOld = 0;
	double vPidOld = 0;
	bool newSonar = false;
	NewPing newSonarR = NewPing(48,49);
	NewPing newSonarFR = NewPing(46, 47);
public:
	Robot(int T = 50, double xPos = 0, double yPos = 0,
		double omegaPos = 0, int base = 23, int diameter = 9);
	void countRobot();
	void goStraight(double desiredSpeed);
	void stop();
	double getSpeed() { return speed; }
	void localisation();
	void localisationSonar();
	double getX() { return x; }
	double getY() { return y; }
	double getOmega() { return omega; }
	bool goTo(double xDesired, double yDesired);
	bool getAngle(double omegaDesired);
	void actuateSonar();
	void printSonar();
	void followWallRight(double desiredDistance, double vBase = 160.0);
	void followWallLeftNewPing(double desiredDistance, int actualDistance,
		double vBase = 100.0);
};

#endif
