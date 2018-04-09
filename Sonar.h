#ifndef SONAR_H
#define SONAR_H

class Sonar {
	int pinTrigger;
	int pinEcho;
	double duration;
	double distance = 0;
	double distanceOld = 0;
	double deltaDistance = 0;
	double treshold = 30;
public:
	Sonar(int pin1, int pin2);
	void computeDistance();
	double getDistance() { return distance; }
	double getDelta() { return deltaDistance; }
};

#endif
