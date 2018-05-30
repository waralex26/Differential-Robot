#ifndef CYLINDER_RECUPERATION_H
#define CYLINDER_RECUPERATION_H

#include "Servo.h"
#include "Arduino.h"

class CylinderRecuperation {
private:
	int m_interruptPin;
	int m_colorPin;
	Servo m_servoSort;
	Servo m_servoEmpty;
	int m_sortWhiteAngle = 90;
	int m_sortBlackAngle = 50;
	int m_openDoor = 80;
	int m_closeDoor = 120;
	int m_numberOfBlack = 0;
	int maxOfBlack = 2;
	bool m_isFull = false;
	long m_timing = millis();
	long m_timingEmpty = millis();
	bool m_isReached = false;
	bool m_isBlack;
	bool m_first = true;
public:
	CylinderRecuperation(int servoSortPin = 5, int interruptPin = 2, int colorPin = 42)
		:m_interruptPin(interruptPin), m_colorPin(colorPin)
	{
		//initialize(servoSortPin);
	}
	static bool s_isInterrupted;
	void interrupted();
	void initialize(int servoSortPin, int servoEmptyPin);
	void loop();
	void empty();
	bool goToBase();
};


#endif
