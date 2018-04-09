#include "Arduino.h"
#include "Sonar.h"

Sonar::Sonar(int pin1, int pin2)
	: pinTrigger(pin1), pinEcho(pin2)
{
	pinMode(pinTrigger, OUTPUT);
	pinMode(pinEcho, INPUT);
}

void Sonar::computeDistance()
{
	digitalWrite(pinTrigger, LOW);
	delayMicroseconds(1);
	digitalWrite(pinTrigger, HIGH);
	delayMicroseconds(5);
	digitalWrite(pinTrigger, LOW);

	duration = pulseIn(pinEcho, HIGH);
	distance = duration / 29.0 / 2.0; // if c=340m/s
	deltaDistance = distance - distanceOld;
	if (distance == 0.29) // we havn't found an explanation yet
		distance = distanceOld;
	//if (abs(deltaDistance) > treshold && distanceOld != 0)
		//distance = distanceOld;
	else
		distanceOld = distance;
	//delayMicroseconds(50);
}