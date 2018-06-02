#include "Arduino.h"
#include "CylinderRecuperation.h"


bool CylinderRecuperation::s_isInterrupted{ false };

void CylinderRecuperation::interrupted() {
	
	
	//Serial.println(digitalRead(m_interruptPin));
	if (digitalRead(m_interruptPin) == 1)
	{
		if (m_first)
		{
			m_isBlack = true;
			m_first = false;
		}
		if (digitalRead(m_colorPin) == 1)
		{
			m_servoSort.write(m_sortWhiteAngle);
			m_isBlack = false;
			m_timing = millis();
		}
	}
	else
	{
		if (m_isBlack)
			++m_numberOfBlack;
		if (m_numberOfBlack >= maxOfBlack)
			m_isFull = true;
		s_isInterrupted = false;
		m_first = true;
	}
}

void CylinderRecuperation::initialize(int servoSortPin,
	int servoEmptyPin)
{
	m_servoSort.attach(servoSortPin);
	m_servoSort.write(m_sortBlackAngle);
	m_servoEmpty.attach(servoEmptyPin);
	m_servoEmpty.write(m_closeDoor);
}

void CylinderRecuperation::loop()
{
	if (s_isInterrupted)
		interrupted();

	if (millis() > m_timing + 2000)
		m_servoSort.write(m_sortBlackAngle);

	if (m_isFull)
		empty();

	if (millis() > m_timingEmpty + 6000)
		m_servoEmpty.write(m_closeDoor);

	//Serial.println(m_numberOfBlack);
}

void CylinderRecuperation::empty()
{
	//Serial.println(m_isReached);
	if (!m_isReached)
	{
		*m_statePtr = 5;
		m_isReached = goToBase();
		//Serial.println(m_isReached);
	}
}

bool CylinderRecuperation::goToBase()
{
	bool isReached = true;
	return isReached;
}

void CylinderRecuperation::openDoor()
{
	//Serial.println(m_isReached);
	m_servoEmpty.write(m_openDoor);
	m_timingEmpty = millis();
	m_isFull = false;
	m_numberOfBlack = 0;
	m_isReached = false;
}