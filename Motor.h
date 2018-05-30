#ifndef MOTOR_H
#define MOTOR_H

class Motor {
	double rpm;
	long timeOld;
	long count;
	double speedWheel;
	long positionOld;
	long positionNew;
	long countOld;
	long timingOld;
	int nummerMotor;
	long timingMotor;
	double voltage;
	bool stopped;
	long tSample;
	double errorOld;
	double voltageOld;
	bool isIncreased;
	Encoder motorEncoder;
	double motorSpeed = 0;
	int motorDirection = 0;
	int mode = 1;

public:
	Motor(int whichMotor, int encoderPin1, int encoderPin2, long T = 50);
	void setSamplingTime(int T) { tSample = T; }
	long readEncoder() { return motorEncoder.read(); }
	bool countMotor();
	double speedometer();
	double getSpeed() { return speedWheel; }
	void motorRun(int voltage, int desiredDirection = 1);
	void motorStop();
	void controllerPid(double desiredSpeed);
	void setSpeed(double desiredSpeed);
	void setVoltage(double v, int desiredDirection);
	void setVoltage(double v);
	void setMode(int desiredMode) { mode = desiredMode;}
	void increase(bool value) { isIncreased = value; }
};


#endif

