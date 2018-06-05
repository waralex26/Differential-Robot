#ifndef BUFFER_H
#define BUFFER_H
#include "Arduino.h"

class Buffer {
private:
	int m_length = 0;
	int m_numberOfElement = 0;
	double *m_buffer = NULL;
	double m_sum = 0;
	bool m_first = true;
public:
	Buffer(int length);
	void push(double value);
	int getLength() { return m_length; }
	double filter();
	void pushFilter(double value);
	void pushFilter2(double value);
	double average();
	double &operator[](int index);
	bool isFull();
	void reset();
	~Buffer();
};


#endif
