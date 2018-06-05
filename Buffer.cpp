#include "Buffer.h"

Buffer::Buffer(int length)
	:m_length(length)
{
	m_buffer = new double[m_length] {0};
}
void Buffer::push(double value)
{
	for (int count = m_length - 1; count>0; --count)
	{
		m_buffer[count] = m_buffer[count - 1];
	}
	if (m_numberOfElement<m_length)
		++m_numberOfElement;
	m_buffer[0] = value;
}

double Buffer::filter()
{
	return 0.5 * m_buffer[0] + 0.25*m_buffer[1] + 0.25*m_buffer[2];
}

void Buffer::pushFilter(double value)
{
	for (int count = m_length - 1; count>0; --count)
	{
		m_buffer[count] = m_buffer[count - 1];
	}
	if (m_numberOfElement<m_length)
		++m_numberOfElement;
	if (m_numberOfElement >= 3)
	{
		m_buffer[0] = value * 0.5 + 0.35 * m_buffer[1] +
			0.15 * m_buffer[2];
	}
	else
		m_buffer[0] = value;
}


void Buffer::pushFilter2(double value)
{
	if (!m_first)

	{
		double valueOut = m_buffer[m_length - 1];
		for (int count = m_length - 1; count > 0; --count)
		{
			m_buffer[count] = m_buffer[count - 1];
		}
		if (m_numberOfElement < m_length)
		{
			++m_numberOfElement;
			m_sum += value;
		}
		else
		{
			m_sum += value;
			m_sum -= valueOut;
		}
		m_buffer[0] = m_sum / m_numberOfElement;
	}
	else m_first = false;
}
double Buffer::average()
{
	int limit = 10;
	double dummy = 0;
	if (m_length < 5)
		limit = m_length;

	for (int count = 0; count < limit; ++count)
	{
		if (m_buffer[count] != 0)
			dummy += m_buffer[count];
	}
	return dummy / m_numberOfElement;
}
bool Buffer::isFull()
{
	return m_numberOfElement >= m_length;
}
void Buffer::reset()
{
	for (int count = 0; count < m_numberOfElement; ++count)
	{
		m_buffer[count] = 0;
	}
	m_numberOfElement = 0;
	m_first = true;
}

double &Buffer::operator[](int index)
{
	return m_buffer[index];
}
Buffer::~Buffer()
{
	delete[] m_buffer;
}