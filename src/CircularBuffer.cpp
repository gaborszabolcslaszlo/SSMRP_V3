
#include "ROBOT_hardware_interface/CircularBuffer.h"
#include <stdbool.h>
#include <stdlib.h>

FastFilters::AVGFilter::AVGFilter(int cnt):CircularBufferClass(cnt)
{}

FastFilters::CircularBufferClass::CircularBufferClass(int cnt)
{
	CircularElement *e = (CircularElement*)malloc(cnt*sizeof(CircularElement));

	for(int i = 0;i<cnt-1;i++)
	{
		e[i].NextElement = &e[i+1];
		e[i].Value = 0.0;
	}
	e[cnt-1].NextElement = &e[0];
	e[cnt-1].Value = 0.0;

	buffer.LastElement = &e[0];
	buffer.Sum = 0.0;
	buffer.Count = cnt;
    buffer.Avg = 0.0;    
}


void FastFilters::CircularBufferClass::AddElement( double value)
{
	buffer.Sum = buffer.Sum - buffer.LastElement->NextElement->Value;
	buffer.LastElement->NextElement->Value = value;
	buffer.Sum = buffer.Sum + buffer.LastElement->NextElement->Value;
	buffer.LastElement = buffer.LastElement->NextElement;
    buffer.Avg = (double)buffer.Sum / (double)buffer.Count;
}

double FastFilters::CircularBufferClass::GetAvg()
{
    return  buffer.Avg;
}
