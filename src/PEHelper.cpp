#include "PEHelper.h"

#include <ctime>
#include <iostream>

#if (WIN32 && _MSC_VER <= 1800)
#include <Windows.h>
uint64_t Helper::performanceTimerFrequency;

void Helper::Init()
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	performanceTimerFrequency = freq.QuadPart;
}


uint64_t Helper::getTime()
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return counter.QuadPart;
}


double Helper::getElapsedMs(uint64_t start, uint64_t end)
{
	return double(end - start) / Helper::performanceTimerFrequency * 1000;
}

#else

void Helper::Init()
{
}

std::chrono::high_resolution_clock::time_point Helper::getTime()
{
	return std::chrono::high_resolution_clock::now();
}

double Helper::getElapsedMs(std::chrono::high_resolution_clock::time_point start, std::chrono::high_resolution_clock::time_point end)
{
 	return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1000000.0;
}
#endif

void Helper::wait()
{
    std::cin.ignore();
}

void Helper::printLine()
{
    std::cout << std::endl;
}
