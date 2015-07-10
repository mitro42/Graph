#ifndef PE_HELPER_H
#define PE_HELPER_H
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
// Print functions
class Helper
{
#if (WIN32 && _MSC_VER <= 1800)
private:
	static uint64_t performanceTimerFrequency;
public:
	static uint64_t getTime();
	static double getElapsedMs(uint64_t start, uint64_t end);
#else
public:
	static std::chrono::high_resolution_clock::time_point getTime();
	static double getElapsedMs(std::chrono::high_resolution_clock::time_point start, std::chrono::high_resolution_clock::time_point end);
#endif

public:
    static void Init();
    static void wait();
    static void printLine();

    template<typename T> static void printLine(T head)
    {
        std::cout << head << std::endl;
    }

    template<typename Th, typename Td> static void printLine(Th head, Td data)
    {
        std::cout << head << " " << data << std::endl;
    }
};
#endif //PE_HELPER_H
