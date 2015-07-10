#ifndef BIGNUMBERDIGARR_H
#define BIGNUMBERDIGARR_H

#include <cstdint>
#include <string>
#include <vector>

bool BigNumberDigArrUnitTests();

class BigNumberDigArr
{
    friend bool BigNumberDigArrUnitTests();
public:
    BigNumberDigArr();    
    explicit BigNumberDigArr(const std::string &s);
    BigNumberDigArr(const BigNumberDigArr &other) = default;
    ~BigNumberDigArr() = default;

    BigNumberDigArr &operator=(const BigNumberDigArr &other) = default;

    std::string toString() const;

    uint64_t toInt() const;

    void add(const BigNumberDigArr& other );
    void add(const uint64_t other );

    void multiply(const BigNumberDigArr& other );
    void multiply(const uint64_t other );

    void multiplyByDigit( int d );
    void shiftDecimalPlace( int d );
    void power( uint32_t pow );
    
    void setValue( uint64_t val );
    int length() const { return getValueDigits(); }

    int getDigit( int pos ) const;
    void setDigit(int pos, uint8_t digit);
    BigNumberDigArr reverse() const;
    bool isPalindrome() const;

    int sumOfDigits() const;

private:
    //void stripZeroesAndSet( const std::string &s );
    void setToZero();
    int getValueDigits() const; 

    void increaseSize(size_t newRequiredSize = 0); // newRequiredSize == 0 means double the allocated size

    std::vector<uint8_t> digitArray;
};

#endif //BIGNUMBERDIGARR_H
