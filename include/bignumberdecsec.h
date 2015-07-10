#ifndef BIGNUMBERDECSEC_H
#define BIGNUMBERDECSEC_H

#include <cstdint>
#include <string>
#include <vector>

bool BigNumberDecSecUnitTests();

class BigNumberDecSec
{
    static const uint32_t Base;
    static const int BasePower;
    friend bool BigNumberDecSecUnitTests();
    friend bool operator<( const BigNumberDecSec &first, const BigNumberDecSec &second );
    friend bool operator>( const BigNumberDecSec &first, const BigNumberDecSec &second );
    friend bool operator==( const BigNumberDecSec &first, const BigNumberDecSec &second );
    friend bool operator!=( const BigNumberDecSec &first, const BigNumberDecSec &second );
public:
    BigNumberDecSec();    
    BigNumberDecSec( const char *s );
    BigNumberDecSec( const BigNumberDecSec &other );
    ~BigNumberDecSec();

    BigNumberDecSec &operator=( const BigNumberDecSec &other );
    

    std::string toString() const;

    uint64_t toInt() const;

    void add(const BigNumberDecSec& other );
    void add(const uint64_t other );

    void multiply(const BigNumberDecSec& other );
    void multiply(const uint64_t other );

    void shiftSection( int d );
    void power( int pow );
    
    void setValue( uint64_t val );
    size_t length() const;

    int getDigit( int pos ) const;
    void setDigit( int pos, uint8_t digit );    

    uint64_t sumOfDigits() const;

private:
    void setToZero();
    size_t getValueSections() const;
    void multiplyBySection( uint32_t sec );

	void increaseSize(size_t newSize = 0); // newSize == 0 means double the allocated size
    std::vector<uint32_t> sections;
};

#endif //BIGNUMBERDECSEC_H
