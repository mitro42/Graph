// BigNumber class
// 1 decimal digit - 1 array element implementation

#include "bignumberdigarr.h"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <sstream>
#include <cmath>

BigNumberDigArr::BigNumberDigArr()
{
    digitArray.resize(50, 0);
}

    
BigNumberDigArr::BigNumberDigArr( const std::string &s )
{
    if (!std::all_of(begin(s), end(s), isdigit))
    {
        throw "BigNumberDigArr( const char *s ): argument contains non-digits";
    }

    digitArray.resize(s.size() * 2, 0);
    std::transform(rbegin(s), rend(s), begin(digitArray), [](const char& c){ return uint8_t(c - '0'); });
}


void BigNumberDigArr::setToZero()
{
    digitArray.assign(digitArray.size(), 0);
}

void BigNumberDigArr::increaseSize(size_t newRequiredSize)
{
    size_t newSize = digitArray.size();
    if (newRequiredSize == 0)
    {
        newSize = 2 * digitArray.size();
    }
    else if (newRequiredSize <= digitArray.size())
    {
        return;
    }
    else
    {
        newSize = newRequiredSize;
    }

    digitArray.resize(newSize, 0);
}


void BigNumberDigArr::setValue( uint64_t val )
{
    int requiredSize = val == 0 ? 1 : int(floor( log(double(val))/log(double(10))) + 1);
    increaseSize( requiredSize );

    setToZero();

    int pos = 0;
    while( val > 0 )
    {
        if (pos >= digitArray.size())
            throw "Not enough allocated digits";

        digitArray[ pos ] = val % 10;
        val /= 10;
        ++pos;
    }
}

uint64_t BigNumberDigArr::toInt() const
{
    uint64_t ret = 0;
    int digits = getValueDigits();
    for (int i = 0; i < digits; ++i)
    {
        ret *= 10;
        ret += digitArray[digits - 1 - i];
    }

    return ret;
}

int BigNumberDigArr::getDigit( int pos ) const
{
    if( pos < digitArray.size() )
    {
        return digitArray[ pos ];
    }
    return 0;
}

void BigNumberDigArr::setDigit(int pos, uint8_t digit)
{    
    while( pos >= digitArray.size() )
    {
        increaseSize();
    }
    digitArray[ pos ] = digit;
}

BigNumberDigArr BigNumberDigArr::reverse() const
{
    BigNumberDigArr ret( *this );

    int digits = getValueDigits();
    for (int i = 0; i < digits; ++i)
    {
        ret.digitArray[digits - 1 - i] = digitArray[i];
    }
    return ret;
}

bool BigNumberDigArr::isPalindrome() const
{
    int digits = getValueDigits();
    for (int i = 0; i < digits / 2; ++i)
    {
        if (digitArray[digits - 1 - i] != digitArray[i])
            return false;
    }
    return true;
}


void BigNumberDigArr::add(const BigNumberDigArr& other )
{   
    uint32_t thisValueDigits = getValueDigits();
    uint32_t otherValueDigits = other.getValueDigits();
    uint32_t digits = thisValueDigits > otherValueDigits ? thisValueDigits : otherValueDigits;
    while( digits >= digitArray.size() )
    {
        increaseSize();
    }

    uint32_t carry = 0;
    for( uint32_t i = 0; i <= digits; ++i )
    {
        uint32_t digitSum = digitArray[i] + other.getDigit(i) + carry;        
        digitArray[ i ] = digitSum % 10;
        carry = digitSum / 10;
    }    
}

void BigNumberDigArr::add( const uint64_t other )
{
    BigNumberDigArr tmp;
    tmp.setValue( other );
    add( tmp );
}


int BigNumberDigArr::getValueDigits() const
{
    for (auto p = digitArray.size(); p-- > 0;)
    {
        if( digitArray[ p ] != 0 )
        {
            return int(p + 1);
        }
    }
    return 0;
}

void BigNumberDigArr::shiftDecimalPlace( int d )
{
    if( d > 0 )
    {
        // if the first d digits contain non-zero, the size of the array must be increased
        // otherwise the shift can be done in place
        if (getValueDigits() + d > digitArray.size() - 1)
        {
            increaseSize(digitArray.size() + d);
        }

        std::rotate(begin(digitArray), begin(digitArray) + (digitArray.size() - d), end(digitArray));
    }
}

void BigNumberDigArr::multiplyByDigit( int d )
{   
    if( d > 9 )
        throw "multiplyByDigit called with more than one digit";

    uint32_t digits = getValueDigits();
    if (digits + 1 >= digitArray.size())
    {
        increaseSize();
    }    

    uint32_t digitResult = 0;
    uint32_t carry = 0;

    for( uint32_t i = 0; i <= digits + 1; ++i )
    {
        digitResult = digitArray[ i ] * d + carry;
        digitArray[ i ] = digitResult % 10;
        carry = digitResult / 10;        
    }
}

void BigNumberDigArr::multiply( const BigNumberDigArr& other )
{
    uint32_t digits = getValueDigits() + other.getValueDigits();
    while (digits >= digitArray.size())
    {
        increaseSize();
    }  
    
    BigNumberDigArr tmp;
    BigNumberDigArr origLeftSide( *this ); 
    setToZero();
        
    uint32_t otherLen = other.getValueDigits();

    for( uint32_t i = 0; i < otherLen; ++i )
    {
        tmp = origLeftSide;
        tmp.multiplyByDigit( other.getDigit( otherLen - 1 - i ));        
        tmp.shiftDecimalPlace( otherLen - 1 - i );
        
        add( tmp );        
    }
}


void BigNumberDigArr::multiply( const uint64_t other )
{
    BigNumberDigArr tmp;
    tmp.setValue( other );
    multiply( tmp );
}



int BigNumberDigArr::sumOfDigits() const
{
    return std::accumulate(begin(digitArray), end(digitArray), int(0));
}


std::string BigNumberDigArr::toString() const
{
    std::string ret = "";
    uint32_t digits = getValueDigits();
    if( digits == 0 )
    {
        ret = "0";
        return ret;
    }
    for( uint32_t i = 0; i < digits; ++i )
    {
        ret = char( '0' + digitArray[ i ] ) + ret;
    }

    return ret;
}

void BigNumberDigArr::power( uint32_t power )
{
    // power calculation is done with the powers of powers of 2
    // E.g if 7^200 = 7^(2^128+2^64+2^8) = 7^(2^128)*7^(2^64)*7^(2^8)

    BigNumberDigArr base = *this;
    /*
    for( uint32_t i = 2; i <= power; ++i )
    {
        multiply( base );
    }
    */

    uint32_t MaxPower2 =  uint32_t(floor((log(float(power))/log(2.0))));
        
    std::vector<BigNumberDigArr> powers(MaxPower2 + 1); // power[i] == base^(2^i) and so power[0] == base;
    
    powers[0] = base;
    
    for( uint32_t k = 1; k <= MaxPower2; ++k )
    {
        powers[ k ] = powers[ k - 1];
        powers[ k ].multiply( powers[ k - 1] );
    }

    setToZero();
    digitArray[0] = 1;

    uint32_t pos = 0;
    while( power )
    {
        if( power % 2 )
        {
            multiply( powers[pos] );
        }
        power >>= 1;
        ++pos;
    }    
}


bool BigNumberDigArrUnitTests()
{
    bool ret = true;

    const BigNumberDigArr one("1");
    const BigNumberDigArr two("2");
    const BigNumberDigArr five("5");
    const BigNumberDigArr nine("9");
    const BigNumberDigArr ten("10");
    const BigNumberDigArr eleven("11");
    const BigNumberDigArr bn22("22");
    const BigNumberDigArr bn37("37");
    const BigNumberDigArr bn99("99");

    // init from string // convert to string
    std::string strA = "0";
    BigNumberDigArr a("0");
    ret &= ( strA == a.toString() );
    ret &= ( 0 == a.toInt() );

    {
        std::string strA = "6543";
        BigNumberDigArr a("6543");
        ret &= ( strA == a.toString() );
        ret &= ( 6543 == a.toInt() );
    }

    // init from int    // convert to string
    strA = "123";
    a.setValue( 123 );
    ret &= ( strA == a.toString() );
    ret &= ( 123 == a.toInt() );

    // init from uint32_t    // convert to string
    strA = "12345678901234567890";
    a.setValue( 12345678901234567890UL );
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890UL == a.toInt() );

    // cctor
    {
        BigNumberDigArr b = a;
        ret &= ( strA == b.toString() );
        ret &= ( 12345678901234567890UL == b.toInt() );
    }
    // check that the original object have not been destroyed
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890UL == a.toInt() );

    // operator=
    {
        BigNumberDigArr b;
        b = a;
        ret &= ( strA == b.toString() );
        ret &= ( 12345678901234567890UL == b.toInt() );
    }
    // check that the original object have not been destroyed
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890UL == a.toInt() );

    // size increase
    auto origSize = a.digitArray.size();
    a.increaseSize();
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890UL == a.toInt() );
    ret &= (origSize * 2 == a.digitArray.size());

    a.increaseSize(5);
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890UL == a.toInt() );
    ret &= (origSize * 2 == a.digitArray.size());

    a.increaseSize(a.digitArray.size() + 5);
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890UL == a.toInt() );
    ret &= (origSize * 2 + 5 == a.digitArray.size());
    
    // setValue 
    a.setValue( 4269 );    
    ret &= ( "4269" == a.toString() );
    ret &= ( 4269 == a.toInt() );


    // shift digit - without size increase
    a.setValue( 1234 );
    a.shiftDecimalPlace( 1 );
    ret &= ( "12340" == a.toString() );
    ret &= ( 12340 == a.toInt() );

    a.shiftDecimalPlace( 0 );
    ret &= ( "12340" == a.toString() );
    ret &= ( 12340 == a.toInt() );
    
    a.shiftDecimalPlace( 2 );
    ret &= ( "1234000" == a.toString() );
    ret &= ( 1234000 == a.toInt() );
    
    // shift digit - with size increase    
    {
        strA = "12345678901234567890";
        BigNumberDigArr c;
        c.setValue( 12345678901234567890UL );
        std::string tenZeroes( 10, '0' );
        c.shiftDecimalPlace( 10 );
        ret &= ( strA + tenZeroes == c.toString() );
        c.shiftDecimalPlace( 10 );
        ret &= ( strA + tenZeroes + tenZeroes == c.toString() );

        // setdigit    // getdigit
        strA = "12345678901234567890";
        c.setValue( 12345678901234567890UL );    
        c.shiftDecimalPlace( 12 );
        c.setDigit( 0, 5 );
        c.setDigit( 1, 3 );
        ret &= ( 5 == c.getDigit( 0 ) );
        ret &= ( 3 == c.getDigit( 1 ) );
        ret &= ( 1 == c.getDigit( 31 ) );
        ret &= ( 2 == c.getDigit( 30 ) );
        ret &= ( strA + tenZeroes + "35" == c.toString() );
        c.setDigit( 100, 5 );
        std::string zero68( 68, '0' );
        ret &= ( "5" + zero68 + strA + tenZeroes + "35" == c.toString() );
    }
        
    // add - without carry 
    {
        BigNumberDigArr d("215");

        d.add( 1 );
        ret &= ( 216 == d.toInt() );
        d.add( one );
        ret &= ( 217 == d.toInt() );
        d.add( 10 );
        ret &= ( 227 == d.toInt() );
        d.add( 10 );
        ret &= ( 237 == d.toInt() );
        d.add( 11 );
        ret &= ( 248 == d.toInt() );
        d.add( eleven );
        ret &= ( 259 == d.toInt() );
        // add - with carry
        d.add( eleven );
        ret &= ( 270 == d.toInt() );
        d.add (1000 );
        ret &= ( 1270 == d.toInt() );
        d.add (100000000000 );
        ret &= ( 100000001270 == d.toInt() );
    }

    // multiply - one digit
    {
        BigNumberDigArr e("215");
        e.multiplyByDigit( 2 );
        ret &= ( 430 == e.toInt() );
        e.multiplyByDigit( 2 );
        ret &= ( 860 == e.toInt() );
        e.multiplyByDigit( 2 );
        ret &= ( 1720 == e.toInt() );
        e.multiplyByDigit( 9 );
        ret &= ( 15480 == e.toInt() );
        e.multiplyByDigit( 9 );
        ret &= ( 139320 == e.toInt() );
        e.multiplyByDigit( 9 );
        ret &= ( 1253880 == e.toInt() );
    }

    // multiply - multiple digits
    {
        BigNumberDigArr f("215");

        f.multiply( 22 );
        ret &= ( 215 * 22 == f.toInt() );
        f.multiply( 22 );
        ret &= ( 215 * 22 * 22 == f.toInt() );
        f.multiply( 37 );
        ret &= ( 215 * 22 * 22 * 37 == f.toInt() );
        f.multiply( 99 );
        ret &= ( 215 * 22 * 22 * 37 * 99 == f.toInt() );
    }

    {   
        BigNumberDigArr f("215");

        f.multiply( bn22 );
        ret &= ( 215 * 22 == f.toInt() );
        f.multiply( bn22 );
        ret &= ( 215 * 22 * 22 == f.toInt() );
        f.multiply( bn37 );
        ret &= ( 215 * 22 * 22 * 37 == f.toInt() );
        f.multiply( bn99 );
        ret &= ( 215 * 22 * 22 * 37 * 99 == f.toInt() );
    }

    // multiply - with size increase
    {
        BigNumberDigArr g("6543");
        BigNumberDigArr h("8765");
        g.multiply( h );
        ret &= ( 6543 * 8765 == g.toInt() );
    }

    // power calculations
    {
        BigNumberDigArr m;
        m.setValue( 2 );
        m.power( 2 );
        ret &= ( 4 == m.toInt() );
        m.setValue( 2 );
        m.power( 4 );
        ret &= ( 16 == m.toInt() );
        m.setValue( 2 );
        m.power( 10 );
        ret &= ( 1024 == m.toInt() );        
        m.power( 2 );
        ret &= ( 1024 * 1024 == m.toInt() );

        m.setValue( 2 );
        m.power( 30 );
        ret &= ( 1024 * 1024 * 1024 == m.toInt() );

        m.setValue( 11 );
        m.power( 2 );
        ret &= ( 121 == m.toInt() );
        m.power( 2 );
        ret &= ( 121 * 121 == m.toInt() );

        m.setValue( 12 );
        m.power( 10 );
        ret &= ( 61917364224L  == m.toInt() );
    }

    // reverse
    {
        BigNumberDigArr o;
        o.setValue( 0 );
        ret &= o.reverse().toInt() == 0;
        o.setValue( 1 );
        ret &= o.reverse().toInt() == 1;
        o.setValue( 6 );
        ret &= o.reverse().toInt() == 6;
        o.setValue( 10 );
        ret &= o.reverse().toInt() == 1;
        o.setValue( 345 );
        ret &= o.reverse().toInt() == 543;
        o.setValue( 123456789 );
        
        ret &= o.reverse().toInt() == 987654321;        
    }

    // isPalindrome
    {
        BigNumberDigArr o;
        o.setValue( 0 );
        ret &= o.isPalindrome();
        o.setValue( 1 );
        ret &= o.isPalindrome();
        o.setValue( 12 );
        ret &= !o.isPalindrome();
        o.setValue( 32 );
        ret &= !o.isPalindrome();
        o.setValue( 22 );
        ret &= o.isPalindrome();
        o.setValue( 212 );
        ret &= o.isPalindrome();
        o.setValue( 412 );
        ret &= !o.isPalindrome();
        o.setValue( 123454321 );
        ret &= o.isPalindrome();
        o.setValue( 1234554321 );
        ret &= o.isPalindrome();
        o.setValue( 1234564321 );
        ret &= !o.isPalindrome();
    }

    if( !ret )
    {
        std::cout << "BigNumberDigArrUnitTests: Some of the unit tests FAILED!" << std::endl;
    }
    else
    {
        std::cout << "BigNumberDigArrUnitTests: All unit tests PASSED!" << std::endl;
    }

    return ret;
}
