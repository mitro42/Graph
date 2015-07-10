// BigNumber class
// decimal segment implementation, base = 10^9
// bignum = bn0 +  bn1 * 10^8+ bn2 * 10^16 + bn3 * 19^24 + ...
// where 0 <= bni < 10^9
// decimal string representation is the concatenation of bnn, bnn-1, ..., bn2, bn1, bn0

#include "bignumberdecsec.h"
#include <algorithm>
#include <cmath>
#include <ctime>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>

//#include <algorithm>

const uint32_t BigNumberDecSec::Base = 1000000000;
const int BigNumberDecSec::BasePower = 9;

BigNumberDecSec::BigNumberDecSec() : sections(4,0)
{
}

BigNumberDecSec::~BigNumberDecSec()
{
}
    
BigNumberDecSec::BigNumberDecSec( const char *s )
{
    size_t strLen = strlen(s);
	for (size_t i = 0; i < strLen; ++i)
    {
        if( s[i] < '0' || s[i] > '9' )
        {
            throw "BigNumberDecSec( const char *s ): argument contains non-digits";
        }
    }

	size_t secLen = strLen / BasePower + 1;
    sections.resize(secLen * 2, 0);
    
    std::string sectStr;
    for( uint32_t i = 0; i < secLen; ++i )
    {
        uint32_t sectValue = 0;
        if( strLen > (i + 1) * BasePower )
        {
            sectStr.assign( &s[ strLen - (i + 1) * BasePower ], BasePower );
        }
        else
        {
            sectStr.assign( s, strLen % BasePower );
        }
        std::istringstream( sectStr ) >> sectValue;
        sections[ i ] = sectValue;
    }
}

BigNumberDecSec::BigNumberDecSec(const BigNumberDecSec &other) : sections(other.sections)
{
}

BigNumberDecSec &BigNumberDecSec::operator=( const BigNumberDecSec &other )
{
    if( this == &other )
    {
        return *this;
    }
    sections = other.sections;
    return *this;    
}


bool operator==( const BigNumberDecSec &first, const BigNumberDecSec &second ) 
{
    size_t vs = first.getValueSections();
    size_t ovs = second.getValueSections();
    if( vs != ovs )
        return false;
    
    return std::equal(first.sections.begin(), first.sections.begin() + vs, second.sections.begin());
}

bool operator!=( const BigNumberDecSec &first, const BigNumberDecSec &second ) 
{
    return !(first == second);
}


bool operator<( const BigNumberDecSec &first, const BigNumberDecSec &second ) 
{
    size_t vs = first.getValueSections();
    size_t ovs = second.getValueSections();
    if( vs < ovs )
        return true;

    if( vs > ovs )
        return false;

    while( vs-- > 0 )
    {
        if( first.sections[ vs ] < second.sections[ vs ] )
            return true;
        else if( first.sections[ vs ] > second.sections[ vs ] )
            return false;
    }
    return false;
}

bool operator>( const BigNumberDecSec &first, const BigNumberDecSec &second ) 
{
    return second < first;
}


void BigNumberDecSec::setToZero()
{
    std::fill(sections.begin(), sections.end(), 0);
}

void BigNumberDecSec::increaseSize(size_t newSize)
{
    size_t newAllocatedSize = 0;
    if( newSize == 0 )
    {
        newAllocatedSize = 2 * sections.size();
    }
    else if (newSize <= sections.size())
    {
        return;
    }
    else
    {
        newAllocatedSize = newSize;
    }
    
    sections.resize(newAllocatedSize);
}


void BigNumberDecSec::setValue( uint64_t val )
{
    uint32_t requiredSize = uint32_t( floor( log(double(val))/log(double(Base))) + 1);
    sections.resize(requiredSize, 0);

    uint32_t pos = 0;
    while( val > 0 )
    {
        sections[ pos ] = val % Base;
        val /= Base;
        ++pos;
    }
}

uint64_t BigNumberDecSec::toInt() const
{
    uint64_t ret = 0;
    size_t secNum = getValueSections();
    for (size_t i = 0; i < secNum; ++i)
    {
        ret *= Base;
        ret += sections[ secNum - 1 - i ];
    }

    return ret;
}

int BigNumberDecSec::getDigit(int) const
{
    throw "Not implemented";
    /*
    if( pos < allocatedSize )
    {
        return sections[ pos ];
    }
    */
    //return 0;
}

void BigNumberDecSec::setDigit(int, uint8_t) 
{    
    throw "Not implemented";
    /*
    ;
    while( pos >= allocatedSize )
    {
        increaseSize();
    }
    sections[ pos ] = digit;
    */
}

void BigNumberDecSec::add(const BigNumberDecSec& other )
{   
    size_t thisValueSections = getValueSections();
    size_t otherValueSections = other.getValueSections();
    size_t vs = thisValueSections > otherValueSections ? thisValueSections : otherValueSections;
    increaseSize(vs);

    uint64_t carry = 0;
    for (size_t i = 0; i < vs; ++i)
    {
        uint64_t toAdd = 0;
        if (i < otherValueSections)
            toAdd = other.sections[i];
        uint64_t sectionSum = sections[i] + toAdd + carry;
        sections[i] = sectionSum % Base;
        carry = sectionSum / Base;
    }
    if (carry > 0)
    {
        thisValueSections = getValueSections();
        if (thisValueSections == int(sections.size()))
        {
            sections.push_back(uint32_t(carry));
        }
        else
        {
            sections[thisValueSections] = uint32_t(carry);
        }
    }

}

void BigNumberDecSec::add( const uint64_t other )
{
    BigNumberDecSec tmp;
    tmp.setValue( other );
    add( tmp );
}


size_t BigNumberDecSec::getValueSections() const
{
    for (size_t p = sections.size(); p-- > 0;)
    {
        if( sections[ p ] != 0 )
        {
            return p + 1;
        }
    }
    return 0;
}

size_t BigNumberDecSec::length() const
{
    auto secNum = getValueSections();
    return (secNum - 1) * BasePower + size_t(floor(log10(double(sections[secNum - 1]))) + 1);
}


void BigNumberDecSec::shiftSection( int d )
{
    if( d > 0 )
    {
        // if the first d sections contain non-zero, the size of the array must be increased
        // otherwise the shift can be done in place
        
        increaseSize(getValueSections() + d);

        std::rotate(sections.begin(), sections.end() - d, sections.end());
        std::fill(sections.begin(), sections.begin() + d, 0);
    }
}

void BigNumberDecSec::multiplyBySection( uint32_t d )
{   
    if( d > Base )
        throw "multiplyBySection called with more than BasePower digits";

    size_t secNum = getValueSections();
    if( secNum + 1 >= sections.size())
    {
        increaseSize();
    }    

    uint64_t secResult = 0;
    uint32_t carry = 0;

    for (size_t i = 0; i <= secNum + 1; ++i)
    {
        secResult = uint64_t(sections[ i ]) * d + carry;
        sections[ i ] = secResult % Base;
        carry = uint32_t( secResult / Base );
    }
}

void BigNumberDecSec::multiply( const BigNumberDecSec& other )
{
    increaseSize(getValueSections() + other.getValueSections());
    
    BigNumberDecSec tmp;
    BigNumberDecSec origLeftSide( *this ); 
    setToZero();
        
    int otherSec = int(other.getValueSections());

    for (int i = 0; i < otherSec; ++i)
    {
        tmp = origLeftSide;
        tmp.multiplyBySection( other.sections[ otherSec - 1 - i ]);        
        tmp.shiftSection( int(otherSec) - 1 - i );
        
        add( tmp );        
    }
}



void BigNumberDecSec::multiply( const uint64_t other )
{
    BigNumberDecSec tmp;
    tmp.setValue( other );
    multiply( tmp );
}



uint64_t BigNumberDecSec::sumOfDigits() const
{   
    uint64_t ret = 0;
        
    for (const auto& section: sections)
    {
        uint32_t sec = section;
        while( sec != 0 )
        {
            ret += sec % 10;
            sec /= 10;
        }        
    }

    return ret;
}


std::string BigNumberDecSec::toString() const
{
    std::string ret = "";
    size_t secNum = getValueSections();
    if( secNum == 0 )
    {
        ret = "0";
        return ret;
    }
    for (size_t i = 0; i < secNum; ++i)
    {
        std::string sectionStr = "";
        uint32_t s = sections[ i ];
        for( uint32_t j = 0; j < BasePower; ++j )
        {
            sectionStr = char( '0' + s % 10 ) + sectionStr;
            s /= 10;
        }
        ret = sectionStr + ret;
    }

    uint32_t k = 0;
    while( k < ret.length() - 1 && ret[ k ] == '0' )
    {
        ++k;
    }
        
    return ret.substr( k, ret.length());
}

void BigNumberDecSec::power( int power )
{
    // power calculation is done with the powers of powers of 2
    // E.g if 7^200 = 7^(2^128+2^64+2^8) = 7^(2^128)*7^(2^64)*7^(2^8)

    BigNumberDecSec base = *this;    
    uint32_t MaxPower2 =  uint32_t( floor((log(float(power))/log(2.0)))); 
    std::vector<BigNumberDecSec> powers(MaxPower2 + 1); // power[i] == base^(2^i) and so power[0] == base;
    
    powers[0] = base;    
    for( uint32_t k = 1; k <= MaxPower2; ++k )
    {
        powers[ k ] = powers[ k - 1];
        powers[ k ].multiply( powers[ k - 1] );
    }

    setToZero();
    sections[0] = 1;

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


bool BigNumberDecSecUnitTests()
{
    bool ret = true;

    const BigNumberDecSec one = "1";
    const BigNumberDecSec two = "2";
    const BigNumberDecSec five = "5";
    const BigNumberDecSec nine = "9";
    const BigNumberDecSec ten = "10";
    const BigNumberDecSec eleven = "11";
    const BigNumberDecSec bn22 = "22";
    const BigNumberDecSec bn37 = "37";
    const BigNumberDecSec bn99 = "99";

    // init from string // convert to string
    std::string strA = "0";
    BigNumberDecSec a = "0";
    ret &= ( strA == a.toString() );
    ret &= ( 0 == a.toInt() );

    {
        std::string strA = "6543";
        BigNumberDecSec a = "6543";
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
    a.setValue( 12345678901234567890L );
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890L == a.toInt() );

    // cctor
    {
        BigNumberDecSec b = a;
        ret &= ( strA == b.toString() );
        ret &= ( 12345678901234567890L == b.toInt() );
    }
    // check that the original object have not been destroyed
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890L == a.toInt() );

    // operator=
    {
        BigNumberDecSec b;
        b = a;
        ret &= ( strA == b.toString() );
        ret &= ( 12345678901234567890L == b.toInt() );
    }
    // check that the original object have not been destroyed
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890L == a.toInt() );

    // size increase
    size_t origSize = a.sections.size();
    a.increaseSize();
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890L == a.toInt() );
    ret &= (origSize * 2 == a.sections.size());

    a.increaseSize(5);
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890L == a.toInt() );
    ret &= (origSize * 2 == a.sections.size());

    a.increaseSize(a.sections.size() + 5);
    ret &= ( strA == a.toString() );
    ret &= ( 12345678901234567890L == a.toInt() );
    ret &= (origSize * 2 + 5 == a.sections.size());
    
    // setValue 
    a.setValue( 4269 );    
    ret &= ( "4269" == a.toString() );
    ret &= ( 4269 == a.toInt() );


    // shift digit - without size increase
    /*
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
        BigNumberDecSec c;
        c.setValue( 12345678901234567890L );
        std::string tenZeroes( 10, '0' );
        c.shiftDecimalPlace( 10 );
        ret &= ( strA + tenZeroes == c.toString() );
        c.shiftDecimalPlace( 10 );
        ret &= ( strA + tenZeroes + tenZeroes == c.toString() );

        // setdigit    // getdigit
        strA = "12345678901234567890";
        c.setValue( 12345678901234567890L );    
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
    */

    // shift section - without size increase
    a.setValue( 1234 );
    a.shiftSection( 1 );
    ret &= ( "1234000000000" == a.toString() );
    ret &= ( 1234000000000L == a.toInt() );

    a.shiftSection( 0 );
    ret &= ( "1234000000000" == a.toString() );
    ret &= ( 1234000000000L == a.toInt() );
        
    // shift digit - with size increase    
    {

        strA = "12345678901234567890";
        BigNumberDecSec c;
        c.setValue( 12345678901234567890L );
        std::string z16( 18, '0' );
        c.shiftSection( 2 );
        ret &= ( strA + z16 == c.toString() );
        c.shiftSection( 4 );
        ret &= ( strA + z16 + z16 + z16 == c.toString() );

        // setdigit    // getdigit
        /*
        strA = "12345678901234567890";
        c.setValue( 12345678901234567890L );    
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
        */
    }
        
    // add - without carry 
    {
        BigNumberDecSec d = "215";

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
        BigNumberDecSec e = "215";
        e.multiplyBySection(2);
        ret &= ( 430 == e.toInt() );
        e.multiplyBySection(2);
        ret &= ( 860 == e.toInt() );
        e.multiplyBySection(2);
        ret &= ( 1720 == e.toInt() );
        e.multiplyBySection(9);
        ret &= ( 15480 == e.toInt() );
        e.multiplyBySection(9);
        ret &= ( 139320 == e.toInt() );
        e.multiplyBySection(9);
        ret &= ( 1253880 == e.toInt() );
    }

    // multiply - multiple digits
    {
        BigNumberDecSec f = "215";

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
        BigNumberDecSec f = "215";

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
        BigNumberDecSec g = "6543";
        BigNumberDecSec h = "8765";
        g.multiply( h );
        ret &= ( 6543 * 8765 == g.toInt() );
    }

    // power calculations
    {
        BigNumberDecSec m;
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

    // operator<
    {
        BigNumberDecSec a;
        BigNumberDecSec b;
        BigNumberDecSec c;
        BigNumberDecSec d;
        a.setValue( 42 );
        a.power( 31 );
        b.setValue( 2 );
        b.power( 27 );
        c = a;
        d.setValue( 4 );

        ret &= ( a > b );
        ret &= ( b < a );
        ret &= ( c > b );
        ret &= !( a > c );
        ret &= !( a < c );
        ret &= !( a < a );
        ret &= ( d < a );
        ret &= ( d < b );
        ret &= ( d < c );
        BigNumberDecSec e;
        BigNumberDecSec f;
        e.setValue( 512 );
        f.setValue( 2 );
        f.power( 27 );
        ret &= ( e < f );
        ret &= !( f < e );
        ret &= ( f > e );
    }

    // operator==
    {
        BigNumberDecSec a;
        BigNumberDecSec b;
        BigNumberDecSec c;
        BigNumberDecSec d;
        a.setValue( 46268 );
        a.power( 32 );
        b.setValue( 23134 );
        b.power( 32 );
        c.setValue( 2 );
        c.power( 32 );
        b.multiply( c );
        
        ret &= ( a == a );
        ret &= ( b == b );
        ret &= ( c == c );
        ret &= ( a == b );
        ret &= !( a != b );
        b.add( 1 );
        ret &= !( a == b );
        ret &= ( a != b );
        a.add( 1 );
        ret &= ( a == b );
        ret &= !( a != b );
        
        a.setValue(0);
        b.setValue(1);
        c.setValue(4);

        ret &= !( a != a );
        ret &= !( b != b );
        ret &= !( c != c );
        ret &= ( a != b );
        ret &= ( a != c );
        ret &= ( b != c );
    }

    // sumOfDigits
    {
        BigNumberDecSec a;
        BigNumberDecSec b;
        BigNumberDecSec c;
        BigNumberDecSec d;
        a.setValue( 123456789 );
        b = a;
        a.multiply( 10000000000 );
        

        ret &= ( a.sumOfDigits() == 45 );
        ret &= ( b.sumOfDigits() == 45 );
        a.add( b );
        ret &= ( a.sumOfDigits() == 90 );
        a.add( 100 );
        ret &= ( a.sumOfDigits() == 91 );
    }

    if( !ret )
    {
        std::cout << "BigNumberDecSec: Some of the unit tests FAILED!" << std::endl;
    }
    else
    {
        std::cout << "BigNumberDecSec: All unit tests PASSED!" << std::endl;
    }

    return ret;
}



