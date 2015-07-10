
#include "PEMath.h"
#include <algorithm>


uint32_t allOnes[] = {
    (1 << 0)- 1, // 0
    (1 << 1) - 1, // 1
    (1 << 2) - 1, // 3
    (1 << 3) - 1, // 7
    (1 << 4) - 1, // 15
    (1 << 5) - 1, // 31
    (1 << 6) - 1, // 63
    (1 << 7) - 1, // 127
    (1 << 8) - 1, // 255
    (1 << 9) - 1, // 511
    (1 << 10) - 1, // 1023
    (1 << 11) - 1, // 4047
    (1 << 12) - 1, // 4095
    (1 << 13) - 1, // 8191
    (1 << 14) - 1, // 16383
    (1 << 15) - 1,
    (1 << 16) - 1,
    (1 << 17) - 1,
    (1 << 18) - 1,
    (1 << 19) - 1,
    (1 << 20) - 1,
    (1 << 21) - 1,
    (1 << 22) - 1,
    (1 << 23) - 1,
    (1 << 24) - 1,
    (1 << 25) - 1,
    (1 << 26) - 1,
    (1 << 27) - 1,
    (1 << 28) - 1,
    (1 << 29) - 1,
    (1 << 30) - 1
};
    
const uint32_t MAX_POW10 = 20;

uint64_t POW10[MAX_POW10] = {
    1UL,
    10UL,
    100UL,
    1000UL,
    10000UL,
    100000UL,
    1000000UL,
    10000000UL,
    100000000UL,
    1000000000UL,
    10000000000UL,
    100000000000UL,
    1000000000000UL,
    10000000000000UL,
    100000000000000UL,
    1000000000000000UL,
    10000000000000000UL,
    100000000000000000UL,
    1000000000000000000UL,
    10000000000000000000UL
};


uint64_t getAlphabeticalValue( const std::string &s )
{
    uint64_t ret = 0;
    for( uint32_t i = 0; i < s.length(); ++i )
    {
        ret += uint32_t(s[i] - 'A') + 1;
    }
    return ret;
}


uint64_t getTriangleNumber( uint32_t n ) // Unit test OK
{
    return (uint64_t)(1 + n) * n / 2;
}

uint64_t getSquareNumber( uint32_t n ) // Unit test OK
{
    return (uint64_t)( n * n );
}

uint64_t getPentagonalNumber( uint32_t n ) // Unit test OK
{
    return (uint64_t)( n * (3*n - 1)) / 2;
}

uint64_t getHexagonalNumber( uint32_t n ) // Unit test OK
{
    return (uint64_t)( n * (2*n - 1));
}

uint64_t getHeptagonalNumber( uint32_t n ) // Unit test OK
{
    return (uint64_t)( n * (5*n - 3) / 2 );
}

uint64_t getOctagonalNumber( uint32_t n ) // Unit test OK
{
    return (uint64_t)( n * (3*n - 2));
}

uint64_t getPolygonalNumber( uint32_t p, uint32_t n )
{
    switch( p )
    {
        case 3: return getTriangleNumber( n ); 
        case 4: return getSquareNumber( n ); 
        case 5: return getPentagonalNumber( n ); 
        case 6: return getHexagonalNumber( n ); 
        case 7: return getHeptagonalNumber( n ); 
        case 8: return getOctagonalNumber( n ); 
        default: throw "Illegal polynomial";
    }
}



bool isTriangleNumber( double t ) // Unit test OK
{
    double n = sqrt( 2*t + 0.25 ) - 0.5;
    return std::abs( floor( n + 0.5 ) - n ) < eps;
}

bool isSquareNumber( double s ) // Unit test OK
{
    double n = sqrt( double( s ) );
    return std::abs( floor( n + 0.5 ) - n ) < eps;
}

bool isPentagonalNumber( double p ) // Unit test OK
{
    double n = (sqrt( 24*p + 1 ) + 1) / 6;
    return std::abs( floor( n +0.5 ) - n ) < eps;
}

bool isHexagonalNumber( double h ) // Unit test OK
{
    double n = (sqrt( h + 1/8.0 ) + 1/sqrt(8.0)) / sqrt(2.0);
    return std::abs( floor( n + 0.5 ) - n ) < eps;
}

bool isHeptagonalNumber( double h ) // Unit test OK
{    
    double n = ( sqrt( 2 * h + 9.0 / 20.0 )  + sqrt( 9.0 / 20.0 )) / sqrt( 5.0 );
    return std::abs( floor( n + 0.5 ) - n ) < eps;
}

bool isOctagonalNumber( double o ) // Unit test OK
{    
    double n = (sqrt( o + 1.0 /3.0 ) + 1.0 / sqrt( 3.0 ) ) / sqrt( 3.0 );
    return std::abs( floor( n + 0.5 ) - n ) < eps;
}


bool isPolygonalNumber( uint32_t p, double n )
{
    switch( p )
    {
        case 3: return isTriangleNumber( n ); 
        case 4: return isSquareNumber( n ); 
        case 5: return isPentagonalNumber( n ); 
        case 6: return isHexagonalNumber( n ); 
        case 7: return isHeptagonalNumber( n ); 
        case 8: return isOctagonalNumber( n ); 
        default: throw "Illegal polynomial";
    }
}


////////////////////  NOT TESTED SECTION BEGIN //////////////////////////////

// Known to be incorrect due to floating point arithmetic in some cases
// E.g. log_10(1000) == 3, but log(1000)/log(10) == 2.999999999996
// Can happen with other values
uint32_t numberOfDigits( uint64_t a, double base ) 
{    
    if( a == 0 ) return 1;
    if (base == 10)
        return numberOfDigits(a);
    return uint32_t(log(double(a))/(log(base))) + 1;  // slow
}



bool hasDuplicateDigits( uint64_t a )
{
    uint64_t digits = 0;

    while( a > 0 )
    {
        uint8_t d = a % 10;
        uint64_t p2 = 1 << (d - 1);
        if( digits & p2 )
            return true;

        digits |= p2;
        a /= 10;
    }
    return false;
}


void countDigits(uint64_t n, std::vector<int> &digitCounter)
{
    digitCounter.assign(10, 0);

    while (n > 0)
    {
        uint8_t d = n % 10;
        digitCounter[d]++;
        n /= 10;
    }
}



std::vector<int> countDigits(uint64_t n)
{
    std::vector<int> digitCounter(10, 0);
    countDigits(n, digitCounter);
    return digitCounter;
}



////////////////////  NOT TESTED SECTION END //////////////////////////////

uint64_t rotateNumberRight( uint64_t n ) // Unit test OK
{
    uint32_t d = n % 10;
    return d * POW10[ numberOfDigits( n ) - 1 ] +  n / 10;
}


uint64_t rotateNumberLeft( uint64_t n ) // Unit test OK
{
    uint64_t p = POW10[ numberOfDigits( n ) - 1 ];
    if( p == 0 )
        return 0;

    uint64_t d = n / p;
    return n % p * 10 + d;
}

uint64_t truncateNumberRight( uint64_t n, uint32_t digits ) // Unit test OK
{
    return n / POW10[ digits ];
}

uint64_t truncateNumberLeft( uint64_t n, uint32_t digits ) // Unit test OK
{
    uint64_t dn = numberOfDigits( n );
    if( dn >= digits )
    {
        return n % POW10[ dn - digits];
    }

    return 0;
}


uint32_t numberOfDigits( uint64_t a ) // Unit test OK
{
    uint32_t i = 0;
    if( a == 0 ) return 1;

    while( i < MAX_POW10 && !(a >= POW10[i] && a < POW10[i+1]) ) 
        i++;
    uint32_t ret = std::min( i + 1, MAX_POW10 );
    return ret;
}


bool isPythagoreanTriplet( int a, int b, int c )
{
    return ( a*a + b*b == c*c );
}


bool isPalindromicNumber( uint64_t n, uint32_t base ) // Unit test OK
{
    std::string numstr(32, 0);
    uint64_t len = ((n == 0)?1:0);

    while (n > 0)
    {
        uint32_t d = n % base;
        numstr[len++] = char('0' + d);
        n /= base;
    }

    for( uint32_t i = 0; i < len / 2 + 1; ++i )
    {
        if( numstr[ i ] != numstr[ len - 1 - i ] )
            return false;
    }

    return true;
}


uint32_t getDigit( uint64_t n, uint32_t d )  // Unit test OK
{
	uint32_t numD = numberOfDigits(n);
	if (numD < d)
		return 0;

	uint32_t ret = uint32_t(truncateNumberRight(n, numD - d) % 10);
    return ret;
}


uint64_t setDigit( uint64_t n, uint32_t d, uint32_t newDigit )
{
    if( newDigit > 9 || d == 0 ) 
        return n;

    uint32_t len = numberOfDigits( n );

    if( d > len )
        return n;

    uint32_t oldDigit = getDigit( n, d );    
    return n - ( oldDigit * POW10[ len - d ] ) + ( newDigit * POW10[ len - d ] );
}


uint64_t gcd( uint64_t a, uint64_t b ) // Unit test OK
{
    uint64_t r1 = a;
    uint64_t r2 = b;

    while( r1 != 0 && r2 != 0 )
    {
        r1 = r1 % r2;

        if( r1 == 0 )
            break;

        r2 = r2 % r1;
    }

    if( r1 == 0 )
        return r2;

    if( r2 == 0 )
        return r1;

    throw "WTF";
}



bool isPanDigital( uint64_t a ) // Unit test OK
{
    uint64_t digits = 0;
    uint64_t len = numberOfDigits( a );

    while( a > 0 )
    {
        uint8_t d = a % 10;
        digits |= 1LL << (d - 1 );
        a /= 10;
    }

    return ( digits == allOnes[ len ] );
}


bool isPanDigitalFromZero( uint64_t a ) // Unit test OK
{
    if( a == 0 )
    {
        
        return true;
    }

    uint64_t digits = 0;
    uint64_t len = numberOfDigits( a );

    while( a > 0 )
    {
        uint8_t d = a % 10;
        digits |= 1LL << d;
        a /= 10;
    }

    return ( digits == allOnes[ len ] );
}
    
bool isPanDigital( uint64_t a, uint64_t b ) // Unit test OK
{
    uint64_t lenB = numberOfDigits( b );
    return isPanDigital( a * POW10[lenB] + b );
}

bool isPanDigital( uint64_t a, uint64_t b, uint64_t c ) // Unit test OK
{
    //uint64_t lenA = numberOfDigits( a );
    uint64_t lenB = numberOfDigits( b );
    uint64_t lenC = numberOfDigits( c );    
    return isPanDigital( a * POW10[lenB + lenC] + b * POW10[lenC] + c );
}

bool isPanDigital( uint64_t a, uint64_t b, uint64_t c, uint64_t digits ) // Unit test OK
{
    uint64_t lenA = numberOfDigits( a );
    uint64_t lenB = numberOfDigits( b );
    uint64_t lenC = numberOfDigits( c );    
    return isPanDigital( a * POW10[lenB + lenC] + b * POW10[lenC] + c ) && digits == (lenA + lenB + lenC );
}




//////////////////////////////////
// Unit tests
//////////////////////////////////

