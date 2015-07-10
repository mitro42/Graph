#include "PEContinuedFraction.h"

#include "bignumberdecsec.h"
#include "PEMath.h"
#include <sstream>


ContinuedFraction::ContinuedFraction(): terms( 0 ), period( 0 ), length( 0 )
{
    terms.resize(10, 0);
}


long ContinuedFraction::getTerm( uint32_t n ) const
{
    if( n < length  )
    {
        return terms[ n ];
    }

    if( period != 0 )
    {
        return terms[ ( n - 1 ) % period + 1];
    }
    return 0;
}


void ContinuedFraction::setTerm( uint32_t n, long term )
{
    if (n >= terms.size())
    {
        terms.resize(n + 1);
    }

    terms[ n ] = term;
    if( n + 1 > length )
    {
        length = n + 1;
    }
}

std::string ContinuedFraction::toString() const
{    
    if (terms.empty())
    {
        return "";
    }
    
    std::stringstream s;
    s << "[";
    s << terms[0];

    if( length != 1 )
    {
        s << ";";
    }

    if( period != 0 )
    {
        s << "(";
    }
    for( uint32_t i = 1; i < length; ++i )
    {
        s << terms[ i ];
        if( i != length - 1 )
        {
            s << ",";
        }
    }

    if( period != 0 )
    {
        s << ")";
    }

    s << "]";
    return s.str();
}


void ContinuedFraction::calculate( double num )
{
    std::vector<double> r(terms.size(), 0);
    uint32_t idx = 0;
    terms[ 0 ] = long( floor( num ));
    r[ 0 ] = num - terms[ 0 ];

    bool cycleFound = false;

    while( r[ idx ] != 0 && !cycleFound )
    {        
        num = 1 / r[ idx ];
        ++idx;
        terms[ idx ] = long( floor( num ));
        r[ idx ] = num - terms[ idx ];
        auto oldTermsSize = terms.size();
        if (idx == oldTermsSize - 1 && r[idx] != 0)
        {
            terms.resize(oldTermsSize * 2, 0);
            r.resize(oldTermsSize * 2, 0);
        }

        for( uint32_t i = 0; i < idx; ++i )
        {
            if( std::abs( r[ i ] - r[ idx ] ) < eps )
            {
                period = idx;
                cycleFound = true;
            }
        }
    }
    length = idx + 1;
}


// based on http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Continued_fraction_expansion
void ContinuedFraction::calculateSquareRoot( long S )
{
    long m0 = 0;
    long d0 = 1;
    long a0 = long( floor( sqrt( double( S ))));
    long sqrtS = a0;
    long m1, d1, a1;        

    length = 1;
    terms[ 0 ] = a0;

    do 
    {
        m1 = d0 * a0 - m0;
        d1 = ( S - m1 * m1 ) / d0;
        a1 = long( floor( double( ( sqrtS + m1 ) / d1 )));

        length++;
        if( length > terms.size())
        {
            terms.resize(length);
        }
        terms[ length - 1 ] = a1;

        m0 = m1;
        d0 = d1;
        a0 = a1;

    } while( a1 != 2 * sqrtS );
    period = length - 1;      
}


void ContinuedFraction::getConvergent( int n, int64_t &numerator, int64_t &denominator ) const
{
    numerator = getTerm( n - 1 );
    denominator = 1;
    for( int i = n - 1; i > 0; --i )
    {
        int64_t temp = numerator;
        numerator = denominator;
        denominator = temp;

        numerator += denominator * getTerm( i - 1 );
    }
}


void ContinuedFraction::getConvergent( int n, BigNumberDecSec &numerator, BigNumberDecSec &denominator ) const
{
    numerator.setValue( getTerm( n - 1 ) );
    denominator.setValue( 1 );
    BigNumberDecSec temp;
    for( int i = n - 1; i > 0; --i )
    {
        temp = numerator;
        numerator = denominator;
        denominator = temp;

        temp.multiply( getTerm( i - 1 ) );
        numerator.add( temp );
    }
}
