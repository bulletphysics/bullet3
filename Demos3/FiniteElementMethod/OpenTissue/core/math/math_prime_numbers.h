#ifndef OPENTISSUE_CORE_MATH_MATH_PRIME_NUMBERS_H
#define OPENTISSUE_CORE_MATH_MATH_PRIME_NUMBERS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <cmath>   //--- for floor and sqrt


namespace OpenTissue
{

  namespace math
  {
    
        
    namespace detail
    {
      
      /**
      * Used exclusively by the Miller-Rabin algorithm. It
      * is essential the same as modular exponentiation a
      * raised to n-1 modulo n.
      *
      * This "version" is specialized to test for notrivial
      * square roots of 1.
      */
      inline bool withness(int a,int n)
      {
        int ndec = n-1;int d  = 1; int k = ndec;
        if(k!=0)
        {
          int c=0;
          while((k&0x80000000)==0)
            k<<=1;c++;
          while(c<32)
          {
            int x =d;
            d=(d*d)%n;
            if((d==1)&&(x!=1)&&(x!=ndec))
              return true;//--- Notrival square root of 1 was discovered.
            if((k&0x80000000)!=0)
              d=(d*a)%n;
            k<<=1;c++;
          }
        }
        if(d!=1)
          return true;
        return false;
      }

    }//namespace detail


    /**
    * Prime Test.
    * This method tests if the specified integer is a prime.
    *
    * Do Always find a correct solution, but is
    * very slow for large numbers.
    *
    * Runs in time 2^(beta/2) where beta is the
    * number of bits of n.
    *
    * @param  n  The number to test.
    * @return    If n was prime then the return value is
    *            true otherwise it is false.
    */
    inline bool trial_division(int n)
    {
      using std::sqrt;
      using std::floor;

      double sqrN = sqrt(static_cast<double>(n));
      int j = static_cast<int>( floor(sqrN) );
      for(int i=2;i<=j;++i)
      {
        if((n%i)==0)
          return false;
      }
      return true;
    }

    /**
    * Modular Exponentiation.
    * Computes a raised to the power of b modular n
    */
    inline int modular_exponentiation(int a,int b,int n)
    {
      int d = 1;
      while(b!=0)
      {
        if((b&1)!=0)
          d=(d*a)%n;
        a = (a*a)%n;
        b>>=1;
      }
      return d;
    }

    /**
    * Prime Test.
    * This method tests if the specified integer is a prime.
    *
    * Extremely fast, does not allways find
    * a correct solution. However it fails
    * rarely and it only makes error of one
    * type.
    *
    * Some base-2 pseudo primes are 341, 561,645 and 1105.
    *
    * @param  n  The number to test.
    * @return    If n was prime then the return value is
    *            true otherwise it is false.
    */
    inline bool pseudo_prime(int n)
    {
      if((n==1)||(n==2))//--- trivial cases
        return true;
      if(modular_exponentiation(2,n-1,n)!=1)
        return false;//--- definitely not a prime, i.e. composite
      return true;//--- possible prime or a base-2 pseudoprime
    }


    /**
    * Closest Prime Search.
    * This method tries to find the closest
    * prime number to n.
    *
    * @param n    The seed number from where the
    *             search should be done from.
    * @return     The closest prime number to n.
    */
    inline int prime_search(int n)
    {
      int l = n;
      if((l%2)==0)
        l--;
      for(;l>1;l=l-2)
      {
        //      if(trial_division(l))
        //      if(pseudo_prime(l))
        if(miller_rabin(l,4))
          break;
      }
      int o = n;
      if((o%2)==0)
        o++;
      int max_val = detail::highest<int>();
      for(;o<max_val;o=o+2)
      {
        //      if(trial_division(o))
        //      if(pseudo_prime(o))
        if(miller_rabin(o,4))
          break;
      }
      //--- now l and o must contain two closest
      //--- primes to n such that l <= n <= o
      //--- figure out which one that is closest
      //--- to n and return it.
      if((n-l)<(o-n))
        return l;
      return o;
    }

    /**
    * Greatest Common Divisor.
    * Computes greatest common divisor with euclids algorithm.
    *
    * @param a
    * @param b
    * @return    Greatest common divisor between a and b.
    */
    inline int gcd_euclid_algorithm(int a,int b)
    {
      if(b==0)
        return a;
      else
        return gcd_euclid_algorithm(b,a%b);
    }
    
    /**
    * Tests if two numbers are relative prime.
    *
    * @return   If a and b are relative prime then the
    *           return value is true otherwise it is
    *           false.
    */
    inline bool is_relative_prime(int a,int b)
    {
      if(gcd_euclid_algorithm(a,b)==1)
        return true;
      return false;
    }

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_PRIME_NUMBERS_H
#endif
