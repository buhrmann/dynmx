/*
 *  MathUtils.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
 
#ifndef _MATH_UTILS_
#define _MATH_UTILS_

#include "Dynmx.h"
#include <math.h>
#include <numeric>
#include <sstream>

const float PI = 3.1415926535897932384626433832795f;
const float TWO_PI = 6.283185307179586f;
const float PI_OVER_180 = 0.017453292519943295769236907684886f;
const float PI_OVER_180_REC = 57.295779513082325f;
const float PI_OVER_TWO = 1.570796326794897f;
const float PI_OVER_FOUR = 0.785398163397448f;

const float DEG_TO_RAD = PI / 180.0f;
const float RAD_TO_DEG = 180.0f / PI;

//----------------------------------------------------------------------------------------------------------------------
#ifdef DYNMX_WIN
static const float asinh(float v)
{
  return log(v + sqrt(v*v + 1));
}
#endif

//----------------------------------------------------------------------------------------------------------------------
static const float sqr(float v) { return v * v; };
static const double sqr(double v) { return v * v; };

//----------------------------------------------------------------------------------------------------------------------
static const float degreesToRadians(float deg) { return deg * DEG_TO_RAD; };

//----------------------------------------------------------------------------------------------------------------------
static const float radiansToDegrees(float rad) { return rad * RAD_TO_DEG; };

//----------------------------------------------------------------------------------------------------------------------
template<class T>
static inline T clamp(T Value, T Min, T Max)
{
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

// Vector sum
//----------------------------------------------------------------------------------------------------------------------
template<class T>
static inline T sum(std::vector<T>& vec)
{
  T sum = std::accumulate(vec.begin(), vec.end(), (T)0); 
  return sum;
}

// Vector mean
//----------------------------------------------------------------------------------------------------------------------
template<class T>
static inline T mean(std::vector<T>& vec)
{
  return sum(vec) / (T) vec.size();
}

// Element-wise sum of two vectors
//----------------------------------------------------------------------------------------------------------------------
template<class T>
static inline T elemWiseSum(std::vector<T>& vec1, std::vector<T>& vec2)
{
  std::vector<T> res;
  res.resize(vec1.size()); 
  return std::transform(vec1.begin(), vec1.end(), vec2.begin(), res.begin(), std::plus<T>());
}

// Cross-correlation of two vectors
// (see http://paulbourke.net/miscellaneous/correlate/)
//----------------------------------------------------------------------------------------------------------------------
template<class T>
struct SqrDist
{
  double operator() (T t1, T t2) { return sqr(t1-t2); };
};


template<class T>
static inline std::vector<double> crossCorrelation(std::vector<T>& v1, std::vector<T>& v2, int minDelay, int maxDelay)
{  
  return crossCorrelation(v1, v2, SqrDist<T>(), minDelay, maxDelay);
}

// Cross-correlation of two vectors
// (see http://paulbourke.net/miscellaneous/correlate/)
//----------------------------------------------------------------------------------------------------------------------
template<class T, class Operator>
static inline std::vector<double> crossCorrelation(std::vector<T>& v1, std::vector<T>& v2, Operator operate, int minDelay, int maxDelay)
{
  assert(maxDelay > minDelay);
  
  int N = v1.size();
  
  // Calculate the correlation series
  std::vector<double> crossCor;
  for (int delay = minDelay; delay < maxDelay; delay++) 
  {
    double corr = 0;
    for (int i = 0; i < N; i++) 
    {
      int j = i + delay;
      // For out of range values extend with first or last value
      T v2Val = j < 0 ? v2[0] : j < N ? v2[j] : v2[N-1];
      double val = operate(v1[i], v2Val);
      corr += val;
    }
    crossCor.push_back(corr);
  }
  
  return crossCor;
}
  

// Cross-correlation of two vectors
// (see http://paulbourke.net/miscellaneous/correlate/)
//----------------------------------------------------------------------------------------------------------------------
template<class T>
static inline std::vector<T> crossCorrelationNormalised(std::vector<T>& v1, std::vector<T>& v2, int maxDelay)
{
  int N = v1.size();
  
  T m1 = mean(v1);
  T m2 = mean(v2);
  
  // Calculate the denominator
  T s1 = 0;
  T s2 = 0;
  for (int i = 0;i < N; i++) 
  {
    s1 += sqr(v1[i] - m1);
    s2 += sqr(v2[i] - m2);
  }
  T denom = sqrt(s1 * s2);  
  
  // Calculate the correlation series
  std::vector<T> crossCor;
  for (int delay = -maxDelay; delay < maxDelay; delay++) 
  {
    T s12 = 0;
    for (int i = 0; i < N; i++) 
    {
      int j = i + delay;
      if (j >= 0 && j < N)
      {
        s12 += (v1[i] - m1) * (v2[j] - m2);
      }
      else 
      {
        s12 += (v1[i] - m1) * (0 - m2);  
      }

    }
    crossCor.push_back(s12 / denom);
  }
}

// Smooth step according to Perlin (http://en.wikipedia.org/wiki/Smoothstep)
//----------------------------------------------------------------------------------------------------------------------
static const float smoothStep(float min, float max, float x)
{
  // Scale, and clamp x to 0..1 range
  x = clamp((x - min) / (max - min), 0.0f, 1.0f);
  // Evaluate polynomial
  return x*x*x*(x*(x*6 - 15) + 10);
}

static const float smoothStepUnitInterval(float x)
{
  return x*x*x*(x*(x*6 - 15) + 10);
}

static const double smoothStep(double min, double max, double x)
{
  // Scale, and clamp x to 0..1 range
  x = clamp((x - min) / (max - min), 0.0, 1.0);
  // Evaluate polynomial
  return x*x*x*(x*(x*6 - 15) + 10);
}

static const double smoothStepUnitInterval(double x)
{
  return x*x*x*(x*(x*6 - 15) + 10);
}

//----------------------------------------------------------------------------------------------------------------------
template <class T>
static inline std::string toString (T& t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}

//----------------------------------------------------------------------------------------------------------------------
static const void secondsToTime(int time, int& hours, int& min, int& sec)
{
  hours = time / 3600;
  time = time % 3600;
  min = time / 60;
  time = time % 60;
  sec = time;  
}

//----------------------------------------------------------------------------------------------------------------------
// Fast random number generator from Numerical Recipes in C					            	
//----------------------------------------------------------------------------------------------------------------------
#define IAA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define EPS 1.2e-7
#define RNMX (1.0-EPS)

//----------------------------------------------------------------------------------------------------------------------
static double ran1(long *idum)
{
	int j;
	long k;
	static long iy=0;
	static long iv[NTAB];
	double temp;
	if (*idum <= 0 || !iy) 
  {
		if (-(*idum) < 1) 
      *idum = 1;
		else 
      *idum = -(*idum);
		for (j=NTAB+7; j>=0; j--) 
    {
			k = (*idum)/IQ;
			*idum=IAA*(*idum-k*IQ)-IR*k;
			if (*idum <0) 
        *idum += IM;
			if (j < NTAB) 
        iv[j] = *idum;
		}
		iy = iv[0];
	}
	k = (*idum)/IQ;
	*idum=IAA*(*idum-k*IQ)-IR*k;
	if (*idum < 0) 
    *idum += IM;
	j = iy/NDIV;
	iy=iv[j];
	iv[j] = *idum;
	if ((temp=AM*iy) > RNMX) 
    return RNMX;
	else 
    return temp;
}

//----------------------------------------------------------------------------------------------------------------------
static double gasdev(long *idum)
{
	static int iset=0;
	static double gset;
	double fac,rsq,v1,v2;
	if (iset == 0) 
  {
		do 
    {
			v1=2.0*ran1(idum)-1.0;
			v2=2.0*ran1(idum)-1.0;
			rsq=v1*v1+v2*v2;
		} while (rsq >= 1.0 || rsq==0.0);
		fac=sqrt(-2.0*log(rsq)/rsq);
		gset=v1*fac;
		iset=1;
		return v2*fac;
	} 
  else 
  {
		iset =0;
		return gset;
	}
}

#endif // _MATH_UTILS_