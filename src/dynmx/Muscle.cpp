/*
 *  Muscle.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Muscle.h"
#include "ArmMuscled.h"
#include "MathUtils.h"

namespace dmx
{

// Parameters describing the Hill curve
//----------------------------------------------------------------------------------------------------------------------
static const float Ksh = 0.25f;  // shape parameter for shortening = a/F0 = b/vmax
static const float Klen = 0.25f; // shape parameter for lengthening
static const float Kmax = 1.5f;  // lenghthening asymptote
static const float Kslope = 2.0f; // multiple of shortening slope at v=0
static const float Ksub = (Klen / Kslope) * ((1 - Kmax) / (1 + Klen));
  
// Width (in terms of normalised muscle length) of the parabola describing active force generation.
// I.e. beyond normalised lengths of 1+width or 1-width, no force will be generated. 
static const float activeForceWidth = 0.5;
// Normalised length beyond which the passive elastic element generates force
static const float passiveForceSlackLength = 1.4;
// Normalised amount of passive force at the length of activeForceWidth, i.e. where active force becomes 0.
static const float passiveForceAtWidth = 0.5;

//----------------------------------------------------------------------------------------------------------------------  
void Muscle::init()
{
  // assume initial setup represents rest configuration
  //m_lengthOpt = m_length;
  
  // Calculate passiveForceGain, such that it is 0.5*F0 at maximum length (where active force is 0)
  // Assumes the passive force is calculated as a scaled parabola, see below.
  m_passiveForceGain = passiveForceAtWidth / sqr((1+activeForceWidth) - passiveForceSlackLength);
  
  m_tauAct = 0.04;
  m_tauDeact = 0.07;
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------  
void Muscle::reset()
{
  updateLengthAndMomentArm();
  
  m_lengthNorm = m_length /  m_lengthOpt;
  m_velocityNorm = m_velocity / m_maxVelocity;
  
  m_passiveForceNorm = 0.0;
  m_activeForceNorm = 0.0;
  m_velocityForceNorm = 0.0;  
  
  m_velocity = 0.0; 
  m_excitation = 0.0;  
  m_activation = 0.0;    
}

//----------------------------------------------------------------------------------------------------------------------  
void Muscle::setParameters(double maxIsoForce, double optimalLength, double maxVelocity)
{
  m_lengthOpt = optimalLength;
  m_maxVelocity = maxVelocity * m_lengthOpt;
  m_maxForce = maxIsoForce;
}


//----------------------------------------------------------------------------------------------------------------------
void Muscle::update(float dt)
{
  double prevLength = m_length;

  updateLengthAndMomentArm();

  m_velocity = (m_length - prevLength) / dt;
  
  // Dimensionless variables for Hill-model
  m_lengthNorm = m_length /  m_lengthOpt;
  m_velocityNorm = m_velocity / m_maxVelocity;
  
  // Calculate individual constitutive forces
  m_passiveForceNorm = calcPassiveForceNorm(m_lengthNorm);
  m_activeForceNorm = calcActiveForceNorm(m_lengthNorm);
  m_velocityForceNorm = calcVelocityForceNorm(m_velocityNorm);
  
  // Muscle activation from neural excitation
   m_activation = calcActivation(m_activation, m_excitation, dt);
  
  // Calculate overall response
  m_force = (m_activation * m_maxForce * m_activeForceNorm * m_velocityForceNorm) + (m_maxForce * m_passiveForceNorm); 
  
}

// Parabola centred on optimal length
//----------------------------------------------------------------------------------------------------------------------
double Muscle::calcActiveForceNorm(double lengthNorm)
{
#define SIMPLE_AFORCE_METHOD 1
#if SIMPLE_AFORCE_METHOD
  // Simple parabola. The 0.5-1.5 range roughly determined from Murray,Delp. That paper also suggests that
  // flexors mostly operate on the ascending leg, while flexors symmetrically on the peak of the parabola.
  if(lengthNorm >= activeForceWidth && lengthNorm <= (1 + activeForceWidth))
  {
    return 1.0 - sqr(2.0 * (lengthNorm - 1.0));
  }
  else 
  {
    return 0.0001;
  }

#else
  // Kistemaker's method
  const double k = 4.0; // 1 / width^2, with width = 0.5
  return (-k * sqr(lengthNorm)) + (2 * k * lengthNorm) - k + 1.0;
#endif
}

//----------------------------------------------------------------------------------------------------------------------  
double Muscle::calcPassiveForceNorm(double lengthNorm)
{
  double slackDist = lengthNorm - passiveForceSlackLength;
  if(slackDist > 0)
  {
    return m_passiveForceGain * sqr(slackDist);
  }
  else 
  {
    return 0.0;
  }
}
  
//---------------------------------------------------------------------------------------------------------------------- 
double Muscle::calcVelocityForceNorm(double velocityNorm)
{
  const double& v = velocityNorm;
	if(v < -1)
  {
		return 0.0;
  }
	else if(v > 0)
  {
    return (Ksub - (Kmax * v)) / (-v + Ksub);
  }
  else 
  {
    return (1 + v) / (1 - v / Ksh); // From MacMahon
  }
}

// Muscle actication from neural excitation
//---------------------------------------------------------------------------------------------------------------------- 
double Muscle::calcActivation(double activation, double excitation, float dt)
{
  const double tau = excitation >= activation ? m_tauAct : m_tauDeact;
  return activation + ((excitation - activation) / tau) * dt;	// else if activity decreasing
}
  
  
} // namespace dmx

