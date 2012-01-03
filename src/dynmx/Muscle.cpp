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
static const float activeForceK = 1 / sqr(activeForceWidth);
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
  m_passiveForceGain = passiveForceAtWidth / sqr((1 + activeForceWidth) - passiveForceSlackLength);
  
  m_tauAct = 0.04;
  m_tauDeact = 0.07;
  
  // Get min and max lengths
  calculateMinMaxLength(m_lengthMin, m_lengthMax);
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------  
void Muscle::reset()
{
  updateLengthAndMomentArm();
  
  m_lengthNorm = m_length /  m_lengthOpt;
  m_velocityNorm = m_velocity / m_maxVelocity;
  m_lengthUnit = lengthToUnitLength(m_length);
  
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
  m_maxVelocity = maxVelocity;// * m_lengthOpt; // Why, is it really specified in l0/s?
  m_maxForce = maxIsoForce;
} 

//----------------------------------------------------------------------------------------------------------------------
void Muscle::update(float dt)
{
  double prevLength = m_length;

  updateLengthAndMomentArm();
  
  // Todo: Temporary check that the min/max muscle length calculations are correct
  double delta = 0.001;
  bool lengthOK = (m_length - m_lengthMin) >= -delta && (m_length - m_lengthMax) < delta ;
  if(!lengthOK)
  {
    std::cout << getName() << std::endl;
    std::cout << "Length: " << m_length << " " << m_lengthMin << " " << m_lengthMax << std::endl;
  }
  assert(lengthOK);
         
  m_velocity = (m_length - prevLength) / dt;
  
  // Dimensionless variables for Hill-model
  m_lengthNorm = m_length /  m_lengthOpt;
  m_velocityNorm = m_velocity / m_maxVelocity;
  
  // Length scaled to unit interval (0,1)
  m_lengthUnit = lengthToUnitLength(m_length);
  assert(m_lengthUnit >= 0.0 && m_lengthUnit <= 1.0);
  
  // Muscle activation from neural excitation
  m_activation = calcActivation(m_activation, m_excitation, dt);
  
  // Calculate individual constitutive forces
  m_passiveForceNorm = calcPassiveForceNorm(m_lengthNorm);
  m_activeForceNorm = calcActiveForceNorm(m_lengthNorm);
  m_velocityForceNorm = calcVelocityForceNorm(m_velocityNorm);
  
  
  // Calculate overall response
  m_force = (m_activation * m_maxForce * m_activeForceNorm * m_velocityForceNorm) + (m_maxForce * m_passiveForceNorm); 
}

// Parabola centred on optimal length
//----------------------------------------------------------------------------------------------------------------------
double Muscle::calcActiveForceNorm(double lengthNorm)
{
#define SIMPLE_AFORCE_METHOD 0
  
  if(lengthNorm >= (1 - activeForceWidth) && lengthNorm <= (1 + activeForceWidth))
  {
#if SIMPLE_AFORCE_METHOD    
    // Simple parabola. The 0.5-1.5 range roughly determined from Murray,Delp. That paper also suggests that
    // flexors mostly operate on the ascending leg, while flexors symmetrically on the peak of the parabola.
    // THIS CODE ONLY WORKS IF ACTIVEFORCEWIDTH=0.5! OTHERWISE USE BELOW CODE!!!    
    return 1.0 - sqr(2.0 * (lengthNorm - 1.0));
#else    
    // Kistemaker's method
    return (-activeForceK * sqr(lengthNorm)) + (2 * activeForceK * lengthNorm) - activeForceK + 1.0;
#endif
  }
  else 
  {
    return 0.0001;
  }
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
  
// Will be called from subclasses, as is virtual function
//----------------------------------------------------------------------------------------------------------------------  
void Muscle::toXml(ci::XmlTree& muscle)
{
  // XMLTree muscle provided by subclass   //ci::XmlTree muscle("Muscle", "");
  muscle.setAttribute("Name", getName());
  muscle.setAttribute("IsFlexor", isFlexor());
  muscle.setAttribute("IsMono", isMonoArticulate());
    
  ci::XmlTree origin("Origin",""); origin.setAttribute("Value", getOrigin()); 
  muscle.push_back(origin);
  
  ci::XmlTree insertion("Insertion",""); insertion.setAttribute("Value", getInsertion()); 
  muscle.push_back(insertion);
  
  ci::XmlTree maxForce("MaxIsoForce",""); maxForce.setAttribute("Value", getForceMax()); 
  muscle.push_back(maxForce);
  
  ci::XmlTree optLength("OptimalLength",""); optLength.setAttribute("Value", getOptimalLength()); 
  muscle.push_back(optLength);
  
  ci::XmlTree maxVel("MaxVelocity",""); maxVel.setAttribute("Value", getVelocityMax()); 
  muscle.push_back(maxVel);
  
  //xml.push_back(muscle);
}
  
} // namespace dmx

