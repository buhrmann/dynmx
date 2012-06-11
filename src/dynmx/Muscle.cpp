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
#include "MuscleMonoWrap.h"
#include "MuscleBiWrap.h"

namespace dmx
{

// Parameters describing the Hill curve
//----------------------------------------------------------------------------------------------------------------------
static const double s_hillShDefault = 0.15;   // shape parameter for shortening = a/F0 = b/vmax
static const double s_hillLnDefault = 0.15;   // shape parameter for lengthening
static const double s_hillMaxDefault = 1.5;   // lenghthening asymptote
static const double s_hillSlopeDefault = 2.0; // multiple of shortening slope at v=0
  
// Width (in terms of normalised muscle length) of the parabola describing active force generation.
// I.e. beyond normalised lengths of 1+width or 1-width, no force will be generated. 
static const double s_activeForceWidth = 0.5;
static const double s_activeForceK = 1 / sqr(s_activeForceWidth);
// Normalised length beyond which the passive elastic element generates force
static const double s_passiveForceSlackLength = 1.4;
// Normalised amount of passive force at the length of activeForceWidth, i.e. where active force becomes 0.
static const double s_passiveForceAtWidth = 0.5;

//----------------------------------------------------------------------------------------------------------------------    
void Muscle::setHillParameters(double hSh, double hLn, double hMax, double hSlope)
{
  m_hillSh = hSh;
  m_hillLn = hLn;
  m_hillMax = hMax;
  m_hillSlope = hSlope;
  m_hillSub = (m_hillLn / m_hillSlope) * ((1 - m_hillMax) / (1 + m_hillLn));
}

//----------------------------------------------------------------------------------------------------------------------  
void Muscle::init()
{
  // Assume initial setup represents rest configuration
  //m_lengthOpt = m_length;
  
  // Calculate passiveForceGain, such that it is 0.5*F0 at maximum length (where active force is 0)
  // Assumes the passive force is calculated as a scaled parabola, see below.
  m_passiveForceGain = s_passiveForceAtWidth / sqr((1 + s_activeForceWidth) - s_passiveForceSlackLength);
  
  // Set default hill parameters
  setHillParameters(s_hillShDefault, s_hillLnDefault, s_hillMaxDefault, s_hillSlopeDefault);
  
  m_tauAct = MUSCLE_DEFAULT_TIMECONST_ACT;
  m_tauDeact = MUSCLE_DEFAULT_TIMECONST_DEACT;
  
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
  m_force = 0.0;
  
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
  
#if DEBUGGING  
  // Todo: Temporary check that the min/max muscle length calculations are correct
  double delta = 0.00001;
  bool lengthOK = (m_length - m_lengthMin) >= -delta && (m_length - m_lengthMax) < delta ;
  if(!lengthOK)
  {
    std::cout << getName() << "\t";
    std::cout << "Length Error: " << m_length << " " << m_lengthMin << " " << m_lengthMax << " " << m_lengthOpt;
    std::cout << ". Angles:" << m_arm->getJointAngle(JT_elbow) << " " << m_arm->getJointAngle(JT_shoulder);
    std::cout << ". Wraps: " << ((MuscleMonoWrap*)this)->getWraps() << std::endl;
    m_length = clamp(m_length, m_lengthMin, m_lengthMax);
  }
  assert(lengthOK);
#endif
         
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
  
  if(lengthNorm >= (1 - s_activeForceWidth) && lengthNorm <= (1 + s_activeForceWidth))
  {
#if SIMPLE_AFORCE_METHOD    
    // Simple parabola. The 0.5-1.5 range roughly determined from Murray,Delp. That paper also suggests that
    // flexors mostly operate on the ascending leg, while flexors symmetrically on the peak of the parabola.
    // THIS CODE ONLY WORKS IF ACTIVEFORCEWIDTH=0.5! OTHERWISE USE BELOW CODE!!!    
    return 1.0 - sqr(2.0 * (lengthNorm - 1.0));
#else    
    // Kistemaker's method
    return (-s_activeForceK * sqr(lengthNorm)) + (2 * s_activeForceK * lengthNorm) - s_activeForceK + 1.0;
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
  double slackDist = lengthNorm - s_passiveForceSlackLength;
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
    return (m_hillSub - (m_hillMax * v)) / (-v + m_hillSub);
  }
  else 
  {
    return (1 + v) / (1 - v / m_hillSh); // From MacMahon
  }
}

// Muscle actication from neural excitation
//---------------------------------------------------------------------------------------------------------------------- 
double Muscle::calcActivation(double activation, double excitation, float dt)
{
  const double tau = excitation >= activation ? m_tauAct : m_tauDeact;
  return activation + ((excitation - activation) / tau) * dt;
}
  
// Will be called from subclasses, as is virtual function
//----------------------------------------------------------------------------------------------------------------------  
void Muscle::toXml(ci::XmlTree& muscle)
{
  // XMLTree muscle provided by subclass   //ci::XmlTree muscle("Muscle", "");
  muscle.setAttribute("Name", getName());
  muscle.setAttribute("IsFlexor", isFlexor());
  muscle.setAttribute("IsMono", isMonoArticulate());
    
  ci::XmlTree attach("Attachment",""); 
  attach.setAttribute("Origin", getOrigin()); 
  attach.setAttribute("Insertion", getInsertion());   
  muscle.push_back(attach);
  
  double Fmax = getForceMax();
  ci::XmlTree maxForce("MaxIsoForce", toString(Fmax));
  muscle.push_back(maxForce);
  
  double Vmax = getVelocityMax();
  ci::XmlTree maxVel("MaxVelocity", toString(Vmax));
  muscle.push_back(maxVel);  
  
  ci::XmlTree length("Length",""); 
  length.setAttribute("Optimal", getOptimalLength()); 
  length.setAttribute("Min", m_lengthMin); 
  length.setAttribute("Max", m_lengthMax); 
  muscle.push_back(length);
  
  ci::XmlTree hillParams("HillParameters",""); 
  hillParams.setAttribute("Shortening", m_hillSh); 
  hillParams.setAttribute("Lengthening", m_hillLn); 
  hillParams.setAttribute("Asymptote", m_hillMax); 
  hillParams.setAttribute("Slope", m_hillSlope); 
  muscle.push_back(hillParams);
}
  
//----------------------------------------------------------------------------------------------------------------------    
void Muscle::record(Recorder& recorder)
{
  recorder.push_back(m_name + "LengthNorm", m_lengthNorm);
  recorder.push_back(m_name + "VelocityNorm", m_velocityNorm);
  recorder.push_back(m_name + "Act", m_activation);
  recorder.push_back(m_name + "FrcPsv", m_passiveForceNorm);
  recorder.push_back(m_name + "FrcAct", m_activeForceNorm);
  recorder.push_back(m_name + "FrcVel", m_velocityForceNorm);
  recorder.push_back(m_name + "Force", m_force);
  
  if(m_isMonoArticulate)
  {
    double ma = ((MuscleMonoWrap*)this)->getMomentArm();
    recorder.push_back(m_name + "MomentArm", ma);
    recorder.push_back(m_name + "Torque", ma * m_force);
  }
  else 
  {
    double maElb = ((MuscleBiWrap*)this)->getMomentArm(JT_elbow);
    double maShd = ((MuscleBiWrap*)this)->getMomentArm(JT_shoulder);
    recorder.push_back(m_name + "MomentArmElb", maElb);
    recorder.push_back(m_name + "MomentArmShd", maShd);
    recorder.push_back(m_name + "TorqueElb", maElb * m_force);
    recorder.push_back(m_name + "TorqueShd", maShd * m_force);
  }

}
  
} // namespace dmx

