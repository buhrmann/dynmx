/*
 *  Muscle.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Muscle.h"
#include "Arm.h"
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


//----------------------------------------------------------------------------------------------------------------------  
void Muscle::init()
{
  transformPathToWorld();
  m_length = calcLength();
  
  // assume initial setup represents rest configuration
  m_lengthOpt = m_length;
  
  // Todo: calculate passiveForceGain, such that it is 0.5*F0 at maximum length (where active force is 0)
  m_passiveForceGain = 1.0;
  
  m_maxVelocity = 10.0 * m_lengthOpt;
}

//----------------------------------------------------------------------------------------------------------------------
void Muscle::addPathPoint(const ci::Vec2f& point, int jointId)
{
  m_path.push_back(MusclePathPoint());
  const size_t i = m_path.size() - 1;
  m_path[i].local = point;
  m_path[i].joint = jointId;
}

//----------------------------------------------------------------------------------------------------------------------
void Muscle::update(float dt)
{
  transformPathToWorld();
  
  double prevLength = m_length;
  m_length = calcLength();
  m_velocity = (m_length - prevLength) / dt;
  
  // Dimensionless variables for Hill-model
  m_lengthNorm = m_length /  m_lengthOpt;
  m_velocityNorm = m_velocity / m_maxVelocity;
  
  
#if 0  
  if(m_length > m_lengthOpt)
  {
    m_force = m_length - m_lengthOpt;
    m_force *= 100.0f;
  }
  else
  { 
    m_force = 0.0f;
  }
#endif  
}

//----------------------------------------------------------------------------------------------------------------------
void Muscle::transformPathToWorld()
{
  ci::Vec3f upperDir (m_arm->getElbowPos());
  ci::Vec3f lowerDir (m_arm->getEffectorPos() - m_arm->getElbowPos());
  
  // Get local "up" direction vectors, orthogonal to bone 
  const ci::Vec3f ortho (0,0,1);
  ci::Vec3f upperUp = upperDir.cross(ortho);
  ci::Vec3f lowerUp = lowerDir.cross(ortho);
  upperUp.normalize();
  lowerUp.normalize();
  
  for(size_t i = 0; i < m_path.size(); ++i)
  {
    ci::Vec3f p;
    
    if (m_path[i].joint == JT_elbow)
    {
      p = ci::Vec3f(m_arm->getPointOnLowerArm(m_path[i].local.x));
      p += lowerUp * m_path[i].local.y;
    }
    else 
    {
      p = ci::Vec3f(m_arm->getPointOnUpperArm(m_path[i].local.x));
      p += upperUp * m_path[i].local.y;
    }

    m_path[i].world = ci::Vec2f(p.x, p.y);    
  }
}

//----------------------------------------------------------------------------------------------------------------------
double Muscle::calcLength()
{
  double length = 0.0;
  const int N = m_path.size() - 1;
  for(size_t i = 0; i < N; ++i)
  {
    const ci::Vec2f diff = m_path[i].world - m_path[i+1].world; 
    length += diff.length();
  }
  return length;
}

// Parabola centred on optimal length
//----------------------------------------------------------------------------------------------------------------------
double Muscle::calcActiveForceNorm()
{
#define SIMPLE_AFORCE_METHOD 1
#if SIMPLE_AFORCE_METHOD
  // Simple parabola. The 0.5-1.5 range roughly determined from Murray,Delp. That paper also suggests that
  // flexors mostly operate on the ascending leg, while flexors symmetrically on the peak of the parabola.
  if(m_lengthNorm >= 0.5 && m_lengthNorm <= 1.5)
  {
    return 1.0 - sqr(2.0 * (m_lengthNorm - 1.0));
  }
  else 
  {
    return 0.0001;
  }

#else
  // Kistemaker's method
  const float k = 4.0; // 1 / width^2
  return (-k * sqr(m_lengthNorm)) + (2 * k * m_lengthNorm) - k + 1.0;
#endif
}

//----------------------------------------------------------------------------------------------------------------------  
double Muscle::calcPassiveForceNorm()
{
  const float slackLengthNorm = 1.4;
  double slackDist = m_lengthNorm - slackLengthNorm;
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
double Muscle::calcVelocityForceNorm()
{
  const float& v = m_velocityNorm;
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

// Activation level filter
//---------------------------------------------------------------------------------------------------------------------- 
double Muscle::updateActivation(float dt)
{
  const float tau = m_excitation >= m_activation ? m_tauAct : m_tauDeact;
  m_activation += ((m_excitation - m_activation) / tau) * dt;	// else if activity decreasing
}


} // namespace dmx

