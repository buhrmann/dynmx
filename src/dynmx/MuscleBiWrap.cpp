/*
 *  MuscleBiWrap.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 7/20/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "MuscleBiWrap.h"
#include "MathUtils.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
MuscleBiWrap::MuscleBiWrap(ArmMuscled* arm, float originDist, float insertDist, bool isFlexor) :
m_originJointDist(originDist),
m_insertJointDist(insertDist)
{
  m_arm = arm;
  m_isFlexor = isFlexor;
}

//----------------------------------------------------------------------------------------------------------------------    
void MuscleBiWrap::init()  
{
  // Muscle path constant data
  const double radShd = m_arm->getJointRadius(JT_shoulder);    
  const double radElb = m_arm->getJointRadius(JT_elbow);
  const double upperArmLength = m_arm->getLength(JT_shoulder);
  m_originCapsuleAngle = acos(radShd / m_originJointDist);
  m_insertCapsuleAngle = acos(radElb / m_insertJointDist);
  m_gammaAngle = acos((radShd - radElb) / upperArmLength);
  m_wrapAngleThresholdShd = PI - m_originCapsuleAngle - m_gammaAngle;
  m_wrapAngleThresholdElb = m_gammaAngle - m_insertCapsuleAngle;
  m_originCapsuleDist = m_originJointDist * sin(m_originCapsuleAngle);
  m_insertCapsuleDist = m_insertJointDist * sin(m_insertCapsuleAngle);
  m_capsuleCapsuleDist = upperArmLength * sin(m_gammaAngle);
  m_isMonoArticulate = false;
  
  // Called last, as init() will in turn call updateLengthAndMomentArm() for which the above data needs to exist.  
  // Also, Muscle::init() will in turn call (this derived class's) reset().
  Muscle::init();
}
  
//----------------------------------------------------------------------------------------------------------------------    
void MuscleBiWrap::reset()  
{
  m_shoulderWraps = false;
  m_elbowWraps = false;  
  m_wrapAngleElb = m_wrapAngleShd = 0;
  
  Muscle::reset();  
}

//----------------------------------------------------------------------------------------------------------------------  
void MuscleBiWrap::updateLengthAndMomentArm()
{
  double shdAngle = m_arm->getJointAngle(JT_shoulder);
  double elbAngle = m_arm->getJointAngle(JT_elbow);  
  const double radShd = m_arm->getJointRadius(JT_shoulder);
  const double radElb = m_arm->getJointRadius(JT_elbow);  
 
  // An extensor's muscle length and moment arm are exactly those of an equivalent flexor,
  // when joint angles are mirrored.
  if(!m_isFlexor)
  {
    shdAngle *= -1;
    elbAngle *= -1;
  }
  
  // Check capsule wrapping
  bool doubleWraps = (shdAngle < m_wrapAngleThresholdShd) && (elbAngle < m_wrapAngleThresholdElb);
  if(doubleWraps)
  {
    // Double wrap condition
    m_shoulderWraps = m_elbowWraps = true;
    m_wrapAngleShd = PI - m_originCapsuleAngle - shdAngle - m_gammaAngle;
    m_wrapAngleElb = m_gammaAngle - m_insertCapsuleAngle - elbAngle;
    m_length = m_originCapsuleDist + m_wrapAngleShd*radShd + m_capsuleCapsuleDist + m_wrapAngleElb*radElb + m_insertCapsuleDist;
    m_momentArms[JT_shoulder] = radShd;
    m_momentArms[JT_elbow] = radElb;
  }
  else 
  {
    // Single or no wrap condition
    
    // These are needed in any case:
    const double upperArmLength = m_arm->getLength(JT_shoulder);    
    const double xa = upperArmLength * cos(shdAngle) + m_insertJointDist * cos(shdAngle + elbAngle);
    const double ya = upperArmLength * sin(shdAngle) + m_insertJointDist * sin(shdAngle + elbAngle);
    const double wShd = atan2(ya, xa + m_originJointDist);
    const double maShd = m_originJointDist * sin(wShd);
    const double wElb = PI_OVER_TWO - shdAngle - elbAngle + wShd;
    const double maElb = m_insertJointDist * cos(wElb);
    
    const bool couldShoulderWrap = ((shdAngle - m_wrapAngleThresholdShd) < (elbAngle - m_wrapAngleThresholdElb));    
    m_shoulderWraps = couldShoulderWrap && (shdAngle < PI_OVER_TWO) && (maShd < radShd);
    if(m_shoulderWraps)
    {
      // Shoulder wrapping    
      m_elbowWraps = false;      
      const double cosElbAngle = cos(elbAngle);
      const double b = sqrt( sqr(upperArmLength) + sqr(m_insertJointDist) + 2*upperArmLength*m_insertJointDist*cosElbAngle );
      const double n = acos(radShd / b);
      const double mu = acos((upperArmLength + m_insertJointDist*cosElbAngle) / b);
      m_wrapAngleShd = PI - n - mu - m_originCapsuleAngle - shdAngle;
      m_wrapAngleElb = 0.0;
      const double l = PI_OVER_TWO - m_originCapsuleAngle - m_wrapAngleShd - shdAngle;
      m_length = m_originCapsuleDist + radShd*m_wrapAngleShd + b*sin(n);
      m_momentArms[JT_shoulder] = radShd;
      m_momentArms[JT_elbow] = radShd + upperArmLength*sin(l);
    }
    else 
    {
      m_elbowWraps = (elbAngle < PI_OVER_TWO) && (maElb < radElb);  
      if(m_elbowWraps)
      {
        // ELbow wrapping
        const double cosShdAngle = cos(shdAngle);
        const double b = sqrt( sqr(upperArmLength) + sqr(m_originJointDist) + 2*upperArmLength*m_originJointDist*cosShdAngle );
        const double xi = asin(radElb / b);
        const double ro = acos((m_originJointDist + upperArmLength*cosShdAngle) / b);
        m_wrapAngleElb = PI_OVER_TWO - shdAngle - elbAngle - m_insertCapsuleAngle + ro + xi;
        m_wrapAngleShd = 0.0;
        m_length = sqrt(sqr(b) - sqr(radElb)) + m_wrapAngleElb*radElb + m_insertCapsuleDist;
        m_momentArms[JT_shoulder] = m_originJointDist * sin(ro + xi);
        m_momentArms[JT_elbow] = radElb;
      }
      else 
      {
        // No wrapping
        m_wrapAngleElb = 0.0;        
        m_wrapAngleShd = 0.0;        
        m_length = sqrt(sqr(xa + m_originJointDist) + sqr(ya));
        m_momentArms[JT_shoulder] = maShd;
        m_momentArms[JT_elbow] = maElb;        
      } // No wrap
    } // Elbow wrap
  } // Single or no wrap
  
}

//----------------------------------------------------------------------------------------------------------------------    
Pos MuscleBiWrap::getOriginWorld()
{
  return Pos(-m_originJointDist, 0.0);
}

//----------------------------------------------------------------------------------------------------------------------  
Pos MuscleBiWrap::getInsertionWorld()
{
  return m_arm->getPointOnLowerArm(m_insertJointDist / m_arm->getLength(JT_elbow));
}

} // namespace dmx

