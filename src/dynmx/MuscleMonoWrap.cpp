/*
 *  MuscleMonoWrap.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 7/20/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "MuscleMonoWrap.h"
#include "MathUtils.h"

namespace dmx
{
  
#define TEST_CONST_MOMEMT_ARM 0

//----------------------------------------------------------------------------------------------------------------------  
MuscleMonoWrap::MuscleMonoWrap(ArmMuscled* arm, float originDist, float insertDist, Joint jointId, bool isFlex) :
  m_joint(jointId)
{
  m_arm = arm;
  m_originJointDist = originDist;
  m_insertJointDist = insertDist;  
  m_isFlexor = isFlex;
}

//----------------------------------------------------------------------------------------------------------------------    
void MuscleMonoWrap::init()  
{
  // Muscle path constant data
  const double r = m_arm->getJointRadius(m_joint);    
  m_originCapsuleAngle = acos(r / m_originJointDist);
  m_insertCapsuleAngle = acos(r / m_insertJointDist);
  m_wrapAngleThreshold = PI - m_originCapsuleAngle - m_insertCapsuleAngle;
  m_originCapsuleDist = m_originJointDist * sin(m_originCapsuleAngle);
  m_insertCapsuleDist = m_insertJointDist * sin(m_insertCapsuleAngle);
  m_isMonoArticulate = true;
  
  // Called last, as init() will in turn call updateLengthAndMomentArm() for which the above data needs to exist.
  // Also, Muscle::init() will in turn call (this derived class's) reset().  
  Muscle::init();
}
  
//----------------------------------------------------------------------------------------------------------------------    
void MuscleMonoWrap::reset()  
{
  m_momentArm = m_arm->getJointRadius(m_joint); // Doesn't really matter as true value is being calculated before use.
  m_muscleWraps = false;   

  Muscle::reset();
}
  
//----------------------------------------------------------------------------------------------------------------------    
void MuscleMonoWrap::calculateMinMaxLength(double& minLength, double& maxLength)
{
  double upperLimit = m_arm->getJointLimitUpper(m_joint);
  double lowerLimit = m_arm->getJointLimitLower(m_joint);
  double midLimit = lowerLimit + (0.5 * (upperLimit - lowerLimit));
  double lowerLength, upperLength, midLength;
  if(m_joint == JT_elbow)
  {
    lowerLength = getLengthFromJointAngles(lowerLimit, 0); // Second joint angle is irrelevant
    upperLength = getLengthFromJointAngles(upperLimit, 0); // Second joint angle is irrelevant
    midLength = getLengthFromJointAngles(midLimit, 0); // Second joint angle is irrelevant
  }
  else 
  {
    lowerLength = getLengthFromJointAngles(0, lowerLimit); // Second joint angle is irrelevant
    upperLength = getLengthFromJointAngles(0, upperLimit); // Second joint angle is irrelevant    
    midLength = getLengthFromJointAngles(0, midLimit); // Second joint angle is irrelevant
  }
  
  minLength = std::min(upperLength, std::min(lowerLength, midLength));
  maxLength = std::max(upperLength, std::max(lowerLength, midLength));
}
  
//----------------------------------------------------------------------------------------------------------------------  
void MuscleMonoWrap::updateLengthAndMomentArm()
{
  // An extensors length and moment arm are exactly those of an equivalent flexor when the joint angle is mirrored.
  double sign = m_isFlexor ? 1.0 : -1.0;
  double jointAngle = sign * m_arm->getJointAngle(m_joint);
  if(jointAngle == 0)
  {
    jointAngle = jointAngle;
  }
  m_muscleWraps = jointAngle < m_wrapAngleThreshold;
  
  if(!m_muscleWraps)
  {
    // Verify muscle length and moment arm
    double cosTheta = cos(jointAngle);
    m_length = sqrt(sqr(m_originJointDist) + sqr(m_insertJointDist) + (2.0 * m_originJointDist * m_insertJointDist * cosTheta));
    double k = std::acos((m_originJointDist + m_insertJointDist*cosTheta) / m_length);
    m_momentArm = m_originJointDist * sin(k);
  }    
  else
  {
    const double r = m_arm->getJointRadius(m_joint);
    double wrapAngle = m_wrapAngleThreshold - jointAngle;
    m_length = m_originCapsuleDist + (r * wrapAngle) + m_insertCapsuleDist;
    m_momentArm = r;     
  }    
  
#if TEST_CONST_MOMEMT_ARM  
  m_momentArm = 0.05;
#endif
}

//----------------------------------------------------------------------------------------------------------------------      
double MuscleMonoWrap::getLengthFromJointAngles(double elbAngle, double shdAngle)
{
  // An extensors length and moment arm are exactly those of an equivalent flexor when the joint angle is mirrored.
  int sign = m_isFlexor ? 1 : -1;  
  double jointAngle = (m_joint == JT_elbow) ? elbAngle : shdAngle;
  jointAngle *= sign;
  bool muscleWraps = jointAngle < m_wrapAngleThreshold;
  
  if(!muscleWraps)
  {
    double cosTheta = cos(jointAngle);
    return sqrt(sqr(m_originJointDist) + sqr(m_insertJointDist) + (2 * m_originJointDist * m_insertJointDist * cosTheta));
  }    
  else
  {
    const double r = m_arm->getJointRadius(m_joint);
    double wrapAngle = m_wrapAngleThreshold - jointAngle;
    return m_originCapsuleDist + (r * wrapAngle) + m_insertCapsuleDist;
  }  
}

//----------------------------------------------------------------------------------------------------------------------    
Pos MuscleMonoWrap::getOriginWorld()
{
  if (m_joint == JT_elbow)
  {
    return m_arm->getPointOnUpperArm(1.0 - m_originJointDist / m_arm->getLength(JT_shoulder));
  }
  else 
  {
    return Pos(-m_originJointDist, 0.0);
  }
}
  
//----------------------------------------------------------------------------------------------------------------------  
Pos MuscleMonoWrap::getInsertionWorld()
{
  if (m_joint == JT_elbow)
  {
    return m_arm->getPointOnLowerArm(m_insertJointDist / m_arm->getLength(JT_elbow));
  }
  else 
  {
    return m_arm->getPointOnUpperArm(m_insertJointDist / m_arm->getLength(JT_shoulder));
  }  
}
  
//----------------------------------------------------------------------------------------------------------------------  
void MuscleMonoWrap::toXml(ci::XmlTree& xml)
{
  ci::XmlTree muscle("Muscle", "");
  muscle.setAttribute("Joint", getJoint() == JT_elbow ? "Elbow" : "Shoulder"); 
  
  // Let Muscle write generic parameters
  Muscle::toXml(muscle);
  
  xml.push_back(muscle);
}
  
} // namespace dmx

