/*
 *  ArmMuscled.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_MUSCLED_
#define _DMX_ARM_MUSCLED_

#include "Arm.h"
#include "Muscle.h"
#include "MathUtils.h"

namespace dmx
{

class EPController;
//class Reflex;
  
//----------------------------------------------------------------------------------------------------------------------
// Extends Arm to add control via PDs. 
//----------------------------------------------------------------------------------------------------------------------
class ArmMuscled : public Arm
{

public:

  // Functions inherited from class Model
  virtual void init();
  virtual void reset(float elbAngle = 0.0f, float shdAngle = 0.0f);
  virtual void update(float timeStep); 
  
//  void update(Pos pos, float timeStep, int elbPos = 1);
//  void update(double desElbAngle, double desShdAngle, float timeStep);
//  void update(double lengthElb0, double lengthElb1, double lengthShd0, double lengthShd1, float timeStep);
  
  // Setters
  void setJointRadii(float elbRad, float shdRad) { m_jointRadius[JT_elbow] = elbRad; m_jointRadius[JT_shoulder] = shdRad; };
  void setMuscleParams(int mId, double origin, double insertion, double maxIsoForce, double optimalLength, double maxVelocity);
  void setDesiredJointAngle(Joint j, double angle);
  
  // Getters
  int getNumMuscles() const { return m_muscles.size(); };
  Muscle* getMuscle(int i) { assert(i < m_muscles.size()); return m_muscles[i]; };
//  Reflex* getReflex(int i) { assert(i < m_reflexes.size()); return m_reflexes[i]; };
  float getJointRadius(int joint) const { return m_jointRadius[joint]; }; 
//  const Pos& getDesiredPos() const { return m_desiredPos; };
  double getDesiredJointAngle(Joint j) const { assert(j==JT_elbow || j==JT_shoulder); return m_desiredAngles[j]; };
//  const std::deque<Pos>& getDesiredTrajectory() const { return m_desiredTrajectory; };
  
  // Store output in human readable format
  virtual void toXml(ci::XmlTree& xml);

protected:

  // Six muscles: 2 per joint and two biarticulated.
  std::vector<Muscle*> m_muscles;
//  std::vector<Reflex*> m_reflexes;

  float m_jointRadius[2];
  
//  Pos m_desiredPos;
  double m_desiredAngles[2];
  
//  std::deque<Pos> m_desiredTrajectory;
 
}; // class ArmPD
  

//----------------------------------------------------------------------------------------------------------------------    
// Inline implementations  
//----------------------------------------------------------------------------------------------------------------------  
inline void ArmMuscled::setMuscleParams(int mId, double origin, double insertion, 
                                 double maxIsoForce, double optimalLength, double maxVelocity)
{ 
  assert(mId >= 0 && mId < m_muscles.size());
  assert(maxIsoForce > 0.0);
  assert(optimalLength > 0.0);
  assert(maxVelocity > 0.0);
  m_muscles[mId]->setOriginInsertion(origin, insertion);
  m_muscles[mId]->setParameters(maxIsoForce, optimalLength, maxVelocity);
}
  
//----------------------------------------------------------------------------------------------------------------------    
inline void ArmMuscled::setDesiredJointAngle(Joint j, double angle)
{
  assert(j==JT_elbow || j==JT_shoulder);
  m_desiredAngles[j] = clamp(angle, getJointLimitLower(j), getJointLimitUpper(j));
}
  
} // namespace dmx

#endif
