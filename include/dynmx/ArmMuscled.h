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
  
//----------------------------------------------------------------------------------------------------------------------
// Extends Arm to add control via PDs. 
//----------------------------------------------------------------------------------------------------------------------
class ArmMuscled : public Arm
{

public:
  
  virtual ~ArmMuscled();

  // Functions inherited from class Model
  virtual void init();
  virtual void reset() { resetTo(0.0f, 0.0f); };
  virtual void update(float timeStep); 
  
  void resetTo(double elbAngle, double shdAngle);  
  
  // Setters
  inline void setJointRadii(double elbRad, double shdRad)
  { 
    m_jointRadius[JT_elbow] = elbRad; 
    m_jointRadius[JT_shoulder] = shdRad; 
  };
  void setMuscleParams(int mId, double origin, double insertion, double maxIsoForce, double optimalLength, double maxVelocity);
  void setDesiredJointAngle(Joint j, double angle);
  
  // Getters
  int getNumMuscles() const { return m_muscles.size(); };
  Muscle* getMuscle(int i) { assert(i < m_muscles.size()); return m_muscles[i]; };
  double getJointRadius(int joint) const { return m_jointRadius[joint]; };
  double getDesiredJointAngle(Joint j) const { assert(j==JT_elbow || j==JT_shoulder); return m_desiredAngles[j]; };
  
  // Store output in human readable format
  virtual void toXml(ci::XmlTree& xml);
  
  // Record all current state
  virtual void record(Recorder& recorder);    
  
  // Output steady state behaviour
  void recordIsometric(Recorder& recorder, const std::vector<float>& musclAct, float jointIncDeg = 1.0);

protected:

  // Six muscles: 2 per joint and two biarticulated.
  std::vector<Muscle*> m_muscles;
  double m_jointRadius[2];
  double m_desiredAngles[2];
 
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
