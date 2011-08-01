/*
 *  MuscleBiWrap.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 7/20/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Muscle.h"
#include "ArmMuscled.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------  
//  A biarticulate muscle wrapping around two joint capsules
//----------------------------------------------------------------------------------------------------------------------    
class MuscleBiWrap : public Muscle
{
  
  friend class Arm3dView;
  
public:
  
  MuscleBiWrap(ArmMuscled* arm, float originDist, float insertDist, bool isFlexor);
  
  virtual void init();  
  virtual void reset();
  
  virtual Pos getOriginWorld();
  virtual Pos getInsertionWorld();  
  
  double getMomentArm(Joint j) const { assert(j == 0 || j == 1); return m_momentArms[j]; };
  bool wrapsShoulder() const { return m_shoulderWraps; };
  bool wrapsElbow() const { return m_elbowWraps; };
  double getWrapAngle(Joint j) const {if(j==JT_elbow) return m_wrapAngleElb; else return m_wrapAngleShd; };
  
protected:
  
  virtual void updateLengthAndMomentArm();
  
  // Constant data related to muscle path
  double m_momentArms[2];
  double m_originJointDist;
  double m_insertJointDist;
  double m_originCapsuleAngle;
  double m_insertCapsuleAngle;
  double m_gammaAngle;
  double m_wrapAngleThresholdShd;
  double m_wrapAngleThresholdElb;
  double m_originCapsuleDist;
  double m_insertCapsuleDist;
  double m_capsuleCapsuleDist;

  // State
  double m_wrapAngleShd;
  double m_wrapAngleElb;  
  bool m_shoulderWraps;  
  bool m_elbowWraps;  
};
  
} // namespace dmx