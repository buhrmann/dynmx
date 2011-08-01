/*
 *  MuscleMonoWrap.h
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
//  A monoarticulate muscle wrapping around the joint capsule
//----------------------------------------------------------------------------------------------------------------------    
class MuscleMonoWrap : public Muscle
{
  
friend class Arm3dView;
  
public:
  
  MuscleMonoWrap(ArmMuscled* arm, float originDist, float insertDist, Joint jointId, bool isFlexor);
  
  virtual void init();
  virtual void reset();
  
  virtual Pos getOriginWorld();
  virtual Pos getInsertionWorld();  
  
  double getMomentArm() const { return m_momentArm; };  
  Joint getJoint() { return m_joint; };
  
protected:
  
  virtual void updateLengthAndMomentArm();
  
  // Constant data related to muscle path
  Joint m_joint;
  double m_originJointDist;
  double m_insertJointDist;
  double m_originCapsuleAngle;
  double m_insertCapsuleAngle;
  double m_wrapAngleThreshold;
  double m_originCapsuleDist;
  double m_insertCapsuleDist;

  double m_momentArm;  
  double m_muscleWraps;  
};
  
} // namespace dmx