/*
 *  MuscleBiWrap.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 7/20/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_MUSCLE_BI_
#define _DMX_MUSCLE_BI_

#include "Muscle.h"
#include "ArmMuscled.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------  
//  A biarticulate muscle wrapping around two joint capsules
//----------------------------------------------------------------------------------------------------------------------    
class MuscleBiWrap : public Muscle
{
  
friend class ArmMuscledViz;
  
public:
  
  MuscleBiWrap(ArmMuscled* arm, double originDist, double insertDist, bool isFlexor);
  
  virtual void init();  
  virtual void reset();
  
  virtual Pos getOriginWorld();
  virtual Pos getInsertionWorld();  
  
  double getMomentArm(Joint j) const { assert(j == 0 || j == 1); return m_momentArms[j]; };
  bool wrapsShoulder() const { return m_shoulderWraps; };
  bool wrapsElbow() const { return m_elbowWraps; };
  double getWrapAngle(Joint j) const {if(j==JT_elbow) return m_wrapAngleElb; else return m_wrapAngleShd; };
  
  virtual double getLengthFromJointAngles(double elbAngle, double shdAngle);  
  
  // Store output in human readable format
  virtual void toXml(ci::XmlTree& xml);    
  
protected:
  
  virtual void updateLengthAndMomentArm();
  virtual void calculateMinMaxLength(double& min, double& max);  
  
  // Constant data related to muscle path
  double m_momentArms[2];
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

#endif