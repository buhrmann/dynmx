/*
 *  ArmReflex.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 10/14/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _DMX_ARM_REFLEX_
#define _DMX_ARM_REFLEX_

#include "ArmMuscled.h"
#include "Reflex.h"

namespace dmx
{
  
class ArmMuscled;
  
//----------------------------------------------------------------------------------------------------------------------
// Extends Arm to add control via PDs. 
//----------------------------------------------------------------------------------------------------------------------
class ArmReflex : public ArmMuscled
{
public:
  
  // Functions inherited from class Model
  virtual void init();
  virtual void reset(float elbAngle = 0.0f, float shdAngle = 0.0f);
  virtual void update(float timeStep); 
  
  // Custom updates
  void update(Pos pos, float timeStep, int elbPos = 1);
  void update(double desElbAngle, double desShdAngle, float timeStep);
  void update(double lengthElb0, double lengthElb1, double lengthShd0, double lengthShd1, float timeStep);  
  
  // Setters

  
  // Getters
  Reflex* getReflex(int i) { assert(i < m_reflexes.size()); return m_reflexes[i]; };
  const Pos& getDesiredPos() const { return m_desiredPos; };
  const std::deque<Pos>& getDesiredTrajectory() const { return m_desiredTrajectory; };
  
  // Store output in human readable format
  virtual void toXml(ci::XmlTree& xml);
  
protected:
  
  std::vector<Reflex*> m_reflexes;
  Pos m_desiredPos;
  std::deque<Pos> m_desiredTrajectory;
  
}; // class 
  
} // namespace

#endif