/*
 *  Arm3dView.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "ArmViz.h"
#include "MathUtils.h"

// Todo: this is nasty!
#include "ArmPD.h"
#include "ArmMuscled.h"
#include "Muscle.h"
#include "MuscleMonoWrap.h"
#include "MuscleBiWrap.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
Vec3f ArmViz::toLocalSpace(Vec3f p)
{
  p -= m_TM.getTranslate().xyz();
  return m_TM.preMultiply(p);
}
  
Vec3f ArmViz::toWorldSpace(Vec3f p)
{
  p = m_TM.postMultiply(p);
  return p + m_TM.getTranslate().xyz();
}
  
//----------------------------------------------------------------------------------------------------------------------
void ArmViz::init()
{
  NodeGroup::init();
}

// Overwrite base class update
//----------------------------------------------------------------------------------------------------------------------
void ArmViz::update()
{
  
#if 0  
  m_upperArm.m_TM.setToIdentity(); 
  m_upperArm.m_TM.translate(Vec3f(m_arm->getPointOnUpperArm(0.5)));  
#if 1  
  // bring capsule into proper default orientation (from z-aligned to x-alignes)
  m_upperArm.m_TM.rotate(Vec3f(0.0f, 1.0f, 0.0f), -PI_OVER_TWO);    
  // now apply real angle from model (in this just modified space)
  m_upperArm.m_TM.rotate(Vec3f(1.0f, 0.0f, 0.0f), shdAngle);    
#else
  // now apply real angle from model (in this just modified space)
  m_upperArm.m_TM.rotate(Vec3f(0.0f, 0.0f, 1.0f), shdAngle);  
#endif

  m_lowerArm.m_TM.setToIdentity(); 
  m_lowerArm.m_TM.translate(Vec3f(m_arm->getPointOnLowerArm(0.5)));  
#if 1  
  // bring capsule into proper default orientation (from z-aligned to x-alignes)
  m_lowerArm.m_TM.rotate(Vec3f(0.0f, 1.0f, 0.0f), -PI_OVER_TWO);    
  // now apply real angle from model (in this just modified space)
  m_lowerArm.m_TM.rotate(Vec3f(1.0f, 0.0f, 0.0f), shdAngle + elbAngle);    
#else
  // now apply real angle from model (in this just modified space)
  m_lowerArm.m_TM.rotate(Vec3f(0.0f, 0.0f, 1.0f), shdAngle + elbAngle);  
#endif  
  
  m_elbow.m_TM.setToIdentity();
  m_elbow.m_TM.rotate(Vec3f(0.0f, 0.0f, 1.0f), shdAngle); 
  m_elbow.m_TM.setTranslate(Vec3f(m_arm->getElbowPos()));
#endif

  
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  glLineWidth(1.0);
  glColor3f(0,0,0);
  ci::gl::drawLine(Vec3f(0,0,0), Vec3f(m_arm->getElbowPos()));  
  ci::gl::drawLine(Vec3f(m_arm->getElbowPos()), Vec3f(m_arm->getEffectorPos()));  
  glLineWidth(1.0);  
  
  glPopMatrix();   
}

} // namespace dmx
