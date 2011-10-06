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
  
//----------------------------------------------------------------------------------------------------------------------
void ArmViz::init()
{
  NodeGroup::init();
  
  Vec4f col (0.5f, 0.5f, 0.5f, 0.95f);
  
  float elbLength = m_arm->getLength(JT_elbow);
  float shdLength = m_arm->getLength(JT_shoulder);
  
#if 0  
  m_lowerArm.m_lx = elbLength;
  m_lowerArm.m_ly = elbLength / 10.0;
  m_lowerArm.m_lz = elbLength / 10.0;
  m_lowerArm.createGeometry();
  m_lowerArm.m_color = col;

  m_upperArm.m_lx = shdLength;
  m_upperArm.m_ly = shdLength / 10.0;
  m_upperArm.m_lz = shdLength / 10.0;  
  m_upperArm.createGeometry();
  m_upperArm.m_color = col;
#else
  m_lowerArm.m_length = elbLength;
  m_lowerArm.m_radius1 = elbLength / 10.0;
  m_lowerArm.m_radius2 = m_lowerArm.m_radius1;
  m_lowerArm.createGeometry();
  m_lowerArm.m_color = col;
  m_lowerArm.m_outlineWidth = 2.0f;
  
  m_upperArm.m_length = shdLength;
  m_upperArm.m_radius1 = shdLength / 10.0;
  m_upperArm.m_radius2 = m_upperArm.m_radius1;
  m_upperArm.createGeometry();
  m_upperArm.m_color = col;  
  m_upperArm.m_outlineWidth = 2.0f;  
#endif
  
  m_elbow.m_radius1 = 1.1 * m_lowerArm.m_radius1;
  m_elbow.m_radius2 = m_elbow.m_radius1;
  m_elbow.m_length = m_lowerArm.m_radius1 * 2;  
  m_elbow.m_slices = 16;
  m_elbow.m_stacks = 1;
  m_elbow.createGeometry();
  m_elbow.m_color = col;
  m_elbow.m_outlineWidth = 2.0f;
    
  m_shoulder.m_radius1 =  1.1f * m_upperArm.m_radius2;
  m_shoulder.m_radius2 = m_shoulder.m_radius1;
  m_shoulder.m_length = m_upperArm.m_radius1 * 2;
  m_shoulder.m_slices = 16;
  m_shoulder.m_stacks = 1;
  m_shoulder.createGeometry();
  m_shoulder.m_color = col;
  m_shoulder.m_outlineWidth = 2.0f;
  
  m_children.push_back(&m_lowerArm);
  m_children.push_back(&m_upperArm);
  m_children.push_back(&m_elbow);
  m_children.push_back(&m_shoulder);
}

// Overwrite base class update
//----------------------------------------------------------------------------------------------------------------------
void ArmViz::update()
{
 
  // Update pose  
  const float shdAngle = m_arm->getJointAngle(JT_shoulder);
  const float elbAngle = m_arm->getJointAngle(JT_elbow);
  
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

  
  // After updating poses, render the scene graph objects
  //NodeGroup::update();  
  
  // Overlay desired state
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // Arm as line drawing
  glLineWidth(4.0);
  glColor3f(0,0,0);
  ci::gl::drawLine(Vec3f(0,0,0), Vec3f(m_arm->getElbowPos()));  
  ci::gl::drawLine(Vec3f(m_arm->getElbowPos()), Vec3f(m_arm->getEffectorPos()));  
  glLineWidth(1.0);  
  
  // Joints and limits
  glPushMatrix();
  glMultMatrixf(m_elbow.m_TM);
   // Draw joint 
  float r = m_elbow.m_radius1; 
  drawDisk(r, 0, 16, 1);
  // Draw limits
  float limMin = m_arm->getJointLimitLower(JT_elbow) * RAD_TO_DEG;
  float limMax = m_arm->getJointLimitUpper(JT_elbow) * RAD_TO_DEG;
  glColor4f(1,1,1,0.5);
  drawPartialDisk(r, r+0.01, 16, 1, 90 - limMin, -(limMax - limMin));
  glPopMatrix();
  
  glColor3f(0,0,0);
  glPushMatrix();
  //glMultMatrixf(m_shoulder.m_TM);
  // Draw joint  
  r = m_shoulder.m_radius1;
  drawDisk(r, 0, 16, 1);
  // Draw limits  
  limMin = m_arm->getJointLimitLower(JT_shoulder) * RAD_TO_DEG;
  limMax = m_arm->getJointLimitUpper(JT_shoulder) * RAD_TO_DEG;  
  glColor4f(1,1,1,0.5);
  drawPartialDisk(r, r+0.01, 16, 1, 90 - limMin, -(limMax - limMin));
  glPopMatrix();
  
  // Undo this Node's transform
  glPopMatrix();   
}

} // namespace dmx
