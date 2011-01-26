/*
 *  Arm3dView.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "ArmView.h"
#include "MathUtils.h"

namespace dmx
{

void Arm3dView::init()
{
  NodeGroup::init();
  
  float elbLength = m_arm->getLength(JT_elbow);
  m_lowerArm.m_radius = elbLength / 10.0f;
  m_lowerArm.m_length = elbLength;
  m_lowerArm.m_resolution = 16;
  m_lowerArm.createGeometry();
  
  float shdLength = m_arm->getLength(JT_shoulder);
  m_upperArm.m_radius = shdLength / 10.0f;
  m_upperArm.m_length = shdLength;
  m_upperArm.m_resolution = 16;
  m_upperArm.createGeometry();

  m_elbow.m_radius = m_upperArm.m_radius + 0.1f * m_upperArm.m_radius;
  m_elbow.createGeometry();
  m_elbow.m_color = Vec4f(0.1f, 0.1f, 0.1f, 1.0);
    
  m_shoulder.m_radius = m_upperArm.m_radius  + 0.1f * m_upperArm.m_radius;
  m_shoulder.createGeometry();
  m_shoulder.m_color= Vec4f(0.1f, 0.1f, 0.1f, 1.0);
    
  m_children.push_back(&m_lowerArm);
  m_children.push_back(&m_upperArm);
  m_children.push_back(&m_elbow);
  m_children.push_back(&m_shoulder);
}

void Arm3dView::update(float time)
{
  // update pose
  Vec4f origin (0, 0, 1, 1);
  m_TM.createRotation(Vec3f(0, 1, 0), PI_OVER_TWO); // 90deg
  m_TM.setRow(3, origin);
  
  const float shdAngle = m_arm->getJointAngle(JT_shoulder);
  Mat4f uArmRot;
  uArmRot.createRotation(Vec3f(0.0f, -1.0f, 0.0f), shdAngle + PI_OVER_TWO);  
  m_upperArm.m_TM = uArmRot;
  
  const float elbAngle = m_arm->getJointAngle(JT_elbow);
  Mat4f lArmRot;
  lArmRot.createRotation(Vec3f(0.0f, -1.0f, 0.0f), elbAngle);
  m_lowerArm.m_TM = uArmRot * lArmRot; 
  
  float x, y;
  m_arm->getPointOnUpperArm(0.5, x, y);
  Vec4f midPos1(x, 0.0f, y, 1.0);
  
  m_arm->getPointOnLowerArm(0.5, x, y);
  Vec4f midPos2(x, 0.0f, y, 1.0); 
  
  m_lowerArm.m_TM.setRow(3, midPos2);
  m_upperArm.m_TM.setRow(3, midPos1);
   
  m_arm->getElbowPos(x, y);
  Vec4f elbPos(x, 0.0f, y, 1.0);
  m_elbow.m_TM.setRow(3, elbPos);
  
  m_shoulder.m_TM.setRow(3, Vec4f(0.0f, 0.0f, 0.0f, 1.0));
  
  // render
  NodeGroup::update();
}

} // namespace dmx
