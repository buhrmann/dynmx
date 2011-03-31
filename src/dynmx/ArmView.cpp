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

void Arm3dView::update()
{
  // update pose
  Vec3f origin (0, 0, 0);
  m_TM = ci::Matrix44f::createRotation(Vec3f(0, 0, 1), PI_OVER_TWO); // 90deg
  m_TM.translate(origin);
  
  const float shdAngle = m_arm->getJointAngle(JT_shoulder);
  m_upperArm.m_TM = Mat4f::createRotation(Vec3f(0.0f, -1.0f, 0.0f), shdAngle + PI_OVER_TWO);  
  
  const float elbAngle = m_arm->getJointAngle(JT_elbow);
  Mat4f lArmRot = Mat4f::createRotation(Vec3f(0.0f, -1.0f, 0.0f), elbAngle);
  m_lowerArm.m_TM = m_upperArm.m_TM * lArmRot; 
  
  float x, y;
  m_arm->getPointOnUpperArm(0.5, x, y);
  Vec4f midPos1(x, y, 0.0f, 1.0);
  
  m_arm->getPointOnLowerArm(0.5, x, y);
  Vec4f midPos2(x, y, 0.0f, 1.0); 
  
  m_lowerArm.m_TM.setRow(3, midPos2);
  m_upperArm.m_TM.setRow(3, midPos1);
   
  m_arm->getElbowPos(x, y);
  Vec4f elbPos(x, y, 0.0f, 1.0);
  m_elbow.m_TM.setRow(3, elbPos);
  
  m_shoulder.m_TM.setRow(3, Vec4f(0.0f, 0.0f, 0.0f, 1.0));
  
  // render
  NodeGroup::update();
}

} // namespace dmx
