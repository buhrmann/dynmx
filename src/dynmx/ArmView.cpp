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
  m_lowerArm.m_color = Vec4f(1.0f, 1.0f, 1.0f, 0.5f);
  
  float shdLength = m_arm->getLength(JT_shoulder);
  m_upperArm.m_radius = shdLength / 10.0f;
  m_upperArm.m_length = shdLength;
  m_upperArm.m_resolution = 16;
  m_upperArm.createGeometry();
  m_upperArm.m_color = Vec4f(1.0f, 1.0f, 1.0f, 0.5f);
  
  m_elbow.m_radius = m_upperArm.m_radius + 0.1f * m_upperArm.m_radius;
  m_elbow.createGeometry();
  m_elbow.m_color = Vec4f(0.1f, 0.1f, 0.1f, 0.5f);
    
  m_shoulder.m_radius = m_upperArm.m_radius  + 0.1f * m_upperArm.m_radius;
  m_shoulder.createGeometry();
  m_shoulder.m_color= Vec4f(0.1f, 0.1f, 0.1f, 0.5f);
    
  m_children.push_back(&m_lowerArm);
  m_children.push_back(&m_upperArm);
  m_children.push_back(&m_elbow);
  m_children.push_back(&m_shoulder);
}

void Arm3dView::update()
{
  // update pose
  Vec3f origin (0, 0, 0);
  m_TM = ci::Matrix44f::createRotation(Vec3f(0, 0, 1), -PI_OVER_TWO); // 90deg
  m_TM.translate(origin);

  Pos pos;
  pos = m_arm->getPointOnUpperArm(0.5);
  Vec3f midPos1(pos.x, pos.y, 0);
  
  pos = m_arm->getPointOnLowerArm(0.5);
  Vec3f midPos2(pos.x, pos.y, 0);
  
  const float shdAngle = m_arm->getJointAngle(JT_shoulder);
  m_upperArm.m_TM.setToIdentity(); 
  m_upperArm.m_TM.translate(midPos1);  
  // bring capsule into proper default orientation (from z-aligned to x-alignes)
  m_upperArm.m_TM.rotate(Vec3f(0.0f, 1.0f, 0.0f), -PI_OVER_TWO);    
  // now apply real angle from model (in this just modified space)
  m_upperArm.m_TM.rotate(Vec3f(1.0f, 0.0f, 0.0f), shdAngle);  
  
  const float elbAngle = m_arm->getJointAngle(JT_elbow);
  m_lowerArm.m_TM.setToIdentity(); 
  m_lowerArm.m_TM.translate(midPos2);  
  // bring capsule into proper default orientation (from z-aligned to x-alignes)
  m_lowerArm.m_TM.rotate(Vec3f(0.0f, 1.0f, 0.0f), -PI_OVER_TWO);    
  // now apply real angle from model (in this just modified space)
  m_lowerArm.m_TM.rotate(Vec3f(1.0f, 0.0f, 0.0f), shdAngle + elbAngle);  
  
  pos = m_arm->getElbowPos();
  Vec3f elbPos(pos.x, pos.y, 0.0f);
  m_elbow.m_TM.setToIdentity(); 
  m_elbow.m_TM.translate(elbPos);
  
  m_shoulder.m_TM.translate(Vec3f(0.0f, 0.0f, 0.0f));
  
  // overlay desired state
  float desElbAngle = m_arm->getDesiredJointAngle(JT_elbow); 
  float desShdAngle = m_arm->getDesiredJointAngle(JT_shoulder); 
  Pos p1, p2;
  m_arm->forwardKinematics(desElbAngle, desShdAngle, p1, p2);
  glColor3f(1,0,0);
  
  // convert into world space
  Vec3f desElbPos(p1.x, p1.y, 0.0f);
  Vec3f desEffPos(p2.x, p2.y, 0.0f);
  desElbPos = m_TM.transformPoint(desElbPos);
  desEffPos = m_TM.transformPoint(desEffPos);
  ci::gl::drawLine(Vec3f(0,0,0), desElbPos);
  ci::gl::drawLine(desElbPos, desEffPos);
  ci::gl::drawSphere(desElbPos, 0.01f);
  ci::gl::drawSphere(desEffPos, 0.01f);
  
  // draw trajectory
  int numTrajPoints = m_arm->getTrajectory().size();
  for(int i = 0; i < numTrajPoints-1; ++i)
  {
    float c = (float)i / (float)numTrajPoints;
    glColor4f(c,c,c,c);
    Pos p1 = m_arm->getTrajectory()[i];
    Pos p2 = m_arm->getTrajectory()[i+1];
    Vec3f trajPos1 (p1.x, p1.y, 0.0f);
    Vec3f trajPos2 (p2.x, p2.y, 0.0f);
    trajPos1 = m_TM.transformPoint(trajPos1);
    trajPos2 = m_TM.transformPoint(trajPos2);
    ci::gl::drawLine(trajPos1, trajPos2);
  }
  
  // render
  NodeGroup::update();

}

} // namespace dmx
