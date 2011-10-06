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
Vec3f Arm3dView::toLocalSpace(Vec3f p)
{
  p -= m_TM.getTranslate().xyz();
  return m_TM.preMultiply(p);
}
  
//----------------------------------------------------------------------------------------------------------------------
void Arm3dView::init()
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
  
  m_elbow.m_radius1 = m_hasMuscles ? ((ArmMuscled*)m_arm)->getJointRadius(JT_elbow) : 1.1 * m_lowerArm.m_radius1;
  m_elbow.m_radius2 = m_elbow.m_radius1;
  m_elbow.m_length = m_lowerArm.m_radius1*2;  
  m_elbow.m_slices = 16;
  m_elbow.m_stacks = 1;
  m_elbow.createGeometry();
  m_elbow.m_color = col;
  m_elbow.m_outlineWidth = 2.0f;
    
  m_shoulder.m_radius1 =  m_hasMuscles ? ((ArmMuscled*)m_arm)->getJointRadius(JT_shoulder) : 1.1f * m_upperArm.m_radius2;
  m_shoulder.m_radius2 = m_shoulder.m_radius1;
  m_shoulder.m_length = m_upperArm.m_radius1*2;
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
void Arm3dView::update()
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
  glDisable(GL_DEPTH_TEST);
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // Desired kinematic state
#if 1
  const bool drawDesired = true;
  if(drawDesired)
  {
    double desElbAngle = ((ArmMuscled*)m_arm)->getDesiredJointAngle(JT_elbow); 
    double desShdAngle = ((ArmMuscled*)m_arm)->getDesiredJointAngle(JT_shoulder); 
    Pos p1, p2;
    m_arm->forwardKinematics(desElbAngle, desShdAngle, p1, p2);
    glColor4f(1.0, 0.25, 0.25, 0.5);
    
    Vec3f desShdPos(0,0,0);
    Vec3f desElbPos(p1);
    Vec3f desEffPos(p2);
    ci::gl::drawLine(desShdPos, desElbPos);
    ci::gl::drawLine(desElbPos, desEffPos);
#if 1 
    glPointSize(4.0);
    glBegin(GL_POINTS);
    glVertex3f(desShdPos.x, desShdPos.y, 0.0);
    glVertex3f(desElbPos.x, desElbPos.y, 0.0);
    glVertex3f(desEffPos.x, desEffPos.y, 0.0);
    glEnd();
#endif    
  }  
#endif  
  
  // Arm as line drawing
  glLineWidth(4.0);
  glColor3f(0,0,0);
  ci::gl::drawLine(Vec3f(0,0,0), Vec3f(m_arm->getElbowPos()));  
  ci::gl::drawLine(Vec3f(m_arm->getElbowPos()), Vec3f(m_arm->getEffectorPos()));  
  glLineWidth(1.0);  
  
  // Joints and limits
  glPushMatrix();
  glMultMatrixf(m_elbow.m_TM);
  float r = m_hasMuscles ? ((ArmMuscled*)m_arm)->getJointRadius(JT_elbow) : 1.1 * m_lowerArm.m_radius1; 
  drawDisk(r, 0, 16, 1);
  float limMin = m_arm->getJointLimitLower(JT_elbow) * RAD_TO_DEG;
  float limMax = m_arm->getJointLimitUpper(JT_elbow) * RAD_TO_DEG;
  glColor4f(1,1,1,0.5);
  drawPartialDisk(r, r+0.01, 16, 1, 90 - limMin, -(limMax - limMin));
  glPopMatrix();
  
  glColor3f(0,0,0);
  glPushMatrix();
  //glMultMatrixf(m_shoulder.m_TM);
  r = m_hasMuscles ? ((ArmMuscled*)m_arm)->getJointRadius(JT_shoulder) : 1.1 * m_upperArm.m_radius1;
  drawDisk(r, 0, 16, 1);
  limMin = m_arm->getJointLimitLower(JT_shoulder) * RAD_TO_DEG;
  limMax = m_arm->getJointLimitUpper(JT_shoulder) * RAD_TO_DEG;  
  glColor4f(1,1,1,0.5);
  drawPartialDisk(r, r+0.01, 16, 1, 90 - limMin, -(limMax - limMin));
  glPopMatrix();
  
  // Trajectory  
  const std::deque<Pos>& effTrajectory = m_arm->getTrajectory();
  const int numPoints =  effTrajectory.size();  
  float lineVerts[numPoints*2];
  float colors[numPoints*4];
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glVertexPointer(2, GL_FLOAT, 0, lineVerts); // 2d positions
  glColorPointer(4, GL_FLOAT, 0, colors);     // 4d colors
  
  // Draw actual trajectory
  for(size_t i = 0; i < numPoints; i++)
  {
    lineVerts[i*2 + 0] = effTrajectory[i].x;
    lineVerts[i*2 + 1] = effTrajectory[i].y;
    float c = (float)i / (float)numPoints;
    colors[i*4 + 0] = c;
    colors[i*4 + 1] = c;
    colors[i*4 + 2] = c;
    colors[i*4 + 3] = c;
  }
  glDrawArrays( GL_LINE_STRIP, 0, numPoints);

  // Draw desired trajectory
  const std::deque<Pos>& desTrajectory = ((ArmMuscled*)m_arm)->getDesiredTrajectory();  
  for(size_t i = 0; i < numPoints; i++)
  {
    lineVerts[i*2 + 0] = desTrajectory[i].x;
    lineVerts[i*2 + 1] = desTrajectory[i].y;
    float c = 0.5f * (float)i / (float)numPoints;
    colors[i*4 + 0] = 1;
    colors[i*4 + 1] = 0;
    colors[i*4 + 2] = 0;
    colors[i*4 + 3] = c;
  }
  glDrawArrays( GL_LINE_STRIP, 0, numPoints);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);
  
  
  // Todo: this is nasty!
  if(m_hasMuscles)
  {
    ArmMuscled* armMusc = (ArmMuscled*)m_arm;
    
    // TEST
    for(size_t i = 0; i < armMusc->getNumMuscles(); ++i)
    {
      float l = armMusc->getMuscle(i)->getNormalisedLength() - 1;
      l = clamp(l, -1.0f, 1.0f);
      setColor3(getColorMapBlueRed(l));
      /*
      if(armMusc->getMuscle(i)->isMonoArticulate())        
        glColor3f(0.9, 0.9, 0.9);
      else
        glColor3f(0.6, 0.6, 0.6);    
       */
      
      Vec3f origin = Vec3f(armMusc->getMuscle(i)->getOriginWorld());
      Vec3f insertion = Vec3f(armMusc->getMuscle(i)->getInsertionWorld());
      
      glPointSize(4.0);
      glBegin(GL_POINTS);
      glVertex3f(origin.x, origin.y, 0);
      glVertex3f(insertion.x, insertion.y, 0);
      glEnd();
      
      if(armMusc->getMuscle(i)->isMonoArticulate())
      {
        MuscleMonoWrap* m = ((MuscleMonoWrap*)armMusc->getMuscle(i));        
        if(!m->m_muscleWraps)
        {
          //ci::Vec2f dir = m->getInsertionWorld() - m->getOriginWorld();
          //dir.normalize();
          // Only hold in the non-wrapping case. Otherwise we need to do a proper projection (dot product).
          //ci::Vec2f closestPoint = m->getOriginWorld() + dir * m->m_originCapsuleDist;
          //ci::Vec2f maVec = closestPoint - m_arm->getElbowPos();
          //float ma = maVec.length();
          //const bool muscleWraps = ma < r && m_arm->getJointAngle(JT_elbow) < PI_OVER_TWO;
          //ci::gl::drawLine(ci::Vec3f(closestPoint), ci::Vec3f(m_arm->getElbowPos())); 
          ci::gl::drawLine(origin, insertion); 
          // Indicate optimal length
          //ci::Vec3f l0Pos = origin + m->getOptimalLength() * (insertion - origin).normalized();
          //drawPoint(l0Pos, 4.0);
        }    
        else
        {
          // Upstream segment
          ci::Vec2f pathDir = (m->m_joint == JT_elbow) ? m_arm->getElbowPos() : ci::Vec2f(1, 0);
          float rotDir = m->m_isFlexor ? 1.0 : -1.0;
          pathDir.rotate(rotDir * (PI_OVER_TWO - m->m_originCapsuleAngle));
          pathDir.normalize();
          ci::Vec2f pathEnd = m->getOriginWorld() + (pathDir * m->m_originCapsuleDist);
          ci::gl::drawLine(origin, ci::Vec3f(pathEnd));

          // Downstream segment
          pathDir = (m->m_joint == JT_elbow) ? 
            (m_arm->getElbowPos() - m_arm->getEffectorPos()) :
            (-m_arm->getElbowPos());
          pathDir.rotate(-rotDir * (PI_OVER_TWO - m->m_insertCapsuleAngle));
          pathDir.normalize();
          pathEnd = m->getInsertionWorld() + (pathDir * m->m_insertCapsuleDist);
          ci::gl::drawLine(insertion, ci::Vec3f(pathEnd));     
        }
      } // is mono
      else 
      {
        glPushAttrib (GL_LINE_BIT);
        glEnable(GL_LINE_STIPPLE);
        glLineStipple (1, 0xAAAA);
        MuscleBiWrap* m = ((MuscleBiWrap*)armMusc->getMuscle(i));
        if (m->wrapsElbow() && m->wrapsShoulder())
        {
          //glColor3f(0,0,1);
          // Upstream segment: always shoulder
          ci::Vec2f pathDir = ci::Vec2f(1, 0);
          float rotDir = m->m_isFlexor ? 1.0 : -1.0;
          pathDir.rotate(rotDir * (PI_OVER_TWO - m->m_originCapsuleAngle));
          pathDir.normalize();
          ci::Vec2f pathEnd = m->getOriginWorld() + (pathDir * m->m_originCapsuleDist);
          ci::gl::drawLine(origin, ci::Vec3f(pathEnd));        
          // Downstream segment: always elbow.
          pathDir = m_arm->getElbowPos() - m_arm->getEffectorPos();
          pathDir.rotate(-rotDir * (PI_OVER_TWO - m->m_insertCapsuleAngle));
          pathDir.normalize();
          pathEnd = m->getInsertionWorld() + (pathDir * m->m_insertCapsuleDist);
          ci::gl::drawLine(insertion, ci::Vec3f(pathEnd));
          // Middle segment
          pathDir = m_arm->getElbowPos();
          pathDir.rotate(rotDir * m->m_gammaAngle);
          pathDir.normalize();
          pathDir = pathDir * m->m_momentArms[JT_shoulder];
          ci::Vec2f pathStart = pathDir;
          
          pathDir = m_arm->getEffectorPos() - m_arm->getElbowPos();
          pathDir.rotate(rotDir * (m->m_insertCapsuleAngle + m->getWrapAngle(JT_elbow)));
          pathDir.normalize();
          pathDir = pathDir * armMusc->getJointRadius(JT_elbow);
          pathEnd = m_arm->getElbowPos() + pathDir;      
          ci::gl::drawLine(ci::Vec3f(pathStart), ci::Vec3f(pathEnd));          
        }
        else if (m->wrapsElbow())
        {
          //glColor3f(0,1,0);
          // Downstream segment: always elbow. Upstream segment from origin to capsule
          ci::Vec2f pathDir = ci::Vec2f(m_arm->getElbowPos() - m_arm->getEffectorPos());
          float rotDir = m->m_isFlexor ? 1.0 : -1.0;
          pathDir.rotate(-rotDir * (PI_OVER_TWO - m->m_insertCapsuleAngle));
          pathDir.normalize();
          ci::Vec2f pathEnd = m->getInsertionWorld() + (pathDir * m->m_insertCapsuleDist);
          ci::gl::drawLine(insertion, ci::Vec3f(pathEnd));

          pathDir = m_arm->getEffectorPos() - m_arm->getElbowPos();
          pathDir.rotate(rotDir * (m->m_insertCapsuleAngle + m->getWrapAngle(JT_elbow)));
          pathDir.normalize();
          pathDir = pathDir * armMusc->getJointRadius(JT_elbow);
          pathEnd = m_arm->getElbowPos() + pathDir;
          ci::gl::drawLine(origin, ci::Vec3f(pathEnd));
        }
        else if (m->wrapsShoulder())
        {
          //glColor3f(1,0,0);
          // Upstream segment: always shoulder
          ci::Vec2f pathDir = ci::Vec2f(1, 0);
          float rotDir = m->m_isFlexor ? 1.0 : -1.0;
          pathDir.rotate(rotDir * (PI_OVER_TWO - m->m_originCapsuleAngle));
          pathDir.normalize();
          ci::Vec2f pathEnd = m->getOriginWorld() + (pathDir * m->m_originCapsuleDist);
          ci::gl::drawLine(origin, ci::Vec3f(pathEnd));
          
          //Downstream segment: from capsule to insertion
          pathDir = m_arm->getElbowPos();
          pathDir.rotate(rotDir * m->m_gammaAngle);
          pathDir.normalize();
          pathDir = pathDir * m->m_momentArms[JT_shoulder];
          ci::Vec2f pathStart = pathDir;
          ci::gl::drawLine(ci::Vec3f(pathStart), insertion);
        }
        else
        {
          // No wrap
          //glColor3f(1,1,1);
          ci::gl::drawLine(origin, insertion);          
        }
        glPopAttrib();
      }
    } // for all muscles
  } // if has muscles
  
  glPopMatrix();
  glEnable(GL_DEPTH_TEST);    
}

} // namespace dmx
