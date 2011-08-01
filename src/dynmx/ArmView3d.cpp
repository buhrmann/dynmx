/*
 *  Arm3dView.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "ArmView3d.h"
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
  
  m_elbow.m_TM.setTranslate(Vec3f(m_arm->getElbowPos()));
  
  // After updating poses, render the scene graph objects
  //NodeGroup::update();  
  
  // Overlay desired state
  glDisable(GL_DEPTH_TEST);
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // Desired kinematic state
#if 0
  const bool drawDesired = true;
  if(drawDesired)
  {
    double desElbAngle = ((ArmPD*)m_arm)->getDesiredJointAngle(JT_elbow); 
    double desShdAngle = ((ArmPD*)m_arm)->getDesiredJointAngle(JT_shoulder); 
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
  
  glPushMatrix();
  glMultMatrixf(m_elbow.m_TM);
  drawDisk(m_elbow.m_radius1, 0, 16, 1);
  glPopMatrix();
  
  glPushMatrix();
  glMultMatrixf(m_shoulder.m_TM);
  drawDisk(m_shoulder.m_radius1, 0, 16, 1);
  glPopMatrix();

  
  // draw trajectory
  int numTrajPoints = m_arm->getTrajectory().size();
  for(int i = 0; i < numTrajPoints - 1; ++i)
  {
    float c = (float)i / (float)numTrajPoints;
    glColor4f(c,c,c,c);
    Vec3f p1 = Vec3f(m_arm->getTrajectory()[i]);
    Vec3f p2 = Vec3f(m_arm->getTrajectory()[i+1]);
    ci::gl::drawLine(p1, p2);
  }

  
  // Todo: this is nasty!
  if(m_hasMuscles)
  {
    ArmMuscled* armMusc = (ArmMuscled*)m_arm;
    
    // TEST
    for(size_t i = 0; i < armMusc->getNumMuscles(); ++i)
    {
      if(armMusc->getMuscle(i)->isMonoArticulate())
        glColor3f(0.9, 0.9, 0.9);
      else
        glColor3f(0.6, 0.6, 0.6);    
      
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
          ci::Vec2f dir = m->getInsertionWorld() - m->getOriginWorld();
          dir.normalize();
          // Only hold in the non-wrapping case. Otherwise we need to do a proper projection (dot product).
          ci::Vec2f closestPoint = m->getOriginWorld() + dir * m->m_originCapsuleDist;
          //ci::Vec2f maVec = closestPoint - m_arm->getElbowPos();
          //float ma = maVec.length();
          //const bool muscleWraps = ma < r && m_arm->getJointAngle(JT_elbow) < PI_OVER_TWO;
          //ci::gl::drawLine(ci::Vec3f(closestPoint), ci::Vec3f(m_arm->getElbowPos())); 
          ci::gl::drawLine(origin, insertion); 
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
      }
    } // for all muscles
  } // if has muscles
  
  glPopMatrix();
  glEnable(GL_DEPTH_TEST);    
}

} // namespace dmx
