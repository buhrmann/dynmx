/*
 *  ArmMuscledViz.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 10/6/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ArmMuscledViz.h"

#include "MathUtils.h"
#include "ArmMuscled.h"
#include "Muscle.h"
#include "MuscleMonoWrap.h"
#include "MuscleBiWrap.h"

#include "CollisionDetection.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void ArmMuscledViz::init()
{
  // Let parent class initialize first
  ArmViz::init();
  
#if 0  
  // Modify the joint render size
  m_elbow.m_radius1 = m_arm->getJointRadius(JT_elbow);
  m_elbow.m_radius2 = m_elbow.m_radius1;
  m_elbow.createGeometry();
  
  m_shoulder.m_radius1 =  m_arm->getJointRadius(JT_shoulder);
  m_shoulder.m_radius2 = m_shoulder.m_radius1;
  m_shoulder.createGeometry();
#endif
  
  m_drawDesiredState = false;
  
  // Setup colors
  m_colors.shortMuscle = ci::ColorA::gray(0.8);
  m_colors.longMuscle = ci::ColorA::gray(0.8);
  m_colors.midMuscle  = ci::ColorA::black();

  m_colors.boneOutline = ci::ColorA::gray(0.5);
  m_colors.boneFill = ci::ColorA(0.9, 0.9, 0.9, 0.5);
  m_colors.jointOutline = ci::ColorA::black();
  m_colors.jointFill = ci::ColorA::white();
  m_colors.limitsFill = ci::ColorA(0.5, 0.5, 0.5, 0.5);
  m_colors.desired = ci::ColorA::gray(0.75);
  m_colors.trajectory = ci::ColorA(235.0/255.0, 89.0/255.0, 55.0/255.0, 1.0);
}

// Overwrite base class update
//----------------------------------------------------------------------------------------------------------------------
void ArmMuscledViz::update()
{
  // Let simple viz do its rendering first
  //ArmViz::update();
  
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);  
  glDisable(GL_DEPTH_TEST);  
  
  // Get shoulder and elbow TMs and other useful data
  const float shdAngle = m_arm->getJointAngle(JT_shoulder);
  const float elbAngle = m_arm->getJointAngle(JT_elbow);
  
  const ci::Vec3f elbPos = Vec3f(m_arm->getElbowPos());
  //const ci::Vec3f effPos = Vec3f(m_arm->getEffectorPos());
  const ci::Vec3f shdPos = Vec3f(0,0,0);
  
  const double elbRad = m_arm->getJointRadius(JT_elbow);
  const double shdRad = m_arm->getJointRadius(JT_shoulder);   

  ci::Matrix44f elbLimTM;
  elbLimTM.setToIdentity();
  elbLimTM.rotate(Vec3f(0.0f, 0.0f, 1.0f), shdAngle); 
  elbLimTM.setTranslate(elbPos);    
  
  ci::Matrix44f elbTM = elbLimTM;
  elbTM.rotate(Vec3f(0,0,1), elbAngle);
  
  ci::Matrix44f shdTM;
  shdTM.setToIdentity();
  shdTM.rotate(Vec3f(0.0f, 0.0f, 1.0f), shdAngle); 
  shdTM.setTranslate(shdPos);      

  // Overall position and orientation
  glPushMatrix();
  glMultMatrixf(*m_pTM);   
  glLineWidth(1.0);
  glColor3f(0,0,0);
  
  // Desired kinematic state
  if(m_drawDesiredState)
  {
    Pos p1, p2;    
    double desElbAngle = m_arm->getDesiredJointAngle(JT_elbow); 
    double desShdAngle = m_arm->getDesiredJointAngle(JT_shoulder); 
    m_arm->forwardKinematics(desElbAngle, desShdAngle, p1, p2);
    
    Vec3f desShdPos(0,0,0);
    Vec3f desElbPos(p1);
    Vec3f desEffPos(p2);
    ci::gl::color(m_colors.desired);
    // Draw bones
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0xAAAA);
    ci::gl::drawLine(desShdPos, desElbPos);
    ci::gl::drawLine(desElbPos, desEffPos);
    glDisable(GL_LINE_STIPPLE);
    
    // Points indicating joints
    glPointSize(4.0);
    glBegin(GL_POINTS);
    glVertex3f(desShdPos.x, desShdPos.y, 0.0);
    glVertex3f(desElbPos.x, desElbPos.y, 0.0);
    glVertex3f(desEffPos.x, desEffPos.y, 0.0);
    glEnd();
  }    
  
  // Draw elbow joint and bone
  glPushMatrix();
  glMultMatrixf(elbTM);
  // bone triangle
  ci::gl::color(m_colors.boneFill);
  ci::Vec3f lt = -elbRad * ci::Vec3f(0,1,0);
  ci::Vec3f rt = elbRad * ci::Vec3f(0,1,0);
  ci::Vec3f bt = m_arm->getLength(JT_elbow) * ci::Vec3f(1,0,0);
  drawTriangle(rt, lt, bt);
  ci::gl::color(m_colors.boneOutline);
  drawTriangle(rt, lt, bt, GL_LINE);
  ci::gl::drawLine(ci::Vec2f(&bt.x), elbRad * ci::Vec2f(1,0));
  // joint disk
  ci::gl::color(m_colors.jointFill);
  ci::gl::drawSolidCircle(ci::Vec2f(0,0), elbRad, 32);
  ci::gl::color(m_colors.jointOutline);
  ci::gl::drawStrokedCircle(ci::Vec2f(0,0), elbRad, 32);

  glPopMatrix();
  
  // Draw elbow limits
  glPushMatrix();
  glMultMatrixf(elbLimTM);
  float limMin = radiansToDegrees(m_arm->getJointLimitLower(JT_elbow));
  float limMax = radiansToDegrees(m_arm->getJointLimitUpper(JT_elbow));
  ci::gl::color(m_colors.limitsFill);
  drawPartialDisk(elbRad, elbRad + 0.01, 16, 1, 90 - limMin, -(limMax - limMin));
  glPopMatrix();
  
  // Draw shoulder joint and bone
  glColor3f(0,0,0);  
  glPushMatrix();
  glMultMatrixf(shdTM);
  // bone triangle
  lt = -shdRad * ci::Vec3f(0,1,0);
  rt = shdRad * ci::Vec3f(0,1,0);
  bt = m_arm->getLength(JT_shoulder) * ci::Vec3f(1,0,0);  
  ci::gl::color(m_colors.boneFill);
  drawTriangle(rt, lt, bt);
  ci::gl::color(m_colors.boneOutline);
  drawTriangle(rt, lt, bt, GL_LINE);
  ci::gl::drawLine(m_arm->getLength(JT_shoulder) * ci::Vec2f(1,0), shdRad * ci::Vec2f(1,0));
  // joint disk
  ci::gl::color(m_colors.jointFill);
  ci::gl::drawSolidCircle(ci::Vec2f(0,0), shdRad, 32);
  ci::gl::color(m_colors.jointOutline);
  ci::gl::drawStrokedCircle(ci::Vec2f(0,0), shdRad, 32);
  glPopMatrix();
  
  // Draw shoulder limits
  limMin = m_arm->getJointLimitLower(JT_shoulder) * RAD_TO_DEG;
  limMax = m_arm->getJointLimitUpper(JT_shoulder) * RAD_TO_DEG;  
  ci::gl::color(m_colors.limitsFill);
  drawPartialDisk(shdRad, shdRad + 0.01, 16, 1, 90 - limMin, -(limMax - limMin));
  glLineWidth(1.0);    
  
  // Trajectory  
  if(m_drawOverlays)
  {
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    const std::deque<Pos>& effTrajectory = m_arm->getTrajectory();
    int numPoints =  effTrajectory.size();  
    float lineVerts[numPoints*2];
    float colors[numPoints*4];
    glVertexPointer(2, GL_FLOAT, 0, lineVerts); // 2d positions
    glColorPointer(4, GL_FLOAT, 0, colors);     // 4d colors
    
    for(size_t i = 0; i < numPoints; i++)
    {
      lineVerts[i*2 + 0] = effTrajectory[i].x;
      lineVerts[i*2 + 1] = effTrajectory[i].y;
      float a = 0.5 * (float)i / (float)numPoints;
      
      colors[i*4 + 0] = m_colors.trajectory[0];
      colors[i*4 + 1] = m_colors.trajectory[1];
      colors[i*4 + 2] = m_colors.trajectory[2];
      colors[i*4 + 3] = a;
    }
    glLineWidth(2.0);
    glDrawArrays( GL_LINE_STRIP, 0, numPoints);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glLineWidth(1.0);  
  }
  
  // Draw muscles
  for(size_t i = 0; i < m_arm->getNumMuscles(); ++i)
  {
    Muscle* muscle = m_arm->getMuscle(i);
    double act = muscle->getActivation();
    
    // Muscle line width dependent on activation
    float lineWidth = act * 32.0;
    glLineWidth(lineWidth);

    // Muscle colour dependent on length
    float l = muscle->getNormalisedLength() - 1;
    l = clamp(l, -1.0f, 1.0f);

    ci::ColorA col = m_colors.midMuscle;
    if(l >= 0)
    {
      col += (m_colors.longMuscle - m_colors.midMuscle) * l;
    }
    else
    {
      col -= (m_colors.shortMuscle - m_colors.midMuscle) * l;
    }    
    ci::gl::color(col);
    
    // Draw origin and insertion points
    Vec3f origin = Vec3f(muscle->getOriginWorld());
    Vec3f insertion = Vec3f(muscle->getInsertionWorld());    
    glPointSize(4.0);
    glBegin(GL_POINTS);
    glVertex3f(origin.x, origin.y, 0);
    glVertex3f(insertion.x, insertion.y, 0);
    glEnd();
    
    if(muscle->isMonoArticulate())
    {
      MuscleMonoWrap* m = ((MuscleMonoWrap*)muscle);        
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
        pathDir = (m->m_joint == JT_elbow) ? (m_arm->getElbowPos() - m_arm->getEffectorPos()) : ( -m_arm->getElbowPos());
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
      MuscleBiWrap* m = ((MuscleBiWrap*)muscle);
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
        pathDir = pathDir * m_arm->getJointRadius(JT_elbow);
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
        pathDir = pathDir * m_arm->getJointRadius(JT_elbow);
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
  
  glLineWidth(1.0);
  
  glPopMatrix();
  glEnable(GL_DEPTH_TEST);    
  
  glPopAttrib();  
}

} // namespace dmx

