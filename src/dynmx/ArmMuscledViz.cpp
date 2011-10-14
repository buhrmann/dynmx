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

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void ArmMuscledViz::init()
{
  // Let parent class initialize first
  ArmViz::init();
  
  // Modify the joint render size
  m_elbow.m_radius1 = m_arm->getJointRadius(JT_elbow);
  m_elbow.m_radius2 = m_elbow.m_radius1;
  m_elbow.createGeometry();
  
  m_shoulder.m_radius1 =  m_arm->getJointRadius(JT_shoulder);
  m_shoulder.m_radius2 = m_shoulder.m_radius1;
  m_shoulder.createGeometry();
  
  m_drawDesiredState = false;
}

// Overwrite base class update
//----------------------------------------------------------------------------------------------------------------------
void ArmMuscledViz::update()
{
  // Let simple viz do its rendering first
  ArmViz::update();
  
  // Overlay desired state
  glDisable(GL_DEPTH_TEST);
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
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
    glColor4f(1.0, 0.25, 0.25, 0.5);    
    // Draw bones
    ci::gl::drawLine(desShdPos, desElbPos);
    ci::gl::drawLine(desElbPos, desEffPos);

    // Points indicating joints
    glPointSize(4.0);
    glBegin(GL_POINTS);
      glVertex3f(desShdPos.x, desShdPos.y, 0.0);
      glVertex3f(desElbPos.x, desElbPos.y, 0.0);
      glVertex3f(desEffPos.x, desEffPos.y, 0.0);
    glEnd();
  }  
  
  // Trajectory  
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  const std::deque<Pos>& effTrajectory = m_arm->getTrajectory();
  int numPoints =  effTrajectory.size();  
  float lineVerts[numPoints*2];
  float colors[numPoints*4];
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
  
  // Draw desired trajectory: assume it's same size as actual trajectory, which it should be  
  /*
  const std::deque<Pos>& desTrajectory = ((ArmMuscled*)m_arm)->getDesiredTrajectory();  
  numPoints =  desTrajectory.size();    
  float lineVertsDes[numPoints*2];
  float colorsDes[numPoints*4];
  glVertexPointer(2, GL_FLOAT, 0, lineVertsDes); // 2d positions
  glColorPointer(4, GL_FLOAT, 0, colorsDes);     // 4d colors  
  for(size_t i = 0; i < numPoints; i++)
  {
    lineVertsDes[i*2 + 0] = desTrajectory[i].x;
    lineVertsDes[i*2 + 1] = desTrajectory[i].y;
    float c = 0.5f * (float)i / (float)numPoints;
    colorsDes[i*4 + 0] = 1;
    colorsDes[i*4 + 1] = 0;
    colorsDes[i*4 + 2] = 0;
    colorsDes[i*4 + 3] = c;
  }
  glDrawArrays( GL_LINE_STRIP, 0, numPoints);
  */
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);

  
  
  // Draw muscles
  for(size_t i = 0; i < m_arm->getNumMuscles(); ++i)
  {
    Muscle* muscle = m_arm->getMuscle(i);

    // Muscle colour dependent on length
    float l = muscle->getNormalisedLength() - 1;
    l = clamp(l, -1.0f, 1.0f);
    setColor3(getColorMapBlueRed(l));
    
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
  
  glPopMatrix();
  glEnable(GL_DEPTH_TEST);    
}

} // namespace dmx

