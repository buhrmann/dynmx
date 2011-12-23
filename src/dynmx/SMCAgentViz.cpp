/*
 *  SMCAgentViz.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCAgentViz.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentViz::init() 
{ 
  NodeGroup::init(); 
  
  m_agentDisk = new Disk(m_agent->getRadius(), 0.0, 32);
  m_agentDisk->m_color = ci::Vec4f(0,0,0,1);
  m_agentDisk->createGeometry();
  m_children.push_back(m_agentDisk);
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgentViz::update()
{
  NodeGroup::update();
  
  // data
  const ci::Vec2f& pos = m_agent->getPosition();
  float angle = m_agent->getAngle();
  
  // pose
  m_pTM->setToIdentity();  
  m_pTM->setTranslate(ci::Vec3f(pos));  
  m_pTM->rotate(ci::Vec3f(0.0f, 0.0f, 1.0f), angle);
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // draw
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);
  glColor3f(0.2, 0.2, 0.2);  
  //ci::gl::drawSolidCircle(ci::Vec2f(0,0), radius, 32);
  
  glPopMatrix();
  
  // This is in world space
  float sensedDistance = m_agent->getDistanceSensor().getDistance();
  glEnable(GL_LINE_STIPPLE);
  glLineStipple(2, 0xAAAA);
  ci::gl::drawLine(pos, sensedDistance * m_agent->getDistanceSensor().getDirection());  
  glDisable(GL_LINE_STIPPLE);
  if(sensedDistance < m_agent->getDistanceSensor().getMaxDistance())
  {  
    drawPoint(ci::Vec3f(m_agent->getDistanceSensor().getCollision()), 5.0f);
  }
  
  glPopAttrib();
}

} // namespace