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
  
  m_agentDisk = new Disk(m_agent->getRadius(), m_agent->getRadius()/2.0, 32);
  m_agentDisk->m_color = ci::Vec4f(0,0,0,1);
  m_agentDisk->createGeometry();
  m_children.push_back(m_agentDisk);
  
  Disk* innerDisk = new Disk(m_agent->getRadius()/2.0, 0, 32);
  innerDisk->m_color = ci::Vec4f(1,1,1, 1);
  innerDisk->createGeometry();
  m_children.push_back(innerDisk);  
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgentViz::update()
{
  // data
  const ci::Vec2f& pos = m_agent->getPosition();
  float angle = m_agent->getAngle();
  
  // pose
  m_pTM->setToIdentity();  
  m_pTM->setTranslate(ci::Vec3f(pos));  
  m_pTM->rotate(ci::Vec3f(0.0f, 0.0f, 1.0f), angle);

  NodeGroup::update();
  
  // draw
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);
  
  // This is in world space
  const DistanceSensor& sensor = m_agent->getDistanceSensor();
  float sensedDistance = sensor.getDistance();
  glEnable(GL_LINE_STIPPLE);
  glLineStipple(2, 0xAAAA);
  glColor3f(0.2, 0.2, 0.2);    
  ci::gl::drawLine(sensor.getPosition(), sensor.getPosition() + sensor.getDirection() * sensedDistance);  
  glDisable(GL_LINE_STIPPLE);
  if(sensedDistance < sensor.getMaxDistance())
  {  
    drawPoint(ci::Vec3f(sensor.getCollision()), 5.0f);
  }
  
  // Sensor in local space
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  glTranslatef(m_agent->getRadius(), 0, 0.001);
  float act = m_agent->getDistanceSensor().getDistanceProportional();
  glColor3f(act,act,act);
  drawDisk(m_agent->getRadius() / 4.0, 0, 16, 2, GLU_FILL);
  glColor3f(0,0,0);
  drawDisk(m_agent->getRadius() / 4.0, 0, 16, 2, GLU_SILHOUETTE);
  glPopMatrix();    
  
  glPopAttrib();
}

} // namespace