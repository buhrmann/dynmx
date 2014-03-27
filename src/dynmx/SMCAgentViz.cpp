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
  
  m_agentDisk = new Disk(m_agent->getRadius(), m_agent->getRadius()/2, 32);
  m_agentDisk->m_color = ci::Vec4f(0,0,0,1);
  m_agentDisk->createGeometry();
  m_children.push_back(m_agentDisk);
  
  Disk* innerDisk = new Disk(m_agent->getRadius()/2.0, 0, 32);
  //innerDisk->translate(cinder::Vec4f(0,0,0,0));
  innerDisk->m_color = ci::Vec4f(0,0,0, 1);
  innerDisk->createGeometry();
  m_children.push_back(innerDisk);
  
  m_paused = false;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentViz::reset()
{
  m_traj.clear();
  
  NodeGroup::reset();
  
  m_steps = 0;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentViz::onKeyPress(cinder::app::KeyEvent e)
{
  if(e.getCode() == ci::app::KeyEvent::KEY_SPACE)
    m_paused = !m_paused;
};
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgentViz::update()
{
  glDisable(GL_DEPTH_TEST);
  
  // data
  const ci::Vec2f& pos = m_agent->getPosition();
  float angle = m_agent->getAngle();
  
  // Update trajectory
  if(!m_paused && m_steps % 2 == 0){
    m_traj.push_back(pos);
  
    if(m_traj.size() >= 800)
    {
      m_traj.pop_front();
    }
  }
  
  m_steps++;
  
  // pose
  m_pTM->setToIdentity();  
  m_pTM->setTranslate(ci::Vec3f(pos));  
  m_pTM->rotate(ci::Vec3f(0.0f, 0.0f, 1.0f), angle);

  NodeGroup::update();
  
  // draw
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);
  
  // Trajectory
  ci::ColorA trajCol (235.0/255.0, 89.0/255.0, 55.0/255.0, 1.0);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  int numPoints =  m_traj.size();
  float lineVerts[numPoints*2];
  float colors[numPoints*4];
  glVertexPointer(2, GL_FLOAT, 0, lineVerts); // 2d positions
  glColorPointer(4, GL_FLOAT, 0, colors);     // 4d colors
  
  for(size_t i = 0; i < numPoints; i++)
  {
    lineVerts[i*2 + 0] = m_traj[i].x;
    lineVerts[i*2 + 1] = m_traj[i].y;
    float a = (float)i / (float)numPoints;
    
    colors[i*4 + 0] = trajCol[0];
    colors[i*4 + 1] = trajCol[1];
    colors[i*4 + 2] = trajCol[2];
    colors[i*4 + 3] = a;
  }
  glLineWidth(2.0);
  glDrawArrays( GL_LINE_STRIP, 0, numPoints);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);
  glLineWidth(1.0);
  
  // This is in world space
  glColor3f(0, 0, 0);
  const float velScale = 0.1f;
  glLineWidth(2.0);
  ci::gl::drawLine(m_agent->getPosition(), m_agent->getPosition() + m_agent->getVelocity() * velScale);
  glLineWidth(1.0);
  
  if(m_agent->hasDistanceSensor())
  {
    DistanceSensor* sensor = m_agent->getDistanceSensor();
    float sensedDistance = sensor->getDistance();
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0xAAAA);
    glColor3f(0.2, 0.2, 0.2);    
    ci::gl::drawLine(sensor->getPosition(), sensor->getPosition() + sensor->getDirection() * sensedDistance);
    glDisable(GL_LINE_STIPPLE);
    
    glColor3f(1,0,0);
    if(sensedDistance < sensor->getMaxDistance())
    {
      drawPoint(ci::Vec3f(sensor->getCollision()), 5.0f);
    }
    
    // Sensor in local space
    glPushMatrix();
    glMultMatrixf(*m_pTM);
    glTranslatef(m_agent->getRadius(), 0, 0.001);
    float act = m_agent->getDistanceSensor()->getDistanceProportional();
    glColor3f(act,act,act);
    drawDisk(m_agent->getRadius() / 4.0, 0, 16, 2, GLU_FILL);
    glColor3f(0,0,0);
    drawDisk(m_agent->getRadius() / 4.0, 0, 16, 2, GLU_SILHOUETTE);
    glPopMatrix();
  }
  
  if(m_agent->hasGradientSensor())
  {
    GradientSensor* sensor = m_agent->getGradientSensor();
    float s = 1.0f - sensor->getActivation();
    
    // Sensor in local space
    glPushMatrix();
    glTranslatef(sensor->getPosition().x, sensor->getPosition().y, 0.0);
    glColor3f(s,s,s);
    drawDisk(m_agent->getRadius() / 4.0, 0, 16, 2, GLU_FILL);
    glColor3f(0,0,0);
    drawDisk(m_agent->getRadius() / 4.0, 0, 16, 2, GLU_SILHOUETTE);
    glPopMatrix();
  }
  
  if(m_agent->hasTorusSensor())
  {
    Sensor* sensor = m_agent->getTorusSensor();
    float s = 1.0f - sensor->getActivation();
    
    // Sensor in local space
    glPushMatrix();
    glTranslatef(sensor->getPosition().x, sensor->getPosition().y, 0.0);
    glColor3f(s,s,s);
    drawDisk(m_agent->getRadius() / 4.0, 0, 16, 2, GLU_FILL);
    glColor3f(0,0,0);
    drawDisk(m_agent->getRadius() / 4.0, 0, 16, 2, GLU_SILHOUETTE);
    glPopMatrix();
  }

  
  // draw positional range
  if(m_agent->positionWraps())
  {
    float p = m_agent->getMaxPosition();
    ci::gl::drawStrokedRect(ci::Rectf(ci::Vec2f(-p,-p), ci::Vec2f(p,p)));
  }

  // Change body color depending on energy level
  ci::Vec3f col = getColorMapRainbow(m_agent->getEnergy() / 5);
  ci::Vec4f col4 = ci::Vec4f(col);
  col4[3] = 0.5;
  m_agentDisk->m_color = col4;
  
  glPopAttrib();
  glEnable(GL_DEPTH_TEST);
}

} // namespace