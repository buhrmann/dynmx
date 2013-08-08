/*
 *  SMCAgentViz1d.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_AGENT_VIZ_1D_
#define _DMX_SMC_AGENT_VIZ_1D_

#include "Scene.h"
#include "SMCAgentMeta1d.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// Visualisation of an SMCAgent
//----------------------------------------------------------------------------------------------------------------------
class SMCAgentViz1d : public NodeGroup
{
public:
  
  SMCAgentViz1d(SMCAgentMeta1d* agent) : m_agent(agent) { init(); };
  
  virtual void update();
  
  int m_agentViewMode;
  
protected:
  
  virtual void init();
  
  float m_r;
  SMCAgentMeta1d* m_agent;
  Cylinder* m_arrow;
  Disk* m_sensor;
  Disk* m_body;
  ci::Font m_font; 
};

//----------------------------------------------------------------------------------------------------------------------
// Inlines
//----------------------------------------------------------------------------------------------------------------------
void SMCAgentViz1d::init()
{
  NodeGroup::init();
  
  m_font = ci::Font(ci::app::loadResource("pf_tempesta_seven.ttf"), 8); 
  
  m_agentViewMode = 2;
  
  m_r = 0.05;
  
  float senrad = 0.2 * m_r;
  m_sensor = new Disk(senrad, 0);
  m_sensor->m_color = ci::Vec4f(1, 1, 1, 1);
  m_sensor->translate(ci::Vec4f(m_r + senrad, 0, 0, 1));
  m_children.push_back(m_sensor);
  
  Disk* sensorFrame = new Disk(senrad, 0, GLU_SILHOUETTE);
  sensorFrame->m_color = ci::Vec4f(0, 0, 0, 1);
  sensorFrame->translate(ci::Vec4f(m_r + senrad, 0, 0, 1));
  m_children.push_back(sensorFrame);
  
  m_body = new Disk(m_r, 0);
  m_body->m_color = ci::Vec4f(0.5, 0.2, 0.5, 0.0);
  m_children.push_back(m_body);

  Disk* bodyFrame = new Disk(m_r, 0, GLU_SILHOUETTE);
  bodyFrame->m_color = ci::Vec4f(0, 0, 0, 1);
  m_children.push_back(bodyFrame);

}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentViz1d::update()
{
  if(m_children.size() <= 4 && m_agentViewMode > 0)
  {
    CTRNNBasicViz* annviz;
    if (m_agentViewMode == 1){
      annviz = new CTRNNWheelViz(m_agent->getCTRNN(), 1.5 * m_r, false);
      annviz->renderConnections(false);
    }
    else{
      annviz = new CTRNNNeuronViz(m_agent->getCTRNN(), 1.8 * m_r, false);
    }
    annviz->rotate(ci::Vec4f(0,0,1,1), PI/2);
    m_children.insert(m_children.begin(), annviz);
  }
  
  // data
  const float p = m_agent->getPosition();
  const float v = m_agent->getVelocity();
  const ci::Vec3f pos = ci::Vec3f(0, p, 0);
  const ci::Vec3f vel = ci::Vec3f(0, v, 0);
  
  // pose
  m_pTM->setToIdentity();
  m_pTM->setTranslate(pos);
  
  // draw
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);
  
  // This is in world space
  // Draw velocity vector
  glColor3f(0, 0, 0);
  const float velScale = 0.1f;
  const float h = 0.01f;
  int dir = sign(v);
  glLineWidth(2.0);
  ci::Vec3f start = pos + ci::Vec3f(0, m_r * dir, 0);
  ci::Vec3f end = start + vel * velScale;
  ci::Vec3f p1 = end + ci::Vec3f(h,0,0);
  ci::Vec3f p2 = end + ci::Vec3f(0, h * dir, 0);
  ci::Vec3f p3 = end + ci::Vec3f(-h,0,0);
  ci::gl::drawLine(start, end);
  if (dir > 0)
    drawTriangle(p1, p2, p3);
  else
    drawTriangle(p1, p3, p2);
  
  glLineWidth(1.0);
  
  // Draw distance sensor ray and collision point
  const DistanceSensor& sensor = m_agent->getDistanceSensor();
  float sensedDistance = sensor.getDistance();
  glEnable(GL_LINE_STIPPLE);
  glLineStipple(2, 0xAAAA);
  glColor3f(0.2, 0.2, 0.2);
  ci::gl::drawLine(sensor.getPosition(), sensor.getPosition() + sensor.getDirection() * sensedDistance);
  glDisable(GL_LINE_STIPPLE);
  
  glColor3f(1,0,0);
  if(sensedDistance < sensor.getMaxDistance())
  {
    drawPoint(ci::Vec3f(sensor.getCollision()), 5.0f);
  }
  
  glPopAttrib();
  
  // Change sensor color depending on distance
  float act = m_agent->getDistanceSensor().getDistanceProportional();
  m_sensor->m_color = ci::Vec4f(act, act, act, 1.f);
  
  // Change body color depending on energy level
  ci::Vec3f col = getColorMapRainbow(m_agent->getEnergy() / 20);
  ci::Vec4f col4 = ci::Vec4f(col);
  col4[3] = 0.1;
  m_body->m_color = col4;

  // Draw rest of agent
  NodeGroup::update();
}

  
} // namespace

#endif