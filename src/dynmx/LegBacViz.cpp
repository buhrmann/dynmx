/*
 *  LegBacViz.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "LegBacViz.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
void LegBacViz::init()
{
  NodeGroup::init();
  
  m_paused = false;
}

//----------------------------------------------------------------------------------------------------------------------
void LegBacViz::reset()
{
  m_traj.clear();
  
  NodeGroup::reset();
  
  m_steps = 0;
}

//----------------------------------------------------------------------------------------------------------------------
void LegBacViz::onKeyPress(cinder::app::KeyEvent e)
{
  if(e.getCode() == ci::app::KeyEvent::KEY_SPACE)
    m_paused = !m_paused;
};

//----------------------------------------------------------------------------------------------------------------------
void LegBacViz::drawLeg()
{
  glColor3f(0,0,0);
  glPushMatrix();
  ci::gl::translate(m_agent->m_leg.m_position);
  drawDisk(1, 0, 16, 2, GLU_FILL);
  glPopMatrix();

  glPushMatrix();
  ci::gl::translate(m_agent->m_leg.m_joint);
  drawDisk(0.75, 0, 16, 2, GLU_FILL);
  glPopMatrix();

  if(m_agent->m_leg.m_footDown)
    glColor3f(1,0,0);
  glPushMatrix();
  ci::gl::translate(m_agent->m_leg.m_foot);
  drawDisk(0.5, 0, 16, 2, GLU_FILL);
  glPopMatrix();
  
  glColor3f(0,0,0);
  ci::gl::drawLine(m_agent->m_leg.m_position, m_agent->m_leg.m_joint);
  ci::gl::drawLine(m_agent->m_leg.m_joint, m_agent->m_leg.m_foot);
}
  
//----------------------------------------------------------------------------------------------------------------------
void LegBacViz::drawBac()
{
  m_pTM->rotate(ci::Vec3f(0.0f, 0.0f, 1.0f), m_agent->m_bac.m_angle);
  
  // Agent in local space
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  glColor4f(0.5, 0.5, 0.5, 0.5);
  drawDisk(m_agent->m_bac.m_radius, 0, 16, 2, GLU_FILL);
  glColor3f(0,0,0);
  drawDisk(m_agent->m_bac.m_radius, 0, 16, 2, GLU_SILHOUETTE);
  
  drawBasis(m_agent->m_bac.m_radius);
  glPopMatrix();

  // Food in world space
  glPushMatrix();
  glColor4f(0.8, 0.8, 0, 0.75);
  ci::gl::drawLine(m_agent->m_bac.m_position, m_agent->m_bac.m_foodPos);
  ci::gl::translate(m_agent->m_bac.m_foodPos);
  drawDisk(m_agent->m_bac.m_radius, 0, 16, 2, GLU_FILL);
  glColor3f(0,0,0);
  drawDisk(m_agent->m_bac.m_radius, 0, 16, 2, GLU_SILHOUETTE);
  glPopMatrix();
}
  
//----------------------------------------------------------------------------------------------------------------------
void LegBacViz::update()
{
  glDisable(GL_DEPTH_TEST);
  
  const ci::Vec2f& pos = m_agent->getPos();
  
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
  
  if (m_agent->m_legged)
    drawLeg();
  else
    drawBac();
   
  glPopAttrib();
  glEnable(GL_DEPTH_TEST);
}
  
} // namespace