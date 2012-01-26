/*
 *  ArmPDView.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 10/14/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_PD_VIEW_
#define _DMX_ARM_PD_VIEW_

#include "ArmView.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------  
class ArmPDView : public ArmView
{
public:
  
  ArmPDView(dmx::ArmPD* arm) : ArmView(arm), m_armPD(arm), m_selectedController(0) {};
  
  virtual void update(float dt);
  virtual void buildGui();
  virtual void draw3d();  
  
protected:
  
  ArmPD* m_armPD;
  int m_selectedController;
  mowa::sgui::FloatVarControl* m_pGainControl;
  mowa::sgui::FloatVarControl* m_dGainControl;  
  
}; // class
  
//----------------------------------------------------------------------------------------------------------------------
// Inline implementations  
//----------------------------------------------------------------------------------------------------------------------  
inline void ArmPDView::update(float dt)
{
  ArmView::update(dt);
  
  m_pGainControl->setControlledVariable(&m_armPD->m_pd[m_selectedController].m_P);
  m_dGainControl->setControlledVariable(&m_armPD->m_pd[m_selectedController].m_D);
}

//----------------------------------------------------------------------------------------------------------------------  
inline void ArmPDView::buildGui()
{
  ArmView::buildGui();
  
  m_gui->addPanel();
  m_gui->addParam("Controller", &m_selectedController, 0, 1, m_selectedController);
  m_pGainControl = m_gui->addParam("P gain", &(m_armPD->m_pd[m_selectedController].m_P), 0.0, 100.0, 0.0);
  m_dGainControl = m_gui->addParam("D gain", &(m_armPD->m_pd[m_selectedController].m_D), 0.0, 100.0, 0.0);
}
  
//----------------------------------------------------------------------------------------------------------------------    
inline void ArmPDView::draw3d()
{
  ArmView::draw3d();
  
  glPushMatrix();
  glMultMatrixf(*m_armViz->m_pTM);

  Vec3f desired (m_armPD->m_desiredPos);
  glColor3f(1,0,0);
  drawPoint(desired, 5);
  
  
  Pos p1, p2;    
  double desElbAngle = m_armPD->getDesiredJointAngle(JT_elbow); 
  double desShdAngle = m_armPD->getDesiredJointAngle(JT_shoulder); 
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
  
  glPopMatrix();
}
  
} // namespace

#endif