/*
 *  SMCView.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_VIEW_
#define _DMX_SMC_VIEW_

#include "View.h"
#include "Scene.h"
#include "SMCAgent.h"
#include "SMCAgentViz.h"
#include "SMCEnvironmentViz.h"
#include "CTRNNViz.h"

#include "cinder/gl/gl.h"
#include "cinder/Matrix.h"
#include "SimpleGUI.h"

namespace dmx
{

// ---------------------------------------------------------------------------------------------------------------------
class SMCView : public dmx::View
{
  
public:
  
  SMCView(SMCAgent* agent) : m_agent(agent), m_fixedFrameRate(DEFAULT_VIEW_FRAME_RATE) {};
  
  // Inherited functions
  virtual void setupScene();
  virtual int getDesiredFrameRate() { return m_fixedFrameRate; };
  virtual void update(float dt);
  
  virtual void draw3d(){};
  virtual void draw2d(){};
  
  virtual void keyDown (ci::app::KeyEvent event);
  
  virtual void buildGui();
  
protected:
  
  SMCAgent* m_agent; 
  SMCAgentViz* m_agentViz;
  SMCEnvironmentViz* m_environViz;
  CTRNNViz* m_ctrnnViz;
  int32_t m_fixedFrameRate;
  float m_gravity;
  
  dmx::Plot* m_plot;
  
}; // class


//----------------------------------------------------------------------------------------------------------------------
// Inline implementations
//----------------------------------------------------------------------------------------------------------------------
inline void SMCView::setupScene()
{
  assert(m_agent);
  
  m_backgroundColor = ci::Vec4f(0.9, 0.9, 0.9, 1.0);
  
  // 3d view
#if 0  
  // Coordinate axes
  Axes* axes = new Axes(0.1);
  axes->createGeometry();
  m_scene3d.m_children.push_back(axes);
#endif
  
#if 0  
  // Background grid
  Grid* grid = new Grid(0.5, 0.5, 10, 10);
  grid->createGeometry();
  grid->m_color = ci::Vec4f(0.8f, 0.8f, 0.8f, 0.25f); 
  grid->translate(ci::Vec4f(0.0, 0.0, -0.002, 1.0f));  
  grid->rotate(ci::Vec3f(1.0, 0.0, 0.0), PI_OVER_TWO);  
  m_scene3d.m_children.push_back(grid);
#endif
  
  // Agent and environment
  m_agentViz = new SMCAgentViz(m_agent);
  m_environViz = new SMCEnvironmentViz(m_agent->getEnvironment());
  m_scene3d.m_children.push_back(m_agentViz);
  m_scene3d.m_children.push_back(m_environViz);
  
  // 2d viz
  m_ctrnnViz = new CTRNNViz(m_agent->getCTRNN(), 150);  
  m_ctrnnViz->translate(ci::Vec4f(100, 350, 0, 1));
  m_scene2d.m_children.push_back(m_ctrnnViz);
  
  // Data plots
  m_plot = new dmx::Plot(400.0, 180, 2, 200);
  m_plot->translate(ci::Vec4f(100, 50, 0, 1)); 
  m_scene2d.m_children.push_back(m_plot);
  
}

//--------------------------------------------------------------------------------------------------------------------  
inline void SMCView::update(float dt)
{
  
  // Add data to plot
  double val = m_agent->getSensedValue();
  m_plot->addPoint(val, 0);
}

//---------------------------------------------------------------------------------------------------------------------
inline void SMCView::keyDown(ci::app::KeyEvent event)
{
  switch(event.getChar())
  {
    case '=':
      m_fixedFrameRate++;
      break;
    case '-':
      m_fixedFrameRate--;
      break;
  }
  // default behaviour in base class
  View::keyDown(event);
}

//--------------------------------------------------------------------------------------------------------------------
inline void SMCView::buildGui() 
{ 
  m_gui->addParam("FPS", &m_fixedFrameRate, -1, 300, m_fixedFrameRate);   
  
  m_gui->addPanel();
  m_gui->addLabel("Agent Controls");
  //m_gui->addParam("Gravity", &m_gravity, 0.0, 9.81, m_arm->getGravity());
};

  
} // namespace dmx

#endif