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
#include "SMCAgentEvo.h"
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
  
  SMCView(SMCAgentEvo* agent) : m_agent(agent), m_fixedFrameRate(DEFAULT_VIEW_FRAME_RATE) {};
  
  // Inherited functions
  virtual void setupScene();
  virtual int getDesiredFrameRate() { return m_fixedFrameRate; };
  virtual void update(float dt);
  
  virtual void draw3d(){};
  virtual void draw2d(){};
  
  virtual void keyDown (ci::app::KeyEvent event);
  
  virtual void buildGui();
  
protected:
  
  SMCAgentEvo* m_agent; 
  SMCAgentViz* m_agentViz;
  SMCEnvironmentViz* m_environViz;
  CTRNNViz* m_ctrnnViz;
  int32_t m_fixedFrameRate;
  float m_gravity;
  
  Plot* m_plot;
  Plot* m_ctrnnPlot; 
  Plot* m_fitnessPlot;
  
}; // class


//----------------------------------------------------------------------------------------------------------------------
// Inline implementations
//----------------------------------------------------------------------------------------------------------------------
inline void SMCView::setupScene()
{
  assert(m_agent);
  
  m_light.toggle();
  
  // Background
  m_backgroundColor = ci::Vec4f(1, 1, 1, 1.0);
  const float g = 0.1f;
  m_background.topLeft = ci::ColorA(g, g, g, 0.5f);
  m_background.topRight= ci::ColorA(g, g, g, 0.5f);
  m_background.bottomLeft = ci::ColorA(1,1,1,0.0);
  m_background.bottomRight = ci::ColorA(1,1,1,0.0);
  
  // 3d view
#if 0
  // Coordinate axes
  Axes* axes = new Axes(0.05);
  axes->createGeometry();
  m_scene3d.m_children.push_back(axes);
#endif
  
#if 1
  // Background grid
  Grid* grid = new Grid(0.6, 1.0, 12, 20);
  grid->createGeometry();
  grid->m_color = ci::Vec4f(0.8f, 0.8f, 0.8f, 0.05f); 
  grid->translate(ci::Vec4f(0.3, 0.0, -0.002, 1.0f));      
  grid->rotate(ci::Vec3f(1.0, 0.0, 0.0), PI_OVER_TWO);
  m_scene3d.m_children.push_back(grid);
#endif
  
  // Agent and environment
  m_agentViz = new SMCAgentViz(m_agent->getAgent());
  m_environViz = new SMCEnvironmentViz(&m_agent->getAgent()->getEnvironment());
  m_scene3d.m_children.push_back(m_agentViz);
  m_scene3d.m_children.push_back(m_environViz);
  
  m_scene3d.translate(ci::Vec4f(-0.15,-0.15,0,1));
  m_scene3d.rotate(ci::Vec4f(0,0,1,1), PI/2);
  
  // 2d viz
  float columnWidth = 300;
  float columnMargin = 5;
  float plotHeight = 100;
  float left = columnWidth + columnMargin;
  NodeGroup* column = new NodeGroup();
  column->setRightAligned(true);
  column->translate(ci::Vec4f(left, columnMargin, 0, 1));

  // Add ctrnnViz to column
  m_ctrnnViz = new CTRNNViz(&m_agent->getAgent()->getCTRNN(), 150, &m_agent->getAgent()->getTopology());  
  column->m_children.push_back(m_ctrnnViz);

  // Add plots to column
  m_ctrnnPlot = new Plot(columnWidth, plotHeight, m_agent->getAgent()->getCTRNN().getSize(), 200);
  m_ctrnnPlot->translate(Vec4f(0, m_ctrnnViz->getHeight() + 16, 0, 1));  
  m_ctrnnPlot->setTitle("Neural outputs");
  column->m_children.push_back(m_ctrnnPlot);    
  
  m_plot = new dmx::Plot(columnWidth, plotHeight, 2, 200);
  m_plot->translate(ci::Vec4f(0, m_ctrnnViz->getHeight() + 16 + plotHeight + 32, 0, 1));
  m_plot->setTitle("Sensor Data");
  column->m_children.push_back(m_plot);

  m_fitnessPlot = new dmx::Plot(columnWidth, plotHeight, 2, 200);
  m_fitnessPlot->translate(ci::Vec4f(0, m_ctrnnViz->getHeight() + 16 + plotHeight + 32 + plotHeight + 32, 0, 1));
  m_fitnessPlot->setTitle("Fitness");
  column->m_children.push_back(m_fitnessPlot);
  
  m_scene2d.m_children.push_back(column);  
}

//--------------------------------------------------------------------------------------------------------------------  
inline void SMCView::update(float dt)
{
  // update data in graph
  for(int i = 0; i < m_agent->getAgent()->getCTRNN().getSize(); i++)
  {
    m_ctrnnPlot->addPoint(m_agent->getAgent()->getCTRNN().getOutput(i), i);
  }   
  
  // Add data to plot
  double val = m_agent->getAgent()->getSensedValue();
  double der = m_agent->getAgent()->getDistanceSensor().getDerivative();
  m_plot->addPoint(val, 0);
  m_plot->addPoint(der, 1);
  
  // Update fitness-related data
  m_fitnessPlot->addPoint(m_agent->getAgent()->getAngle(), 0);
  //m_fitnessPlot->addPoint(m_agent->getAngleWithHeading(m_agent->getEnvironment().getObjects()[0]->getPosition()), 1);
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