/*
 *  SMCVArmiew.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_ARM_VIEW_
#define _DMX_SMC_ARM_VIEW_

#include "View.h"
#include "Scene.h"
#include "ArmViz.h"
#include "SMCArm.h"
#include "SMCEnvironmentViz.h"
#include "CTRNNViz.h"

#include "cinder/gl/gl.h"
#include "cinder/Matrix.h"
#include "SimpleGUI.h"

namespace dmx
{
  
// ---------------------------------------------------------------------------------------------------------------------
class SMCArmView : public dmx::View
{
  
public:
  
  SMCArmView(SMCArm* arm) : m_arm(arm), m_fixedFrameRate(DEFAULT_VIEW_FRAME_RATE) {};
  
  // Inherited functions
  virtual void setupScene();
  virtual int getDesiredFrameRate() { return m_fixedFrameRate; };
  virtual void update(float dt);
  
  virtual void draw3d();
  virtual void draw2d(){};
  
  virtual void keyDown (ci::app::KeyEvent event);
  
  virtual void buildGui();
  
protected:
  
  SMCArm* m_arm;
  ArmViz* m_armViz;
  SMCEnvironmentViz* m_environViz;
  CTRNNViz* m_ctrnnViz;
  int32_t m_fixedFrameRate;
  
  Plot* m_plot;
  Plot* m_ctrnnPlot;
  Plot* m_fitnessPlot;
  
}; // class


//----------------------------------------------------------------------------------------------------------------------
// Inline implementations
//----------------------------------------------------------------------------------------------------------------------
inline void SMCArmView::setupScene()
{
  assert(m_arm);
  
  m_light.toggle();
  
  // Background
  m_backgroundColor = ci::Vec4f(1, 1, 1, 1.0);
  const float g = 0.1f;
  m_background.topLeft = ci::ColorA(g, g, g, 0.5f);
  m_background.topRight= ci::ColorA(g, g, g, 0.5f);
  m_background.bottomLeft = ci::ColorA(1,1,1,0.0);
  m_background.bottomRight = ci::ColorA(1,1,1,0.0);
  
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
  m_armViz = new ArmViz(&m_arm->getArm());
  m_environViz = new SMCEnvironmentViz(m_arm->getEnvironment());
  
  m_scene3d.m_children.push_back(m_armViz);
  m_scene3d.m_children.push_back(m_environViz);
  
  //m_scene3d.translate(ci::Vec4f(-0.15,-0.15,0,1));
  //m_scene3d.rotate(ci::Vec4f(0,0,1,1), -PI_OVER_TWO);
  
  // 2d viz
  float columnWidth1 = 200;
  float columnWidth2 = 300;
  float columnMargin = 5;
  float plotHeight = 100;
  float vSpace = 32;
  
  NodeGroup* column1 = new NodeGroup();
  column1->setRightAligned(true);
  column1->translate(ci::Vec4f(columnWidth1 + columnWidth2 + 2 * columnMargin, columnMargin, 0, 1));
  
  // Add ctrnnViz to column
  m_ctrnnViz = new CTRNNViz(m_arm->getCTRNN(), columnWidth1, &m_arm->getTopology());
  column1->m_children.push_back(m_ctrnnViz);
  
  NodeGroup* column2 = new NodeGroup();
  column2->setRightAligned(true);
  column2->translate(ci::Vec4f(columnWidth2 + columnMargin, columnMargin, 0, 1));
  
  // Add plots to column
  m_ctrnnPlot = new Plot(columnWidth2, plotHeight, m_arm->getCTRNN()->getSize(), 200);
  m_ctrnnPlot->setTitle("Neural outputs");
  column2->m_children.push_back(m_ctrnnPlot);
  
  m_plot = new dmx::Plot(columnWidth2, plotHeight, 2, 200);
  m_plot->translate(ci::Vec4f(0, plotHeight + vSpace, 0, 1));
  m_plot->setTitle("Sensor Data");
  column2->m_children.push_back(m_plot);
  
  m_fitnessPlot = new dmx::Plot(columnWidth2, plotHeight, 2, 200);
  m_fitnessPlot->translate(ci::Vec4f(0, 2 * plotHeight + 2 * vSpace, 0, 1));
  m_fitnessPlot->setTitle("Fitness");
  column2->m_children.push_back(m_fitnessPlot);
  
  m_scene2d.m_children.push_back(column1);
  m_scene2d.m_children.push_back(column2);
  
}

//--------------------------------------------------------------------------------------------------------------------
inline void SMCArmView::update(float dt)
{
  // update data in graph
  for(int i = 0; i < m_arm->getCTRNN()->getSize(); i++)
  {
    m_ctrnnPlot->addPoint(m_arm->getCTRNN()->getOutput(i), i);
  }
  
  // Add data to plot
  double val = m_arm->getSensedValue();
  m_plot->addPoint(val, 0);
  
  // Update fitness-related data
  //m_fitnessPlot->addPoint(m_agent->getPosition(), 0);
}

  //---------------------------------------------------------------------------------------------------------------------
inline void SMCArmView::draw3d()
{
  // Draw distance sensor ray and collision point
  const DistanceSensor& sensor = m_arm->getDistanceSensor();
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
}

//---------------------------------------------------------------------------------------------------------------------
inline void SMCArmView::keyDown(ci::app::KeyEvent event)
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
inline void SMCArmView::buildGui() 
{ 
  m_gui->addParam("FPS", &m_fixedFrameRate, -1, 300, m_fixedFrameRate);   
  
  m_gui->addPanel();
  m_gui->addLabel("Agent Controls");
  //m_gui->addButton("View mode", &m_agentViz->m_agentViewMode);
};

  
} // namespace dmx

#endif