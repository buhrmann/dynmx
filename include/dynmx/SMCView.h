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
#include "MathUtils.h"
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
  virtual void reset();
  
  virtual void draw3d(){};
  virtual void draw2d(){};
  
  virtual void keyDown (ci::app::KeyEvent event);
  bool resetCam(ci::app::MouseEvent event);
  
  virtual void buildGui();
  
protected:
  
  SMCAgentEvo* m_agent; 
  SMCAgentViz* m_agentViz;
  SMCEnvironmentViz* m_environViz;
  CTRNNViz* m_ctrnnViz;
  int32_t m_fixedFrameRate;
  //float m_gravity;
  
  Plot* m_plot;
  Plot* m_ctrnnPlot;
  Plot* m_energyPlot;
  Plot* m_fitnessPlot;
  
  NodeGroup* m_column1;
  NodeGroup* m_column2;
  
  mowa::sgui::LabelControl* m_timeLabel;
  
  ci::Vec3f m_scenePos;
  bool m_followCam;
  
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
  
#if 0
  // Background grid
  Grid* grid = new Grid(1.2, 1.2, 1.2/0.05, 1.2/0.05);
  grid->createGeometry();
  grid->m_color = ci::Vec4f(0.8f, 0.8f, 0.8f, 0.05f); 
  grid->translate(ci::Vec4f(0.0, 0.0, -0.002, 1.0f));
  grid->rotate(ci::Vec3f(1.0, 0.0, 0.0), PI_OVER_TWO);
  m_scene3d.m_children.push_back(grid);
#endif
  
  // Agent and environment
  m_agentViz = new SMCAgentViz(m_agent->getAgent());
  m_environViz = new SMCEnvironmentViz(&m_agent->getAgent()->getEnvironment());
  m_scene3d.m_children.push_back(m_environViz);
  m_scene3d.m_children.push_back(m_agentViz);
  
  m_scenePos = ci::Vec3f(-0.15, 0.0, 0);
  m_scene3d.translate(ci::Vec4f(m_scenePos, 1));
  
  m_cam.setEyePoint(Vec3f(0.0f, 0.0f, 1.5f));
  m_cam.setWorldUp(ci::Vec3f(0,0,1));
  m_followCam = false;
  
  //m_scene3d.rotate(ci::Vec4f(0,0,1,1), PI/2);
  
  // 2d viz
  float columnWidth1 = 200;
  float columnWidth2 = 200;
  float columnMargin = 5;
  float plotHeight = 100;
  float vSpace = 32;

  // Add ctrnnViz to column
  m_column1 = new NodeGroup();
  m_column1->setRightAligned(true);
  m_column1->translate(ci::Vec4f(columnWidth1 + columnWidth2 + 2 * columnMargin, columnMargin, 0, 1));

  m_ctrnnViz = new CTRNNViz(&m_agent->getAgent()->getCTRNN(), columnWidth1, &m_agent->getAgent()->getTopology());
  m_column1->m_children.push_back(m_ctrnnViz);

  // Add plots to column
  m_column2 = new NodeGroup();
  m_column2->setRightAligned(true);
  m_column2->translate(ci::Vec4f(columnWidth2 + columnMargin, columnMargin, 0, 1));

  m_ctrnnPlot = new Plot(columnWidth2, plotHeight, m_agent->getAgent()->getCTRNN().getSize(), 200);
  //m_ctrnnPlot->translate(Vec4f(0, m_ctrnnViz->getHeight() + 16, 0, 1));
  m_ctrnnPlot->setTitle("Neural outputs");
  m_column2->m_children.push_back(m_ctrnnPlot);
  
  m_plot = new dmx::Plot(columnWidth2, plotHeight, 4, 200);
  m_plot->translate(ci::Vec4f(0, plotHeight + vSpace, 0, 1));
  m_plot->setTitle("Sensor Data");
  m_column2->m_children.push_back(m_plot);

  m_fitnessPlot = new dmx::Plot(columnWidth2, plotHeight, 2, 200);
  m_fitnessPlot->translate(ci::Vec4f(0, 2 * plotHeight + 2 * vSpace, 0, 1));
  m_fitnessPlot->setTitle("Fitness");
  m_column2->m_children.push_back(m_fitnessPlot);
  
  m_energyPlot = new dmx::Plot(columnWidth2, plotHeight, 2, 200);
  m_energyPlot->translate(ci::Vec4f(0, 3 * plotHeight + (3 * vSpace), 0, 1));
  m_energyPlot->setTitle("Energy");
  m_column2->m_children.push_back(m_energyPlot);
  
  m_scene2d.m_children.push_back(m_column1);
  m_scene2d.m_children.push_back(m_column2);
}

//--------------------------------------------------------------------------------------------------------------------  
inline void SMCView::update(float dt)
{
  float time = m_agent->getAgent()->getTime();
  if(time <= 0.01f)
  {
    reset();
  }
  
  char tstr [5];
  sprintf(tstr, "Time: %2.2f", time);
  m_timeLabel->setText(tstr);
  
  // update data in graph
  for(int i = 0; i < m_agent->getAgent()->getCTRNN().getSize(); i++)
  {
    m_ctrnnPlot->addPoint(m_agent->getAgent()->getCTRNN().getOutput(i), i);
  }   
  
  // Add data to plot
  if (m_agent->getAgent()->hasDistanceSensor())
  {
    double val = m_agent->getAgent()->getSensedValue();
    double der = m_agent->getAgent()->getDistanceSensor()->getDerivative();
    m_plot->addPoint(val, 0);
    m_plot->addPoint(der, 1);
  }
  
  if (m_agent->getAgent()->hasGradientSensor())
  {
    m_plot->addPoint(radiansToDegrees(m_agent->getAgent()->getAngularSpeed()), 1);
    double v = m_agent->getAgent()->getGradientSensor()->getLevel();
    m_plot->addPoint(v, 2);
  }

  
  // Update fitness-related data
  m_fitnessPlot->addPoint(m_agent->m_fitness, 0);
  m_fitnessPlot->addPoint(m_agent->m_fitnessInst, 1);
  
  // Energy
  m_energyPlot->addPoint(m_agent->getAgent()->getEnergy(), 0);
  m_energyPlot->addPoint(m_agent->getAgent()->getSensedFood(), 1);
  
  if(m_followCam)
  {
    ci::Vec3f pos = ci::Vec3f(m_agent->getAgent()->getPosition()) - m_scenePos;
    m_cam.setEyePoint(pos + ci::Vec3f(0, 0, 1.5));
    m_cam.setCenterOfInterestPoint(pos);
    m_cam.setWorldUp(ci::Vec3f(0,0,1));
    m_cam3d.setCurrentCam(m_cam);
  }
}
  
//--------------------------------------------------------------------------------------------------------------------
inline void SMCView::reset()
{
  m_agentViz->reset();
  m_environViz->reset();
  m_ctrnnViz->reset();
  m_scene2d.reset();
}

//---------------------------------------------------------------------------------------------------------------------
bool SMCView::resetCam(ci::app::MouseEvent event)
{
  m_cam.setEyePoint(Vec3f(0.0f, 0.0f, 1.5f));
  m_cam.setWorldUp(ci::Vec3f(0,0,1));
  m_cam.setCenterOfInterestPoint(ci::Vec3f(0,0,0));
  m_cam3d.setCurrentCam(m_cam);
  m_followCam = false;
  return false;
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
  char tstr [5];
  sprintf(tstr, "Time: %2.2f", m_agent->getAgent()->getTime());
  m_timeLabel = m_gui->addLabel(tstr);
  m_gui->addParam("FPS", &m_fixedFrameRate, -1, 300, m_fixedFrameRate);   

  m_gui->addPanel();
  m_gui->addLabel("View Controls");
  m_gui->addParam("Follow Cam", &m_followCam);
  m_gui->addButton("Reset Cam")->registerClick(this, &SMCView::resetCam);
  
  m_gui->addPanel();
  m_gui->addLabel("Agent Controls");
  //m_gui->addParam("Gravity", &m_gravity, 0.0, 9.81, m_arm->getGravity());
};

  
} // namespace dmx

#endif