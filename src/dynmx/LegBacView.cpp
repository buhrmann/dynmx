//
//  LegBacView.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 26/08/14.
//
//

#include "LegBacView.h"
#include "LegBac.h"
#include "CTRNN.h"
#include "AdapNN.h"

namespace dmx
{

// ---------------------------------------------------------------------------------------------------------------------
void LegBacView::setupScene()
{
  assert(m_agent);
  
  m_light.toggle();
  m_cam.setEyePoint(Vec3f(0.0f, 0.0f, 25.f));
  m_cam3d.setCurrentCam(m_cam);
  m_followCam = false;
  
  // Background
  m_backgroundColor = ci::Vec4f(1, 1, 1, 1.0);
  const float g = 0.1f;
  m_background.topLeft = ci::ColorA(g, g, g, 0.5f);
  m_background.topRight= ci::ColorA(g, g, g, 0.5f);
  m_background.bottomLeft = ci::ColorA(1,1,1,0.0);
  m_background.bottomRight = ci::ColorA(1,1,1,0.0);
  
  // Background grid
  Grid* grid = new Grid(20, 20, 40, 40);
  grid->createGeometry();
  grid->m_color = ci::Vec4f(0.8f, 0.8f, 0.8f, 0.05f);
  grid->translate(ci::Vec4f(0.3, 0.0, -0.002, 1.0f));
  grid->rotate(ci::Vec3f(1.0, 0.0, 0.0), PI_OVER_TWO);
  m_scene3d.m_children.push_back(grid);
  
  // Coordinate axes
  Axes* axes = new Axes(0.5);
  axes->createGeometry();
  m_scene3d.m_children.push_back(axes);
  
  // Agent and environment
  m_agentViz = new LegBacViz(m_agent);
  m_scene3d.m_children.push_back(m_agentViz);
  
  m_scenePos = ci::Vec3f(-0.15, -0.15, 0);
  m_scene3d.translate(ci::Vec4f(m_scenePos, 1));
  //m_scene3d.rotate(ci::Vec4f(0,0,1,1), PI);
  
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
  m_ctrnnViz = new CTRNNViz(m_agent->getCTRNN(), columnWidth1, m_agent->getTopology());
  column1->m_children.push_back(m_ctrnnViz);
  
  NodeGroup* column2 = new NodeGroup();
  column2->setRightAligned(true);
  column2->translate(ci::Vec4f(columnWidth2 + columnMargin, columnMargin, 0, 1));
  
  // Add plots to column
  m_ctrnnPlot = new Plot(columnWidth2, plotHeight, m_agent->getCTRNN()->getSize(), 200);
  m_ctrnnPlot->setTitle("Neural outputs");
  column2->m_children.push_back(m_ctrnnPlot);
  
  m_plot = new dmx::Plot(columnWidth2, plotHeight, 6, 200);
  m_plot->translate(ci::Vec4f(0, plotHeight + vSpace, 0, 1));
  m_plot->setTitle("Other Data");
  column2->m_children.push_back(m_plot);
  
  m_fitnessPlot = new dmx::Plot(columnWidth2, plotHeight, 2, 200);
  m_fitnessPlot->translate(ci::Vec4f(0, 2 * plotHeight + 2 * vSpace, 0, 1));
  m_fitnessPlot->setTitle("Fitness");
  column2->m_children.push_back(m_fitnessPlot);
  
  m_scene2d.m_children.push_back(column1);
  m_scene2d.m_children.push_back(column2);
  
}

//--------------------------------------------------------------------------------------------------------------------
void LegBacView::update(float dt)
{
  float time = m_agent->getTime();
  char tstr [16];
  sprintf(tstr, "Time: %2.2f", time);
  m_timeLabel->setText(tstr);
  
  if(time < 0.01)
    reset();
  
  // update data in graph
  for(int i = 0; i < m_agent->getCTRNN()->getSize(); i++)
  {
    m_ctrnnPlot->addPoint(m_agent->getCTRNN()->getOutput(i), i);
  }
  
  // Add data to plot
  //double val = m_agent->getSensedValue();
  if(m_agent->m_legged)
  {
    m_plot->addPoint(m_agent->m_leg.m_sensor, 0);
    m_plot->addPoint(m_agent->m_leg.m_force, 1);
    m_plot->addPoint(m_agent->m_leg.m_velocity.x, 2);
    m_plot->addPoint(m_agent->m_leg.m_angSpeed, 3);
  }
  //m_plot->addPoint(val, 0);
  
  // Update fitness-related data
  if(!m_agent->m_legged)
    m_fitnessPlot->addPoint(((AdapNN*)m_agent->getCTRNN())->getReward(), 0);
  
  if(m_followCam)
  {
    ci::Vec3f pos = ci::Vec3f(m_agent->getPos()) - m_scenePos;
    m_cam.setEyePoint(pos + ci::Vec3f(0, 0, 25));
    m_cam.setCenterOfInterestPoint(pos);
    m_cam.setWorldUp(ci::Vec3f(0,0,1));
    m_cam3d.setCurrentCam(m_cam);
  }
}
  
//--------------------------------------------------------------------------------------------------------------------
void LegBacView::reset()
{
  m_agentViz->reset();
  m_ctrnnViz->reset();
  m_scene2d.reset();
}
  
//---------------------------------------------------------------------------------------------------------------------
bool LegBacView::resetCam(ci::app::MouseEvent event)
{
  m_cam.setEyePoint(Vec3f(0.0f, 0.0f, 25));
  m_cam.setWorldUp(ci::Vec3f(0,0,1));
  m_cam.setCenterOfInterestPoint(ci::Vec3f(0,0,0));
  m_cam3d.setCurrentCam(m_cam);
  m_followCam = false;
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
void LegBacView::keyDown(ci::app::KeyEvent event)
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
void LegBacView::buildGui()
{
  m_gui->addParam("FPS", &m_fixedFrameRate, -1, 300, m_fixedFrameRate);
  
  m_gui->addPanel();
  m_gui->addLabel("Agent Controls");
  char tstr [5];
  sprintf(tstr, "Time: %2.2f", m_agent->getTime());
  m_timeLabel = m_gui->addLabel(tstr);
  
  m_gui->addPanel();
  m_gui->addLabel("View Controls");
  m_gui->addParam("Follow Cam", &m_followCam);
  m_gui->addButton("Reset Cam")->registerClick(this, &LegBacView::resetCam);

  //m_gui->addButton("View mode", &m_agentViz->m_agentViewMode);
}
  
} // namespace
