/*
 *  ArmReflexView.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 10/14/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_REFLEX_VIEW_
#define _DMX_ARM_REFLEX_VIEW_

#include "ArmMuscledView.h"
#include "ArmReflexViz.h"
#include "ArmReflex.h"

#define MAX_NUM_MUSCLES 6

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
class ArmReflexView : public ArmMuscledView
{
public:
  
  ArmReflexView(ArmReflex* arm) : ArmMuscledView(arm), m_armRx(arm), m_selectedReflex(0), m_previousReflex(0) {};
  
  virtual void setupScene();
  virtual void update(float dt);
  virtual void buildGui();
  
protected:
  
  ArmReflex* m_armRx;
  int m_selectedReflex;
  int m_previousReflex;
  float m_reflexGains[3]; // P,V,D
  
  dmx::Plot* m_reflexPlot;  
  
}; // class

//----------------------------------------------------------------------------------------------------------------------  
// Inline implementation
//----------------------------------------------------------------------------------------------------------------------    
inline void ArmReflexView::setupScene()
{
  ArmMuscledView::setupScene();
  
  // Todo: This is very ugly!!!
  delete m_armViz;
  m_armViz = new ArmReflexViz(m_armRx);
  m_armViz->rotate(ci::Vec4f(0,0,1,1), -PI_OVER_TWO);
  m_armViz->translate(ci::Vec4f(-0.25, 0, 0, 1));
  m_scene3d.m_children[0] = m_armViz;  
  
  ((ArmMuscledViz*)m_armViz)->setDrawDesired(true);
  
  m_reflexGains[0] = m_armRx->getReflex(m_selectedReflex)->m_Kspp[0];
  m_reflexGains[1] = m_armRx->getReflex(m_selectedReflex)->m_Kspv[0];
  m_reflexGains[2] = m_armRx->getReflex(m_selectedReflex)->m_Kspd[0];
  
  // Create a plot for reflex data
  m_reflexPlot = new dmx::Plot(600.0, 180, 4, 200);
  m_reflexPlot->translate(ci::Vec4f(75, 50 + 180 + 20, 0, 1));      
  m_scene2d.m_children.push_back(m_reflexPlot);
}

//----------------------------------------------------------------------------------------------------------------------  
inline void ArmReflexView::update(float dt)
{
  ArmMuscledView::update(dt);
  
  m_armRx->setTarget(dmx::Pos(m_target));
  
  // Control reflex parameters visually
  if(m_selectedReflex != m_previousReflex)
  {
    m_reflexGains[0] = m_armRx->getReflex(m_selectedReflex)->m_Kspp[0];
    m_reflexGains[1] = m_armRx->getReflex(m_selectedReflex)->m_Kspv[0];
    m_reflexGains[2] = m_armRx->getReflex(m_selectedReflex)->m_Kspd[0];
    m_previousReflex = m_selectedReflex;
  }
  
  m_armRx->getReflex(m_selectedReflex)->setSpindleParameters(m_reflexGains[0], m_reflexGains[0], 
                                                             m_reflexGains[1], m_reflexGains[1], 
                                                             m_reflexGains[2], m_reflexGains[2], 
                                                             1, 1);
  
  // Add data to plot
  m_reflexPlot->addPoint(0.5 * (m_armRx->getReflex(m_selectedReflex)->getAlphaOutput(0) + 
                                m_armRx->getReflex(m_selectedReflex)->getAlphaOutput(1)), 0);
  m_reflexPlot->addPoint(m_armRx->getReflex(m_selectedReflex)->getCoContraction(0), 1);
  
  m_reflexPlot->addPoint(m_armRx->getReflex(m_selectedReflex)->getAlphaOutput(0), 2);
  m_reflexPlot->addPoint(m_armRx->getReflex(m_selectedReflex)->getAlphaOutput(1), 3);  
}

//----------------------------------------------------------------------------------------------------------------------      
inline void ArmReflexView::buildGui()
{
  ArmMuscledView::buildGui();
  
  m_gui->addPanel();
  m_gui->addLabel("Reflex Controls");
  m_gui->addParam("Joint", &m_selectedReflex, 0, 1, m_selectedReflex);
  m_gui->addParam("P gain", &m_reflexGains[0], 0, 10, 0);
  m_gui->addParam("V gain", &m_reflexGains[1], 0, 10, 0);
  m_gui->addParam("D gain", &m_reflexGains[2], 0, 10, 0);
}

} // namespace

#endif