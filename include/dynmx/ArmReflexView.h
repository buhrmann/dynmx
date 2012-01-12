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
  
  dmx::Plot* m_reflexPlot1;
  dmx::Plot* m_reflexPlot2;
  dmx::Plot* m_jointTrajPlot;
  dmx::Plot* m_muscleTraj;
  dmx::Plot* m_effTraj;
  
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
  m_reflexPlot1 = new dmx::Plot(300.0, 180, 6, 200);
  m_reflexPlot1->translate(ci::Vec4f(0, 180 + 20, 0, 1));   
  m_reflexPlot1->setTitle("Reflex Elbow");
  m_uiColumn->m_children.push_back(m_reflexPlot1);
  
  m_reflexPlot2 = new dmx::Plot(300.0, 180, 6, 200);
  m_reflexPlot2->translate(ci::Vec4f(0, 400, 0, 1));   
  m_reflexPlot2->setTitle("Reflex Shoulder");
  m_uiColumn->m_children.push_back(m_reflexPlot2);  
  
  m_jointTrajPlot = new dmx::Plot(300.0, 180, 6, 200);
  m_jointTrajPlot->translate(ci::Vec4f(0, 600, 0, 1));   
  m_jointTrajPlot->setTitle("Joint Trajectory");
  m_uiColumn->m_children.push_back(m_jointTrajPlot);    
}
  
// TODO: Remove this test code
struct MySqrDist
{
  double operator() (float t1, float t2) { return sqr(t1-t2); };
};  

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
  
/*  m_armRx->getReflex(m_selectedReflex)->setSpindleParameters(m_reflexGains[0], m_reflexGains[0], 
                                                             m_reflexGains[1], m_reflexGains[1], 
                                                             m_reflexGains[2], m_reflexGains[2], 
                                                             1, 1);
 */ 
  
  // Add data to plot
  m_reflexPlot1->addPoint(m_armRx->getReflex(0)->getAlphaOutput(0), 0);
  m_reflexPlot1->addPoint(m_armRx->getReflex(0)->getPositionError(0), 1);
  m_reflexPlot1->addPoint(max(-m_armRx->getReflex(0)->getContractionVelocity(0), 0.0), 2);  
  //m_reflexPlot1->addPoint(m_armRx->getReflex(0)->getVelocityError(0), 2);
  
  //m_reflexPlot1->addPoint(m_armRx->getReflex(0)->getAlphaOutput(1), 3);
  //m_reflexPlot1->addPoint(m_armRx->getReflex(0)->getPositionError(1), 4);
  //m_reflexPlot1->addPoint(m_armRx->getReflex(0)->getVelocityError(1), 5);
  
  m_reflexPlot2->addPoint(m_armRx->getReflex(1)->getAlphaOutput(0), 0);
  m_reflexPlot2->addPoint(m_armRx->getReflex(1)->getPositionError(0), 1);
  //m_reflexPlot2->addPoint(m_armRx->getReflex(1)->getVelocityError(0), 2);
  m_reflexPlot2->addPoint(m_armRx->getReflex(1)->getAlphaOutput(1), 3);
  m_reflexPlot2->addPoint(m_armRx->getReflex(1)->getPositionError(1), 4);
  //m_reflexPlot2->addPoint(m_armRx->getReflex(1)->getVelocityError(1), 5);
  
  
  // Add desired muscle length to existing plot of actual length 
  m_musclePlot->addPoint(m_armRx->getReflex(0)->getDesiredLength(0), 4);
  m_musclePlot->addPoint(m_armRx->getReflex(0)->getDesiredLength(1), 5);
  m_musclePlot->addPoint(m_armRx->getReflex(1)->getDesiredLength(0), 6);
  m_musclePlot->addPoint(m_armRx->getReflex(1)->getDesiredLength(1), 7);
  
  // HACK
  // Retrieve desired trajectory
  EvoArmCoCon* evoArmCC =  (EvoArmCoCon*) ((GATester*) ((dmx::App*)ci::app::App::get())->m_model)->getEvolvable();
  ci::Vec2f desiredPos = evoArmCC->m_currentDesiredPos;
  ci::Vec2f desiredAng = evoArmCC->m_currentDesiredAngles;

  // Desired angles
  m_jointTrajPlot->addPoint(desiredAng.x, 0);
  m_jointTrajPlot->addPoint(desiredAng.y, 1);
  // Commanded angles
  m_jointTrajPlot->addPoint(m_armRx->getDesiredJointAngle(JT_elbow), 2);
  m_jointTrajPlot->addPoint(m_armRx->getDesiredJointAngle(JT_shoulder), 3);
  // Actual angles
  m_jointTrajPlot->addPoint(m_armRx->getJointAngle(JT_elbow), 4);
  m_jointTrajPlot->addPoint(m_armRx->getJointAngle(JT_shoulder), 5);
  
#if 0
  // TODO: remove test code
  if(evoArmCC->getTime() > 1.2)
  {
    std::vector<float>& d1 = m_musclePlot->getData(0); // actual muscle length
    std::vector<float>& d2 = m_musclePlot->getData(4); // desired muscle length
    int minDelay = -100;
    int maxDelay = 100;
    std::vector<double> cc = crossCorrelation(d1, d2, MySqrDist(), minDelay, maxDelay);
    std::vector<double>::iterator optElem = min_element(cc.begin(), cc.end());
    double optVal = *optElem;
    int optId = std::distance(cc.begin(), optElem);
    int bestDelay = minDelay + optId;
  }
#endif
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