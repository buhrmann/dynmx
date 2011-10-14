/*
 *  ArmMuscledView.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 10/14/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_MUSCLED_VIEW_
#define _DMX_ARM_MUSCLED_VIEW_

#include "ArmView.h"
#include "ArmMuscled.h"
#include "ArmMuscledViz.h"

#define MAX_NUM_MUSCLES 6

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
class ArmMuscledView : public ArmView
{
public:
  
  ArmMuscledView(ArmMuscled* arm) : ArmView(arm), m_armMd(arm), m_selectedMuscle(0) {};
  
  virtual void setupScene();
  virtual void update(float dt);
  virtual void buildGui();
  
protected:
  
  ArmMuscled* m_armMd;
  int m_selectedMuscle;
  float m_excitation[MAX_NUM_MUSCLES];  
  mowa::sgui::LabelControl* m_muscleLabel;
  mowa::sgui::DoubleVarControl* m_fmaxControl;  
  
  dmx::Plot* m_musclePlot;
  
}; // class

//----------------------------------------------------------------------------------------------------------------------  
// Inline implementation
//----------------------------------------------------------------------------------------------------------------------    
inline void ArmMuscledView::setupScene()
{
  assert(m_arm);
  
  // 3d view
  m_armViz = new ArmMuscledViz(m_armMd);
  m_armViz->rotate(ci::Vec4f(0,0,1,1), -PI_OVER_TWO);
  m_armViz->translate(ci::Vec4f(-0.25, 0, 0, 1));
  m_scene3d.m_children.push_back(m_armViz);
  
  m_trackMouse = false;
  
  std::fill(m_excitation, m_excitation + MAX_NUM_MUSCLES, 0.0);  
  
  // Create a plot for muscle data
  m_musclePlot = new dmx::Plot(600.0, 180, MAX_NUM_MUSCLES, 200);
  m_musclePlot->translate(ci::Vec4f(75, 50, 0, 1)); 
  m_scene2d.m_children.push_back(m_musclePlot);
  
}

//----------------------------------------------------------------------------------------------------------------------  
inline void ArmMuscledView::update(float dt)
{
  ArmView::update(dt);
  
  // Change which muscle the gui controls
  std::string muscleName = m_armMd->getMuscle(m_selectedMuscle)->getName();
  if(m_muscleLabel->getText() != muscleName)
  {
    m_muscleLabel->setText(muscleName);
    m_fmaxControl->setControlledVariable(&m_armMd->getMuscle(m_selectedMuscle)->m_maxForce);
  }
  
  // Visual control of muscle activation
  for(int i = 0; i < m_armMd->getNumMuscles(); ++i)
  {
    m_armMd->getMuscle(i)->setExcitation(m_excitation[i]);
  }
  
  // Add data to plot
  for(int i = 0; i < m_armMd->getNumMuscles(); i++)
  {
    double val = m_armMd->getMuscle(i)->getNormalisedLength();
    m_musclePlot->addPoint(val, i);
  }  
  
}

//----------------------------------------------------------------------------------------------------------------------      
inline void ArmMuscledView::buildGui()
{
  ArmView::buildGui();

  for(int i = 0; i < m_armMd->getNumMuscles(); i++)
  {
    m_gui->addParam(m_armMd->getMuscle(i)->getName(), &m_excitation[i], 0.0, 1.0, 0.0);
  }
  
  m_gui->addPanel();
  m_gui->addParam("Muscle", &m_selectedMuscle, 0, 5, m_selectedMuscle);
  m_muscleLabel = m_gui->addLabel("Selected Muscle");
  m_fmaxControl = m_gui->addParam("Fmax", &(m_armMd->getMuscle(m_selectedMuscle)->m_maxForce), 0.0, 3000.0, m_armMd->getMuscle(m_selectedMuscle)->m_maxForce);
  
}

} // namespace

#endif