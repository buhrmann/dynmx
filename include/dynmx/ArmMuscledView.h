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
  
  virtual void createArmViz();
  
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
inline void ArmMuscledView::createArmViz()
{
  m_armViz = new ArmMuscledViz(m_armMd);
}

//----------------------------------------------------------------------------------------------------------------------      
inline void ArmMuscledView::setupScene()
{
  assert(m_arm);
  
  // Let parent setup first
  ArmView::setupScene();
  
  // Create a plot for muscle data
  m_musclePlot = new dmx::Plot(UI_COLUMN_WIDTH, 180, MAX_NUM_MUSCLES*2, m_numPointsPerPlot);
  m_musclePlot->setTitle("Muscle lengths");
  for(int i = 0; i < m_armMd->getNumMuscles(); i++)
  {
    char str[32];
    sprintf(str, "%s: ", m_armMd->getMuscle(i)->getName().c_str());
    m_musclePlot->setLabel(i, str);
  }
  m_uiColumn->m_children.push_back(m_musclePlot);
  

  std::fill(m_excitation, m_excitation + MAX_NUM_MUSCLES, 0.0);  
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
    double val = m_armMd->getMuscle(i)->getUnitLength();
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