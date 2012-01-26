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
  m_armViz->translate(ci::Vec4f(-0.35, -0.3, 0, 1));
  m_scene3d.m_children.push_back(m_armViz);
  
  m_trackMouse = false;
  
  std::fill(m_excitation, m_excitation + MAX_NUM_MUSCLES, 0.0);  
  
  // 2d viz
  float columnWidth = 300;
  float columnMargin = 5;
  float left = columnWidth + columnMargin;
  int pointsPerPlot = SETTINGS->getChild("Config/Globals/PlotDuration").getAttributeValue<float>("Value") * 
                      SETTINGS->getChild("Config/Globals/FrameRate").getAttributeValue<int>("Value");
  
  m_uiColumn = new NodeGroup();
  m_uiColumn->setRightAligned(true);
  m_uiColumn->translate(ci::Vec4f(left, columnMargin, 0, 1));
  m_scene2d.m_children.push_back(m_uiColumn);

  m_uiColumn2 = new NodeGroup();
  m_uiColumn2->setRightAligned(true);
  m_uiColumn2->translate(ci::Vec4f(left+left, columnMargin, 0, 1));
  m_scene2d.m_children.push_back(m_uiColumn2);  
  
  // Create a plot for muscle data
  m_musclePlot = new dmx::Plot(columnWidth, 180, MAX_NUM_MUSCLES*2, pointsPerPlot);
  m_musclePlot->setTitle("Muscle lengths");
  for(int i = 0; i < m_armMd->getNumMuscles(); i++)
  {
    char str[32];
    sprintf(str, "%s: ", m_armMd->getMuscle(i)->getName().c_str());
    m_musclePlot->setLabel(i, str);
  }
  m_uiColumn->m_children.push_back(m_musclePlot);

  // Create a plot for effector movement
  m_armEffPlot = new dmx::Plot(columnWidth, 180, 6, pointsPerPlot);
  m_armEffPlot->setTitle("Effector kinematics");
  m_armEffPlot->setLabel(0, "x");
  m_armEffPlot->setLabel(1, "y");
  
  // Create a plot for arm data (different forces acting on arm e.g.)
  m_armElbPlot = new dmx::Plot(columnWidth, 180, 6, pointsPerPlot);
  m_armElbPlot->translate(ci::Vec4f(0, 180 + 20, 0, 1));  
  m_armElbPlot->setTitle("Elbow forces");
  m_armElbPlot->setLabel(0, "applied");
  m_armElbPlot->setLabel(1, "inertia");
  m_armElbPlot->setLabel(2, "interaction");
  m_armElbPlot->setLabel(3, "damping"); 
  m_armElbPlot->setLabel(4, "total");   
  
  m_armShdPlot = new dmx::Plot(columnWidth, 180, 6, pointsPerPlot);
  m_armShdPlot->translate(ci::Vec4f(0, 400, 0, 1));
  m_armShdPlot->setTitle("Shoulder forces");
  m_armShdPlot->setLabel(0, "applied");  
  m_armShdPlot->setLabel(1, "inertia");
  m_armShdPlot->setLabel(2, "interaction");
  m_armShdPlot->setLabel(3, "damping");
  m_armShdPlot->setLabel(4, "total");   
  
  m_uiColumn2->m_children.push_back(m_armEffPlot);  
  m_uiColumn2->m_children.push_back(m_armElbPlot);
  m_uiColumn2->m_children.push_back(m_armShdPlot);  
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