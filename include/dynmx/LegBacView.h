/*
 *  LegBacView.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_LEGBACVIEW_
#define _DMX_LEGBACVIEW_

#include "View.h"
#include "Scene.h"
#include "CTRNNViz.h"
#include "LegBacViz.h"

#include "cinder/gl/gl.h"
#include "cinder/Matrix.h"
#include "cinder/gl/Texture.h"
#include "SimpleGUI.h"

namespace dmx
{
  
class LegBac;
  
// ---------------------------------------------------------------------------------------------------------------------
class LegBacView : public View
{
  
public:
  
  LegBacView(LegBac* agent) : m_agent(agent), m_fixedFrameRate(DEFAULT_VIEW_FRAME_RATE) {};
  virtual ~LegBacView(){};
  
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
  
  LegBac* m_agent;
  LegBacViz* m_agentViz;
  CTRNNViz* m_ctrnnViz;
  int32_t m_fixedFrameRate;
  
  Plot* m_plot;
  Plot* m_ctrnnPlot;
  Plot* m_fitnessPlot;
  
  mowa::sgui::LabelControl* m_timeLabel;
  
  ci::Vec3f m_scenePos;
  bool m_followCam;
  
  float m_lrate;
  float m_ndecay;
  float m_wdecay;
  float m_synscale;
  float m_noiseVar;
  bool m_useReward;
  bool m_nncontrol;
  
}; // class

  
  
} // namespace dmx

#endif