/*
 *  SMCAgentViz.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_LEGBAC_VIZ_
#define _DMX_LEGBAC_VIZ_

#include "Scene.h"
#include "LegBac.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// Visualisation of a LegBac agent
//----------------------------------------------------------------------------------------------------------------------
class LegBacViz : public NodeGroup
{
public:
  
  LegBacViz(LegBac* agent) : m_agent(agent) { init(); };
  
  virtual void update();
  virtual void reset();
  virtual void onKeyPress(cinder::app::KeyEvent e);
  
protected:
  
  virtual void init();
  void drawLeg();
  void drawBac();
  
  LegBac* m_agent;
  Disk* m_agentDisk;
  std::deque<ci::Vec2f> m_traj;
  
  int m_steps;
  bool m_paused;
  
}; // class
  
} // namespace

#endif