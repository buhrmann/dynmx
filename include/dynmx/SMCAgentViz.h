/*
 *  SMCAgentViz.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_AGENT_VIZ_
#define _DMX_SMC_AGENT_VIZ_

#include "Scene.h"
#include "SMCAgent.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// Visualisation of an SMCAgent
//----------------------------------------------------------------------------------------------------------------------
class SMCAgentViz : public NodeGroup
{
public:
  
  SMCAgentViz(SMCAgent* agent) : m_agent(agent) { init(); };
  
  virtual void update();
  
protected:
  
  virtual void init();
  
  SMCAgent* m_agent;
  Disk* m_agentDisk;
  
}; // class
  
} // namespace
    
#endif