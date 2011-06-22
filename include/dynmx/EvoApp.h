/*
 *  EvoApp.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 21/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
 
#ifndef _DNM_EVO_APP_
#define _DNM_EVO_APP_

namespace dmx
{

class Evolvable;
class App;

//----------------------------------------------------------------------------------------------------------------------
// A class managing a GA experiment that can be run to evolve or test, both in visual as well as non-visual mode. 
//----------------------------------------------------------------------------------------------------------------------
class EvoApp
{
public:
  EvoApp(Evolvable* evolvable, App* app = 0) : m_evolvable(evolvable), m_app(app) {};
  
  void setApp(App* app) { m_app = app; };
  
  void start(bool evolve = true);
  
protected:

  void runVisual(bool evolve);
  void runNonVisual(bool evolve);
    
  Evolvable* m_evolvable;
  App* m_app;
};

} // namespace dmx

#endif