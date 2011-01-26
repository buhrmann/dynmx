/*
 *  ofxMVC.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 12/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_MVC_
#define _DMX_MVC_

#include "cinder/app/AppBasic.h"
#include "Model.h"
#include "View.h"

namespace dmx
{

class App : public cinder::app::AppBasic
{

public:

  // inherited from ci::AppBasic
  virtual void update(float dt) { m_model->update(dt); };
  virtual void setup() { m_model->init(); m_view->init(); }; 
	void prepareSettings( Settings *settings ) {};    
  virtual void draw() { m_view->draw(); };
  
  virtual void mouseMove(ci::app::MouseEvent event) { m_view->mouseMove(event); };
  virtual void mouseDrag(ci::app::MouseEvent event) { m_view->mouseDrag(event); };
  virtual void mouseDown(ci::app::MouseEvent event) { m_view->mouseDown(event); };
  virtual void mouseUp(ci::app::MouseEvent event)   { m_view->mouseUp(event); };  
	virtual void keyDown(ci::app::KeyEvent event)     { m_view->keyDown(event); };  
  virtual void keyUp(ci::app::KeyEvent event)       { m_view->keyUp(event); };  
  virtual void resize(ci::app::ResizeEvent event)   { m_view->resize(event); };
  
  Model* m_model;
  View* m_view;
};

}

CINDER_APP_BASIC( dmx::App, ci::app::RendererGl )

#endif
