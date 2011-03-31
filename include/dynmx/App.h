/*
 *  ofxMVC.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 12/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_APP_
#define _DMX_APP_

#include "cinder/app/AppBasic.h"
#include "Model.h"
#include "View.h"

namespace dmx
{

class App : public cinder::app::AppBasic
{

public:

  // inherited from ci::AppBasic
  virtual void update() 
  {
    // get duration of frame
    double currentTime = getElapsedSeconds();
    double deltaTime = currentTime - m_prevTime;
    m_prevTime = currentTime;
    
    update(deltaTime); 
  };
  
  virtual void update(float dt) { m_model->update(dt); };
  
  virtual void setup()
  { 
    m_prevTime = getElapsedSeconds();
    m_model->init(); 
    if(m_view) 
      m_view->init(); 
  }; 
  
	virtual void prepareSettings( Settings *settings )
  {
  	settings->setWindowSize( 800, 600 );
    settings->setFrameRate( 60.0f );
    settings->setFullScreen( false );
  };
      
  virtual void draw() { if(m_view) m_view->draw(); };
  
  virtual void mouseMove(ci::app::MouseEvent event) { m_view->mouseMove(event); };
  virtual void mouseDrag(ci::app::MouseEvent event) { m_view->mouseDrag(event); };
  virtual void mouseDown(ci::app::MouseEvent event) { m_view->mouseDown(event); };
  virtual void mouseUp(ci::app::MouseEvent event)   { m_view->mouseUp(event); };  
	virtual void keyDown(ci::app::KeyEvent event)     { m_view->keyDown(event); };  
  virtual void keyUp(ci::app::KeyEvent event)       { m_view->keyUp(event); };  
  virtual void resize(ci::app::ResizeEvent event)   { m_view->resize(event); };
  
  Model* m_model;
  View* m_view;
  
protected:
  double m_prevTime;
  
}; // class App

} // namespace dmx

#endif
