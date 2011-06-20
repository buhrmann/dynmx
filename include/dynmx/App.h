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
#include "Dynmx.h"
#include "Model.h"
#include "View.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------  
// Base class for apps that combine a model and a (opengl) view
//----------------------------------------------------------------------------------------------------------------------  
class App : public cinder::app::AppBasic
{

public:

  // Inherited from ci::AppBasic
  //----------------------------------------------------------------------------------------------------------------------
  virtual void update();
  virtual void update(float dt);
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setup();

  //--------------------------------------------------------------------------------------------------------------------
	virtual void prepareSettings( Settings *settings );
  
  //--------------------------------------------------------------------------------------------------------------------
  void setFixedTimeStep(float dt);
  
  //--------------------------------------------------------------------------------------------------------------------
  void togglePause();
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw();
  
  // Mouse and keyboard interaction
  //--------------------------------------------------------------------------------------------------------------------
  virtual void mouseMove(ci::app::MouseEvent event);
  virtual void mouseDrag(ci::app::MouseEvent event);
  virtual void mouseDown(ci::app::MouseEvent event);
  virtual void mouseUp(ci::app::MouseEvent event);
	virtual void keyDown(ci::app::KeyEvent event);
  virtual void keyUp(ci::app::KeyEvent event); 
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void resize(ci::app::ResizeEvent event);
  
  Model* m_model;
  View* m_view;
  
protected:
  
  double m_prevTime;      /// For calculating elapsed time
  bool m_paused;          /// Pause the model, while allowing interaction with view
  float m_fixedTimeStep;  /// When using fixed timeStep instead of elapsed time since last frame 
  
}; // class App
  
  
//----------------------------------------------------------------------------------------------------------------------
// Inline implementations
//----------------------------------------------------------------------------------------------------------------------  

//----------------------------------------------------------------------------------------------------------------------
inline void App::update() 
{
  // Calculate duration of frame
#if 0  
  double currentTime = getElapsedSeconds();
  double deltaTime = currentTime - m_prevTime;
  m_prevTime = currentTime;
  update(deltaTime); 
#endif
  
  update(m_fixedTimeStep);
}

//--------------------------------------------------------------------------------------------------------------------
void App::update(float dt) 
{ 
  if(!m_paused)
  {
    m_model->update(dt); 
  }
}

//--------------------------------------------------------------------------------------------------------------------
void App::setup()
{ 
  m_paused = false;
  m_fixedTimeStep = 1.0f / 100.0f;
  
  m_prevTime = getElapsedSeconds();
  m_model->init(); 
  if(m_view)
  {
    m_view->init(); 
  }
} 

//--------------------------------------------------------------------------------------------------------------------
void App::prepareSettings( Settings *settings )
{
  settings->setWindowSize( 800, 600 );
  settings->setFrameRate( 60.0f );
  settings->setFullScreen( false );
}

//--------------------------------------------------------------------------------------------------------------------
void App::setFixedTimeStep(float dt)            { m_fixedTimeStep = dt; }
void App::togglePause()                         { m_paused = !m_paused; }
void App::draw()                                { if(m_view) m_view->draw();}
void App::resize(ci::app::ResizeEvent event)    { m_view->resize(event); };
void App::mouseMove(ci::app::MouseEvent event)  { m_view->mouseMove(event); }
void App::mouseDrag(ci::app::MouseEvent event)  { m_view->mouseDrag(event); }
void App::mouseDown(ci::app::MouseEvent event)  { m_view->mouseDown(event); }
void App::mouseUp(ci::app::MouseEvent event)    { m_view->mouseUp(event); }  
void App::keyDown(ci::app::KeyEvent event)      { m_view->keyDown(event); }  

//--------------------------------------------------------------------------------------------------------------------
void App::keyUp(ci::app::KeyEvent event) 
{
  switch(event.getChar())
  {
    case ' ':
      m_paused = !m_paused;  
      break;
    case 's':
    {
      if(m_paused)
        m_model->update(m_fixedTimeStep);
    }
      break;
  }
  
  // let view respond to key
  m_view->keyUp(event); 
}

} // namespace dmx

#endif
