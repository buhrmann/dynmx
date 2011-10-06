/*
 *  App.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 21/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
 
#include "App.h"
#include "cinder/app/KeyEvent.h"

namespace dmx
{

//--------------------------------------------------------------------------------------------------------------------
void App::setup()
{ 
  m_paused = false;
  m_useFixedTimeStep = true;

  // Read global config
  float dt = DEFAULT_TIMESTEP;
  if (SETTINGS->hasChild("Config/Globals/FrameRate"))
  {
    dt = 1.0f / SETTINGS->getChild("Config/Globals/FrameRate").getAttributeValue<int>("Value");
  }
  m_fixedTimeStep = dt;
  
  m_prevTime = getElapsedSeconds();
  //m_model->init(); 
  if(m_view)
  {
    m_view->init(); 
  }
} 

//----------------------------------------------------------------------------------------------------------------------
inline void App::update() 
{
  // Calculate duration of frame
  double currentTime = getElapsedSeconds();
  m_elapsedTime = currentTime - m_prevTime;
  m_prevTime = currentTime;
  
  const int desFrameRate = m_view->getDesiredFrameRate(); 
  if(desFrameRate > 0)
  {
    update(1.0f / desFrameRate);  
  }
  else if(m_useFixedTimeStep)
  {
    update(m_fixedTimeStep);
  }
  else
  {
    update(m_elapsedTime); 
  }  
}

//--------------------------------------------------------------------------------------------------------------------
void App::update(float dt) 
{ 
  if(m_model->hasFinished())
  {
    quit();
  }
  
  if(!m_paused)
  {
    m_model->update(dt); 
    m_view->update(dt);
  }
}

//--------------------------------------------------------------------------------------------------------------------
void App::prepareSettings( Settings *settings )
{
  int w = 800;
  int h = 600;
  if (SETTINGS->hasChild("Config/Globals/WindowSize"))
  {
    w = SETTINGS->getChild("Config/Globals/WindowSize").getAttributeValue<int>("Width");
    h = SETTINGS->getChild("Config/Globals/WindowSize").getAttributeValue<int>("Height");    
  }
  settings->setWindowSize( w, h );
  settings->setFrameRate( 60.0f );
  settings->setFullScreen( false );
}

//--------------------------------------------------------------------------------------------------------------------
void App::setFixedTimeStep(float dt)            { m_fixedTimeStep = dt; }
void App::togglePause()                         { m_paused = !m_paused; m_view->pause(m_paused); }
void App::draw()                                { m_view->draw(); }
void App::resize(ci::app::ResizeEvent event)    { m_view->resize(event); }
void App::mouseMove(ci::app::MouseEvent event)  { m_view->mouseMove(event); }
void App::mouseDrag(ci::app::MouseEvent event)  { m_view->mouseDrag(event); }
void App::mouseDown(ci::app::MouseEvent event)  { m_view->mouseDown(event); }
void App::mouseUp(ci::app::MouseEvent event)    { m_view->mouseUp(event); }  
void App::keyDown(ci::app::KeyEvent event)      { m_view->keyDown(event); }  

//--------------------------------------------------------------------------------------------------------------------
void App::keyUp(ci::app::KeyEvent event) 
{
  switch(event.getCode())
  {
    case ci::app::KeyEvent::KEY_SPACE:
      togglePause();  
      break;
    case ci::app::KeyEvent::KEY_RETURN:
    {
      if(m_paused)
      {
        m_model->update(m_fixedTimeStep);
        m_view->draw();
      }
      break;
    }
    case ci::app::KeyEvent::KEY_r:
      m_model->reset();
      break;
  }
  
  // let view respond to key
  m_view->keyUp(event); 
}

} // namespace dmx
