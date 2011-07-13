/*
 *  App.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 21/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
 
#include "App.h"

namespace dmx
{

//--------------------------------------------------------------------------------------------------------------------
void App::setup()
{ 
  m_paused = true;
  m_useFixedTimeStep = true;
  m_fixedTimeStep = 1.0f / 100.0f;
  
  m_prevTime = getElapsedSeconds();
  m_model->init(); 
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
  if(!m_paused)
  {
    m_model->update(dt); 
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
void App::draw()                                { if(m_view) m_view->draw(); }
void App::resize(ci::app::ResizeEvent event)    { m_view->resize(event); }
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
      break;
    }
    case 'i':
      m_model->init();
      break;
  }
  
  // let view respond to key
  m_view->keyUp(event); 
}

} // namespace dmx
