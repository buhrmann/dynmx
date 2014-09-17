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
  std::cout << "App::setup()" << std::endl;
  
  m_paused = false;
  m_useFixedTimeStep = true;  
  m_prevTime = getElapsedSeconds();
  
  //m_model->init(); 
  if(m_view)
  {
    m_view->init(); 
  }
} 

//----------------------------------------------------------------------------------------------------------------------
void App::update() 
{
  // Calculate duration of frame
  double currentTime = getElapsedSeconds();
  m_elapsedTime = currentTime - m_prevTime;
  m_prevTime = currentTime;
  
  const int desFrameRate = m_view->getDesiredFrameRate(); 
  float dt;
  if(desFrameRate > 0)
  {
    dt = 1.0f / desFrameRate;  
  }
  else if(m_useFixedTimeStep)
  {
    dt = m_fixedTimeStep;
  }
  else
  {
    dt = m_elapsedTime; 
  }  
  
  update(dt);
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
  std::cout << "App::prepareSettings()" << std::endl;
  
  // Window size
  int w = 800;
  int h = 600;
  if (SETTINGS->hasChild("Config/Globals/WindowSize"))
  {
    w = SETTINGS->getChild("Config/Globals/WindowSize")["Width"].as<int>();
    h = SETTINGS->getChild("Config/Globals/WindowSize")["Height"].as<int>();    
  }
  settings->setWindowSize( w, h );
  
  // Frame rate
  int fps = DEFAULT_FRAMERATE;
  if (SETTINGS->hasChild("Config/Globals/FrameRate"))
  {
    fps = SETTINGS->getChild("Config/Globals/FrameRate")["Value"].as<int>();
  }
  m_fixedTimeStep = 1.0 / (float) fps;  // Simulation integration time step
  //settings->setFrameRate(fps);          // How many FPS the app gets updated with
  
  // Fullscreen
  if (SETTINGS->hasChild("Config/Globals/FullScreen"))
  {
    settings->setFullScreen(SETTINGS->getChild("Config/Globals/FullScreen")["Value"].as<bool>());
  }
  
  // VSync
  bool vsync;
  if (SETTINGS->hasChild("Config/Globals/VSync"))
  {
    vsync = SETTINGS->getChild("Config/Globals/VSync")["Value"].as<bool>();
  }
  else
  {
    vsync = false;
  }
  m_view->setVSync(vsync);
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
        m_view->update(m_fixedTimeStep);
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
