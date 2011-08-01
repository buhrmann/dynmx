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

  virtual ~App() {};

  // Inherited from ci::AppBasic
  //----------------------------------------------------------------------------------------------------------------------
  virtual void update();
  virtual void update(float dt);
  
  virtual void shutDown() { m_model->shutDown(); };
  
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
  
  std::string m_name;
  double m_prevTime;      /// For calculating elapsed time
  double m_elapsedTime;   /// stores elapsed time
  float m_fixedTimeStep;  /// When using fixed timeStep instead of elapsed time since last frame  
  bool m_paused;          /// Pause the model, while allowing interaction with view
  bool m_useFixedTimeStep;
  
}; // class App

} // namespace dmx

#endif
