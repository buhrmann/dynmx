/*
 *  ofxView.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 12/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_VIEW_
#define _DMX_VIEW_

#include "Dynmx.h"
#include "Scene.h"
#include "Light.h"

#include "cinder/Camera.h"
#include "cinder/MayaCamUI.h"
#include "cinder/params/Params.h"
#include "cinder/gl/Texture.h"
#include "SimpleGUI.h"

namespace dmx
{
// indicated that the view doesn't want to override the app's default frame rate
#define DEFAULT_VIEW_FRAME_RATE -1
#define USE_SIMPLE_GUI 1

//----------------------------------------------------------------------------------------------------------------------  
// Interface for a view 3d+2d view of a simulation
//----------------------------------------------------------------------------------------------------------------------
class View
{

public:
  
  virtual ~View();

  // functions to be implemented by subclasses
  virtual void setupScene() = 0; 
  virtual void draw3d() = 0;
  virtual void draw2d() = 0;
  virtual void buildGui() = 0;
      
  // core functionality
  virtual void init();
  virtual void update(float dt){}; // E.g. for updating internal calculated data after a simulation step
  virtual void draw();
  virtual void reset() { m_scene2d.reset(); m_scene3d.reset(); };
  
  virtual int getDesiredFrameRate() { return DEFAULT_VIEW_FRAME_RATE; } ;

  virtual void mouseMove(ci::app::MouseEvent event);
  virtual void mouseDrag(ci::app::MouseEvent event);
  virtual void mouseDown(ci::app::MouseEvent event);
  virtual void mouseUp(ci::app::MouseEvent event);  
	virtual void keyDown(ci::app::KeyEvent event);  
  virtual void keyUp(ci::app::KeyEvent event);  
  virtual void resize(ci::app::ResizeEvent event);
  virtual int pick(int x, int y);
  
  CameraPersp& getCameraPersp() { return m_cam; };
  
  // Getter and Setter for name, to be used by Factories e.g.
  const std::string& getName() const { return m_name; };
  void setName(const std::string& name) { m_name = name; };
  
  void pause(bool p) { m_paused = p; };
  
  void setVSync(bool vsync) { if(vsync) ci::gl::enableVerticalSync(); else ci::gl::disableVerticalSync(); };
  
protected: 
  
  ci::Vec3f screenToWorld(const ci::Vec2i& point);
  ci::Vec3f unproject(const ci::Vec3f& point);

  // scene graph stuff
  NodeGroup m_scene3d;
  NodeGroup m_scene2d;
  NodeGeometry* m_selectedNode;
  Light m_light;
  ci::CameraPersp m_cam;
  ci::MayaCamUI	m_cam3d;
  ci::CameraOrtho m_cam2d;
	ci::Vec2i m_mouse; 
  ci::Vec3f m_mouseWorld;
  ci::Vec4f m_backgroundColor;
  ci::Area m_viewport;
  ci::Rectf m_windowSize;  
  
#if USE_SIMPLE_GUI  
  mowa::sgui::SimpleGUI* m_gui;
#else
  ci::params::InterfaceGl m_gui;
#endif
  
  struct BackGround
  {
    ci::ColorA topLeft;
    ci::ColorA topRight;
    ci::ColorA bottomLeft;
    ci::ColorA bottomRight;
  };

  struct
  {
    bool m_drawWireframe:1;
    bool m_drawOutlines:1;
    bool m_drawShadows:1;
    bool m_draw3d:1;
    bool m_draw2d:1;
    bool m_showMenu;
  };
  
  BackGround m_background;

  // stuff for picking objects in the scene
  static const int m_selectBufSize;
  GLuint m_selectionBuffer[128];
  
  std::string m_name;
  ci::Font m_font;
  
  bool m_paused;
};

}

#endif