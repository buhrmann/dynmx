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

#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/MayaCamUI.h"

#include "Scene.h"
#include "Light.h"

namespace dmx
{

class View
{
public:

  // functions to be implemented by subclasses
  virtual void setupScene() = 0; 
  virtual void draw3d() = 0;
  virtual void draw2d() = 0;
  virtual void buildGui() = 0;
      
  // core functionality
  virtual void init();
  virtual void draw();

  virtual void mouseMove(ci::app::MouseEvent event);
  virtual void mouseDrag(ci::app::MouseEvent event);
  virtual void mouseDown(ci::app::MouseEvent event);
  virtual void mouseUp(ci::app::MouseEvent event);  
	virtual void keyDown(ci::app::KeyEvent event);  
  virtual void keyUp(ci::app::KeyEvent event);  
  virtual void resize(ci::app::ResizeEvent event);
  virtual int pick(int x, int y);
  
  // scene graph stuff
  NodeGroup m_scene3d;
  NodeGroup m_scene2d;
  NodeGeometry* m_selectedNode;
  Light m_light;
  ci::MayaCamUI	m_cam3d;
  ci::CameraOrtho m_cam2d;
	ci::Vec2i m_mouse; 
  ci::Vec4f m_backgroundColor;

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
  };
  
  BackGround m_background;

  // stuff for picking objects in the scene
  static const int m_selectBufSize;
  GLuint m_selectionBuffer[128];
};

}

#endif