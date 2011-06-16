/*
 *  ofxView.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 12/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "View.h"

namespace dmx
{

using namespace cinder;
using namespace cinder::app;

const int View::m_selectBufSize = 128;

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
void View::init()
{

  m_backgroundColor = Vec4f(0.3f, 0.3f, 0.3f, 1.0);
  const float g = 0.6f;
  m_background.topLeft = ci::ColorA(g, g, g, 0.5f);
  m_background.topRight= ci::ColorA(g, g, g, 0.5f);
  m_background.bottomLeft = ci::ColorA(1,1,1,0.5);
  m_background.bottomRight = ci::ColorA(1,1,1,0.5);
  
  // setup openGL modes
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_CULL_FACE);
  glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);	// Use The Good Calculations
  RenderState::g_shadingMode = GL_SMOOTH;

  m_drawWireframe = false;
  m_drawOutlines = true;
  m_drawShadows = true;
  m_draw3d = true;
  m_draw2d = true;

  // use simpler color control !
  glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
  glEnable ( GL_COLOR_MATERIAL ) ;

  // setup the picking buffer: //
  glSelectBuffer (m_selectBufSize, m_selectionBuffer);
  m_selectedNode = 0;

  // a light
  m_light.m_TM.setRow(3, Vec4f(-10.0f, -10.0f, 10.0f, 1.0f));
  m_light.m_TM.setRow(0, Vec4f(-1, -1, -1, 1));
  m_scene3d.m_children.push_back(&m_light);
  
  // set up the cameras
	CameraPersp cam;
	cam.setEyePoint(Vec3f(0.0f, 1.0f, 2.0f));
	cam.setCenterOfInterestPoint(Vec3f(0.0f, 0.0f, 0.0f));
  cam.setWorldUp(Vec3f(0,1,0));
	cam.setPerspective(60.0f, getWindowAspectRatio(), 1.0f, 1000.0f);
	m_cam3d.setCurrentCam(cam);
    
  //Vec3f centre(ci::app::getWindowWidth() / 2.0f,  ci::app::getWindowHeight() / 2.0f, 0.0f);
	//m_cam2d.setEyePoint(centre + Vec3f(0, 0, -1));
	//m_cam2d.setCenterOfInterestPoint( Vec3f(0.0f, 0.0f, 0.0f) );
	//m_cam2d.setPerspective( 60.0f, getWindowAspectRatio(), 1.0f, 1000.0f );
  m_cam2d.setOrtho(0, ci::app::getWindowWidth(), ci::app::getWindowHeight(), 0, 0.01f, 100.0f);

  m_hud = params::InterfaceGl( "Tweakbar", Vec2i( 200, 400 ) );
  buildGui();
  
  setupScene();
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
void View::draw()
{
  // Clear the background
	glClearColor(m_backgroundColor.x, m_backgroundColor.y, m_backgroundColor.z, 1.0f);  
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
    
  // small viewport for picking
  if(RenderState::g_renderPass == RenderState::RENDER_PICKING)
  {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity(); 
    // Set perspective viewing transformation
    GLint viewport[4];
  	glGetIntegerv (GL_VIEWPORT, viewport);
    gluPickMatrix (
      (GLdouble) m_mouse.x,
      (GLdouble) (viewport[3] - m_mouse.y),
      10, 10,
      viewport);
    gluPerspective(60, getWindowAspectRatio(), 1, 50);
  }
  else
  {
    gl::setProjection(m_cam3d.getCamera());
  }
  
  gl::setModelView(m_cam3d.getCamera());
      
  // picking
  if(RenderState::g_renderPass == RenderState::RENDER_PICKING)
  {
    if(m_draw3d)
    {
      glInitNames();
      // draw TODO: find most minimal way to draw for picking
      RenderState::startSurfaceMode();
      m_scene3d.update();
      RenderState::stopSurfaceMode();
    }
  }
  // normal rendering
  else
  {
    if(m_draw3d)
    {
      // solid object render
      RenderState::startSurfaceMode();
      m_scene3d.update();
      RenderState::stopSurfaceMode();

       // wireframes
      if(m_drawWireframe)
      {
        RenderState::startWireframeMode();
        m_scene3d.update();
        RenderState::stopWireframeMode();
      }

      // outlines
      if(m_drawOutlines)
      {
        RenderState::startOutlineMode();
        m_scene3d.update();
        RenderState::stopOutlineMode();
      }

      // shadows
      if(m_drawShadows)
      {
        Vec4f origin(0,0,0,1);
        Vec4f normal(0,0,1,1);
        Vec4f dir = m_light.m_TM.getRow(0);
        //Vec3f dir = Vec3f(m_light.m_TM.getRow(0).x, m_light.m_TM.getRow(0).y, m_light.m_TM.getRow(0).z);
        RenderState::startShadowMode(normal, origin, dir);
        m_scene3d.update();
        RenderState::stopShadowMode();
      }

      // custom 3d drawing
      draw3d();
    }

    // 2D stuff
    glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT | GL_COLOR_BUFFER_BIT | GL_POLYGON_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_COLOR, GL_ONE);
    glDisable(GL_CULL_FACE);

    //gl::setMatrices(m_cam2d);
    gl::setMatricesWindow(getWindowWidth(), getWindowHeight(), true);
    
    // custom 2d drawing
    if(m_draw2d)
    {
      // draw a background quad with potentially four differently coloured corners
      dmx::drawRectangle(getWindowWidth(), getWindowHeight(), m_background.topLeft, m_background.topRight,
        m_background.bottomLeft, m_background.bottomRight);
      m_scene2d.update();
      draw2d();
    }

    // finally draw the GUI on top
    glDisable(GL_BLEND);
    ci::params::InterfaceGl::draw();
    
    glPopAttrib();
  } // when in render mode
}

//----------------------------------------------------------------------------------------------------------------------
// select an object from the scene graph with mouse
//----------------------------------------------------------------------------------------------------------------------
int View::pick(int x, int y)
{
  // setup render mode and draw scene
  glRenderMode( GL_SELECT );
  RenderState::g_renderPass = RenderState::RENDER_PICKING;
  draw();
  int hits = glRenderMode( GL_RENDER );
  RenderState::g_renderPass = RenderState::RENDER_SURFACE;

  // process hits: find closest (assumes not hierarchical names for now)
  GLuint names, *ptr, minZ, *ptrNames, numberOfNames;

  ptr = (GLuint *) m_selectionBuffer;
  minZ = 0xffffffff;
  if(hits > 0)
  {
    for (unsigned int i = 0; i < hits; i++)
    {
      names = *ptr;
      ptr++;
      if (*ptr < minZ)
      {
        numberOfNames = names;
        minZ = *ptr;
        ptrNames = ptr + 2;
      }
      ptr += names + 2;
    }

    // turn off currently selected node
    if(m_selectedNode != 0)
    {
      m_selectedNode->m_picked = false;
    }
    // change to newly selected
    m_selectedNode = (NodeGeometry*) m_scene3d.getNode(*ptrNames);
    // have to check as some geometry is un-selectable
    if(m_selectedNode)
    {
      printf("Picked node: %i \n", m_selectedNode->m_uniqueID);
      m_selectedNode->m_picked = true;
    }
  }
  return hits;
}

//----------------------------------------------------------------------------------------------------------------------
void View::keyDown(KeyEvent event)
{
  switch(event.getChar()) 
  {
    case '2': 
      m_draw2d = !m_draw2d; 
      break;
    case '3': 
      m_draw3d = !m_draw3d; 
      break;
    case '=': 
      //gui.nextPage(); 
      //gui.show(); 
      break;
    case ' ': 
      //gui.toggleDraw();
      //m_easyCam3d.fix(gui.isOn());
      break;
    case '[': 
      //gui.prevPage(); 
      break;
    case ']': 
      //gui.nextPage(); 
      break;
    //case 'p': gui.nextPageWithBlank(); break;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void View::keyUp(KeyEvent event)
{
  m_scene2d.onKeyPress(event);
}

//----------------------------------------------------------------------------------------------------------------------
void View::mouseMove(MouseEvent event)
{
  // keep track of the mouse
  m_mouse = event.getPos();
  
  m_scene2d.onMouseMove(Vec3f(m_mouse.x, m_mouse.y, 0));  
}

//----------------------------------------------------------------------------------------------------------------------
void View::mouseDrag(MouseEvent event)
{
  // gui
  //if(!gui.isOn())
  {
    // keep track of the mouse
    m_mouse = event.getPos();

    // let the camera handle the interaction
    m_cam3d.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() ); 
  }
}

//----------------------------------------------------------------------------------------------------------------------
void View::mouseDown(MouseEvent event)
{
  int N = pick(event.getX(), event.getY());
  // only update mouse/camera when we didn't hit anything
  if(N == 0)
  {
    bool handled = false;//gui->mousePressed(x, y, button);

    // deactivate currently selected node, if gui wasn't pressed rather than the scene
    if(!handled)
    {
      if(m_selectedNode != 0)
        m_selectedNode->m_picked = false;
      m_selectedNode = 0;
    }
  }

  // let the camera handle the interaction
  m_cam3d.mouseDown( event.getPos() );
}

//----------------------------------------------------------------------------------------------------------------------
void View::mouseUp(MouseEvent event)
{
}

//----------------------------------------------------------------------------------------------------------------------
void View::resize(ResizeEvent event)
{
	// adjust aspect ratio
	CameraPersp cam = m_cam3d.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	m_cam3d.setCurrentCam( cam );
}

} // namespace dmx


