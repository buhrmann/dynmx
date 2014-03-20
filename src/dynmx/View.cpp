/*
 *  ofxView.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 12/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "View.h"
#include "cinder/ImageIo.h"
#include "cinder/gl/gl.h"
#include "cinder/app/KeyEvent.h"

namespace dmx
{

using namespace cinder;
using namespace cinder::app;

const int View::m_selectBufSize = 128;

//----------------------------------------------------------------------------------------------------------------------
View::~View()
{
  
}
  
//----------------------------------------------------------------------------------------------------------------------
void View::init()
{
  m_paused = false;

  m_backgroundColor = Vec4f(0.3f, 0.3f, 0.3f, 1.0);
  const float g = 0.6f;
  m_background.topLeft = ci::ColorA(g, g, g, 0.5f);
  m_background.topRight= ci::ColorA(g, g, g, 0.5f);
  m_background.bottomLeft = ci::ColorA(1,1,1,0.0);
  m_background.bottomRight = ci::ColorA(1,1,1,0.0);
  
  m_font = ci::Font(ci::app::loadResource("pf_tempesta_seven.ttf"), 24);  
  
  ci::gl::disableVerticalSync();
  
  // setup openGL modes
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_LINE_SMOOTH); 
  glEnable(GL_CULL_FACE);
  glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);	// Use The Good Calculations
  RenderState::g_shadingMode = GL_SMOOTH;

  m_drawWireframe = false;
  m_drawOutlines = false;
  m_drawShadows = false;
  m_draw3d = true;
  m_draw2d = false;
  m_showMenu = false;

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
	m_cam.setEyePoint(Vec3f(0.0f, 0.0f, 1.0f));
	m_cam.setCenterOfInterestPoint(Vec3f(0.0f, 0.0f, 0.0f));
  m_cam.setWorldUp(Vec3f(0,1,0));
	m_cam.setPerspective(60.0f, getWindowAspectRatio(), 0.001f, 100.0f);
	m_cam3d.setCurrentCam(m_cam);
    
  m_cam2d.setOrtho(0, ci::app::getWindowWidth(), ci::app::getWindowHeight(), 0, 0.001f, 100.0f);

#if USE_SIMPLE_GUI  
  m_gui = new mowa::sgui::SimpleGUI(ci::app::App::get());
  mowa::sgui::SimpleGUI::lightColor = ColorA(1, 1, 153/255, 1);
  m_gui->addColumn();
  m_gui->addLabel("Controls");
#else
  m_gui = params::InterfaceGl( "Tweakbar", Vec2i( 200, 400 ) );
  m_gui.setOptions("", "iconified=true");
#endif  
  
  buildGui();
  
  setupScene();
}

//----------------------------------------------------------------------------------------------------------------------
void View::draw()
{

  if(getWindowWidth() <= 0)
    return;
  
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
  
  // Unproject 2d mouse position into 3d mouse coordinates
  m_mouseWorld = screenToWorld(m_mouse);
      
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
      
#if 0      
      // draw projected mouse position
      glPushMatrix();
      glTranslatef(m_mouseWorld);
      drawFrame(0.1, 0.1);
      glPopMatrix();
#endif
    }

    // 2D stuff
    glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT | GL_COLOR_BUFFER_BIT | GL_POLYGON_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glDisable(GL_CULL_FACE);

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
    
    //ci::gl::drawString("dynmx", ci::Vec2f(25,75), ci::ColorA(1,1,1,0.5), m_font);

    // finally draw the GUI on top
    glDisable(GL_BLEND);
#if USE_SIMPLE_GUI
    if(m_showMenu)
    {
      m_gui->draw();
    }
#else    
    ci::params::InterfaceGl::draw();
#endif
    
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
  GLuint names, *ptr, minZ, numberOfNames;
  GLuint *ptrNames = NULL;

  ptr = (GLuint *) m_selectionBuffer;
  minZ = 0xffffffff;
  if(hits > 0)
  {
    for (int i = 0; i < hits; i++)
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
  switch(event.getCode()) 
  {
    case ci::app::KeyEvent::KEY_2: 
      m_draw2d = !m_draw2d; 
      break;
    case ci::app::KeyEvent::KEY_3: 
      m_draw3d = !m_draw3d; 
      break;
    case ci::app::KeyEvent::KEY_w:
      m_drawWireframe = !m_drawWireframe;
      break;
    case ci::app::KeyEvent::KEY_o:
      m_drawOutlines = !m_drawOutlines;
      break;
    case ci::app::KeyEvent::KEY_s:
      m_drawShadows = !m_drawShadows;
      break;      
    case ci::app::KeyEvent::KEY_TAB:
      m_showMenu = !m_showMenu;
#if !USE_SIMPLE_GUI      
      if(m_showMenu)
        m_gui.setOptions("", "iconified=false");
      else
        m_gui.setOptions("", "iconified=true");
#endif      
      break;
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
  
  m_scene2d.onMouseDrag(event);  
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
  m_scene2d.onMousePress(Vec3f(event.getX(), event.getY(), 0));
}

//----------------------------------------------------------------------------------------------------------------------
void View::resize(ResizeEvent event)
{
	// adjust aspect ratio
	CameraPersp cam = m_cam3d.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	m_cam3d.setCurrentCam( cam );
  
  // Update window size
  m_windowSize = Rectf(0.0f, 0.0f, (float)event.getWidth(), (float)event.getHeight());  

  // let ui interface adjust
  m_scene2d.onResize(event);  
}

//----------------------------------------------------------------------------------------------------------------------  
ci::Vec3f View::screenToWorld(const ci::Vec2i& point)
{
	// Find near and far plane intersections
	Vec3f point3f = Vec3f((float)point.x, m_windowSize.getHeight() * 0.5f - (float)point.y, 0.0f);
	Vec3f nearPlane = unproject(point3f);
	Vec3f farPlane = unproject(Vec3f(point3f.x, point3f.y, 1.0f));
  
	// Calculate X, Y and return point
	float theta = (0.0f - nearPlane.z) / (farPlane.z - nearPlane.z);
	return Vec3f(nearPlane.x + theta * (farPlane.x - nearPlane.x), 
               nearPlane.y + theta * (farPlane.y - nearPlane.y), 
               0.0f);
}
  
//----------------------------------------------------------------------------------------------------------------------  
ci::Vec3f View::unproject(const ci::Vec3f& point)
{
	// Find the inverse Modelview-Projection-Matrix
	Matrix44f mInvMVP = ci::gl::getProjection() * ci::gl::getModelView();
	mInvMVP.invert();
  
	// Transform to normalized coordinates in the range [-1, 1]
	Vec4f pointNormal;
  ci::Area viewport = ci::gl::getViewport();
	pointNormal.x = (point.x - viewport.getX1()) / viewport.getWidth() * 2.0f - 1.0f;
	pointNormal.y = (point.y - viewport.getY1()) / viewport.getHeight() * 2.0f;
	pointNormal.z = 2.0f * point.z - 1.0f;
	pointNormal.w = 1.0f;
  
	// Find the object's coordinates
	Vec4f pointCoord = mInvMVP * pointNormal;
	if (pointCoord.w != 0.0f)
  {
		pointCoord.w = 1.0f / pointCoord.w;
  }
  
	// Return coordinate
	return Vec3f(pointCoord.x * pointCoord.w, 
               pointCoord.y * pointCoord.w, 
               pointCoord.z * pointCoord.w);
  
}

} // namespace dmx


