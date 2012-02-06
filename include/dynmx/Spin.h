/*
 *  Spin.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 05/02/2012.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SPINNING_WHEEL_
#define _DMX_SPINNING_WHEEL_

#include "Model.h"
#include "Scene.h"
#include "View.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Spinning Color Wheel Model
//----------------------------------------------------------------------------------------------------------------------
class SpinningWheel : public Model
{

friend class SpinningWheelViz;

public:
  SpinningWheel() : m_time(0.0f), m_speed(0.0f), m_radius(300), m_numSegments(5) { init(); };
  
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  
  void setSpeed(float s) { m_speed = s; };
  
protected:

  float m_time;
  float m_speed;
  float m_radius;
  int m_numSegments;
  std::vector<float> m_segmentSizes;
  std::vector<ci::Color> m_segmentColors;
  ci::Matrix44f m_mat;
};

//----------------------------------------------------------------------------------------------------------------------
// Spinning Color Wheel Viz
//----------------------------------------------------------------------------------------------------------------------
class SpinningWheelViz : public NodeGeometry
{
public:

  SpinningWheelViz(SpinningWheel* wheel) : m_wheel(wheel) { init(); };
  virtual void createGeometry();
  virtual void update();

protected:
  virtual void init();
  
  SpinningWheel* m_wheel;
};


// ---------------------------------------------------------------------------------------------------------------------
class SpinningWheelView : public dmx::View
{

public:

  //--------------------------------------------------------------------------------------------------------------------
  SpinningWheelView(SpinningWheel* wheel) : m_wheel(wheel) {};
  
  // functions to be implemented by subclasses
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setupScene()
  {   
    m_wheelViz = new SpinningWheelViz(m_wheel);
    
    m_container = new NodeGroup();
    m_container->translate(ci::Vec4f(500, 400, 0, 1));
    m_container->m_children.push_back(m_wheelViz);
    m_scene2d.m_children.push_back(m_container);
  };
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw3d(){};
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void draw2d()
  {
    m_wheel->setSpeed(m_speed);
  };
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void keyDown (ci::app::KeyEvent event)
  {
    View::keyDown(event);
    switch(event.getChar()) 
    {
      case 'r':
        m_wheel->reset();
        m_wheelViz->createGeometry();
      default:
        break;
    }
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void buildGui()
  { 
    m_gui->addPanel();
    m_gui->addLabel("Wheel Controls");
    m_gui->addParam("Speed", &m_speed, 0.0f, 100.0f, 0);
  };
  
  float m_speed;
  SpinningWheel* m_wheel; 
  SpinningWheelViz* m_wheelViz;
  NodeGroup* m_container;
};

} // namespace

#endif