/*
 *  TestView.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 12/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
 
#ifndef _OFX_TEST_VIEW_
#define _OFX_TEST_VIEW_

#include "View.h"

// forward declarations
class TestModel;

// ---------------------------------------------------------------------------------------------------------------------
class TestView : public dmx::View
{

public:

  TestView() : m_model(0) {};
  TestView(TestModel* model) : m_model(model) {};
  
  // functions to be implemented by subclasses
  virtual void setupScene()
  {
    //m_backgroundColor.Set(244.0/255.0, 242.0/255.0, 229.0/255.0);
    //m_backgroundColor = ci::Vec4f(0.3, 0.3, 0.3);
    
    dmx::Sphere* sphere = new dmx::Sphere(0.1, 16);
    sphere->createGeometry();
    sphere->translate(ci::Vec3f(0,0,1));
    sphere->m_outlineColor = ci::Vec4f(1.0, 1.0, 1.0, 0.3);
    m_scene3d.m_children.push_back(sphere);
    
    dmx::Grid* grid = new dmx::Grid(10, 10, 10, 10);
    grid->createGeometry();
    m_scene3d.m_children.push_back(grid);
    
    dmx::Axes* axes = new dmx::Axes();
    axes->createGeometry();
    m_scene3d.m_children.push_back(axes);
    
    //m_font.loadFont("Times New Roman.ttf", 28);
  };
  
  virtual void draw3d()
  {    
    glPushMatrix();
      glTranslatef(0, 0, 0);
      dmx::setColor4(ci::Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
      dmx::drawSphere(0.05f, 16, 16);
    glPopMatrix();  
  };
  
  virtual void draw2d()
  {
    if(m_model)
    {
      //ofSetColor(255, 130, 0);
      //ofFill();		// draw "filled shapes"
      //ofCircle(100, 100, m_model->m_val*100);
      glPushMatrix();
      glTranslatef(0, 0, 0);
      dmx::drawDisk(100, m_model->m_val*50 + 50, 16, 16);
      glPopMatrix();
      
      glColor4f(0.05, 0.05, 0.05, 0.95f);      
      glPushMatrix();
      glScalef(1, -1, 1);
      //m_font.drawString("TrueType Font", m_winWidth/2.0, -m_winHeight/2.0);
      glPopMatrix();
    }
  };
  virtual void buildGui(){};
  
  TestModel* m_model;
  //ofTrueTypeFont m_font;
};

#endif
