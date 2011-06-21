/*
 *  TestView.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 12/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _OFX_TEST_VIEW_CTRNN_
#define _OFX_TEST_VIEW_CTRNN_

#include "View.h"
#include "CTRNN.h"
#include "CTRNNViz.h"

// ---------------------------------------------------------------------------------------------------------------------
class TestViewCTRNN : public dmx::View
{

public:

  //--------------------------------------------------------------------------------------------------------------------
  TestViewCTRNN() : m_ctrnn(0) {};
  TestViewCTRNN(CTRNN* ctrnn) : m_ctrnn(ctrnn) {};
  
  // functions to be implemented by subclasses
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setupScene()
  {
    assert(m_ctrnn);
    
    m_ctrnnView = new dmx::CTRNNViz(m_ctrnn, 175);
    m_ctrnnView->translate(ci::Vec4f(50, 50, 0, 1));
    m_scene2d.m_children.push_back(m_ctrnnView);
  };
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw3d(){};
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void draw2d(){};
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void keyDown (ci::app::KeyEvent event)
  {
    View::keyDown(event);
    switch(event.getChar()) 
    {
      case 'r': 
        m_ctrnn->randomizeWeights(-10, 10);
        m_ctrnn->randomizeBiases(-10.0f, 10.0f);
        m_ctrnn->randomizeTimeConstants(2.0 / 30.0, 5.0f);         
        break;
      case 'c':
        m_ctrnn->setCenterCrossing();
        break;
    }
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void buildGui() { /*gui.addPage("Default gui page");*/ };
  
  
  CTRNN* m_ctrnn; 
  dmx::CTRNNViz* m_ctrnnView;
};

#endif
