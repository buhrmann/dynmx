/*
 *  TestViewEvolvableCTRNN
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 01/03/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _OFX_TEST_VIEW_EVOLVABLE_CTRNN_
#define _OFX_TEST_VIEW_EVOLVABLE_CTRNN_

#include "View.h"
#include "CTRNNViz.h"
#include "TestEvolvableCTRNN.h"

// ---------------------------------------------------------------------------------------------------------------------
class TestViewEvolvableCTRNN : public dmx::View
{

public:

  //--------------------------------------------------------------------------------------------------------------------
  TestViewEvolvableCTRNN() : m_evoCtrnn(0) {};
  TestViewEvolvableCTRNN(TestEvolvableCTRNN* evo) : m_evoCtrnn(evo) {};
  
  // functions to be implemented by subclasses
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setupScene()
  {
    assert(m_evoCtrnn);
    
    m_ctrnnView = new dmx::CTRNNViz(m_evoCtrnn->m_ctrnn, 175);
    m_ctrnnView->translate(ci::Vec3f(50, 50, 0));
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
        m_evoCtrnn->m_ctrnn->randomizeWeights(-10, 10);
        m_evoCtrnn->m_ctrnn->randomizeBiases(-10.0f, 10.0f);
        m_evoCtrnn->m_ctrnn->randomizeTimeConstants(2.0 / 30.0, 5.0f);         
        break;
      case 'c':
        m_evoCtrnn->m_ctrnn->setCenterCrossing();
        break;
    }
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void buildGui() { /*gui.addPage("Default gui page");*/ };
  
  
  TestEvolvableCTRNN* m_evoCtrnn; 
  dmx::CTRNNViz* m_ctrnnView;
};

#endif
