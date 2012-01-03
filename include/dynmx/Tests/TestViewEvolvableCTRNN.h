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
#include "GAViz.h"
#include "CTRNNViz.h"
#include "TestEvolvableCTRNN.h"

// ---------------------------------------------------------------------------------------------------------------------
class TestViewEvolvableCTRNN : public dmx::View
{

public:

  //--------------------------------------------------------------------------------------------------------------------
  TestViewEvolvableCTRNN() : m_gaRunner(0) {};
  TestViewEvolvableCTRNN(dmx::Model* gaRunner) : m_gaRunner((dmx::GARunner*)gaRunner)
  {
    m_evoCtrnn = (TestEvolvableCTRNN*) m_gaRunner->getEvolvable();
  };
  
  // Functions to be implemented by subclasses
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setupScene()
  {
    assert(m_evoCtrnn);
    
    m_ctrnnViz = new dmx::CTRNNViz(m_evoCtrnn->m_ctrnn, 175);
    m_ctrnnViz->translate(ci::Vec4f(50, 50, 0, 1));
    m_scene2d.m_children.push_back(m_ctrnnViz);
    
#if 0  
    m_gaViz = new dmx::GAViz(m_gaRunner, 500.0);
    m_gaViz->translate(ci::Vec4f(50, 50, 0, 1));    
    m_scene2d.m_children.push_back(m_gaViz);	    
#else
    m_matrixViz = new dmx::RealMatrixViz(
    m_gaRunner->getGA()->getPopulation(), 
    m_gaRunner->getGA()->getPopulationSize(), 
    m_gaRunner->getGA()->getGenomeSize(),
      250.0, 1.0);
    m_matrixViz->translate(ci::Vec4f(50, 250, 0, 1));
    m_scene2d.m_children.push_back(m_matrixViz); 
#endif    
  };
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw3d(){};
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void draw2d()
  {   
    glColor3f(0,0,0);
    //dmx::drawSquaresVB(750, 300, 300);

  };
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void keyDown (ci::app::KeyEvent event)
  {
    View::keyDown(event);
    switch(event.getChar()) 
    {
    default:
      break;
    }
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void buildGui() { /*gui.addPage("Default gui page");*/ };
  
  
  TestEvolvableCTRNN* m_evoCtrnn; 
  dmx::GARunner* m_gaRunner;
  dmx::CTRNNViz* m_ctrnnViz;
  dmx::GAViz* m_gaViz;
  dmx::RealMatrixViz* m_matrixViz;
};

#endif
