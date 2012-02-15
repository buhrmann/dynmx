/*
 *  GAViz.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 01/03/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _GA_VIZ_
#define _GA_VIZ_

#include "Scene.h"
#include "GARunner.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
class GAViz : public NodeGroup
{
public:
  
  GAViz(GARunner* gaRunner, float width) : m_gaRunner(gaRunner), m_width(width) { init(); };
  
  void setGARunner(GARunner* gaRunner) { m_gaRunner = gaRunner; };
  virtual void update();
  virtual void onMouseMove(const ci::Vec4f& mPos);
  virtual void onKeyPress(ci::app::KeyEvent e);
  
protected:
  virtual void init();
  
  int m_selected;               // selected individual/genome
  ci::Vec2f m_mPos;             // local mouse position
  uint8_t m_mode;               // draw mode
  float m_width;                // width of this widget
  float m_unitSize;             // width of an individual
  GARunner* m_gaRunner;         // pointer to the model
  MatrixView<double>* m_genomes;// individuals 
  VectorView<float>* m_fitness;// 
  Plot* m_plot;
};

} // namespace dmx

#endif