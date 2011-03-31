/*
 *  CTRNNViz.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _OFX_CTRNN_VIEW_
#define _OFX_CTRNN_VIEW_

#include "Scene.h"
#include "CTRNN.h"
//#include "cinder/gl/Texture.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
class CTRNNViz : public NodeGroup
{
public:
  
  CTRNNViz(CTRNN* ctrnn, float width) : m_ctrnn(ctrnn), m_width(width) { init(); };
  
  void setCTRNN(CTRNN* ctrnn) { m_ctrnn = ctrnn; };
  virtual void update();
  virtual void onMouseMove(const Vec3f& mPos);
  virtual void onKeyPress(ci::app::KeyEvent e);
  
protected:
  virtual void init();
  
  int m_selected;
  float m_angle;
  ci::Vec2f m_mPos;
  
  uint8_t m_mode;
  float m_width;
  float m_unitSize;
  CTRNN* m_ctrnn;
  MatrixView<double>* m_weights;
  VectorView<double>* m_outputs;
  Plot* m_plot;
//  ci::gl::Texture m_texture;
};

} // namespace dmx

#endif