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
  virtual void onMouseMove(const Vec4f& mPos);
  virtual void onMousePress(const Vec4f& mPos);
  
  float getHeight() { return m_height; };
  
protected:
  virtual void init();
  void renderAsMatrix();
  void renderAsWheel();
  
  int m_selected;
  float m_angle;
  ci::Vec2f m_mPos;
  
  uint8_t m_mode;
  float m_width;
  float m_height;
  float m_labelHeight;
  float m_lineHeight;
  float m_padding;
  float m_unitSize;
  float m_label2y;
  float m_label3y;
  
  ci::Font m_font;
  ci::Color m_textColor;
  ci::ColorA m_labelColor;
  
  ci::Rectf m_modeButton;
  
  CTRNN* m_ctrnn;
  MatrixView<double>* m_weights;
  VectorView<double>* m_outputs;
};

} // namespace dmx

#endif