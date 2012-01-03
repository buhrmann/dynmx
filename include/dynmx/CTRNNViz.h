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

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Base class for basic, not necessarily GUI-type CTRNN viz
//----------------------------------------------------------------------------------------------------------------------
class CTRNNBasicViz : public Node
{
public:

  CTRNNBasicViz(CTRNN* ctrnn, float width, bool text = true) : 
    m_ctrnn(ctrnn), m_width(width), m_renderText(text) { init(); };  

  void setCTRNN(CTRNN* ctrnn) { m_ctrnn = ctrnn; };  
  int getSelectedNeuron() { return m_selected; };

protected:
  virtual void init(){ m_selected = -1; m_font = ci::Font(ci::app::loadResource("pf_tempesta_seven.ttf"), 8);  };
  
  CTRNN* m_ctrnn;  
  ci::Font m_font;  
  float m_width;
  int m_selected;
  bool m_renderText;
};

//----------------------------------------------------------------------------------------------------------------------
// Basic re-usable 2d visualization drawing individual neurons along a circle
//----------------------------------------------------------------------------------------------------------------------
class CTRNNNeuronViz : public CTRNNBasicViz
{
public:
  CTRNNNeuronViz(CTRNN* ctrnn, float width, bool text = true) : CTRNNBasicViz(ctrnn, width, text) {};  
    
  virtual void update();
  virtual void onMouseMove(const Vec4f& mPos);
};

//----------------------------------------------------------------------------------------------------------------------
// Basic re-usable 2d visualization drawing individual neurons along a circle
//----------------------------------------------------------------------------------------------------------------------
class CTRNNWheelViz : public CTRNNBasicViz
{
public:

  CTRNNWheelViz(CTRNN* ctrnn, float width, bool text = true) : CTRNNBasicViz(ctrnn, width, text) {};

  virtual void update();
  virtual void onMouseMove(const Vec4f& mPos);
};
  
//----------------------------------------------------------------------------------------------------------------------
// Composite view combining different visualizations and data output in gui-like element
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
  void renderAsNeurons();
  
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
  CTRNNNeuronViz* m_neuronViz;
  CTRNNWheelViz* m_wheelViz;
};

} // namespace dmx

#endif