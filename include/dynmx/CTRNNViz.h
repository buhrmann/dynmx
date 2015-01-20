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
#include "Topology.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Base class for basic, not necessarily GUI-type CTRNN viz
//----------------------------------------------------------------------------------------------------------------------
class CTRNNBasicViz : public Node
{
public:

  CTRNNBasicViz(const CTRNN* ctrnn, float width, bool text = true) : 
    m_ctrnn(ctrnn), m_width(width), m_renderText(text), m_renderConnections(true) { init(); };

  void renderConnections(bool r) { m_renderConnections = r; };
  void setCTRNN(const CTRNN* ctrnn) { m_ctrnn = ctrnn; };  
  int getSelectedNeuron() { return m_selected; };

protected:
  virtual void init(){
    m_selected = -1;
    //m_font = ci::Font(ci::app::loadResource("pf_tempesta_seven.ttf"), 8);
    m_font = ci::Font("PF Tempesta Seven", 8);
  };
  
  const CTRNN* m_ctrnn;  
  ci::Font m_font;
  float m_width;
  int m_selected;
  bool m_renderText;
  bool m_renderConnections;
};

//----------------------------------------------------------------------------------------------------------------------
// Basic re-usable 2d visualization drawing individual neurons along a circle
//----------------------------------------------------------------------------------------------------------------------
class CTRNNNeuronViz : public CTRNNBasicViz
{
public:
  CTRNNNeuronViz(const CTRNN* ctrnn, float width, bool text = true) : CTRNNBasicViz(ctrnn, width, text) {};  
    
  virtual void update();
  virtual void onMouseMove(const Vec4f& mPos);
};

//----------------------------------------------------------------------------------------------------------------------
// Basic re-usable 2d visualization drawing individual neurons along a circle
//----------------------------------------------------------------------------------------------------------------------
class CTRNNWheelViz : public CTRNNBasicViz
{
public:

  CTRNNWheelViz(const CTRNN* ctrnn, float width, bool text = true) : CTRNNBasicViz(ctrnn, width, text) {};

  virtual void update();
  virtual void onMouseMove(const Vec4f& mPos);
};
  
//----------------------------------------------------------------------------------------------------------------------
// Basic re-usable 2d visualization drawing individual neurons according to a given topology
//----------------------------------------------------------------------------------------------------------------------
class CTRNNLayerViz : public CTRNNBasicViz
{
public:
  
  CTRNNLayerViz(const CTRNN* ctrnn, const Topology* top, float width, bool text = true);
  
  virtual void update();
  virtual void onMouseMove(const Vec4f& mPos);
  
protected:
  const Topology* m_topology;
  std::vector<ci::Vec3f> m_pos;
  int m_maxLayerN;
  int m_r;
};
  
//----------------------------------------------------------------------------------------------------------------------
// Composite view combining different visualizations and data output in gui-like element
//----------------------------------------------------------------------------------------------------------------------
class CTRNNViz : public NodeGroup
{
public:
  
  enum RenderMode
  {
    kRM_Matrix,
    kRM_Wheel,
    kRM_Ring,
    kRM_Layers,
    kRM_NumModes
  };
  
  enum WeightRenderMode
  {
    kWM_Weights = 0,
    kWM_dW,
    kWM_LRate,
    kWM_DRate,
    kWM_SScale,
    kWM_Num
  };
  
  static std::string s_weightModeNames [kWM_Num];
  
  CTRNNViz(const CTRNN* ctrnn, float width, const Topology* top = 0) : m_ctrnn(ctrnn), m_width(width), m_topology(top) { init(); };
  
  void setCTRNN(const CTRNN* ctrnn) { m_ctrnn = ctrnn; };
  virtual void update();
  virtual void onMouseMove(const Vec4f& mPos);
  virtual void onMousePress(const Vec4f& mPos);
  
  float getHeight() { return m_height; };
  
  static void drawNeuron(int i, const CTRNN* ctrnn, float inner, float outer);
  static double getOutputNorm(int i, const CTRNN* ctrnn);  
  
protected:
  virtual void init();
  void renderAs(int m);
  void renderAsMatrix();
  
  int m_selected;
  float m_angle;
  ci::Vec2f m_mPos;
  
  uint8_t m_mode;
  uint8_t m_weightMode;
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
  ci::Rectf m_weightModeButton;
  
  const CTRNN* m_ctrnn;
  const Topology* m_topology;
  MatrixView<double>* m_weights;
  VectorView<double>* m_outputs;
  CTRNNNeuronViz* m_neuronViz;
  CTRNNWheelViz* m_wheelViz;
  CTRNNLayerViz* m_layerViz;
};

} // namespace dmx

#endif