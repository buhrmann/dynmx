/*
 *  CTRNNView.h
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
//----------------------------------------------------------------------------------------------------------------------
class CTRNNView : public NodeGroup
{
public:
  
  CTRNNView(CTRNN* ctrnn, float width) : m_ctrnn(ctrnn), m_width(width) { init(); };
  
  void setCTRNN(CTRNN* ctrnn) { m_ctrnn = ctrnn; };
  virtual void update() const;
  
protected:
  virtual void init();
  
  float m_width;
  float m_unitSize;
  CTRNN* m_ctrnn;
  MatrixView<double>* m_weights;
  VectorView<double>* m_outputs;
  Plot* m_plot;
};

} // namespace dmx

#endif