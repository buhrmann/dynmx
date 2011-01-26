/*
 *  CTRNNView.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "CTRNNView.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void CTRNNView::init()
{
  NodeGroup::init();

  m_unitSize = m_width / m_ctrnn->size;
  
  // output vector
  m_outputs = new VectorView<double>(
    m_ctrnn->outputs, m_ctrnn->size, m_width, 1.0); 
  m_outputs->translate(Vec3f(0, -m_unitSize, 0));
  
  // weight matrix
  m_weights = new MatrixView<double>(
    m_ctrnn->weights, m_ctrnn->size, m_ctrnn->size, m_width, 10.0);
  m_weights->translate(Vec3f(0, -4 * m_unitSize, 0));
    
  // output plot
  m_plot = new Plot(300.0, 180, m_ctrnn->size, 100);
  m_plot->translate(Vec3f(m_width * 1.5, -180, 0));  
  
  // add to this
  m_children.push_back(m_outputs);
  m_children.push_back(m_weights);
  m_children.push_back(m_plot);
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNView::update() const
{ 
  // update data in graph
  for(int i = 0; i < m_ctrnn->size; i++)
  {
    m_plot->addPoint(m_ctrnn->getOutput(i), i);
  }  
  
  // draw stuff not already in child nodes
  Vec3f pos = m_pTM->getTranslation();
  glColor3f(0,0,0);
  
  // outputs section
  drawString(pos, "CTRNN");
  pos += Vec3f(0, -10, 0);
  //drawLine(pos, pos + Vec3f(m_width, 0, 0));

  // weights matrix section
  pos += Vec3f(0, -3 * m_unitSize + 10, 0);
  drawString(pos, "weights");
  pos += Vec3f(0, -10, 0);
  //drawLine(pos, pos + Vec3f(m_width, 0, 0));
  
  // bottom section
  pos += Vec3f(0, - m_width - m_unitSize, 0);
  //drawLine(pos, pos + Vec3f(m_width, 0, 0));
  const int& i = m_outputs->m_iSel;
  if(i != -1)
  {
    pos += Vec3f(0, -15, 0);
    char str [128];
    sprintf(str, "N%i b: %2.2f t: %2.2f", i, m_ctrnn->biases[i], m_ctrnn->taus[i]);
    //drawString(str, pos, 0.8);
  }
  
  const int& iW = m_weights->m_iSel;
  const int& jW = m_weights->m_jSel;
  if(iW != -1 && jW != -1)
  {
    pos += Vec3f(0, -15, 0);
    char str [128];
    sprintf(str, "w%i,%i: %2.2f", iW, jW, m_ctrnn->weights[iW][jW]);
    //drawString(str, pos, 0.8);
  }  
  
  // render children
  NodeGroup::update();
}

} // namespace dmx


