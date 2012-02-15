/*
 *  GAViz.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 01/03/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "GAViz.h"

#include "cinder/gl/gl.h"
#include "cinder/Text.h"
#include "cinder/Utilities.h"
#include "cinder/ImageIo.h"
#include "cinder/Font.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void GAViz::init()
{
  NodeGroup::init();
  
  m_mode = 0;
  m_selected = -1;
  m_unitSize = m_width / m_gaRunner->getGA()->getPopulationSize();
  
  // fitnesses
  m_fitness = new VectorView<float>(
    m_gaRunner->getGA()->getFitnesses(), 
    m_gaRunner->getGA()->getPopulationSize(), 
    m_width, 1.0f); 
  m_fitness->translate(Vec4f(0.0f, 0.5f * m_unitSize, 0.0f, 1.0f));
  
  // genomes
  m_genomes = new MatrixView<double>(
    m_gaRunner->getGA()->getPopulation(), 
    m_gaRunner->getGA()->getPopulationSize(), 
    m_gaRunner->getGA()->getGenomeSize(),
     m_width, 10.0);
  m_genomes->translate(Vec4f(0, 3 * m_unitSize, 0, 1));
    
  // output plot
  m_plot = new Plot(300.0, 180, 2, 100);
  m_plot->translate(Vec4f(m_width * 1.5, 0, 0, 1));  
    
  // add to this
  m_children.push_back(m_fitness);
  m_children.push_back(m_genomes);
  m_children.push_back(m_plot);
}

//----------------------------------------------------------------------------------------------------------------------
void GAViz::update()
{ 
  ci::Font font("Arial", 11);

  // update data in graph
//  for(int i = 0; i < m_ctrnn->size; i++)
//  {
//    m_plot->addPoint(m_ctrnn->getOutput(i), i);
//  } 
  
  
  
  // draw as matrix
  if(m_mode == 0)
  {    
    ci::Color textColor (0,0,0);
    // draw stuff not already in child nodes
    ci::Vec2f pos = ci::Vec2f(m_pTM->getTranslate());
    glColor3f(0,0,0);
    
    // fitnesses section
    pos -= ci::Vec2f(0, 10); 
    ci::gl::drawString("Genetic Algorithm", ci::Vec2f(pos.x, pos.y), textColor, font);
    pos += ci::Vec2f(0, 10.5);
    ci::gl::drawLine(pos, pos + ci::Vec2f(m_width, 0));

    // genomes section
    pos += ci::Vec2f(0, m_unitSize + 10);
    pos += ci::Vec2f(0, 10.5);
    ci::gl::drawLine(pos, pos + ci::Vec2f(m_width, 0));
    
    // bottom section
    pos += ci::Vec2f(0, m_width + m_unitSize + 0.5);
    ci::gl::drawLine(pos, pos + ci::Vec2f(m_width, 0));
    const int& i = m_fitness->m_iSel;
    if(i != -1)
    {
      pos += ci::Vec2f(0, 15);
      char str [128];
      sprintf(str, "Genome %i: fitness = %1.4f", i, m_gaRunner->getGA()->getFitnesses()[i]);
      ci::gl::drawString(str, pos, ci::Color(0,0,0), font); 
    }
    
    glColor3f(0,0,0);
    const int& iW = m_genomes->m_iSel;
    const int& jW = m_genomes->m_jSel;
    char str [128];  
    if(iW != -1 && jW != -1)
    {
      pos += ci::Vec2f(0, 15);
      sprintf(str, "weight %i > %i: %2.2f", iW, jW, m_gaRunner->getGA()->getPopulation()[iW][jW]);
      ci::gl::drawString(str, ci::Vec2f(pos.x, pos.y), textColor, font);
    }  
    
    // render children
    NodeGroup::update();
  } // draw as matrix
}

//----------------------------------------------------------------------------------------------------------------------
  void GAViz::onMouseMove(const ci::Vec4f& mPos)
{
//  const float r3 = m_width / 2;
//  const float r2 = r3 - 5;
//  const float r1 = r2 - 10;
//  
//  // left upper corner of bounding box at 0,0:    
//  ci::Vec3f radiusOffset(r3, r3 + 10.5, 0);
//  
//  // update mouse info: transform mouse position into local space
//  Vec3f posLocal = mPos - m_pTM->getTranslation();
//  posLocal -= radiusOffset;
//  m_mPos = ci::Vec2f(posLocal.x, posLocal.y);
//  float radius = posLocal.length();
//  if(radius > r1 && radius < r3)
//  {
//    float angle = radiansToDegrees(atan2(posLocal.x, posLocal.y));
//    if(angle < 0) angle += 360.0f;
//    m_angle = angle;
//    m_selected = (int)angle / (360 / (int) m_ctrnn->size);
//  }
  
  NodeGroup::onMouseMove(mPos);
}

//----------------------------------------------------------------------------------------------------------------------
void GAViz::onKeyPress(ci::app::KeyEvent e)
{
  const uint8_t numModes = 2;
  switch(e.getChar())
  {
  case 'n':
    m_mode = (m_mode + 1) % numModes;
    break;
  }
  
  NodeGroup::onKeyPress(e);
}

} // namespace dmx