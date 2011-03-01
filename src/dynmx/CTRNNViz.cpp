/*
 *  CTRNNViz.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#include "CTRNNViz.h" 
#include "MathUtils.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Text.h"
#include "cinder/Utilities.h"
#include "cinder/ImageIo.h"
#include "cinder/Font.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::init()
{
  NodeGroup::init();
  
  m_mode = 0;
  m_selected = -1;
  m_angle = 0;
  
  m_unitSize = m_width / m_ctrnn->size;
  
  // output vector
  m_outputs = new VectorView<double>(m_ctrnn->outputs, m_ctrnn->size, m_width, 1.0); 
  m_outputs->translate(Vec3f(0, 0.5 * m_unitSize, 0));
  
  // weight matrix
  m_weights = new MatrixView<double>(
    m_ctrnn->weights, m_ctrnn->size, m_ctrnn->size, m_width, 10.0);
  m_weights->translate(Vec3f(0, 3 * m_unitSize, 0));
    
  // output plot
  m_plot = new Plot(300.0, 180, m_ctrnn->size, 100);
  m_plot->translate(Vec3f(m_width * 1.5, 0, 0));  
  
//	ci::TextLayout layout;
//	layout.clear( ci::ColorA( 0.2f, 0.2f, 0.2f, 0.2f ) );
//	layout.setFont( ci::Font( "Arial", 24 ) );  
//  layout.addLine( std::string( "CTRNN: " ));  
//	ci::Surface8u rendered = layout.render( true, false);
//	m_texture = ci::gl::Texture( rendered );
    
  // add to this
  m_children.push_back(m_outputs);
  m_children.push_back(m_weights);
  m_children.push_back(m_plot);
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::update() const
{ 
  ci::Font font("Arial", 11);

  // update data in graph
  for(int i = 0; i < m_ctrnn->size; i++)
  {
    m_plot->addPoint(m_ctrnn->getOutput(i), i);
  } 
  
//	ci::gl::enableAlphaBlending( false );
//	ci::gl::color( ci::Color::white() );
//	ci::gl::draw( m_texture, ci::Vec2f( 10, 10 ) );
  
  // draw as wheel
  if(m_mode == 1)
  {
    const float segmentSizeDeg = 360.0f / m_ctrnn->size;
    const float segmentSizeRad = degreesToRadians(segmentSizeDeg);
    const float r3 = m_width / 2;
    const float r2 = r3 - 5;
    const float r1 = r2 - 10;
    
    glPushMatrix();
    // plot data
    ci::gl::translate(m_pTM->getTranslation());    
    m_children[2]->update();    
    
    // outputs section
    ci::Vec2f pos (0, -10); 
    ci::gl::drawString("CTRNN", pos, ci::ColorA(0,0,0), font);
    pos += ci::Vec2f(0, 10.5);
    glColor3f(0,0,0);
    ci::gl::drawLine(pos, pos + ci::Vec2f(m_width, 0));
    
    // left upper corner of wheel's bounding box at 0,0:    
    ci::Vec3f radiusOffset(r3, r3 + 10.5, 0);
    ci::gl::translate(radiusOffset);
        
    // connectivity
    for(int i = 0; i < m_ctrnn->size; i++)
    {
      // draw only the selected neuron's connections
      if(i == m_selected)
      {
        float angle = -((float)i + 0.5f) * segmentSizeRad + PI_OVER_TWO; 
        Vec3f midPoint ((r1+5) * cosf(angle), (r1+5) * sinf(angle), 0.0f);
        for(int j = 0; j < m_ctrnn->size; j++)
        {
          float otherAngle = -((float)j + 0.5f) * segmentSizeRad + PI_OVER_TWO; 
          Vec3f otherMidPoint ((r1+5) * cosf(otherAngle), (r1+5) * sinf(otherAngle), 0.0f);
          const float weight = m_ctrnn->weights[j][i];
          glLineWidth(fabs(weight));
          if(weight < 0)
            glColor3f(235.0/255.0, 0.0/255.0, 103.0/255.0); 
          else
            glColor3f(0,0,0);
          ci::gl::drawLine(midPoint, otherMidPoint);
        }   
      }   
    }
    glLineWidth(1.0f);
    
    // outputs
    for(int i = 0; i < m_ctrnn->size; i++)
    {
      // white background
      glColor3f(1,1,1);
      dmx::drawPartialDisk(r1, r2, 8, 1, i * segmentSizeDeg, segmentSizeDeg, GLU_FILL);    
      // actual amount
      glColor3f(181.0/255.0, 206.0/255.0, 26.0/255.0);
      float outputWidth = m_ctrnn->outputs[i] * segmentSizeDeg;
      dmx::drawPartialDisk(r1, r2, 8, 1, i * segmentSizeDeg, outputWidth, GLU_FILL); 
      // outline segment
      glColor3f(0,0,0);
      dmx::drawPartialDisk(r1, r2, 8, 1, i * segmentSizeDeg, segmentSizeDeg, GLU_SILHOUETTE);   
    }
    
    // indicators
    for(int i = 0; i < m_ctrnn->size; i++)
    {
      glColor3f(0,0,0);
      if(i == m_selected)
      {
        dmx::drawPartialDisk(r2, r3, 8, 1, i * segmentSizeDeg, segmentSizeDeg, GLU_FILL);  
      }
      else 
      {
        dmx::drawPartialDisk(r2, r3, 8, 1, i * segmentSizeDeg, segmentSizeDeg, GLU_SILHOUETTE); 
      }
    }  
    
    // text output
    pos = ci::Vec2f(-r3, r3 + m_unitSize + 0.5);
    ci::gl::drawLine(pos, pos + ci::Vec2f(m_width, 0.0));    
    if(m_selected >= 0)
    {
      pos += ci::Vec2f(0, 10);
      char str [128];
      sprintf(str, "Node: %i", m_selected);
      ci::gl::drawString(str, pos, ci::Color(0,0,0), font);
      sprintf(str, "bias: %2.2f | tau: %2.2f | out: %1.3f", m_ctrnn->biases[m_selected], m_ctrnn->taus[m_selected], 
        m_ctrnn->outputs[m_selected]);
      pos += ci::Vec2f(0, 15);
      ci::gl::drawString(str, pos, ci::Color(0,0,0), font);
    }
    
    glPopMatrix();
    return;
  } // draw as wheel
  
  // draw as matrix
  if(m_mode == 0)
  {    
    ci::Color textColor (0,0,0);
    // draw stuff not already in child nodes
    ci::Vec2f pos = ci::Vec2f(m_pTM->getTranslation());
    glColor3f(0,0,0);
    
    // outputs section
    pos -= ci::Vec2f(0, 10); 
    ci::gl::drawString("CTRNN", ci::Vec2f(pos.x, pos.y), textColor, font);
    pos += ci::Vec2f(0, 10.5);
    ci::gl::drawLine(pos, pos + ci::Vec2f(m_width, 0));

    // weights matrix section
    pos += ci::Vec2f(0, m_unitSize + 10);
    pos += ci::Vec2f(0, 10.5);
    ci::gl::drawLine(pos, pos + ci::Vec2f(m_width, 0));
    
    // bottom section
    pos += ci::Vec2f(0, m_width + m_unitSize + 0.5);
    ci::gl::drawLine(pos, pos + ci::Vec2f(m_width, 0));
    const int& i = m_outputs->m_iSel;
    if(i != -1)
    {
      pos += ci::Vec2f(0, 15);
      char str [128];
      sprintf(str, "Node: %i", i);
      ci::gl::drawString(str, pos, ci::Color(0,0,0), font);
      sprintf(str, "bias: %2.2f | tau: %2.2f | out: %1.3f", m_ctrnn->biases[i], m_ctrnn->taus[i], 
        m_ctrnn->outputs[i]);
      pos += ci::Vec2f(0, 15);
      ci::gl::drawString(str, pos, ci::Color(0,0,0), font);      
    }
    
    glColor3f(0,0,0);
    const int& iW = m_weights->m_iSel;
    const int& jW = m_weights->m_jSel;
    char str [128];  
    if(iW != -1 && jW != -1)
    {
      pos += ci::Vec2f(0, 15);
      sprintf(str, "weight %i > %i: %2.2f", iW, jW, m_ctrnn->weights[iW][jW]);
      ci::gl::drawString(str, ci::Vec2f(pos.x, pos.y), textColor, font);
    }  
    
//    pos += Vec3f(0, 15, 0);
//    sprintf(str, "x:%f, y: %f, angle: %f, sel: %i", m_mPos.x, m_mPos.y, m_angle, m_selected);    
//    ci::gl::drawString(str, ci::Vec2f(pos.x, pos.y), textColor);
//    
//    glColor3f(1,0,0);
//    dmx::drawPoint(Vec3f(m_mPos), 10);
    
    // render children
    NodeGroup::update();
  } // draw as matrix
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::onMouseMove(const Vec3f& mPos)
{
  const float r3 = m_width / 2;
  const float r2 = r3 - 5;
  const float r1 = r2 - 10;
  
  // left upper corner of bounding box at 0,0:    
  ci::Vec3f radiusOffset(r3, r3 + 10.5, 0);
  
  // update mouse info: transform mouse position into local space
  Vec3f posLocal = mPos - m_pTM->getTranslation();
  posLocal -= radiusOffset;
  m_mPos = ci::Vec2f(posLocal.x, posLocal.y);
  float radius = posLocal.length();
  if(radius > r1 && radius < r3)
  {
    float angle = radiansToDegrees(atan2(posLocal.x, posLocal.y));
    if(angle < 0) angle += 360.0f;
    m_angle = angle;
    m_selected = (int)angle / (360 / (int) m_ctrnn->size);
  }
  
  NodeGroup::onMouseMove(mPos);
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::onKeyPress(ci::app::KeyEvent e)
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


