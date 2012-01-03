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

#define CTRNNVIZ_NUM_MODES 3

//----------------------------------------------------------------------------------------------------------------------
// CTRNNNeuronViz implementation
//----------------------------------------------------------------------------------------------------------------------
void CTRNNNeuronViz::update()
{
  // left upper corner of circle's bounding box at 0,0:
  const float segmentSize = TWO_PI / m_ctrnn->size;
  const float networkRadius = m_width / 2.5;  
  const float neuronRadius = networkRadius / m_ctrnn->size;

  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // connectivity
  for(int i = 0; i < m_ctrnn->size; i++)
  {
    float angle = -((float)i + 0.5f) * segmentSize + PI_OVER_TWO;   
    Vec3f midPoint (networkRadius * cosf(angle), networkRadius * sinf(angle), 0.0f);

    // draw only the selected neuron's connections
    if(i == m_selected || m_selected == -1)
    {
      // draw connections    
      for(int j = 0; j < m_ctrnn->size; j++)
      {
        float otherAngle = -((float)j + 0.5f) * segmentSize + PI_OVER_TWO; 
        Vec3f otherMidPoint (networkRadius * cosf(otherAngle), networkRadius * sinf(otherAngle), 0.0f);
        const float weight = m_ctrnn->weights[j][i];
        glLineWidth(fabs(weight));
        if(weight < 0)
          glColor3f(235.0/255.0, 0.0/255.0, 103.0/255.0); 
        else
          glColor3f(0,0,0);
        ci::gl::drawLine(midPoint, otherMidPoint);
      }   
      
      // render text 
      if(m_renderText && i == m_selected)
      {
        // box
        glColor4f(0,0,0, 0.5f);
        const float lineHeight = 13;
        const float padding = 3;
        const float textBoxH = 3 * lineHeight + 2 * padding;
        ci::Vec2f textBoxPos = ci::Vec2f(midPoint);
        ci::gl::drawSolidRect(ci::Rectf(textBoxPos.x, textBoxPos.y, textBoxPos.x + m_width, textBoxPos.y + textBoxH));
        
        // text
        ci::Color textColor(1,1,1);
        ci::Vec2f textPos = textBoxPos + ci::Vec2f(padding, padding);
        char str [128];
        sprintf(str, "Node: %i", i);
        ci::gl::drawString(str, textPos, textColor, m_font);
        
        sprintf(str, "b: %2.2f | t: %2.2f | g: %2.2f", m_ctrnn->biases[i], m_ctrnn->taus[i], m_ctrnn->gains[i]);
        ci::gl::drawString(str, textPos + ci::Vec2f(0, lineHeight), textColor, m_font);      
        
        sprintf(str, "in: %1.3f | out: %1.3f", m_ctrnn->externalinputs[i], m_ctrnn->outputs[i]);
        ci::gl::drawString(str, textPos + ci::Vec2f(0, 2 * lineHeight), textColor, m_font);            
      }
    }
  }
  
  glLineWidth(1.0f);
  float outerRadius = neuronRadius + 6;
  for(int i = 0; i < m_ctrnn->size; i++)
  {
    ci::Color col = m_ctrnn->states[i] > 0 ? ci::Color(0,0,0) : ci::Color(235.0/255.0, 0.0/255.0, 103.0/255.0);

    float angle = -((float)i + 0.5f) * segmentSize + PI_OVER_TWO;   
    Vec3f midPoint (networkRadius * cosf(angle), networkRadius * sinf(angle), 0.0f);
    glPushMatrix();
    glTranslatef(midPoint);
    // white background    
    glColor3f(1,1,1);
    drawDisk(outerRadius, 0.0, 32, 1);    
    ci::gl::color(col);
    // disk size indicating output value
    drawDisk(neuronRadius * m_ctrnn->outputs[i], 0.0, 32, 1);
    // ring size indicating external input value
    glColor3f(181.0/255.0, 206.0/255.0, 26.0/255.0);    
    float width = radiansToDegrees(m_ctrnn->externalinputs[i] * TWO_PI);
    dmx::drawPartialDisk(neuronRadius, outerRadius, 8, 1, 0, width, GLU_FILL);     
    glColor3f(0,0,0);
    ci::gl::drawStrokedCircle(ci::Vec2f(0,0), outerRadius, 32);
    ci::gl::drawStrokedCircle(ci::Vec2f(0,0), neuronRadius, 32);    
    glPopMatrix();
  }  
  
  glPopMatrix();
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNNeuronViz::onMouseMove(const Vec4f& mPos)
{  
  m_selected = -1;

  // Update mouse info: transform mouse position into local space angle and radius
  Vec4f posLocal = mPos - m_pTM->getTranslate();
  float angle = radiansToDegrees(atan2(posLocal.x, posLocal.y));
  if(angle < 0) 
  {
    angle += 360.0f;
  }

  const float networkRadius = m_width / 2.5;  
  const float neuronRadius = networkRadius / m_ctrnn->size + 6;   
  int neuron = (int)angle / (360 / (int) m_ctrnn->size);    
  float neuronAngle = -((float)neuron + 0.5f) * TWO_PI / m_ctrnn->size + PI_OVER_TWO;   
  Vec4f neuronPos (networkRadius * cosf(neuronAngle), networkRadius * sinf(neuronAngle), 0.0f, 1.0);
  float dist = neuronPos.distance(posLocal);
     
  if(dist < neuronRadius)
  {
    m_selected = neuron;
  }
}

//----------------------------------------------------------------------------------------------------------------------
// CTRNNNeuronViz implementation
//----------------------------------------------------------------------------------------------------------------------
void CTRNNWheelViz::update()
{
  const float segmentSizeDeg = 360.0f / m_ctrnn->size;
  const float segmentSizeRad = degreesToRadians(segmentSizeDeg);
  const float r3 = m_width / 2;
  const float r2 = r3 - 5;
  const float r1 = r2 - 10;
  
  // left upper corner of wheel's bounding box at 0,0:    
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // connectivity
  for(int i = 0; i < m_ctrnn->size; i++)
  {
    // draw only the selected neuron's connections
    if(i == m_selected || m_selected == -1)
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
    dmx::drawPartialDisk(r2, r3, 8, 1, i * segmentSizeDeg, m_ctrnn->externalinputs[i] * segmentSizeDeg, GLU_FILL);  
    
    if(i == m_selected)
      glColor3f(235.0/255.0, 0.0/255.0, 103.0/255.0);
    else
      glColor3f(0,0,0);
    
    dmx::drawPartialDisk(r2, r3, 8, 1, i * segmentSizeDeg, segmentSizeDeg, GLU_SILHOUETTE); 
  }  
  glPopMatrix();
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNWheelViz::onMouseMove(const Vec4f& mPos)
{  
  m_selected = -1;
  const float r3 = m_width / 2;
  const float r2 = r3 - 5;
  const float r1 = r2 - 10;
    
  // update mouse info: transform mouse position into local space angle and radius
  Vec4f posLocal = mPos - m_pTM->getTranslate();
  float radius = posLocal.length();
  float angle = radiansToDegrees(atan2(posLocal.x, posLocal.y));
  if(angle < 0) angle += 360.0f;
  if(radius > r1 && radius < r3)
  {
    m_selected = (int)angle / (360 / (int) m_ctrnn->size);
  }
}


//----------------------------------------------------------------------------------------------------------------------
// CTRNNViz implementation
//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::init()
{
  NodeGroup::init();
  
  m_font = ci::Font(ci::app::loadResource("pf_tempesta_seven.ttf"), 8);
  
  m_mode = 0;
  m_selected = -1;
  m_angle = 0;
  m_labelHeight = 16;
  m_padding = 3;
  m_lineHeight = m_labelHeight - m_padding;  
  m_unitSize = m_width / m_ctrnn->size;

  m_textColor = ci::Color(1,1,1);
  m_labelColor = ci::ColorA(0.0f, 0.0f, 0.0f, 0.5f);
  
  float buttonSize = m_labelHeight - 2 * m_padding;  
  ci::Vec2f butPos(m_width - buttonSize - m_padding, m_padding);
  m_modeButton = ci::Rectf(butPos.x, butPos.y, butPos.x + buttonSize, butPos.y + buttonSize);
  
  m_label2y = ((int)(m_labelHeight + 2 * m_padding + m_unitSize)) ;
  m_label3y = m_label2y + m_labelHeight + m_padding + m_width + m_padding;
  m_height =  m_label3y + m_labelHeight + m_padding + 3 * m_lineHeight + 2 * m_padding;
  
  // output vector
  m_outputs = new VectorView<double>(m_ctrnn->outputs, m_ctrnn->size, m_width, 1.0); 
  m_outputs->translate(Vec4f(0, m_labelHeight + m_padding, 0, 1));
  m_children.push_back(m_outputs);
  
  // weight matrix
  m_weights = new MatrixView<double>(m_ctrnn->weights, m_ctrnn->size, m_ctrnn->size, m_width, 10.0);
  m_weights->translate(Vec4f(0, m_label2y + m_labelHeight + m_padding , 0, 1));
  m_children.push_back(m_weights);  
  
  // neuronViz
  m_neuronViz = new CTRNNNeuronViz(m_ctrnn, m_width, false);
  m_neuronViz->translate(ci::Vec4f(m_width / 2, m_label2y - m_labelHeight + (m_label3y - m_label2y) / 2, 0, 1));
  m_children.push_back(m_neuronViz);
  
  // wheelViz
  m_wheelViz = new CTRNNWheelViz(m_ctrnn, m_width, false);
  m_wheelViz->translate(ci::Vec4f(m_width / 2, m_label3y / 2, 0, 1));
  m_children.push_back(m_wheelViz);  
}

//----------------------------------------------------------------------------------------------------------------------  
void CTRNNViz::renderAsNeurons()
{
  m_children[2]->update();
}

//----------------------------------------------------------------------------------------------------------------------  
void CTRNNViz::renderAsWheel()
{
  m_children[3]->update();  
}

//----------------------------------------------------------------------------------------------------------------------  
void CTRNNViz::renderAsMatrix()
{
  ci::Vec2f textPadding (m_padding, m_padding);
  
  // Weights matrix label
  ci::Vec2f wLabelPos = ci::Vec2f(0, m_label2y);
  ci::gl::drawSolidRect(ci::Rectf(wLabelPos.x, wLabelPos.y, wLabelPos.x + m_width, wLabelPos.y + m_labelHeight));         
  ci::gl::drawString("CTRNN weights ", wLabelPos + textPadding, m_textColor, m_font);    
  
  // Render children (vector and matrix viz)
  for(int i = 0; i < 2; i++)
  {
    m_children[i]->update();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::update()
{ 
  ci::Vec2f textPadding (m_padding, m_padding);
  
  // Move to correct position
  glPushMatrix();
  dmx::translate(m_pTM->getTranslate());  
  
  // outputs label    
  ci::gl::color(m_labelColor);
  ci::gl::drawSolidRect(ci::Rectf(0, 0, m_width, m_labelHeight));         
  ci::gl::drawString("CTRNN outputs", textPadding, m_textColor, m_font);  
  
  // Mode button
  ci::gl::color(0.7,0.7,0.7);
  ci::gl::drawSolidRect(m_modeButton);
  
  // Draw according to selected mode
  ci::gl::color(m_labelColor);  
  if(m_mode == 1)
  {
    renderAsWheel();
  } 
  else if (m_mode == 2)
  {     
    renderAsMatrix();
  }
  else
  {
    renderAsNeurons();
  }
  
  // data label
  ci::gl::color(m_labelColor);  
  ci::Vec2f dLabelPos = ci::Vec2f(0, m_label3y);
  ci::gl::drawSolidRect(ci::Rectf(dLabelPos.x, dLabelPos.y, dLabelPos.x + m_width, dLabelPos.y + m_labelHeight));         
  ci::gl::drawString("CTRNN data ", dLabelPos + textPadding, m_textColor, m_font);    
  
  // text box
  ci::Vec2f textBoxPos = dLabelPos + ci::Vec2f(0, m_labelHeight + m_padding);
  const float textBoxH = 3 * m_lineHeight + 2 * m_padding;
  ci::gl::drawSolidRect(ci::Rectf(textBoxPos.x, textBoxPos.y, textBoxPos.x + m_width, textBoxPos.y + textBoxH));
  
  // text
  ci::Vec2f textPos = textBoxPos + textPadding;
  const int i = (m_mode == 2) ? m_outputs->m_iSel : (m_mode == 1) ? m_wheelViz->getSelectedNeuron() : m_neuronViz->getSelectedNeuron();
  char str [128];
  if(i != -1)
  {
    sprintf(str, "Node: %i", i);
    ci::gl::drawString(str, textPos, m_textColor, m_font);
    
    sprintf(str, "b: %2.2f | t: %2.2f | g: %2.2f", m_ctrnn->biases[i], m_ctrnn->taus[i], m_ctrnn->gains[i]);
    ci::gl::drawString(str, textPos + ci::Vec2f(0, m_lineHeight), m_textColor, m_font);      
    
    sprintf(str, "in: %1.3f | out: %1.3f", m_ctrnn->externalinputs[i], m_ctrnn->outputs[i]);
    ci::gl::drawString(str, textPos + ci::Vec2f(0, 2 * m_lineHeight), m_textColor, m_font);            
  }
  
  if(m_mode == 2)
  {
    const int& iW = m_weights->m_iSel;
    const int& jW = m_weights->m_jSel; 
    if(iW != -1 && jW != -1)
    {
      char str [128];       
      sprintf(str, "weight %i > %i: %2.2f", iW, jW, m_ctrnn->weights[iW][jW]);
      ci::gl::drawString(str, textPos, m_textColor, m_font);
    }
  }
  
  glPopMatrix();
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::onMouseMove(const Vec4f& mPos)
{
  NodeGroup::onMouseMove(mPos);
}
  
  
//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::onMousePress(const Vec4f& mPos)
{
  ci::Vec4f localPos = toLocalPos(mPos);
  
  // Button triggers view mode
  if(m_modeButton.contains(ci::Vec2f(localPos)) ) 
  {
    m_mode = (m_mode + 1) % CTRNNVIZ_NUM_MODES;
  }
  
  NodeGroup::onMousePress(mPos);
}


} // namespace dmx


