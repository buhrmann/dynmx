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
// CTRNNNeuronViz implementation
//----------------------------------------------------------------------------------------------------------------------
void CTRNNNeuronViz::update()
{
  int N = m_ctrnn->getSize();
  
  // left upper corner of circle's bounding box at 0,0:
  const float segmentSize = TWO_PI / N;
  const float networkRadius = m_width / 2.5;  
  const float neuronRadius = networkRadius / N;

  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // connectivity
  for(int i = 0; i < N; i++)
  {
    float angle = -((float)i + 0.5f) * segmentSize + PI_OVER_TWO;   
    Vec3f midPoint (networkRadius * cosf(angle), networkRadius * sinf(angle), 0.0f);

    // draw only the selected neuron's connections
    if(i == m_selected || m_selected == -1)
    {
      // draw connections    
      for(int j = 0; j < N; j++)
      {
        const double eps = 0.001;
        const float weight = m_ctrnn->getWeight(j,i);        
        if(fabs(weight) > eps)
        {
          float otherAngle = -((float)j + 0.5f) * segmentSize + PI_OVER_TWO; 
          Vec3f otherMidPoint (networkRadius * cosf(otherAngle), networkRadius * sinf(otherAngle), 0.0f);

          glLineWidth(fabs(weight));
          if(weight < 0)
            glColor3f(235.0/255.0, 0.0/255.0, 103.0/255.0); 
          else
            glColor3f(0,0,0);
          ci::gl::drawLine(midPoint, otherMidPoint);
        }
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
        
        sprintf(str, "b: %2.2f | t: %2.2f | g: %2.2f", m_ctrnn->getBias(i), m_ctrnn->getTimeConstant(i), m_ctrnn->getGain(i));
        ci::gl::drawString(str, textPos + ci::Vec2f(0, lineHeight), textColor, m_font);      
        
        sprintf(str, "in: %1.3f | out: %1.3f", m_ctrnn->getExternalInput(i), m_ctrnn->getOutput(i));
        ci::gl::drawString(str, textPos + ci::Vec2f(0, 2 * lineHeight), textColor, m_font);            
      }
    }
  }
  
  glLineWidth(1.0f);
  float outerRadius = neuronRadius + 6;
  for(int i = 0; i < N; i++)
  {
    ci::Color col = m_ctrnn->getState(i) > 0 ? ci::Color(0,0,0) : ci::Color(235.0/255.0, 0.0/255.0, 103.0/255.0);

    float angle = -((float)i + 0.5f) * segmentSize + PI_OVER_TWO;   
    Vec3f midPoint (networkRadius * cosf(angle), networkRadius * sinf(angle), 0.0f);
    glPushMatrix();
    glTranslatef(midPoint);
    // white background    
    glColor3f(1,1,1);
    drawDisk(outerRadius, 0.0, 32, 1);    
    ci::gl::color(col);
    // disk size indicating output value
    drawDisk(neuronRadius * clamp(m_ctrnn->getOutput(i),0.0,1.0), 0.0, 32, 1);
    // ring size indicating external input value
    glColor3f(181.0/255.0, 206.0/255.0, 26.0/255.0);    
    float width = radiansToDegrees(m_ctrnn->getExternalInput(i) * TWO_PI);
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
  int N = m_ctrnn->getSize();
  m_selected = -1;

  // Update mouse info: transform mouse position into local space angle and radius
  Vec4f posLocal = mPos - m_pTM->getTranslate();
  float angle = radiansToDegrees(atan2(posLocal.x, posLocal.y));
  if(angle < 0) 
  {
    angle += 360.0f;
  }

  const float networkRadius = m_width / 2.5;  
  const float neuronRadius = networkRadius / N + 6;   
  int neuron = (int)angle / (360 / (int) N);    
  float neuronAngle = -((float)neuron + 0.5f) * TWO_PI / N + PI_OVER_TWO;   
  Vec4f neuronPos (networkRadius * cosf(neuronAngle), networkRadius * sinf(neuronAngle), 0.0f, 1.0);
  float dist = neuronPos.distance(posLocal);
     
  if(dist < neuronRadius)
  {
    m_selected = neuron;
  }
}

//----------------------------------------------------------------------------------------------------------------------
// CTRNNWheelViz implementation
//----------------------------------------------------------------------------------------------------------------------
void CTRNNWheelViz::update()
{
  int N = m_ctrnn->getSize();
  
  const float segmentSizeDeg = 360.0f / N;
  const float segmentSizeRad = degreesToRadians(segmentSizeDeg);
  const float r3 = m_width / 2;
  const float r2 = r3 - 5;
  const float r1 = r2 - 10;
  
  // left upper corner of wheel's bounding box at 0,0:    
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // connectivity
  for(int i = 0; i < N; i++)
  {
    // draw only the selected neuron's connections
    if(i == m_selected || m_selected == -1)
    {
      float angle = -((float)i + 0.5f) * segmentSizeRad + PI_OVER_TWO; 
      Vec3f midPoint ((r1+5) * cosf(angle), (r1+5) * sinf(angle), 0.0f);
      for(int j = 0; j < N; j++)
      {
        const double eps = 0.001;
        const float weight = m_ctrnn->getWeight(j, i);        
        if(fabs(weight) > eps)
        {
          float otherAngle = -((float)j + 0.5f) * segmentSizeRad + PI_OVER_TWO; 
          Vec3f otherMidPoint ((r1+5) * cosf(otherAngle), (r1+5) * sinf(otherAngle), 0.0f);
          glLineWidth(fabs(weight));
          if(weight < 0)
            glColor3f(235.0/255.0, 0.0/255.0, 103.0/255.0); 
          else
            glColor3f(0,0,0);
          ci::gl::drawLine(midPoint, otherMidPoint);
        }
      }   
    }   
  }
  glLineWidth(1.0f);
  
  // outputs
  for(int i = 0; i < N; i++)
  {
    // white background
    glColor3f(1,1,1);
    dmx::drawPartialDisk(r1, r2, 8, 1, i * segmentSizeDeg, segmentSizeDeg, GLU_FILL);
    // actual amount
    glColor3f(181.0/255.0, 206.0/255.0, 26.0/255.0);
    float outputWidth = m_ctrnn->getOutput(i) * segmentSizeDeg;
    dmx::drawPartialDisk(r1, r2, 8, 1, i * segmentSizeDeg, outputWidth, GLU_FILL); 
    // outline segment
    glColor3f(0,0,0);
    dmx::drawPartialDisk(r1, r2, 8, 1, i * segmentSizeDeg, segmentSizeDeg, GLU_SILHOUETTE);   
  }
  
  // indicators
  for(int i = 0; i < N; i++)
  {
    glColor3f(0,0,0);
    dmx::drawPartialDisk(r2, r3, 8, 1, i * segmentSizeDeg, m_ctrnn->getExternalInput(i) * segmentSizeDeg, GLU_FILL);  
    
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
    m_selected = (int)angle / (360 / (int) m_ctrnn->getSize());
  }
}

//----------------------------------------------------------------------------------------------------------------------
// CTRNNLayerViz implementation
//----------------------------------------------------------------------------------------------------------------------
CTRNNLayerViz::CTRNNLayerViz(const CTRNN* ctrnn, const Topology* top, float width, bool text) : 
  CTRNNBasicViz(ctrnn, width, text), 
  m_topology(top) 
{
  int numInputs = m_topology->getInputsAreNeurons() ? m_topology->getNumInputs() : 0;
  int numHidden = m_topology->getNumHidden();
  int numOutputs = m_topology->getNumOutputs();
  m_maxLayerN = std::max(numInputs, std::max(numHidden, numOutputs));
  
  int space = m_width / m_maxLayerN;   
  m_r = space / 3;
  
  // Pre-calculate all neuron's positions
  // Inputs
  int x;
  int y = 5.0/12.0 * m_width; // not quite 1/2
  int xOffset = -space * (numInputs - 1) / 2;
  for(int i = 0; i < numInputs; i++)
  {
    x = xOffset + i * space;
    m_pos.push_back(ci::Vec3f(x, y, 0));
  }
  
  // Hidden
  y = 0.0;
  xOffset = -space * (numHidden - 1) / 2;
  float midPoint = (float)(numHidden - 1) / 2.0;
  for(int i = 0; i < numHidden; i++)
  {
    x = xOffset + i * space;
    // triangular arrangement so weights become discernable    
    int yOffset = -space/3 * (midPoint - std::abs((float)i - midPoint)); 
    m_pos.push_back(ci::Vec3f(x, y + yOffset, 0));
  }
  
  // Output
  y = -5.0/12.0 * m_width; // not quite 1/2
  xOffset = -space * (numOutputs - 1) / 2.0;
  midPoint = (float)(numOutputs - 1) / 2.0;
  for(int i = 0; i < numOutputs; i++)
  {
    x = xOffset + i * space;
    // triangular arrangement so weights become discernable    
    int yOffset = -space/3 * (midPoint - std::abs((float)i - midPoint));     
    m_pos.push_back(ci::Vec3f(x, y + yOffset, 0));
  }  
};

//----------------------------------------------------------------------------------------------------------------------    
void CTRNNLayerViz::update()
{
  const float innerRadius = m_r - 6;
  
  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // connectivity
  for(int i = 0; i < m_pos.size(); i++)
  {
    // draw only the selected neuron's connections
    if(i == m_selected || m_selected == -1)
    {
      for(int j = 0; j < m_pos.size(); j++)
      {
        const double eps = 0.001;
        const float weight = m_ctrnn->getWeight(j, i);        
        if(fabs(weight) > eps)
        {
          glLineWidth(fabs(weight));
          if(weight < 0)
            glColor3f(235.0/255.0, 0.0/255.0, 103.0/255.0); 
          else
            glColor3f(0,0,0);
          ci::gl::drawLine(m_pos[j], m_pos[i]);
        }
      }   
    }   
  }
  glLineWidth(1.0);  
  
  // Neurons
  for(int i = 0; i < m_pos.size(); i++)
  {
    glPushMatrix();
    glTranslatef(m_pos[i]);
    CTRNNViz::drawNeuron(i, m_ctrnn, innerRadius, m_r);
    glPopMatrix();
  }
  
  glPopMatrix();
}

//----------------------------------------------------------------------------------------------------------------------
void CTRNNLayerViz::onMouseMove(const Vec4f& mPos)
{
  m_selected = -1;
  
  // update mouse info: transform mouse position into local space angle and radius
  Vec4f posLocal = mPos - m_pTM->getTranslate();
  for(int i = 0; i < m_pos.size(); ++i)
  {
    if(posLocal.distanceSquared(m_pos[i]) < (m_r * m_r))
    {
      m_selected = i;
    }
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
// CTRNNViz implementation  
//----------------------------------------------------------------------------------------------------------------------  

// Mormalises inputs to 1, if input neurons function as placeholders  
//----------------------------------------------------------------------------------------------------------------------  
double CTRNNViz::getOutputNorm(int i, const CTRNN* ctrnn)
{
  double out = ctrnn->getOutput(i);
  if(out > 1)
    out /= ctrnn->getGain(i);
  
  return out;
}

//----------------------------------------------------------------------------------------------------------------------  
void CTRNNViz::drawNeuron(int i, const CTRNN* ctrnn, float inner, float outer)
{
  // white background    
  glColor3f(1,1,1);
  drawDisk(outer, 0.0, 32, 1);    
  
  // disk size indicating output value
  ci::Color col = ctrnn->getState(i) > 0 ? ci::Color(0,0,0) : ci::Color(235.0/255.0, 0.0/255.0, 103.0/255.0);    
  ci::gl::color(col);    
  drawDisk(inner * clamp(getOutputNorm(i, ctrnn),0.0,1.0), 0.0, 32, 1);
  
  // ring size indicating external input value
  glColor3f(181.0/255.0, 206.0/255.0, 26.0/255.0);    
  float width = radiansToDegrees(ctrnn->getExternalInput(i) * TWO_PI);
  dmx::drawPartialDisk(inner, outer, 8, 1, 0, width, GLU_FILL);     
  glColor3f(0,0,0);
  ci::gl::drawStrokedCircle(ci::Vec2f(0,0), outer, 32);
  ci::gl::drawStrokedCircle(ci::Vec2f(0,0), inner, 32);      
}
  
//----------------------------------------------------------------------------------------------------------------------
void CTRNNViz::init()
{
  NodeGroup::init();
  
  m_font = ci::Font(ci::app::loadResource("pf_tempesta_seven.ttf"), 8);
  
  m_mode = kRM_Layers;
  m_selected = -1;
  m_angle = 0;
  m_labelHeight = 16;
  m_padding = 3;
  m_lineHeight = m_labelHeight - m_padding;  
  m_unitSize = m_width / m_ctrnn->getSize();

  m_textColor = ci::Color(1,1,1);
  m_labelColor = ci::ColorA(0.0f, 0.0f, 0.0f, 0.5f);
  
  float buttonSize = m_labelHeight - 2 * m_padding;  
  ci::Vec2f butPos(m_width - buttonSize - m_padding, m_padding);
  m_modeButton = ci::Rectf(butPos.x, butPos.y, butPos.x + buttonSize, butPos.y + buttonSize);
  
  m_label2y = ((int)(m_labelHeight + 2 * m_padding + m_unitSize)) ;
  m_label3y = m_label2y + m_labelHeight + m_padding + m_width + m_padding;
  m_height =  m_label3y + m_labelHeight + m_padding + 3 * m_lineHeight + 2 * m_padding;
  
  // output vector
  int N = m_ctrnn->getSize();
  m_outputs = new VectorView<double>(m_ctrnn->getOutputs(), N, m_width, 1.0); 
  m_outputs->translate(Vec4f(0, m_labelHeight + m_padding, 0, 1));
  m_children.push_back(m_outputs);
  
  // weight matrix
  m_weights = new MatrixView<double>(m_ctrnn->getWeights(), N, N, m_width, 20.0);
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
  
  // layerViz
  if(m_topology)
  {
    m_layerViz = new CTRNNLayerViz(m_ctrnn, m_topology, m_width, false);
    m_layerViz->translate(ci::Vec4f(m_width / 2, m_label3y / 2, 0, 1));
    m_children.push_back(m_layerViz);      
  }
}

//----------------------------------------------------------------------------------------------------------------------  
void CTRNNViz::renderAs(int m)
{
  switch (m) 
  {
    case kRM_Matrix:
      renderAsMatrix();
      break;
    case kRM_Ring:
      m_children[2]->update();
      break;      
    case kRM_Wheel:
      m_children[3]->update();
      break;
    case kRM_Layers:
      m_children[4]->update();
      break;
  }
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
  ci::gl::color(0.7f, 0.7f, 0.7f);
  ci::gl::drawSolidRect(m_modeButton);
  
  // Draw according to selected mode
  ci::gl::color(m_labelColor);  
  renderAs(m_mode);
  
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
  
  // get selected node from current viz
  int i;
  switch (m_mode) 
  {
    case kRM_Matrix:
    default:
      i = m_outputs->m_iSel;
      break;
    case kRM_Wheel:
      i = m_wheelViz->getSelectedNeuron();
      break;
    case kRM_Ring:
      i = m_neuronViz->getSelectedNeuron();
      break;
    case kRM_Layers:
      i = m_layerViz->getSelectedNeuron();
      break;      
  }
  
  char str [128];
  if(i != -1)
  {
    sprintf(str, "Node: %i", i);
    ci::gl::drawString(str, textPos, m_textColor, m_font);
    
    sprintf(str, "b: %2.2f | t: %2.2f | g: %2.2f", m_ctrnn->getBias(i), m_ctrnn->getTimeConstant(i), m_ctrnn->getGain(i));
    ci::gl::drawString(str, textPos + ci::Vec2f(0, m_lineHeight), m_textColor, m_font);      
    
    sprintf(str, "in: %1.3f | out: %1.3f", m_ctrnn->getExternalInput(i), m_ctrnn->getOutput(i));
    ci::gl::drawString(str, textPos + ci::Vec2f(0, 2 * m_lineHeight), m_textColor, m_font);            
  }
  
  if(m_mode == kRM_Matrix)
  {
    const int& iW = m_weights->m_iSel;
    const int& jW = m_weights->m_jSel; 
    if(iW != -1 && jW != -1)
    {      
      sprintf(str, "weight %i > %i: %2.2f", iW, jW, m_ctrnn->getWeight(iW, jW));
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
    // We have n viz in children (but matrix viz consists of two, therefore one less)
    m_mode = (m_mode + 1) % (m_children.size() - 1);
  }
  
  NodeGroup::onMousePress(mPos);
}


} // namespace dmx


