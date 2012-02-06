/*
 *  Spin.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 05/02/2012.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "Spin.h"
#include "Random.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void SpinningWheel::init()
{
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void SpinningWheel::reset()
{
  m_numSegments = UniformRandomInteger(0, 10);
  float max = 360.0 / m_numSegments;
  float min = max / 2.0;
  float total = 0;
  m_segmentSizes.clear();
  for(int i = 0; i < m_numSegments-1; i++)
  {
    float size = UniformRandom(min, max);
    m_segmentSizes.push_back(size);
    ci::Color col (UniformRandom(0, 1), UniformRandom(0, 1), UniformRandom(0, 1));
    m_segmentColors.push_back(col);
    total += size;
  }
  m_segmentSizes.push_back(360.0 - total);

}

//----------------------------------------------------------------------------------------------------------------------
void SpinningWheel::update(float dt)
{
  m_time += dt;
  m_mat.rotate(ci::Vec3f(0,0,1), m_speed*dt);  
}

//----------------------------------------------------------------------------------------------------------------------
void SpinningWheelViz::init()
{
  attachDriver(&m_wheel->m_mat);
  createGeometry();
}

//----------------------------------------------------------------------------------------------------------------------
void SpinningWheelViz::createGeometry()
{
  if(m_dl > -1)
  {
    glDeleteLists(m_dl, 1);
  }
  
  m_dl = glGenLists(1);
  glNewList(m_dl, GL_COMPILE);

  float total = 0;
  for(int i = 0; i < m_wheel->m_numSegments; i++)
  {
    ci::gl::color(m_wheel->m_segmentColors[i]);
    float size = m_wheel->m_segmentSizes[i];
    dmx::drawPartialDisk(0, m_wheel->m_radius, 8, 1, total, size, GLU_FILL);    
    total += size;
  }

  glEndList();
}

//----------------------------------------------------------------------------------------------------------------------
void SpinningWheelViz::update()
{
  NodeGeometry::update();
}

}