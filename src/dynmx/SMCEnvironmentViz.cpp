/*
 *  SMCEnvironmentViz.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCEnvironmentViz.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
void SMCEnvironmentViz::update()
{
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);
  
  glColor3f(0.1, 0.1, 0.1);
  
  for(int i = 0; i < m_environment->getCircles().size(); i++)
  {
    const Circle& circle = m_environment->getCircles()[i];
    ci::gl::drawSolidCircle(circle.getPosition(), circle.getRadius(), 32);
  }

  for(int i = 0; i < m_environment->getTriangles().size(); i++)
  {
    const Triangle& tri = m_environment->getTriangles()[i];
    drawTriangle(ci::Vec3f(tri.p1), ci::Vec3f(tri.p2), ci::Vec3f(tri.p3));
  }
  
  glPopAttrib();
}
  
} // namespace