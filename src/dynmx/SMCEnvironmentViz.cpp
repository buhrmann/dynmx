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
  
#define DMX_ENV_DRAW_POS 0

//----------------------------------------------------------------------------------------------------------------------  
void SMCEnvironmentViz::drawGaussian(const Gaussian& g) const
{
  const int numPoints = 50;
  ci::Vec2f start, end;
  
  // Three sigmas cover approx. 99% of the gaussian
  float a = -3.0 * g.getW();
  float inc = 6.0 * g.getW() / numPoints;
  
  ci::Path2d path;
  path.moveTo(g.getPointAt(a));
  for(int i = 0; i < numPoints; ++i)
  {
    end = g.getPointAt((a+inc));
    path.lineTo(end);
    a += inc;
  }
  path.close();
  
  glColor4f(0.5,0.5,0.5,0.25);
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  ci::gl::drawSolid(path);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCEnvironmentViz::update()
{
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);

  for(int i = 0; i < m_environment->getObjects().size(); i++)
  {
    const Positionable& obj = *m_environment->getObjects()[i];
    if(obj.isVisible())
    {
      ci::gl::color(obj.getColor());
      
      // Object specific collision detection
      if(obj.getType() == Positionable::kObj_Line)
      {
        const Line& line = (Line&)obj;
        ci::gl::drawLine(line.getStart(), line.getEnd());
      }
      else if (obj.getType() == Positionable::kObj_Triangle)
      {
        const Triangle& tri = (Triangle&)obj;        
        drawTriangle(ci::Vec3f(tri.p1), ci::Vec3f(tri.p2), ci::Vec3f(tri.p3));
      }
      else if (obj.getType() == Positionable::kObj_Circle)
      {
        const Circle& circle = (Circle&)obj;        
        ci::gl::drawSolidCircle(circle.getPosition(), circle.getRadius(), 32);
      }
      else if (obj.getType() == Positionable::kObj_Gaussian)
      {
        const Gaussian& gaus = (Gaussian&)obj;        
        drawGaussian(gaus);
      }      
      
#if DMX_ENV_DRAW_POS      
      dmx::drawPoint(ci::Vec3f(obj.getPosition()), 4);
#endif
    }
  }
  
  glPopAttrib();
}
  
} // namespace