/*
 *  SMCEnvironment.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_ENVIRONMENT_
#define _DMX_SMC_ENVIRONMENT_

#include "Dynmx.h"
#include "Model.h"
#include "cinder/Vector.h"

namespace dmx
{
 
//----------------------------------------------------------------------------------------------------------------------
// A triangle
//----------------------------------------------------------------------------------------------------------------------
struct Triangle
{
  Triangle(const ci::Vec2f& v1, const ci::Vec2f& v2, const ci::Vec2f& v3) : p1(v1), p2(v2), p3(v3) {};
  ci::Vec2f p1, p2, p3;
};
  
//----------------------------------------------------------------------------------------------------------------------
// A circle
//----------------------------------------------------------------------------------------------------------------------  
struct Circle
{
  Circle(const ci::Vec2f& p, float r) : position(p), radius(r) {};
  ci::Vec2f position;
  float radius;
};
  
//----------------------------------------------------------------------------------------------------------------------
// An environment that can contain circles and triangles
//----------------------------------------------------------------------------------------------------------------------
class SMCEnvironment : public Model
{
  
public:
  
  SMCEnvironment() {};
  ~SMCEnvironment() {};
  
  // Inherited from class Model
  virtual void init() {};
  virtual void reset() {};
  virtual void update(float dt) {};
  
  void addCircle(const Circle& c) { m_circles.push_back(c); };
  void addTriangle(const Triangle& t) { m_triangles.push_back(t); };
  
  std::vector<Circle>& getCircles() { return m_circles; };
  const std::vector<Circle>& getCircles() const { return m_circles; };
  std::vector<Triangle>& getTriangles() { return m_triangles; };
  const std::vector<Triangle>& getTriangles() const { return m_triangles; };
  
protected:
  
  std::vector<Triangle> m_triangles;
  std::vector<Circle> m_circles;
};
  
} // namespace

#endif
