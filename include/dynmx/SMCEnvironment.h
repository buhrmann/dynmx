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
// A positionable
//----------------------------------------------------------------------------------------------------------------------
class Positionable
{
public:
  virtual ~Positionable(){};
  Positionable(const ci::Vec2f& pos = ci::Vec2f(0,0)) : m_position(pos) {};
  
  virtual const ci::Vec2f& getPosition() const { return m_position; };
  virtual ci::Vec2f getPosition() { return m_position; };
  virtual void setPosition(const ci::Vec2f& pos) { m_position = pos; };

protected:  
  ci::Vec2f m_position;
};
  
  
//----------------------------------------------------------------------------------------------------------------------
// A triangle with position in its centre
//----------------------------------------------------------------------------------------------------------------------
class Triangle : public Positionable
{
public:  
  virtual ~Triangle(){};
  Triangle(const ci::Vec2f& pos, float size) : Positionable(pos), m_size(size)
  {
    setPosition(pos);
  };
  
  virtual void setPosition(const ci::Vec2f& pos)
  {
    Positionable::setPosition(pos);
    p1 = pos + ci::Vec2f(-m_size/2, 0); 
    p3 = pos + ci::Vec2f(m_size/2, m_size/2); 
    p2 = pos + ci::Vec2f(m_size/2, -m_size/2);
  }
  
  ci::Vec2f p1, p2, p3;

protected:  
  float m_size;
};
  
//----------------------------------------------------------------------------------------------------------------------
// A circle
//----------------------------------------------------------------------------------------------------------------------  
class Circle : public Positionable
{
public:
  virtual ~Circle(){};
  Circle(const ci::Vec2f& pos, float r) : Positionable(pos), m_radius(r) {};
  float getRadius() const { return m_radius; };
  void setRadius(float r) { m_radius = r; };
  
protected:
  float m_radius;
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
