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
#include "cinder/Color.h"
#include "MathUtils.h"

namespace dmx
{
 
//----------------------------------------------------------------------------------------------------------------------
// A positionable
//----------------------------------------------------------------------------------------------------------------------
class Positionable
{
public:
  
  enum EnvObjType
  {
    kObj_Line,
    kObj_Triangle,
    kObj_Circle,
    kObj_Gradient,
    kObj_Torus,
    kObj_Gaussian,
    kObj_NumTypes
  };
  
  virtual ~Positionable(){};
  Positionable(const ci::Vec2f& pos = ci::Vec2f(0,0)) : 
  m_position(pos), m_positionMean(pos), m_visible(true), m_positionVar(0,0), m_angle(0), m_angleMean(0), m_angleVar(0), m_color(ci::Color::black()) { };
  
  virtual const ci::Vec2f& getPosition() const { return m_position; };
  virtual ci::Vec2f getPosition() { return m_position; };
  virtual const ci::Vec2f& getPositionVar() const { return m_positionVar; };
  virtual ci::Vec2f getPositionVar() { return m_positionVar; };
  virtual const ci::Vec2f& getPositionMean() const { return m_positionMean; };
  virtual ci::Vec2f getPositionMean() { return m_positionMean; };
  virtual void setPosition(const ci::Vec2f& pos) { m_position = pos; };
  virtual void setPositionMean(const ci::Vec2f& pos) { m_positionMean = pos; };
  void setPositionVariance(const ci::Vec2f& var) { m_positionVar = var; };
  void randomisePosition();
  
  virtual float getAngle() const { return m_angle; };
  virtual const float getAngleVar() const { return m_angleVar; };
  virtual const float getAngleMean() const { return m_angleMean; };
  virtual void setAngle(float ang) { m_angle = ang; };
  virtual void setAngleMean(float ang) { m_angleMean = ang; };  
  void setAngleVariance(float var) { m_angleVar = var; };
  void randomiseAngle();
  
  void randomisePose() { randomisePosition(); randomiseAngle(); };

  void setVisibility(bool v) {m_visible = v; };  
  void toggleVisibility() { m_visible = !m_visible; };
  bool isVisible() const { return m_visible; };
  
  void setColor(const ci::ColorA& col) { m_color = col; };
  const ci::ColorA& getColor() const { return m_color; };
  
  EnvObjType getType() const { return m_type; };

protected:  
  ci::Vec2f m_position;
  ci::Vec2f m_positionMean;
  ci::Vec2f m_positionVar;
  ci::ColorA m_color;
  float m_angle;
  float m_angleMean;
  float m_angleVar;
  
  bool m_visible;
  EnvObjType m_type;
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
    m_type = kObj_Triangle;
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
  Circle(const ci::Vec2f& pos, float r) : Positionable(pos), m_radius(r) { m_type = kObj_Circle; };
  float getRadius() const { return m_radius; };
  void setRadius(float r) { m_radius = r; };
  
protected:
  float m_radius;
};
  
//----------------------------------------------------------------------------------------------------------------------
// A radial gradient
//----------------------------------------------------------------------------------------------------------------------
class Gradient : public Circle
{
public:
  virtual ~Gradient(){};
  Gradient(const ci::Vec2f& pos, float r) : Circle(pos, r) { m_type = kObj_Gradient; m_color = ci::ColorA(0.6,0.6,0.6,0.5);};
};
  
//----------------------------------------------------------------------------------------------------------------------
// A circle with width
//----------------------------------------------------------------------------------------------------------------------
class Torus : public Circle
{
public:
  Torus(const ci::Vec2f& pos, float r, float w) : Circle(pos, r), m_width(w) { m_type = kObj_Torus; m_peak = 1.0f; m_color = ci::ColorA(0.0,0.6,0.6,0.1);};
  virtual ~Torus(){};
  
  void setPeak(float v) { m_peak = v; };
  
  bool isPointWithin(const ci::Vec2f& p) const { return fabs(relDistanceFromCentralLine(p)) < m_width; };
  float relDistanceFromCentralLine(const ci::Vec2f& p) const { return p.distance(m_position) - m_radius; };
  float getWidth() const { return m_width; };
  float getPeak() const { return m_peak; };
  
protected:
  float m_width;
  float m_peak;
};
  
//----------------------------------------------------------------------------------------------------------------------
// A gaussian centered around pos
//----------------------------------------------------------------------------------------------------------------------  
class Gaussian : public Positionable
{
public:
  virtual ~Gaussian(){};
  Gaussian(const ci::Vec2f& pos, float w, float h, const ci::Vec2f& dir) : Positionable(pos), m_w(w), m_wSq(2*w*w), m_h(h) 
  { 
    m_type = kObj_Gaussian; 
    m_dir = dir;
    m_ortho = dir;
    m_ortho.rotate(dmx::PI_OVER_TWO);
    m_cutoff = 3.0 * m_w;
  };
  
  void setW(float w) { m_w = w; m_wSq = 2*w*w; m_cutoff = 10.0 * m_w;};
  void setH(float h) { m_h = h; };
  
  float getW() const { return m_w; };
  float getH() const { return m_h; };
  const ci::Vec2f& getDir() const { return m_dir; };
  float getValueAt(float x) const { return std::fabs(x) > m_cutoff ? 0.0 : m_h * exp(-(x*x) / m_wSq); };
  ci::Vec2f getPointAt(float x) const { return m_position + (m_dir * x) + m_ortho * getValueAt(x); };
  ci::Vec2f getPointFromCoords(float x, float y) const { return m_position + (m_dir * x) + (m_ortho * y); };
  void randomise(bool w, bool h);
  
protected:
  float m_w;
  float m_wSq;
  float m_cutoff;
  float m_h;
  ci::Vec2f m_dir;
  ci::Vec2f m_ortho;
};  

//----------------------------------------------------------------------------------------------------------------------
// A circle
//----------------------------------------------------------------------------------------------------------------------  
class Line : public Positionable
{
public:
  virtual ~Line(){};
  Line(const ci::Vec2f& pos, float angle, float length) : Positionable(pos), m_length(length)
  {
    setAngle(angle);
    m_angleMean = angle;
    m_type = kObj_Line;
  };
  
  virtual void setPosition(const ci::Vec2f& pos)
  {   
    ci::Vec2f diff = pos - m_position;
    m_position += diff;
    m_start += diff;
    m_end += diff;    
  }
  
  virtual void setAngle(float angle)
  {
    m_angle = angle;
    ci::Vec2f dir (cos(angle), sin(angle));
    m_start = m_position - 0.5 * m_length * dir;
    m_end =   m_position + 0.5 * m_length * dir; 
  }
  
  const ci::Vec2f& getStart() const { return m_start; };
  const ci::Vec2f& getEnd() const { return m_end; };
  float getLength() const { return m_length; };
  
protected:
  float m_length;
  ci::Vec2f m_start;
  ci::Vec2f m_end;
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
  
  const std::vector<Positionable*>& getObjects () const { return m_objects; };
  std::vector<Positionable*>& getObjects () { return m_objects; };  
  
  void fromXml(const ci::XmlTree& xml);
  
protected:
  
  std::vector<Positionable*> m_objects;  
};
  
} // namespace

#endif
