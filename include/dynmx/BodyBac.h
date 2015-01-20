//
//  BodyBac.h
//  dynmx
//
//  Created by Thomas Buhrmann on 25/08/14.
//
//

#ifndef dynmx_BodyBac_h
#define dynmx_BodyBac_h

#include "Dynmx.h"
#include "Model.h"
#include "Recorder.h"
#include "CTRNN.h"

#include "cinder/Vector.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Simple light sensor
//----------------------------------------------------------------------------------------------------------------------
class LightSensor
{
  
public:
  
  LightSensor(float angle, float offset) : m_angle(angle), m_offset(offset), m_gain(1), m_signal(0)
  {
    m_position.set(0,0);
    m_direction.set(1,0);
  };

  float sense(const ci::Vec2f& lightPos);
  void setPosition(const ci::Vec2f& newPos, float a);
  
  float m_angle;
  float m_offset;
  float m_gain;
  
  float m_signal;
  ci::Vec2f m_position;
  ci::Vec2f m_direction;
  
  static float s_sigma;
  static float s_maxSensorAng;
};
  
  
//----------------------------------------------------------------------------------------------------------------------
// Simple phototactic bacterial agent
//----------------------------------------------------------------------------------------------------------------------
class LegBac;
class LegBacViz;
class LegBacView;
  
class BodyBac : public Model
{
  
friend LegBac;
friend LegBacViz;
friend LegBacView;
  
public:
  
  BodyBac();
  ~BodyBac() {};
  
  // Inherited from class Model
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  virtual bool hasFinished() { return m_fitnesses.size() >= m_maxNumFoods; };
  
  void resetPosition();
  void randomiseFood();
  void setNet(CTRNN* net) { m_net = net; };
  void setFoodPresentation(int num, float duration) { m_maxNumFoods = num; m_foodDur = duration; };

  float getReward() const { return m_reward; };
  float getFitness() const { return m_fitness; };
  float getFinalFitness();
  
  
protected:
  
  // Parameters
  float m_radius;
  float m_maxSpeed;
  float m_maxAngSpeed;
  float m_foodDur;
  int m_maxNumFoods;
  
  CTRNN* m_net;
  std::vector<LightSensor> m_sensors;
  
  // State
  float m_time;
  float m_angle;
  float m_fitness;
  float m_reward;
  std::vector<float> m_fitnesses;
  float m_initDist;
  
  ci::Vec2f m_position;
  ci::Vec2f m_velocity;
  ci::Vec2f m_foodPos;
};
  
} // namespace

#endif
