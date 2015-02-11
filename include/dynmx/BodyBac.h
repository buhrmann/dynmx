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
    m_inverted = false;
  };

  float sense(const std::vector<ci::Vec2f>& lights);
  void setPosition(const ci::Vec2f& newPos, float a);
  void setAngle(float a) { m_angle = a; };
  void inverted(bool i) { m_inverted = i; };
  
  bool m_inverted;
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
  virtual bool hasFinished() { return m_fitnesses.size() >= m_maxNumLights; };
  void nextTrial(int t);
  
  void resetPosition();
  void randomiseLights();
  void setNet(CTRNN* net) { m_net = net; };
  void setLightPresentation(int num, float duration) { m_maxNumLights = num; m_lightDur = duration; };
  
  // Mutations
  void resetMorphology();
  void invertVision();
  void invertSensorFct();
  void shiftVision(float degrees=180);
  void invertMotors();
  
  void useRewardInput(bool r) { m_useRewardInput = r; };

  float getReward() const { return m_reward; };
  float getFitness() const { return m_fitness; };
  float getFinalFitness();
  
  
protected:
  
  // Parameters
  float m_radius;
  float m_maxSpeed;
  float m_maxAngSpeed;
  float m_lightDur;
  float m_sensorAngRange;
  int m_maxNumLights;
  
  CTRNN* m_net;
  std::vector<LightSensor> m_sensors;
  int m_motorId1;
  int m_motorId2;
  bool m_useRewardInput;
  
  // State
  int m_rewardLight;
  float m_time;
  float m_angle;
  float m_fitness;
  float m_reward;
  std::vector<float> m_fitnesses;
  float m_initDist;
  
  ci::Vec2f m_position;
  ci::Vec2f m_velocity;
  std::vector<ci::Vec2f> m_lightPos;
};
  
} // namespace

#endif
