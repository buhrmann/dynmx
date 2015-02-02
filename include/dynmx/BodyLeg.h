//
//  BodyLeg.h
//  dynmx
//
//  Created by Thomas Buhrmann on 25/08/14.
//
//

#ifndef dynmx_BodyLeg_h
#define dynmx_BodyLeg_h

#include "Dynmx.h"
#include "Model.h"
#include "Recorder.h"
#include "CTRNN.h"

#include "cinder/Vector.h"

namespace dmx
{

class LegBac;
class LegBacViz;
class LegBacView;
  
//----------------------------------------------------------------------------------------------------------------------
// A one-legged agent
//----------------------------------------------------------------------------------------------------------------------
class BodyLeg : public Model
{
  
friend LegBac;  
friend LegBacViz;
friend LegBacView;
  
public:
  
  BodyLeg();
  ~BodyLeg() {};
  
  // Inherited from class Model
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  virtual bool hasFinished() { return m_time > m_duration; };
  
  void setNet(CTRNN* net) { m_net = net; };
  void setDuration(float d) { m_duration = d; };
  void setNumTrials(int n) { m_numTrials = n;};
  void nextTrial(int t);
  
  float getReward() const { return m_reward; };
  float getFitness() const { return m_fitness; };
  float getFinalFitness();
  
protected:
  
  CTRNN* m_net;
  
  // Parameters
  float s_legLength;
  float s_maxFootDist;
  float s_maxLegForce;
  float s_forwardAngleLimit;
  float s_backwardAngleLimit;
  float s_maxVelocity;
  float s_maxTorque;
  float s_maxAngSpeed;
  float s_maxAsymVel;
  float s_minLegSensor;
  float s_maxLegSensor;
  
  int m_numTrials;
  float m_duration;
  float m_startingAngle;
  
  // State
  float m_time;
  float m_reward;
  float m_fitness;
  bool m_footDown;
  float m_angle;
  float m_angSpeed;
  float m_force;
  float m_sensor;
  float m_forwardForce;
  float m_backwardForce;
  float m_sensorRange;
  float m_sensorOffset;
  
  ci::Vec2f m_position;
  ci::Vec2f m_joint;
  ci::Vec2f m_foot;
  ci::Vec2f m_velocity;
  
};
  
} // namespace

#endif
