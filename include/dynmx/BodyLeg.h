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
  
  void setNet(CTRNN* net) { m_net = net; };
  
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
  
//----------------------------------------------------------------------------------------------------------------------
// Implmenetation
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
inline BodyLeg::BodyLeg()
{
  init();
}
  
//----------------------------------------------------------------------------------------------------------------------
inline void BodyLeg::init()
{
  s_legLength = 15.0f;
  s_maxFootDist = 20.0f;
  s_maxLegForce = 0.05f;
  s_forwardAngleLimit = PI/6;
  s_backwardAngleLimit = -PI/6;
  s_maxVelocity = 6.0;
  s_maxTorque = 0.5;
  s_maxAngSpeed = 1.0;
  s_maxAsymVel = 0.627;
  s_minLegSensor = -1.0;
  s_maxLegSensor = 0.524;

  m_sensorRange = s_maxLegSensor - s_minLegSensor;
  m_sensorOffset = s_minLegSensor / m_sensorRange;
}

//----------------------------------------------------------------------------------------------------------------------
inline void BodyLeg::reset()
{
  m_force = 0;
  m_position.set(0,0);
  m_velocity.set(0,0);
	m_footDown = false;
  
	m_angle = s_forwardAngleLimit; //UniformRandom(s_backwardAngleLimit, s_forwardAngleLimit);
	m_sensor = (m_angle / m_sensorRange) - m_sensorOffset; // Normalised sensor
	m_angSpeed = m_forwardForce = m_backwardForce = 0.0;
	m_joint = m_position;
  const float jointOffset = 0.0;
  m_joint.y += jointOffset;
  
	m_foot.x = m_joint.x + s_legLength * sin(m_angle);
	m_foot.y = m_joint.y + s_legLength * cos(m_angle);
  
  m_reward = 0;
}

//----------------------------------------------------------------------------------------------------------------------
inline void BodyLeg::update(float dt)
{
  // Sense
	m_sensor = (m_angle / m_sensorRange) - m_sensorOffset;
  
  // Update net
  m_net->setExternalInput(0, m_sensor);
  m_net->updateDynamic(dt);
  
  m_force = 0.0f;
	float newangle = 0.0f;
	
  const int N = m_net->getSize();
  const int idFoot = N - 1;
  const int idFwd = N - 2;
  const int idBwd = N - 3;
  
	if (m_net->getOutput(idFoot) > 0.5)
  {
    m_footDown = true;
    m_angSpeed = 0;
  }
	else
  {
    m_footDown = false;
  }
  
	m_forwardForce = m_net->getOutput(idFwd) * s_maxLegForce;
  m_backwardForce = m_net->getOutput(idBwd) * s_maxLegForce;

	// Compute the force applied to the body
	if (m_footDown && m_angle >= s_backwardAngleLimit && m_angle <= s_forwardAngleLimit)
  {
		m_force = m_forwardForce - m_backwardForce;
  }
  
	// Update the position of the body
	m_velocity.x += dt * m_force;
  m_velocity.x = clamp(m_velocity.x, -s_maxVelocity, s_maxVelocity);

	m_position += dt * m_velocity;
  
	// Update the leg geometry
	m_joint += dt * m_velocity;
	if (m_footDown)
  {
		newangle = atan2(m_foot.x - m_joint.x, m_foot.y - m_joint.y);
		m_angSpeed = (newangle - m_angle) / dt;
		m_angle = newangle;
	}
	else
  {
		m_velocity.set(0,0);
		m_angSpeed	+= dt * s_maxTorque * (m_backwardForce -  m_forwardForce);
    m_angSpeed = clamp(m_angSpeed, -s_maxAngSpeed, s_maxAngSpeed);

		m_angle += dt * m_angSpeed;
		if (m_angle < s_backwardAngleLimit)
    {
      m_angle = s_backwardAngleLimit;
      m_angSpeed = 0;
    }
		else if (m_angle > s_forwardAngleLimit)
    {
      m_angle = s_forwardAngleLimit;
      m_angSpeed = 0;
    }
    
		m_foot.x = m_joint.x + s_legLength * sin(m_angle);
		m_foot.y = m_joint.y + s_legLength * cos(m_angle);
	}
  
	// If the foot is too far back, the body becomes "unstable" and forward motion ceases
	if (m_joint.distance(m_foot) > s_maxFootDist)
    m_velocity.x = 0.0;
}

//----------------------------------------------------------------------------------------------------------------------
inline float BodyLeg::getFinalFitness()
{
  return fabs(m_position.x);
}

  
} // namespace

#endif
