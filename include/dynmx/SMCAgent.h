/*
 *  SMCAgent.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_AGENT_
#define _DMX_SMC_AGENT_

#include "Dynmx.h"
#include "Model.h"
#include "DistanceSensor.h"
#include "GradientSensor.h"
#include "TorusSensor.h"
#include "SMCEnvironment.h"
#include "CTRNN.h"
#include "Topology.h"
#include "Recorder.h"

#include "cinder/Vector.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class SMCAgent : public Model
{
  
public:
  
  enum SMCSensorMode
  {
    kSensorMode_Absolute = 0,
    kSensorMode_Derivative,
    kSensorMode_AbsAndDelayed,
    kNumSensorModes
  };

  SMCAgent();
  SMCAgent(const Topology& top);
  ~SMCAgent();
  
  // Inherited from class Model
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  
  // Setters
  void setMaxSpeed(float s) { m_maxSpeed = s; };
  void setMaxAngularSpeed(float s) { m_maxAngularSpeed = s; };
  void setMaxAngle(float a) { m_maxAngle = a; };
  void setMaxPosition(float p) { m_maxPosition = p; };
  void setPositionWraps(bool w) { m_positionWraps = w; };
  void setAngleWraps(bool w) { m_angleWraps = w; };
  void setSensorMode(int mode) { m_sensorMode = mode; };
  void setSensorMode(const std::string& mode);
  void setPosition(const ci::Vec2f& pos);
  void setAngle(float a) { m_angle = a; };
  void setEnergy(float e) { m_energy = e; };
  
  // Getters
  SMCEnvironment& getEnvironment() { return m_environment; };
  CTRNN& getCTRNN() { return *m_ctrnn; };
  const Topology& getTopology() const { return m_topology; };  
  DistanceSensor* getDistanceSensor() const { return m_distanceSensor; };
  GradientSensor* getGradientSensor() const { return m_gradientSensor; };
  TorusSensor* getTorusSensor() const { return m_torusSensor; };
  const ci::Vec2f& getPosition() { return m_position; };
  bool hasDistanceSensor() { return m_distanceSensor != NULL; };
  bool hasGradientSensor() { return m_gradientSensor != NULL; };
  bool hasTorusSensor() { return m_torusSensor != NULL; };
  float getAngle() { return m_angle; };
  float getAngularSpeed() { return m_angularSpeed; };
  float getRadius() { return m_radius; };
  float getSensedValue() { return m_sensedValue; };
  float getTime() { return m_time; };  
  float getAngleWithHeading(ci::Vec2f pos);
  float getMaxPosition() { return m_maxPosition; };
  float getMaxSpeed() { return m_maxSpeed; };
  float getMaxAngularSpeed() { return m_maxAngularSpeed; };
  float getEnergy() { return m_energy; };
  float getSensedFood() { return m_food; };
  const ci::Vec2f& getVelocity() const { return m_velocity; };
  bool angleWraps() const { return m_angleWraps; };
  bool positionWraps() const { return m_positionWraps; };
  
  virtual void toXml(ci::XmlTree& xml);
  virtual void record(Recorder& recorder);  
  
protected:
  
  void updateSensors(float dt);
  float updateDistanceSensor(float dt);
  float updateGradientSensor(float dt);
  float updateTorusSensor(float dt);
  
  float getSensedEnergy();
  
  SMCEnvironment m_environment;
  CTRNN* m_ctrnn;
  Topology m_topology;  
  DistanceSensor* m_distanceSensor;
  GradientSensor* m_gradientSensor;
  TorusSensor* m_torusSensor;
  int m_sensorMode;

  // States
  ci::Vec2f m_position;
  ci::Vec2f m_velocity;
  float m_angle;  
  float m_angularSpeed;
  float m_sensedValue;
  float m_sensedValueDerivative;
    
  // Params
  float m_maxSpeed;
  float m_maxAngularSpeed;
  float m_radius;
  float m_maxAngle;
  float m_maxPosition;

  bool m_positionWraps;
  bool m_angleWraps;
  
  float m_energy;
  float m_energyInitial;
  float m_energySpeedTresh;
  float m_evMin;
  float m_evMax;
  float m_food;
  
  float m_time;
  
}; // class
  
} // namespace

#endif