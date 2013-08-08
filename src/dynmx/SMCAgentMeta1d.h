/*
 *  SMCAgentEvo.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/22/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_AGENT_META_1D_
#define _DMX_SMC_AGENT_META_1D_

#include "GARunner.h"
#include "Dynmx.h"
#include "Model.h"
#include "DistanceSensor.h"
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
class SMCAgentMeta1d : public Evolvable
{
  
public:
  
  enum SMCSensorMode
  {
    kSensorMode_Absolute = 0,
    kSensorMode_Derivative,
    kSensorMode_AbsAndDelayed,
    kNumSensorModes
  };
  
  SMCAgentMeta1d();
  ~SMCAgentMeta1d();
  
  // Inherited from Model
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  
  // Inherited from Evolvable
  virtual int getNumGenes();
  virtual void decodeGenome(const double* genome);
  virtual float getFitness();
  virtual void nextTrial(int trial = 0);
  virtual bool hasFinished();
  
  virtual void toXml(ci::XmlTree& xml);
  virtual void record(Recorder& recorder);
  
  // Setters
  void setMaxSensorDistance(float d) { m_distanceSensor.setMaxDistance(d); };
  void setMaxSpeed(float s) { m_maxSpeed = s; };
  void setMaxPosition(float p) { m_maxPosition = p; };
  void setPositionWraps(bool w) { m_positionWraps = w; };
  void setSensorMode(int mode) { m_sensorMode = mode; };
  void setSensorMode(const std::string& mode);
  void setPosition(float pos);
  void setEnergy(float e) { m_energy = e; };
  
  // Getters
  SMCEnvironment* getEnvironment() { return &m_environment; };
  const CTRNN* getCTRNN() { return m_ctrnn; };
  const Topology& getTopology() const { return m_topology; };
  const DistanceSensor& getDistanceSensor() const { return m_distanceSensor; };
  DistanceSensor& getDistanceSensor() { return m_distanceSensor; };
  
  float getPosition() { return m_position; };
  float getVelocity() const { return m_velocity; };
  float getSensedValue() { return m_sensedValue; };
  float getSensedFood() { return m_food; };
  float getTime() { return m_time; };
  float getMaxPosition() { return m_maxPosition; };
  float getMaxSpeed() { return m_maxSpeed; };
  float getEnergy() { return m_energy; };
  
protected:
  
  void updateFitness();
  void updateSensor(float dt);
  
  CTRNN* m_ctrnn;
  SMCEnvironment m_environment;
  Topology m_topology;
  DistanceSensor m_distanceSensor;
  int m_sensorMode;
  NetLimits m_netLimits;
  
  // States
  float m_position;
  float m_velocity;
  float m_sensedValue;
  float m_sensedValueDerivative;
  float m_energy;
  float m_food;
  float m_time;
  
  // Params
  float m_maxSpeed;
  float m_maxPosition;
  bool m_positionWraps;

  // Fitness related
  float m_fitness;
  float m_trialDuration;
  float m_fitnessEvalDelay;
};
  
} // namespace

#endif