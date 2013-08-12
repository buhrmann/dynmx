//
//  SMCArm.h
//  dynmx
//
//  Created by Thomas Buhrmann on 8/9/13.
//
//

#ifndef dynmx_SMCArm_h
#define dynmx_SMCArm_h


#include "GARunner.h"
#include "Dynmx.h"
#include "Model.h"
#include "ArmPD.h"
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
class SMCArm : public Evolvable
{
public:
  
  enum SMCSensorMode
  {
    kSensorMode_Absolute = 0,
    kSensorMode_Derivative,
    kSensorMode_AbsAndDelayed,
    kNumSensorModes
  };
  
  SMCArm();
  ~SMCArm();
  
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
  void setSensorMode(int mode) { m_sensorMode = mode; };
  void setSensorMode(const std::string& mode);
  
  // Getters
  const ArmPD& getArm() const { return m_arm; };
  ArmPD& getArm() { return m_arm; };
  SMCEnvironment* getEnvironment() { return &m_environment; };
  const CTRNN* getCTRNN() { return m_ctrnn; };
  const Topology& getTopology() const { return m_topology; };
  const DistanceSensor& getDistanceSensor() const { return m_distanceSensor; };
  DistanceSensor& getDistanceSensor() { return m_distanceSensor; };
  
  float getSensedValue() { return m_sensedValue; };
  float getTime() { return m_time; };
  
protected:
  
  void updateFitness();
  void updateSensor(float dt);
  
  ArmPD m_arm;
  CTRNN* m_ctrnn;
  SMCEnvironment m_environment;
  Topology m_topology;
  DistanceSensor m_distanceSensor;
  int m_sensorMode;
  NetLimits m_netLimits;
  
  // States
  float m_sensedValue;
  float m_time;
  
  // Params
  
  
  // Fitness related
  float m_fitness;
  float m_trialDuration;
  float m_fitnessEvalDelay;

};
  
} // namespace
#endif
