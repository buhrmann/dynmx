/*
 *  LegBac.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/22/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_LEGBAC_
#define _DMX_LEGBAC_

#include "GARunner.h"
#include "Dynmx.h"
#include "Model.h"
#include "CTRNN.h"
#include "Topology.h"
#include "Recorder.h"
#include "BodyLeg.h"
#include "BodyBac.h"

#include "cinder/Vector.h"

namespace dmx
{
  
class LegBacViz;
class LegBacView;
  
//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class LegBac : public Evolvable
{
  
public:
  
  friend LegBacViz;
  friend LegBacView;
  
  LegBac();
  ~LegBac();
  
  // Inherited from Model
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  virtual void endOfEvaluation(float fit);
  
  // Inherited from Evolvable
  virtual int getNumGenes();
  virtual void processGenome(double* genome);
  virtual void decodeGenome(const double* genome);
  virtual float getFitness();
  virtual void nextTrial(int trial = 0);
  virtual bool hasFinished();
  
  virtual void toXml(ci::XmlTree& xml);
  virtual void record(Recorder& recorder);
  
  // Getters
  const CTRNN* getCTRNN() { return m_ctrnn; };
  const Topology* getTopology() const { return m_topology; };
  
  float getTime() { return m_time; };
  
  bool isLegged() const { return m_legged; };
  const BodyLeg& getLeg() const { return m_leg; };
  const BodyBac& getBac() const { return m_bac; };
  const ci::Vec2f& getPos() const { if (m_legged) return m_leg.m_position; else return m_bac.m_position; };
  
  
protected:
  
  CTRNN* m_ctrnn;
  Topology* m_topology;
  
  BodyLeg m_leg;
  BodyBac m_bac;

  bool m_adaptive;
  
  // States
  bool m_legged;
  float m_time;
  ci::Vec2f m_position;
  ci::Vec2f m_velocity;
  
  // Params
  
  // Fitness related
  int m_numLegTrials;
  int m_numBacTrials;
  float m_fitness;
  float m_legDuration;
  float m_legFitMax;
  float m_bacFitMax;
};
  
} // namespace

#endif