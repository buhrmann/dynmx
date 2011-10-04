/*
 *  TestModelEvolvableArm.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/12/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TEST_MODEL_EVOLVABLE_ARM_
#define _DMX_TEST_MODEL_EVOLVABLE_ARM_

#include "Dynmx.h"
#include "Model.h"
#include "GARunner.h"
#include "Arm.h"
#include "MuscleMonoWrap.h"
#include "MuscleBiWrap.h"
#include "ArmMuscled.h"
#include "EPController.h"


//----------------------------------------------------------------------------------------------------------------------
class TestModelEvolvableArm : public dmx::Evolvable
{
public:
  
  TestModelEvolvableArm() : m_arm(0), m_time(0) { m_arm = new dmx::ArmMuscled(); init();};
  
  // Implementation of dmx::Evolvable

  // 2 X joint radii, 4 muscle params (L0, Fmax, origin, insertion), 3 feedback gains
  virtual int getNumGenes() { return 2 + m_arm->getNumMuscles() * (4 + 3); };
  
  virtual void decodeGenome(const double* genome)
  {
    double elbRad = 0.02 + 0.03 * genome[0];
    double shdRad = 0.02 + 0.03 * genome[1];
    m_arm->setJointRadii(elbRad, shdRad);

    int start = 2;
    for(int i = 0; i < m_arm->getNumMuscles(); ++i)
    {
      // Muscle parameters
      //double origin = m_arm->getMuscle(i)->getOrigin();
      //double insertion = m_arm->getMuscle(i)->getInsertion();      
      double origin = 0.1 + 0.15 * genome[start + 0];
      double insertion =  0.075 + 0.125 * genome[start + 1];
      double maxIsoForce = 2500 * genome[start + 2]; 
      double optimalLength = 0.1 + 0.9 * genome[start + 3];          
      m_arm->setMuscleParams(i, origin, insertion, maxIsoForce, optimalLength, 10.0); 
      if(m_arm->getMuscle(i)->isMonoArticulate())
      {
        ((dmx::MuscleMonoWrap*)m_arm->getMuscle(i))->init();
      }
      else 
      {
        ((dmx::MuscleBiWrap*)m_arm->getMuscle(i))->init();        
      }

      // Reflex parameters
      double pGain = 30.0 * genome[start + 4];
      double vGain = 20.0 * genome[start + 5];
      double dGain = 10.0 * genome[start + 6];
      m_arm->getEPController(i)->setGains(pGain, vGain, dGain);
      start += 7;
    }
  };
  
  virtual float getFitness() { return -sqrt(m_fitness / (m_time / (1.0/30.0))); };
  
  
  virtual void init() 
  { 
    assert(m_arm); 
    m_arm->init(); 
    m_saved = false;
    m_fitness = 0.0f;
    
    reset();    
  }
  
  virtual void reset()
  {
    // Calculate starting position
    dmx::Pos start (0.4+0.15, 0.0);
    double initialElbowAngle = 0.0f * PI;
    double initialShoulderAngle = 0.0f * PI;    
    m_arm->inverseKinematics(start, 1, initialElbowAngle, initialShoulderAngle);
    m_arm->reset(initialElbowAngle, initialShoulderAngle);
    m_fitness = 0.0f;
    m_time = 0.0f;
  }

  
  virtual void update(float dt)
  { 
    m_time += dt;
    
    const float radius = 0.15f;
    const float offset = 0.4f;    
    
    double angle = PI_OVER_TWO + 1.0f * m_time;
    float x = offset + radius * sinf(angle);
    float y = radius * cosf(angle);
    dmx::Pos desPos(x,y);
    m_arm->updatePosition(desPos, dt);
    
    // Fitness evaluation
    dmx::Pos diff = m_arm->getEffectorPos() - desPos;
    float dist = diff.length();
    m_fitness += sqr(dist);
  };
  
  virtual void fromXML()
  {
    if(dmx::SETTINGS->hasChild("Config/GA/Eval"))
    {
      std::string fnm = dmx::SETTINGS->getChild("Config/GA/Eval/LoadFrom").getAttributeValue<std::string>("Value");
      ci::XmlTree prevResults(ci::loadFile(fnm));
      fromXML(prevResults);
    }
  }
  
  virtual void fromXML(const ci::XmlTree& xml)
  {
    assert(xml.hasChild("GAProgress"));
    
    ci::XmlTree::ConstIter lastGeneration = xml.begin("GAProgress/Generation");
    for(ci::XmlTree::ConstIter generation = xml.begin("GAProgress/Generation"); generation != xml.end(); ++generation)
    {
      lastGeneration = generation;
    }
    const ci::XmlTree& genome = lastGeneration->getChild("BestGenome");
    int numGenes = xml.getChild("GAProgress").getAttributeValue<int>("NumGenes");
    double genes[numGenes];
    int i = 0;
    for(ci::XmlTree::ConstIter gene = genome.begin(); gene != genome.end(); ++gene)
    {
      genes[i] = gene->getAttributeValue<float>("Value");
      i++;
    }
    decodeGenome(&genes[0]);
  }
  
  
  dmx::ArmMuscled* m_arm;
  bool m_saved;
  float m_time;
  float m_fitness;
};

#endif