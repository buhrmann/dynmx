//
//  BacteriumEvo.h
//  dynmx
//
//  Created by Thomas Buhrmann on 27/02/14.
//
//

#ifndef dynmx_BacteriumEvo_h
#define dynmx_BacteriumEvo_h

#include "SMCAgentEvo.h"
#include "Bacterium.h"
#include "GARunner.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class BacteriumEvo : public SMCAgentEvo
{
  
public:
  
  enum PhFitAcc
  {
    kFitAcc_Avg,
    kFitAcc_Min,
    kFitAcc_Mult
  };
  
  BacteriumEvo(Bacterium* b) : SMCAgentEvo(b) { init(); nextTrial(0); };
  
  virtual void init();
  virtual void reset();
  virtual float getFitness();
  virtual bool hasFinished() { return m_phaseTime == 0 && m_phase==m_numPhases;};
  virtual void nextStage(int stage);
  virtual void nextTrial(int t);
  virtual void endOfEvaluation(float f) { m_trial = 0; };
  
  virtual void update(float dt);

protected:
  
  virtual void updateFitness(float dt);
  
  bool isInnerPhase();
  void nextPhase();
  void trialTaxisGrWidth(int t);
  void trialFoodPos(int t);
  
  int m_phase;
  int m_trial;
  float m_phaseDuration;
  int m_numPhases;
  float m_phaseTime;
  float m_phaseFit;
  int m_phaseFitAcc;
  std::vector<float> m_phaseFits;
  
  float m_phaseInitialDist;
  
  int m_numTests;
  int m_numEnvirons;
  
  float m_randInitProp;
};
  
} // namespace

#endif
