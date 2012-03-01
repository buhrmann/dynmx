/*
 *  EvoArmCoConDecoder.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 2/16/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "EvoArmCoCon.h"

namespace dmx
{

// Returns number of genes decoded
//----------------------------------------------------------------------------------------------------------------------  
int EvoArmCoCon::decodeMuscle(int mId, const double* genome, int start)  
{
  Muscle* m = m_arm->getMuscle(mId);
  
  // Elbow muscles should be weaker than shoulder muscles
  bool isElbowMuscle = false;
  if(m->isMonoArticulate())
  {
    isElbowMuscle = ((MuscleMonoWrap*)m)->getJoint() == JT_elbow;
  }
  const float forceScalar = isElbowMuscle ? 0.75f : 1.0f;
    
  const DecodeLimits::MuscleLimits& limits = m_decodeLimits.muscle;
  const double origin = map01To(genome[start + 0], limits.attach);
  const double insertion =  map01To(genome[start + 1], limits.attach);
  const double maxIsoForce = map01To(forceScalar * genome[start + 2], limits.force); 
  const double optimalLength = map01To(genome[start + 3], limits.optLength);
  const double maxVel = map01To(genome[start + 4], limits.maxVel);
  m_arm->setMuscleParams(mId, origin, insertion, maxIsoForce, optimalLength, maxVel); 
  
  // Muscles need to be reinitialized so that correct values can be precalculated from new parameters
  if(m->isMonoArticulate())
  {
    ((MuscleMonoWrap*)m)->init();
  }
  else 
  {
    ((MuscleBiWrap*)m)->init();
  }
  
  // Need to set length again after initialisation of muscle to be able to use min, max lengths !
  double lOpt0 = m->unitLengthToLength(optimalLength);
  m->setParameters(maxIsoForce, lOpt0, maxVel);
  
  if(m_evolveHillParams)
  {
    double hSh = 0.1 + 0.4 * genome[start + 5];
    double hLn = 0.1 + 0.4 * genome[start + 6];
    double hMax = 1.1 + 0.7 * genome[start + 7];
    double hSlp = 2.0;
    m->setHillParameters(hSh, hLn, hMax, hSlp);
  }
  
  return m_evolveHillParams ? 8 : 5;
}
  
  
//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::decodeGenome(const double* genome)
{
  const int numMuscles = m_arm->getNumMuscles();
  const int numReflexes = m_arm->getNumReflexes();
  
  // Joint parameters
  double elbRad = 0.02 + 0.03 * genome[0];
  double shdRad = 0.02 + 0.03 * genome[1];
  m_arm->setJointRadii(elbRad, shdRad);  
  m_arm->setFriction(0.01 + 0.19 * genome[2], 0.01 + 0.19 * genome[3]);
  int start = 4;
  
  // Decode muscles
  if(m_symmetricMuscles)
  {
    // The two muscles in each reflex share the same parameters
    for(int i = 0; i < numReflexes; ++i)
    {
      int numDecoded = decodeMuscle(i*2, genome, start);
      decodeMuscle(i*2+1, genome, start);
      start += numDecoded;
    }
  }
  else
  {
    // Each muscles has its own parameters    
    for(int i = 0; i < numMuscles; ++i)
    {
      int numDecoded = decodeMuscle(i, genome, start);
      start += numDecoded;
    }
  }
  
  // Spindle parameters
  const DecodeLimits::SpindleLimits& lim = m_decodeLimits.spindle;
  if(m_evolveSpindles)
  {
    int pI = 0;
    int vI = 1;
    int dI = m_evolveVelRef ? 2 : 1;
    int eI = m_evolveVelRef ? 3 : 2;
    
    if(m_evolveUniformSpindles)
    {      
      m_arm->getReflex(0)->setSpindleParameters(map01To(genome[start + pI],lim.pos), map01To(genome[start + pI],lim.pos), 
                                                map01To(genome[start + vI],lim.vel), map01To(genome[start + vI],lim.vel), 
                                                map01To(genome[start + dI],lim.dmp), map01To(genome[start + dI],lim.dmp), 
                                                map01To(genome[start + eI],lim.exp), map01To(genome[start + eI],lim.exp));  
      start += m_evolveVelRef ? 4 : 3;

      m_arm->getReflex(1)->setSpindleParameters(map01To(genome[start + pI],lim.pos), map01To(genome[start + pI],lim.pos), 
                                                map01To(genome[start + vI],lim.vel), map01To(genome[start + vI],lim.vel), 
                                                map01To(genome[start + dI],lim.dmp), map01To(genome[start + dI],lim.dmp), 
                                                map01To(genome[start + eI],lim.exp), map01To(genome[start + eI],lim.exp));        
      start += m_evolveVelRef ? 4 : 3;
    }
    else 
    {
      int ofs = m_evolveVelRef ? 4 : 3;
      m_arm->getReflex(0)->setSpindleParameters(map01To(genome[start + pI],lim.pos), map01To(genome[start + ofs + pI],lim.pos), 
                                                map01To(genome[start + vI],lim.vel), map01To(genome[start + ofs + vI],lim.vel), 
                                                map01To(genome[start + dI],lim.dmp), map01To(genome[start + ofs + dI],lim.dmp), 
                                                map01To(genome[start + eI],lim.exp), map01To(genome[start + ofs + eI],lim.exp));  
      
      start += m_evolveVelRef ? 8 : 6;
      
      m_arm->getReflex(1)->setSpindleParameters(map01To(genome[start + pI],lim.pos), map01To(genome[start + ofs + pI],lim.pos), 
                                                map01To(genome[start + vI],lim.vel), map01To(genome[start + ofs + vI],lim.vel), 
                                                map01To(genome[start + dI],lim.dmp), map01To(genome[start + ofs + dI],lim.dmp), 
                                                map01To(genome[start + eI],lim.exp), map01To(genome[start + ofs + eI],lim.exp));        
      start += m_evolveVelRef ? 8 : 6;  
    }
    
    // Alpha MN params: input from spindles
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setMotoNeuronParameters(map01To(genome[start+0],lim.weight), map01To(genome[start+0],lim.weight)); 
      m_arm->getReflex(1)->setMotoNeuronParameters(map01To(genome[start+1],lim.weight), map01To(genome[start+1],lim.weight));
      start += 2;
    }
    else 
    {
      m_arm->getReflex(0)->setMotoNeuronParameters(map01To(genome[start+0],lim.weight), map01To(genome[start+1],lim.weight)); 
      m_arm->getReflex(1)->setMotoNeuronParameters(map01To(genome[start+2],lim.weight), map01To(genome[start+3],lim.weight));
      start += 4;    
    }
  }  
  
  // Range for neural parameters
  float maxW = 10.0;
  float maxB = 10.0;
  float maxT = 100.0;  
  
  // IaIn parameters  
  if(m_evolveIAIN)
  {
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setIaInParameters(genome[start+0] * maxW, genome[start+0] * maxW,        // spindle->ia
                                             genome[start+1] * maxW, genome[start+1] * maxW,        // ia->ia
                                             genome[start+2] * maxW, genome[start+2] * maxW,        // desired contr->ia
                                             genome[start+3] * maxW, genome[start+3] * maxW,        // rn->ia
                                             genome[start+4] * maxW, genome[start+4] * maxW,        // ia->mn
                                             10 + genome[start+5] * maxT, 10 + genome[start+5] * maxT,        // time constants
                                             -maxB + (genome[start+6] * 2 * maxB), -maxB + (genome[start+6] * 2 * maxB));       // biases
      start += 7;
      
      m_arm->getReflex(1)->setIaInParameters(genome[start+0] * maxW, genome[start+0] * maxW,        // spindle->ia
                                             genome[start+1] * maxW, genome[start+1] * maxW,        // ia->ia
                                             genome[start+2] * maxW, genome[start+2] * maxW,        // desired contr->ia
                                             genome[start+3] * maxW, genome[start+3] * maxW,        // rn->ia
                                             genome[start+4] * maxW, genome[start+4] * maxW,        // ia->mn
                                             10 + genome[start+5] * maxT, 10 + genome[start+5] * maxT,        // time constants
                                             -maxB + (genome[start+6] * 2 * maxB), -maxB + (genome[start+6] * 2 * maxB));       // biases
      start += 7;
    }
    else 
    {
      m_arm->getReflex(0)->setIaInParameters(genome[start+0] * maxW, genome[start+7] * maxW,        // spindle->ia
                                             genome[start+1] * maxW, genome[start+8] * maxW,        // ia->ia
                                             genome[start+2] * maxW, genome[start+9] * maxW,        // desired contr->ia
                                             genome[start+3] * maxW, genome[start+10] * maxW,        // rn->ia
                                             genome[start+4] * maxW, genome[start+11] * maxW,        // ia->mn
                                             10 + genome[start+5] * maxT, 10 + genome[start+12] * maxT,        // time constants
                                             -maxB + (genome[start+6] * 2 * maxB), -maxB + (genome[start+13] * 2 * maxB));       // biases
      start += 14;
      
      m_arm->getReflex(1)->setIaInParameters(genome[start+0] * maxW, genome[start+7] * maxW,        // spindle->ia
                                             genome[start+1] * maxW, genome[start+8] * maxW,        // ia->ia
                                             genome[start+2] * maxW, genome[start+9] * maxW,        // desired contr->ia
                                             genome[start+3] * maxW, genome[start+10] * maxW,        // rn->ia
                                             genome[start+4] * maxW, genome[start+11] * maxW,        // ia->mn
                                             10 + genome[start+5] * maxT, 10 + genome[start+12] * maxT,        // time constants
                                             -maxB + (genome[start+6] * 2 * maxB), -maxB + (genome[start+13] * 2 * maxB));       // biases
      start += 14;      
    }
  }
  
  
  // Renshaw param
  if(m_evolveRenshaw)
  {
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setRenshawParameters(genome[start+0], genome[start+0], 
                                                genome[start+1], genome[start+1], 
                                                genome[start+2], genome[start+2], 
                                                10 + genome[start+3] * maxT, 10 + genome[start+3] * maxT,
                                                -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+4] * 2 * maxB));  
      start += 5;
      
      m_arm->getReflex(1)->setRenshawParameters(genome[start+0], genome[start+0], 
                                                genome[start+1], genome[start+1], 
                                                genome[start+2], genome[start+2], 
                                                10 + genome[start+3] * maxT, 10 + genome[start+3] * maxT,
                                                -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+4] * 2 * maxB));  
      start += 5;  
    }
    else 
    {      
      m_arm->getReflex(0)->setRenshawParameters(genome[start+0], genome[start+5], 
                                                genome[start+1], genome[start+6], 
                                                genome[start+2], genome[start+7], 
                                                10 + genome[start+3] * maxT, 10 + genome[start+8] * maxT,
                                                -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+9] * 2 * maxB));  
      start += 10;
      
      m_arm->getReflex(1)->setRenshawParameters(genome[start+0], genome[start+5], 
                                                genome[start+1], genome[start+6], 
                                                genome[start+2], genome[start+7], 
                                                10 + genome[start+3] * maxT, 10 + genome[start+8] * maxT,
                                                -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+9] * 2 * maxB));  
      start += 10;
      
    }

  }
  
  // IbIn params
  if(m_evolveIBIN)
  {
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setIbInParameters(genome[start+0], genome[start+0], 
                                             genome[start+1], genome[start+1], 
                                             genome[start+2], genome[start+2], 
                                             10 + genome[start+3] * maxT, 10 + genome[start+3] * maxT,
                                             -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+4] * 2 * maxB)); 
      start += 5;
      
      m_arm->getReflex(1)->setIbInParameters(genome[start+0], genome[start+0], 
                                             genome[start+1], genome[start+1], 
                                             genome[start+2], genome[start+2], 
                                             10 + genome[start+3] * maxT, 10 + genome[start+3] * maxT,
                                             -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+4] * 2 * maxB));
      start += 5;    
    }
    else
    {
      m_arm->getReflex(0)->setIbInParameters(genome[start+0], genome[start+5], 
                                             genome[start+1], genome[start+6], 
                                             genome[start+2], genome[start+7], 
                                             10 + genome[start+3] * maxT, 10 + genome[start+8] * maxT,
                                             -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+9] * 2 * maxB)); 
      start += 10;

      m_arm->getReflex(1)->setIbInParameters(genome[start+0], genome[start+5], 
                                             genome[start+1], genome[start+6], 
                                             genome[start+2], genome[start+7], 
                                             10 + genome[start+3] * maxT, 10 + genome[start+8] * maxT,
                                             -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+9] * 2 * maxB)); 
      start += 10;
      
    }
  }
  
  // IFV gains
  if(m_evolveIFV)
  {
    const float maxIC = 10.0f;
    const float maxICb = 1.0f;
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setInertiaCompensationParameters(genome[start+0] * maxIC, genome[start+0] * maxIC, 
                                                            genome[start+1] * maxICb, genome[start+1] * maxICb);
      
      m_arm->getReflex(1)->setInertiaCompensationParameters(genome[start+2] * maxIC, genome[start+2] * maxIC, 
                                                            genome[start+3] * maxICb, genome[start+3] * maxICb);  
      start += 4;
    }
    else
    {
      m_arm->getReflex(0)->setInertiaCompensationParameters(genome[start+0] * maxIC, genome[start+2] * maxIC, 
                                                            genome[start+1] * maxICb, genome[start+3] * maxICb);
      
      m_arm->getReflex(1)->setInertiaCompensationParameters(genome[start+4] * maxIC, genome[start+6] * maxIC, 
                                                            genome[start+5] * maxICb, genome[start+7] * maxICb);  
      start += 8;
    }
  }
  
  // Open loop activations: one per muscle per move
  // Decoded vector is arranged like this: [mv1m1 mv1m2 mv1m3 ... mv1mM | mv2m1 ... | mvNmM]
  if(m_evolveOpenLoop)
  {
    m_openLoopParams.clear(); // This instance could repeatedly be decoded!
    
    const int N = m_symmetricMuscles ? (m_numMoves * numReflexes) : (m_numMoves * numMuscles);
    for (int i = 0; i < N; ++i)
    {
      m_openLoopParams.push_back(m_maxOpenLoop * genome[start + i]);
      
      if(m_symmetricMuscles)
      {
        // We have to duplicate each muscle activation (one encoded, but two identical needed for symmetry)
        m_openLoopParams.push_back(m_maxOpenLoop * genome[start + i]);
      }
    }
    start += N;
  }
  
  // Duration of commanded ramp as proportion of desired movement time
  m_rampDurationFactor = 0.4 + 0.6 * genome[start];
  start += 1;
  
  // Minimum co-contraction
  if(m_enableCoconIncrease)
  {
    m_minCocontraction = 0.05 + 0.15 * genome[start];
    start += 1;
  }
  
  // Intersegmental inputs
  // Encoding works essentially the same as openLoop
  if(m_evolveIntersegmentInputs)
  { 
    m_intersegParams.clear(); // This instance could repeatedly be decoded!
    
    int N = m_symmetricMuscles ? (m_numMoves * numReflexes) : (m_numMoves * numMuscles);
    for(int i = 0; i < N; ++i)
    {
      double Wisep = map01To(genome[start + i], 0.0, m_maxInterseg);
      m_intersegParams.push_back(Wisep);
      
      // We have to duplicate each muscle activation (one encoded, but two identical needed for symmetry)
      if(m_symmetricMuscles)
      {
        m_intersegParams.push_back(Wisep);
      }
    }
    start += N;
  }
  
  assert(start == getNumGenes());
  
  createTrajectories();
};  
  
//----------------------------------------------------------------------------------------------------------------------
int EvoArmCoCon::getNumGenes() 
{ 
  int numGenes = 0;
  const int numReflexes = m_arm->getNumReflexes();
  const int numMuscles = m_arm->getNumMuscles();
  
  // Ramp durationfactor: duration of commanded ramp as proportion of desired movement time
  numGenes += 1;
  
  // 2 X joint radii, 2 x friction
  numGenes += 2 + 2;
  
  // 5 muscle params (L0, Fmax, vmax, origin, insertion)
  if(m_symmetricMuscles)
    numGenes += numReflexes * 5;
  else
    numGenes += numMuscles * 5;
  
  // Muscle hill parameters (hSh, hLn, hMax)
  if(m_evolveHillParams)
  {
    if(m_symmetricMuscles)
      numGenes += numReflexes * 3;  
    else
      numGenes += numMuscles * 3;
  }
  
  if(m_evolveSpindles)
  {
    // Spindle gains: p,v,d, exponent
    const int numSpindleParams = m_evolveVelRef ? 4 : 3;
    if(m_evolveUniformSpindles)
      numGenes += numReflexes * numSpindleParams; // Spindles differ only between elbow and shoulder
    else
      numGenes += numMuscles * numSpindleParams;
    
    // Weights to aMN
    if(m_symmetricMuscles)
      numGenes += numReflexes;
    else
      numGenes += numMuscles;
  }
  
  // Min cocontraction throughout rest and movement periods
  if(m_enableCoconIncrease)
    numGenes += 1;
  
  // Parameters for IaIn neurons: Bias, tau, weights from ia, rn, sp, amn. Plus input to MN.
  if(m_evolveIAIN)
  {
    if(m_symmetricMuscles)
      numGenes += numReflexes * 7;
    else
      numGenes += numMuscles * 7;
  }
  
  // Renshaw neurons: Bias, tau, weights from mn, rn. Plus input to MN.
  if(m_evolveRenshaw)
  {
    if(m_symmetricMuscles)
      numGenes += numReflexes * 5;
    else
      numGenes += numMuscles * 5;
  }
  
  // IbIn neurons: Bias, tau, weights from golgi, ib. Plus input to MN.
  if(m_evolveIBIN)
  {
    if(m_symmetricMuscles)
      numGenes += numReflexes * 5;
    else
      numGenes += numMuscles * 5;
  }
  
  // Parameters of ifv neurons
  if(m_evolveIFV)
  {
    if(m_symmetricMuscles)
      numGenes += numReflexes * 2;
    else
      numGenes += numMuscles * 2;
  }
  
  // Open-loop activation (one set for each target pose)
  if(m_evolveOpenLoop)
  {
    if(m_symmetricMuscles)
      numGenes += numReflexes * m_numMoves;
    else
      numGenes += numMuscles * m_numMoves;  
  }
  
  // Intersegmental inputs (weight to mn)
  if(m_evolveIntersegmentInputs)
  {
    if(m_symmetricMuscles)
      numGenes += m_numMoves * numReflexes;
    else
      numGenes += m_numMoves * numMuscles;
  }
  
  return numGenes;
};
  
  
}
