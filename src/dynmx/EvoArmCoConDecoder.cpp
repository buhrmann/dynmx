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
int EvoArmCoCon::decodeMuscle(int mId, const double* genome, int I)  
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
  const double origin = map01To(genome[I + 0], limits.attach);
  const double insertion =  map01To(genome[I + 1], limits.attach);
  const double maxIsoForce = map01To(forceScalar * genome[I + 2], limits.force); 
  const double optimalLength = map01To(genome[I + 3], limits.optLength);
  const double maxVel = map01To(genome[I + 4], limits.maxVel);
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
    double hSh = 0.1 + 0.4 * genome[I + 5];
    double hLn = 0.1 + 0.4 * genome[I + 6];
    double hMax = 1.1 + 0.7 * genome[I + 7];
    double hSlp = 2.0;
    m->setHillParameters(hSh, hLn, hMax, hSlp);
  }
  
  return m_evolveHillParams ? 8 : 5;
}
  
  
//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::decodeGenome(const double* genome)
{
  int I = 0;
  
  const int numMuscles = m_arm->getNumMuscles();
  const int numReflexes = m_arm->getNumReflexes();
  
  // Joint friction
  if(m_evolveFriction)
  {  
    double elbFriction = map01To(genome[I++], m_decodeLimits.arm.jointFriction);
    double shdFriction = map01To(genome[I++], m_decodeLimits.arm.jointFriction);  
    m_arm->setFriction(elbFriction, shdFriction);
  }
  
  // Decode muscles
  if(m_evolveMuscles)
  {
    // Joint parameters
    double elbRad = map01To(genome[I++], m_decodeLimits.arm.jointRadius);
    double shdRad = map01To(genome[I++], m_decodeLimits.arm.jointRadius);  
    m_arm->setJointRadii(elbRad, shdRad);  
    
    if(m_symmetricMuscles)
    {
      // The two muscles in each reflex share the same parameters
      for(int i = 0; i < numReflexes; ++i)
      {
        int numDecoded = decodeMuscle(i*2, genome, I);
        decodeMuscle(i*2+1, genome, I);
        I += numDecoded;
      }
    }
    else
    {
      // Each muscles has its own parameters    
      for(int i = 0; i < numMuscles; ++i)
      {
        int numDecoded = decodeMuscle(i, genome, I);
        I += numDecoded;
      }
    }
  }
//  else 
//  {
//    // Need to init, as otherwise the new jointRadii are not compatible with precalculated values
//    for(int i = 0; i < numMuscles; ++i)
//    {
//      m_arm->getMuscle(i)->init();
//    }
//  }

  
  // Spindle parameters
  const DecodeLimits::SpindleLimits& lim = m_decodeLimits.spindle;
  if(m_evolveSpindles)
  {
    int pI = 0;
    int dI = 1;
    int edI = 2;
    int vI = 3;
    int evI = 4;
    
    int numSpindleParams = m_evolveVelRef ? 5 : 3;
    
    // 1: identical for antagonists. 2: identical for all muscles.
    if(m_evolveUniformSpindles >= 1)
    {      
      double kp = map01To(genome[I + pI], lim.pos);
      double kd = map01To(genome[I + dI], lim.dmp);
      double expD = map01To(genome[I + edI], lim.exp);
      double kv = m_evolveVelRef ? map01To(genome[I + vI], lim.vel) : 0.0;      
      double expV = m_evolveVelRef ? map01To(genome[I + evI], lim.exp) : 1.0;
      
      m_arm->getReflex(0)->setSpindleParameters(kp, kp, kv, kv, kd, kd, expV, expV, expD, expD);

      // Increase for next reflex, otherwise use same parameters
      if(m_evolveUniformSpindles == 1)
      {
        I += numSpindleParams;
      }

      kp = map01To(genome[I + pI], lim.pos);
      kd = map01To(genome[I + dI], lim.dmp);
      expD = map01To(genome[I + edI], lim.exp);
      kv = m_evolveVelRef ? map01To(genome[I + vI], lim.vel) : 0.0;      
      expV = m_evolveVelRef ? map01To(genome[I + evI], lim.exp) : 1.0;
      
      m_arm->getReflex(1)->setSpindleParameters(kp, kp, kv, kv, kd, kd, expV, expV, expD, expD);
      
      I += numSpindleParams;
    }
    else 
    {
      int ofs = numSpindleParams;
      double kp[2] = { map01To(genome[I + pI], lim.pos), map01To(genome[I + ofs + pI], lim.pos) };
      double kd[2] = { map01To(genome[I + dI], lim.dmp), map01To(genome[I + ofs + dI], lim.dmp) };
      double expD[2] = { map01To(genome[I + edI], lim.exp), map01To(genome[I + ofs + edI], lim.exp) };
      double kv[2] = {0,0};
      double expV[2] = {1,1};
      if(m_evolveVelRef)
      {
        kv[0] = map01To(genome[I + vI], lim.vel); kv[1] = map01To(genome[I + ofs + vI], lim.vel);        
        expV[0] = map01To(genome[I + evI], lim.exp); expV[1] = map01To(genome[I + ofs + evI], lim.exp);;        
      }      
      
      m_arm->getReflex(0)->setSpindleParameters(kp[0], kp[1], kv[0], kv[1], kd[0], kd[1], expV[0], expV[1], expD[0], expD[1]);
      I += 2 * numSpindleParams;
      
      // Same again for next reflex
      {
        double kp[2] = { map01To(genome[I + pI], lim.pos), map01To(genome[I + ofs + pI], lim.pos) };
        double kd[2] = { map01To(genome[I + dI], lim.dmp), map01To(genome[I + ofs + dI], lim.dmp) };
        double expD[2] = { map01To(genome[I + edI], lim.exp), map01To(genome[I + ofs + edI], lim.exp) };
        double kv[2] = {0,0};
        double expV[2] = {1,1};
        if(m_evolveVelRef)
        {
          kv[0] = map01To(genome[I + vI], lim.vel); kv[1] = map01To(genome[I + ofs + vI], lim.vel);        
          expV[0] = map01To(genome[I + evI], lim.exp); expV[1] = map01To(genome[I + ofs + evI], lim.exp);;        
        }      
        
        m_arm->getReflex(1)->setSpindleParameters(kp[0], kp[1], kv[0], kv[1], kd[0], kd[1], expV[0], expV[1], expD[0], expD[1]);
        I += 2 * numSpindleParams;
      }
    }
    
    // Alpha MN params: input from spindles
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setMotoNeuronParameters(map01To(genome[I+0],lim.weight), map01To(genome[I+0],lim.weight)); 
      m_arm->getReflex(1)->setMotoNeuronParameters(map01To(genome[I+1],lim.weight), map01To(genome[I+1],lim.weight));
      I += 2;
    }
    else 
    {
      if(m_evolveUniformSpindleWeights > 0)
      {
        m_arm->getReflex(0)->setMotoNeuronParameters(map01To(genome[I+0],lim.weight), map01To(genome[I+0],lim.weight)); 
        m_arm->getReflex(1)->setMotoNeuronParameters(map01To(genome[I+1],lim.weight), map01To(genome[I+1],lim.weight));
        I += 2;
      }
      else
      {
        m_arm->getReflex(0)->setMotoNeuronParameters(map01To(genome[I+0],lim.weight), map01To(genome[I+1],lim.weight)); 
        m_arm->getReflex(1)->setMotoNeuronParameters(map01To(genome[I+2],lim.weight), map01To(genome[I+3],lim.weight));
        I += 4;    
      }
    }
  }  
  
  // Range for neural parameters
  float maxW = 10.0;
  float maxB = 10.0;
  float maxT = 200.0;  
  
  // IaIn parameters  
  if(m_evolveIAIN)
  {
    const int numIaParams = m_evolveIAINsimple ? 4 : 7;
    if(m_evolveIAINsym)
    {
      if(m_evolveIAINsimple)
      {
        // Symmetric and simple
        m_arm->getReflex(0)->setIaInParametersMinimal(genome[I+0] * maxW, genome[I+0] * maxW,        // spindle->ia
                                               genome[I+1] * maxW, genome[I+1] * maxW,        // ia->ia
                                               genome[I+2] * maxW, genome[I+2] * maxW,        // ia->mn
                                               10 + genome[I+3] * maxT, 10 + genome[I+3] * maxT);  // time constants
        I += numIaParams;
        
        m_arm->getReflex(1)->setIaInParametersMinimal(genome[I+0] * maxW, genome[I+0] * maxW,        // spindle->ia
                                               genome[I+1] * maxW, genome[I+1] * maxW,        // ia->ia
                                               genome[I+4] * maxW, genome[I+4] * maxW,        // ia->mn
                                               10 + genome[I+3] * maxT, 10 + genome[I+3] * maxT);  // time constants
        I += numIaParams;
      }
      else 
      {
        // Symmetric and complex
        m_arm->getReflex(0)->setIaInParameters(genome[I+0] * maxW, genome[I+0] * maxW,        // spindle->ia
                                               genome[I+1] * maxW, genome[I+1] * maxW,        // ia->ia
                                               genome[I+2] * maxW, genome[I+2] * maxW,        // desired contr->ia
                                               genome[I+3] * maxW, genome[I+3] * maxW,        // rn->ia
                                               genome[I+4] * maxW, genome[I+4] * maxW,        // ia->mn
                                               10 + genome[I+5] * maxT, 10 + genome[I+5] * maxT,        // time constants
                                               -maxB + (genome[I+6] * 2 * maxB), -maxB + (genome[I+6] * 2 * maxB));       // biases
        I += numIaParams;
        
        m_arm->getReflex(1)->setIaInParameters(genome[I+0] * maxW, genome[I+0] * maxW,        // spindle->ia
                                               genome[I+1] * maxW, genome[I+1] * maxW,        // ia->ia
                                               genome[I+2] * maxW, genome[I+2] * maxW,        // desired contr->ia
                                               genome[I+3] * maxW, genome[I+3] * maxW,        // rn->ia
                                               genome[I+4] * maxW, genome[I+4] * maxW,        // ia->mn
                                               10 + genome[I+5] * maxT, 10 + genome[I+5] * maxT,        // time constants
                                               -maxB + (genome[I+6] * 2 * maxB), -maxB + (genome[I+6] * 2 * maxB));       // biases
        I += numIaParams;
      }
    }
    else 
    {
      if(m_evolveIAINsimple)
      {
        // Non-symmetric and simple
        m_arm->getReflex(0)->setIaInParametersMinimal(genome[I+0] * maxW, genome[I+4] * maxW,        // spindle->ia
                                               genome[I+1] * maxW, genome[I+5] * maxW,        // ia->ia
                                               genome[I+2] * maxW, genome[I+6] * maxW,        // ia->mn
                                               10 + genome[I+3] * maxT, 10 + genome[I+7] * maxT);  // time constants
        I += 2 * numIaParams;
        
        m_arm->getReflex(1)->setIaInParametersMinimal(genome[I+0] * maxW, genome[I+4] * maxW,        // spindle->ia
                                               genome[I+1] * maxW, genome[I+5] * maxW,        // ia->ia
                                               genome[I+2] * maxW, genome[I+6] * maxW,        // ia->mn
                                               10 + genome[I+3] * maxT, 10 + genome[I+7] * maxT);  // time constants
        I += 2 * numIaParams;         
      }
      else
      {
        // Non-symmetric and complex
        m_arm->getReflex(0)->setIaInParameters(genome[I+0] * maxW, genome[I+7] * maxW,        // spindle->ia
                                               genome[I+1] * maxW, genome[I+8] * maxW,        // ia->ia
                                               genome[I+2] * maxW, genome[I+9] * maxW,        // desired contr->ia
                                               genome[I+3] * maxW, genome[I+10] * maxW,        // rn->ia
                                               genome[I+4] * maxW, genome[I+11] * maxW,        // ia->mn
                                               10 + genome[I+5] * maxT, 10 + genome[I+12] * maxT,        // time constants
                                               -maxB + (genome[I+6] * 2 * maxB), -maxB + (genome[I+13] * 2 * maxB));       // biases
        I += 2 * numIaParams;
        
        m_arm->getReflex(1)->setIaInParameters(genome[I+0] * maxW, genome[I+7] * maxW,        // spindle->ia
                                               genome[I+1] * maxW, genome[I+8] * maxW,        // ia->ia
                                               genome[I+2] * maxW, genome[I+9] * maxW,        // desired contr->ia
                                               genome[I+3] * maxW, genome[I+10] * maxW,        // rn->ia
                                               genome[I+4] * maxW, genome[I+11] * maxW,        // ia->mn
                                               10 + genome[I+5] * maxT, 10 + genome[I+12] * maxT,        // time constants
                                               -maxB + (genome[I+6] * 2 * maxB), -maxB + (genome[I+13] * 2 * maxB));       // biases
        I += 2 * numIaParams;     
      }
    }
  }
  
  
  // Renshaw param
  if(m_evolveRenshaw)
  {
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setRenshawParameters(genome[I+0], genome[I+0], 
                                                genome[I+1], genome[I+1], 
                                                genome[I+2], genome[I+2], 
                                                10 + genome[I+3] * maxT, 10 + genome[I+3] * maxT,
                                                -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+4] * 2 * maxB));  
      I += 5;
      
      m_arm->getReflex(1)->setRenshawParameters(genome[I+0], genome[I+0], 
                                                genome[I+1], genome[I+1], 
                                                genome[I+2], genome[I+2], 
                                                10 + genome[I+3] * maxT, 10 + genome[I+3] * maxT,
                                                -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+4] * 2 * maxB));  
      I += 5;  
    }
    else 
    {      
      m_arm->getReflex(0)->setRenshawParameters(genome[I+0], genome[I+5], 
                                                genome[I+1], genome[I+6], 
                                                genome[I+2], genome[I+7], 
                                                10 + genome[I+3] * maxT, 10 + genome[I+8] * maxT,
                                                -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+9] * 2 * maxB));  
      I += 10;
      
      m_arm->getReflex(1)->setRenshawParameters(genome[I+0], genome[I+5], 
                                                genome[I+1], genome[I+6], 
                                                genome[I+2], genome[I+7], 
                                                10 + genome[I+3] * maxT, 10 + genome[I+8] * maxT,
                                                -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+9] * 2 * maxB));  
      I += 10;
      
    }

  }
  
  // IbIn params
  if(m_evolveIBIN)
  {
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setIbInParameters(genome[I+0], genome[I+0], 
                                             genome[I+1], genome[I+1], 
                                             genome[I+2], genome[I+2], 
                                             10 + genome[I+3] * maxT, 10 + genome[I+3] * maxT,
                                             -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+4] * 2 * maxB)); 
      I += 5;
      
      m_arm->getReflex(1)->setIbInParameters(genome[I+0], genome[I+0], 
                                             genome[I+1], genome[I+1], 
                                             genome[I+2], genome[I+2], 
                                             10 + genome[I+3] * maxT, 10 + genome[I+3] * maxT,
                                             -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+4] * 2 * maxB));
      I += 5;    
    }
    else
    {
      m_arm->getReflex(0)->setIbInParameters(genome[I+0], genome[I+5], 
                                             genome[I+1], genome[I+6], 
                                             genome[I+2], genome[I+7], 
                                             10 + genome[I+3] * maxT, 10 + genome[I+8] * maxT,
                                             -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+9] * 2 * maxB)); 
      I += 10;

      m_arm->getReflex(1)->setIbInParameters(genome[I+0], genome[I+5], 
                                             genome[I+1], genome[I+6], 
                                             genome[I+2], genome[I+7], 
                                             10 + genome[I+3] * maxT, 10 + genome[I+8] * maxT,
                                             -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+9] * 2 * maxB)); 
      I += 10;
      
    }
  }
  
  // IFV gains
  if(m_evolveIFV)
  {
    const float maxIC = 10.0f;
    const float maxICb = 1.0f;
    if(m_symmetricMuscles)
    {
      m_arm->getReflex(0)->setInertiaCompensationParameters(genome[I+0] * maxIC, genome[I+0] * maxIC, 
                                                            genome[I+1] * maxICb, genome[I+1] * maxICb);
      
      m_arm->getReflex(1)->setInertiaCompensationParameters(genome[I+2] * maxIC, genome[I+2] * maxIC, 
                                                            genome[I+3] * maxICb, genome[I+3] * maxICb);  
      I += 4;
    }
    else
    {
      m_arm->getReflex(0)->setInertiaCompensationParameters(genome[I+0] * maxIC, genome[I+2] * maxIC, 
                                                            genome[I+1] * maxICb, genome[I+3] * maxICb);
      
      m_arm->getReflex(1)->setInertiaCompensationParameters(genome[I+4] * maxIC, genome[I+6] * maxIC, 
                                                            genome[I+5] * maxICb, genome[I+7] * maxICb);  
      I += 8;
    }
  }
  
  // Open loop activations: one per muscle per move
  // Decoded vector is arranged like this: [mv1m1 mv1m2 mv1m3 ... mv1mM | mv2m1 ... | mvNmM]
  if(m_evolveOpenLoop)
  {
    m_openLoopParams.clear(); // This instance could repeatedly be decoded!
    
    int numDecoded = m_numMoves * numMuscles;
    int numEncoded = m_numMoves;
    if(m_openLoopSymmetry == 1) 
    {
      numEncoded *= numReflexes;
    }
    else if(m_openLoopSymmetry == 0)
    {
      numEncoded *= numMuscles;
    }
       
    int numCopies = numDecoded / numEncoded;

    for (int i = 0; i < numEncoded; ++i)
    {
      double olParam = m_maxOpenLoop * genome[I + i];
      
      for(int j = 0; j < numCopies; j++)
      {
        m_openLoopParams.push_back(olParam);        
      }
    }
    
    I += numEncoded;
  }
  
  // Duration of commanded ramp as proportion of desired movement time
  if(m_evolveRampDuration)  
    m_rampDurationFactor = 0.4 + 0.6 * genome[I++];
  
  // Minimum co-contraction
  if(m_enableCoconIncrease)
  {
    m_minCocontraction = 0.05 + 0.15 * genome[I++];
  }
  
  // Intersegmental inputs
  // Encoding works essentially the same as openLoop
  if(m_evolveIntersegmentInputs)
  { 
    m_intersegParams.clear(); // This instance could repeatedly be decoded!
    
    int N = m_symmetricMuscles ? (m_numMoves * numReflexes) : (m_numMoves * numMuscles);
    for(int i = 0; i < N; ++i)
    {
      double Wisep = map01To(genome[I + i], 0.0, m_maxInterseg);
      m_intersegParams.push_back(Wisep);
      
      // We have to duplicate each muscle activation (one encoded, but two identical needed for symmetry)
      if(m_symmetricMuscles)
      {
        m_intersegParams.push_back(Wisep);
      }
    }
    I += N;
  }
  
  assert(I == getNumGenes());
  
  createTrajectories();
  
  // Save to file data charaterising arm's isometric response
  const bool recordIsometric = SETTINGS->getChild("Config/GA/Evolvable/recordIsometric").getValue<bool>();
  if(recordIsometric)
  {
    Recorder recorder;
    std::vector<float> muscleAct (4);
    std::fill(muscleAct.begin(), muscleAct.end(), 0.5);
    const float jointInc = 1.0;
    m_arm->recordIsometric(recorder, muscleAct, jointInc);
    recorder.saveTo(dmx::DATA_DIR + "Isometric.txt");
  }  
};  
  
//----------------------------------------------------------------------------------------------------------------------
int EvoArmCoCon::getNumGenes() 
{ 
  int numGenes = 0;
  const int numReflexes = m_arm->getNumReflexes();
  const int numMuscles = m_arm->getNumMuscles();
  
  // Ramp durationfactor: duration of commanded ramp as proportion of desired movement time
  if(m_evolveRampDuration)
    numGenes += 1;
  
  // 2 x friction
  if(m_evolveFriction)
    numGenes += 2;
  
  // 5 muscle params (L0, Fmax, vmax, origin, insertion)
  if(m_evolveMuscles)
  {
    // 2 X joint radii (only makes sense if we evolve muscles too)
    numGenes += 2;    
    
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
  }
  
  if(m_evolveSpindles)
  {
    // Spindle gains: p,v,d, exponent1, exp 2
    const int numSpindleParams = m_evolveVelRef ? 5 : 3;
    
    if(m_evolveUniformSpindles == 1)
      numGenes += numReflexes * numSpindleParams; // Spindles differ only between elbow and shoulder
    else if (m_evolveUniformSpindles == 2)
      numGenes += 1 * numSpindleParams;
    else
      numGenes += numMuscles * numSpindleParams;
    
    // Weights to aMN
    if(m_symmetricMuscles)
    {
      numGenes += numReflexes;
    }
    else
    {
      if(m_evolveUniformSpindleWeights)
        numGenes += numReflexes;
      else
        numGenes += numMuscles;
    }
  }
  
  // Min cocontraction throughout rest and movement periods
  if(m_enableCoconIncrease)
    numGenes += 1;
  
  // Parameters for single IaIn neurons
  // Complex: Bias, tau, weights from ia, rn, sp, amn and input to MN.
  // Simple: tau, weights from ia, sp, and input to MN.  
  if(m_evolveIAIN)
  {
    const int numIaParams = m_evolveIAINsimple ? 4 : 7;
    if(m_evolveIAINsym)
      numGenes += numReflexes * numIaParams;
    else
      numGenes += numMuscles * numIaParams;
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
    if(m_openLoopSymmetry == 1)
      numGenes += numReflexes * m_numMoves;
    else if(m_openLoopSymmetry == 2)
      numGenes += 1 * m_numMoves;
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
