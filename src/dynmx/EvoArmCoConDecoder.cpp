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
  int numDecoded = 0;
  
  if(m_evolveMuscles)
  {
    // Elbow muscles should be weaker than shoulder muscles
    bool isElbowMuscle = false;
    if(m->isMonoArticulate())
    {
      isElbowMuscle = ((MuscleMonoWrap*)m)->getJoint() == JT_elbow;
    }
    const float forceScalar = isElbowMuscle ? 0.75f : 1.0f;
    
    const MuscleLimits& limits = m_decodeLimits.muscle;
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
    
    numDecoded += 5;
  }
  
  if(m_evolveHillParams)
  {
    double hSh = 0.1 + 0.4 * genome[I + numDecoded + 0]; // Was 0.9
    double hLn = 0.1 + 0.4 * genome[I + numDecoded + 1]; // Was 0.9
    double hMax = 1.1 + 0.7 * genome[I + numDecoded + 2];
    double hSlp = 2.0;
    m->setHillParameters(hSh, hLn, hMax, hSlp);
    numDecoded += 3;
  }
  
  return numDecoded;
}
  
//----------------------------------------------------------------------------------------------------------------------  
void EvoArmCoCon::decodeSpindles(int move/*=0*/)
{
  const int numSpindleParams = m_evolveVelRef ? 5 : 3;
  const int symMult = (m_evolveSymmetricSpindles == 0) ? m_arm->getNumMuscles() : (m_evolveSymmetricSpindles == 1) ? m_arm->getNumReflexes() : 1;
  const int moveMult = m_evolveSpindlesPerMove ? move : 0;
  
  int startId = moveMult * symMult * numSpindleParams;    
  bool symmetric = m_evolveSymmetricSpindles >= 1;
  int numDecoded = m_arm->getReflex(0)->decodeSpindleParams(m_spindleParams, startId, symmetric, m_evolveVelRef, m_decodeLimits.spindle);
  
  // If symmetry == 2, we reuse same parameters for second reflex, otherwise increment start index:
  if(m_evolveSymmetricSpindles < 2)
  {
    startId += numDecoded;
  }
  
  m_arm->getReflex(1)->decodeSpindleParams(m_spindleParams, startId, symmetric, m_evolveVelRef, m_decodeLimits.spindle);
}
  
//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::decodeGenome(const double* genome)
{
  int I = 0;
  int numGenesEncoded = getNumGenes();
  
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
  }
  
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
  
  
//  else 
//  {
//    // Need to init, as otherwise the new jointRadii are not compatible with precalculated values
//    for(int i = 0; i < numMuscles; ++i)
//    {
//      m_arm->getMuscle(i)->init();
//    }
//  }

  
  // Spindle parameters
  if(m_evolveSpindles)
  {
    // Copy spindle params to stored vector
    const int numSpindleParams = m_evolveVelRef ? 5 : 3;
    const int moveMult = m_evolveSpindlesPerMove ? m_numMoves : 1;
    const int symMult = (m_evolveSymmetricSpindles == 0) ? numMuscles : (m_evolveSymmetricSpindles == 1) ? numReflexes : 1;
    const int numGenes = moveMult * symMult * numSpindleParams;
    m_spindleParams.clear();
    for(int i = 0; i < numGenes; ++i)
    {
      m_spindleParams.push_back(genome[I+i]);
    }
    I += numGenes;    
    
    // Now decode first set anyway. There should be at least one.
    decodeSpindles(0);
                
    // Alpha MN params: input from spindles
    const Reflex::SpindleLimits& lim = m_decodeLimits.spindle;
    if(m_symmetricMuscles || m_evolveSymmetricSpindleWeights > 0)
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
  
  // Range for neural parameters
  float maxW = 1.0;
  float maxB = 0.1;
  float maxT = 100.0;  // These really encode 1/tau, i.e. tau in [1/(minT+maxt), 1/minT]. Was 90.0 and 10.0;
  float minT = 1.0;
  
  // IaIn parameters  
  if(m_evolveIAIN)
  {
    const int numIaParams = m_evolveIAINsimple ? 4 : 7;
    if(m_evolveIAINsym)
    {
      if(m_evolveIAINsimple)
      {
        // Symmetric and simple
        for(int i = 0; i <= 1; ++i)
        {
          m_arm->getReflex(i)->setIaInParametersMinimal(genome[I+0] * maxW, genome[I+0] * maxW,        // spindle->ia
                                                 genome[I+1] * maxW, genome[I+1] * maxW,        // ia->ia
                                                 genome[I+2] * maxW, genome[I+2] * maxW,        // ia->mn
                                                 minT + genome[I+3] * maxT, minT + genome[I+3] * maxT);  // time constants
          I += numIaParams;
        }
      }
      else 
      {
        // Symmetric and complex
        for(int i = 0; i <= 1; ++i)
        {
          m_arm->getReflex(i)->setIaInParameters(genome[I+0] * maxW, genome[I+0] * maxW,        // spindle->ia
                                                 genome[I+1] * maxW, genome[I+1] * maxW,        // ia->ia
                                                 genome[I+2] * maxW, genome[I+2] * maxW,        // desired contr->ia
                                                 genome[I+3] * maxW, genome[I+3] * maxW,        // rn->ia
                                                 genome[I+4] * maxW, genome[I+4] * maxW,        // ia->mn
                                                 minT + genome[I+5] * maxT, minT + genome[I+5] * maxT,        // time constants
                                                 -maxB + (genome[I+6] * 2 * maxB), -maxB + (genome[I+6] * 2 * maxB));       // biases
          I += numIaParams;
        }
      }
    }
    else 
    {
      if(m_evolveIAINsimple)
      {
        // Non-symmetric and simple
        for(int i = 0; i <= 1; ++i)
        {
          m_arm->getReflex(i)->setIaInParametersMinimal(genome[I+0] * maxW, genome[I+4] * maxW,        // spindle->ia
                                                 genome[I+1] * maxW, genome[I+5] * maxW,        // ia->ia
                                                 genome[I+2] * maxW, genome[I+6] * maxW,        // ia->mn
                                                 minT + genome[I+3] * maxT, minT + genome[I+7] * maxT);  // time constants
          I += 2 * numIaParams;
        }
      }
      else
      {
        // Non-symmetric and complex
        for(int i = 0; i <= 1; ++i)
        {
          m_arm->getReflex(i)->setIaInParameters(genome[I+0] * maxW, genome[I+7] * maxW,        // spindle->ia
                                                 genome[I+1] * maxW, genome[I+8] * maxW,        // ia->ia
                                                 genome[I+2] * maxW, genome[I+9] * maxW,        // desired contr->ia
                                                 genome[I+3] * maxW, genome[I+10] * maxW,        // rn->ia
                                                 genome[I+4] * maxW, genome[I+11] * maxW,        // ia->mn
                                                 minT + genome[I+5] * maxT, minT + genome[I+12] * maxT,        // time constants
                                                 -maxB + (genome[I+6] * 2 * maxB), -maxB + (genome[I+13] * 2 * maxB));       // biases
          I += 2 * numIaParams;
        }
      }
    }
  }
  
  
  // Renshaw param
  if(m_evolveRenshaw)
  {
    if(m_evolveRenshawSym)
    {
      for(int i = 0; i <= 1; ++i)
      {
        m_arm->getReflex(i)->setRenshawParameters(genome[I+0] * maxW, genome[I+0] * maxW,
                                                  genome[I+1] * maxW, genome[I+1] * maxW,
                                                  genome[I+2] * maxW, genome[I+2] * maxW,
                                                  minT + genome[I+3] * maxT, minT + genome[I+3] * maxT,
                                                  -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+4] * 2 * maxB));  
        I += 5;
      }
    }
    else 
    {
      for(int i = 0; i <= 1; ++i)
      {
        m_arm->getReflex(i)->setRenshawParameters(genome[I+0] * maxW, genome[I+5] * maxW,
                                                  genome[I+1] * maxW, genome[I+6] * maxW,
                                                  genome[I+2] * maxW, genome[I+7] * maxW,
                                                  minT + genome[I+3] * maxT, minT + genome[I+8] * maxT,
                                                  -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+9] * 2 * maxB));  
        I += 10;
      }
    }
  }
  
  // IbIn params
  if(m_evolveIBIN)
  {
    if(m_evolveIBINsym)
    {
      if(m_evolveIBINderiv)
      {
        // Symmetric and with derivative
        for(int i = 0; i <= 1; ++i)
        {
          m_arm->getReflex(i)->setIbInParametersDeriv(genome[I+0] * maxW, genome[I+0] * maxW,
                                                 genome[I+1] * maxW, genome[I+1] * maxW,
                                                 genome[I+2] * maxW, genome[I+2] * maxW,
                                                 genome[I+3] * maxW, genome[I+3] * maxW,
                                                 minT + genome[I+4] * maxT, minT + genome[I+4] * maxT,
                                                 -maxB + (genome[I+5] * 2 * maxB), -maxB + (genome[I+5] * 2 * maxB));
          I += 6;
        }
        
      }
      else
      {
        // Symmetric and without derivative
        for(int i = 0; i <= 1; ++i)
        {
          m_arm->getReflex(i)->setIbInParameters(genome[I+0] * maxW, genome[I+0] * maxW,
                                                 genome[I+1] * maxW, genome[I+1] * maxW, 
                                                 genome[I+2] * maxW, genome[I+2] * maxW,
                                                 minT + genome[I+3] * maxT, minT + genome[I+3] * maxT,
                                                 -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+4] * 2 * maxB)); 
          I += 5;
        }
      }
    }
    else
    {
      if(m_evolveIBINderiv)
      {
        // Non-symmetric and with derivative
        for(int i = 0; i <= 1; ++i)
        {
          m_arm->getReflex(i)->setIbInParametersDeriv(genome[I+0] * maxW, genome[I+6] * maxW,
                                                 genome[I+1] * maxW, genome[I+7] * maxW,
                                                 genome[I+2] * maxW, genome[I+8] * maxW,
                                                 genome[I+3] * maxW, genome[I+9] * maxW,
                                                 minT + genome[I+4] * maxT, minT + genome[I+10] * maxT,
                                                 -maxB + (genome[I+5] * 2 * maxB), -maxB + (genome[I+11] * 2 * maxB));
          I += 12;
        }
      }
      else
      {
        // Non-symmetric and without derivative
        for(int i = 0; i <= 1; ++i)
        {
          m_arm->getReflex(i)->setIbInParameters(genome[I+0] * maxW, genome[I+5] * maxW,
                                                 genome[I+1] * maxW, genome[I+6] * maxW,
                                                 genome[I+2] * maxW, genome[I+7] * maxW,
                                                 minT + genome[I+3] * maxT, minT + genome[I+8] * maxT,
                                                 -maxB + (genome[I+4] * 2 * maxB), -maxB + (genome[I+9] * 2 * maxB)); 
          I += 10;
        }
      }
    }
  }
  
  // IBIn intersegmental connections
  if(m_evolveIBInterSeg)
  {
    m_IbIntersegWeights.clear();
    if(m_evolveIBINsym)
    {
     for(int i = 0; i <= 1; ++i)
     {
       m_IbIntersegWeights.push_back(map01To(genome[I+0],-maxW, maxW));
       m_IbIntersegWeights.push_back(map01To(genome[I+1],-maxW, maxW));
       m_IbIntersegWeights.push_back(map01To(genome[I+1],-maxW, maxW));
       m_IbIntersegWeights.push_back(map01To(genome[I+0],-maxW, maxW));
       
       m_IbIntersegWeights.push_back(map01To(genome[I+2],-maxW, maxW));
       m_IbIntersegWeights.push_back(map01To(genome[I+3],-maxW, maxW));
       m_IbIntersegWeights.push_back(map01To(genome[I+3],-maxW, maxW));
       m_IbIntersegWeights.push_back(map01To(genome[I+2],-maxW, maxW));
       
       m_arm->getReflex(i)->setIbIntersegParameters(m_IbIntersegWeights, i*8);
       
       I += 4;
     }
    }
    else
    {
     for(int i = 0; i <= 1; ++i)
     {
       for(int j = 0; j < 8; ++j)
       {
          m_IbIntersegWeights.push_back(map01To(genome[I++],-maxW, maxW));
       }
       
       m_arm->getReflex(i)->setIbIntersegParameters(m_IbIntersegWeights, i*8);
     }
    }
  }
  
  if(m_flipIBIntersegWeights == 1)
  {
    if(m_evolveIBINsym)
    {
      for(int i = 0; i <= 1; ++i)
      {
        m_IbIntersegSwitches.push_back(genome[I+0] > 0.5 ? 1 : -1);
        m_IbIntersegSwitches.push_back(genome[I+1] > 0.5 ? 1 : -1);
        m_IbIntersegSwitches.push_back(genome[I+1] > 0.5 ? 1 : -1);
        m_IbIntersegSwitches.push_back(genome[I+0] > 0.5 ? 1 : -1);
        
        m_IbIntersegSwitches.push_back(genome[I+2] > 0.5 ? 1 : -1);
        m_IbIntersegSwitches.push_back(genome[I+3] > 0.5 ? 1 : -1);
        m_IbIntersegSwitches.push_back(genome[I+3] > 0.5 ? 1 : -1);
        m_IbIntersegSwitches.push_back(genome[I+2] > 0.5 ? 1 : -1);
        
        I += 4;
      }
    }
    else
    {
      for(int i = 0; i <= 1; ++i)
      {
        for(int j = 0; j <= 7; ++j)
        {
          m_IbIntersegSwitches.push_back(genome[I++] > 0.5 ? 1 : -1);
        }
      }
    }
  }
  else if (m_flipIBIntersegWeights == 2)
  {
    if(m_evolveIBINsym)
    {
      for(int i = 0; i <= 1; ++i)
      {
        m_IbIntersegWeights.push_back(map01To(genome[I+0],-maxW, maxW));
        m_IbIntersegWeights.push_back(map01To(genome[I+1],-maxW, maxW));
        m_IbIntersegWeights.push_back(map01To(genome[I+1],-maxW, maxW));
        m_IbIntersegWeights.push_back(map01To(genome[I+0],-maxW, maxW));
        
        m_IbIntersegWeights.push_back(map01To(genome[I+2],-maxW, maxW));
        m_IbIntersegWeights.push_back(map01To(genome[I+3],-maxW, maxW));
        m_IbIntersegWeights.push_back(map01To(genome[I+3],-maxW, maxW));
        m_IbIntersegWeights.push_back(map01To(genome[I+2],-maxW, maxW));
        
        I += 4;
      }
    }
    else
    {
      for(int i = 0; i <= 1; ++i)
      {
        for(int j = 0; j < 8; ++j)
        {
          m_IbIntersegWeights.push_back(map01To(genome[I++],-maxW, maxW));
        }
      }
    }
  }

  
  // Reciprocal excitation of autogenic inhibition reflex (+IbAgMnAn) and Ia to IbIn
  if(m_evolveIBRecExcIa)
  {
    if(m_evolveIBINsym)
    {
      for(int i = 0; i <= 1; ++i)
      {
        m_arm->getReflex(i)->setIbRecExcIaParameters(genome[I+0] * maxW, genome[I+0] * maxW,
                                                     genome[I+1] * maxW, genome[I+1] * maxW);
        I += 2;
      }
    }
    else
    {
      for(int i = 0; i <= 1; ++i)
      {
        m_arm->getReflex(i)->setIbRecExcIaParameters(genome[I+0] * maxW, genome[I+1] * maxW,
                                                     genome[I+2] * maxW, genome[I+3] * maxW);
        I += 4;
      }
    }
  }

  
  // Bias and tau for aMN if simulated as neuron
  if(m_evolveMNAsNeuron)
  {
    if(m_evolveMNsym)
    {
      for(int i = 0; i <= 1; ++i)
      {
        m_arm->getReflex(i)->setMNAsNeuron(minT + genome[I+0] * maxT, minT + genome[I+0] * maxT,
                                           -maxB + (genome[I+1] * 2 * maxB), -maxB + (genome[I+1] * 2 * maxB));
        I += 2;
      }
    }
    else
    {
      for(int i = 0; i <= 1; ++i)
      {
        m_arm->getReflex(i)->setMNAsNeuron(minT + genome[I+0] * maxT, minT + genome[I+2] * maxT,
                                           -maxB + (genome[I+1] * 2 * maxB), -maxB + (genome[I+3] * 2 * maxB));
        I += 4;
      }
    }
      
  }
  
  // Slopes of neural activation function
  if(m_evolveSigmoidSlope)
  {
    double kia[2] = {1, 1};
    double kib[2] = {1, 1};
    double krn[2] = {1, 1};
    double kmn[2] = {1, 1};
    double minK = 0.1;
    double maxK = 10.0;
    
    for(int i = 0; i <= 1; ++i)
    {
      kia[0] = map01To(genome[I++], minK, maxK);
      kia[1] = m_evolveIAINsym ? kia[0] : map01To(genome[I++], minK, maxK);

      kib[0] = map01To(genome[I++], minK, maxK);
      kib[1] = m_evolveIBINsym ? kib[0] : map01To(genome[I++], minK, maxK);
      
      krn[0] = map01To(genome[I++], minK, maxK);
      krn[1] = m_evolveRenshawSym ? krn[0] : map01To(genome[I++], minK, maxK);

      if(m_evolveMNAsNeuron)
      {
        kmn[0] = map01To(genome[I++], minK, maxK);
        kmn[1] = m_evolveMNsym ? kmn[0] : map01To(genome[I++], minK, maxK);
      }
      
      m_arm->getReflex(i)->setSigmoidSlopes(kia[0], kia[1], kib[0], kib[1], krn[0], krn[1], kmn[0], kmn[1]);
      
    } // for all reflexes
  } // if evolve slopes
  
  if(m_goMode == Reflex::kGo_weighted)
  {
    for(int i = 0; i <= 1; ++i)
    {
      m_arm->getReflex(i)->setGoParameters(map01To(genome[I+0], -maxW, 0),
                                           map01To(genome[I+1], -maxW, 0),
                                           map01To(genome[I+2], -maxW, 0),
                                           map01To(genome[I+3], -maxW, 0));
      I += 4;
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
    
    if(m_openLoopTauMaxAct > 0)
    {
      const double tauAct = std::max(0.01, m_openLoopTauMaxAct * genome[I]);
      const double tauDeact = std::max(0.01, m_openLoopTauMaxDeact * genome[I + 1]);
      m_arm->getReflex(0)->setOpenLoopTimeConstant(tauAct, tauDeact);
      m_arm->getReflex(1)->setOpenLoopTimeConstant(tauAct, tauDeact);      
      I += 2;
    }
  }
  
  // Duration of commanded ramp as proportion of desired movement time
  if(m_evolveRampDuration) 
  {
    m_rampDurations.clear();
    int numRamps = m_evolveRampDurationPerMove ? m_numMoves : 1;
    for(int i = 0; i < numRamps; ++i)
    {
      m_rampDurations.push_back(0.4 + 0.6 * genome[I++]);
    }
  }
  
  
  // Intersegmental inputs
  // Encoding works essentially the same as openLoop
  if(m_evolveIntersegmentInputs)
  { 
    m_intersegParams.clear(); // This instance could repeatedly be decoded!
    
    int moveMult = m_evolveIntersegPerMove ? m_numMoves : 1;
    int N = m_evolveIntersegSym ? (moveMult * numReflexes) : (moveMult * numMuscles);
    for(int i = 0; i < N; ++i)
    {
      double Wisep = map01To(genome[I + i], -m_maxInterseg, m_maxInterseg);
      m_intersegParams.push_back(Wisep);
      
      // We have to duplicate each muscle activation (one encoded, but two identical needed for symmetry)
      if(m_evolveIntersegSym)
      {
        m_intersegParams.push_back(Wisep);
      }
    }
    I += N;
  }
  
  if(m_evolveDistalCommandDelay)
  {
    m_distalDelays.clear();
    for(int i = 0; i < m_numMoves; ++i)
    {
      m_distalDelays.push_back(m_maxDistalDelay * genome[I++]);
    }
  }
  
  assert(I == numGenesEncoded);
  
  createTrajectories();
  
  // Save to file data charaterising arm's isometric response
  const bool recordIsometric = SETTINGS->getChild("Config/GA/Evolvable/recordIsometric").getValue<bool>();
  if(recordIsometric)
  {
    Recorder recorder;
    std::vector<float> muscleAct (4);
    std::fill(muscleAct.begin(), muscleAct.end(), 1.0);
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
  {
    numGenes += m_evolveRampDurationPerMove ? m_numMoves : 1;
  }
  
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
  }
  
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
    // Spindle gains: p,v,d, exponent1, exp 2
    const int numSpindleParams = m_evolveVelRef ? 5 : 3;
    const int moveMult = m_evolveSpindlesPerMove ? m_numMoves : 1;
    
    if(m_evolveSymmetricSpindles == 1)
      numGenes += moveMult * numReflexes * numSpindleParams; // Spindles differ only between elbow and shoulder
    else if (m_evolveSymmetricSpindles == 2)
      numGenes += moveMult * 1 * numSpindleParams;
    else
      numGenes += moveMult * numMuscles * numSpindleParams;
    
    // Weights to aMN
    if(m_symmetricMuscles || m_evolveSymmetricSpindleWeights)
    {
      numGenes += numReflexes;
    }
    else
    {
      numGenes += numMuscles;
    }
  }
  
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
    if(m_evolveRenshawSym)
      numGenes += numReflexes * 5;
    else
      numGenes += numMuscles * 5;
  }
  
  // IbIn neurons: Bias, tau, weights from golgi, ib. Plus input to MN.
  // If derivative is used, one more param per neuron.
  if(m_evolveIBIN)
  {
    int numIbParams = m_evolveIBINderiv ? 6 : 5;
    if(m_evolveIBINsym)
      numGenes += numReflexes * numIbParams;
    else
      numGenes += numMuscles * numIbParams;
  }
  
  // Ib intersegmental connections
  if(m_evolveIBInterSeg)
  {
    const int numIbIs = 4; // 2 to each MN and 2 to each IbIn
    const int numParams = m_evolveIBINsym ? numReflexes * numIbIs : numMuscles * numIbIs;
    numGenes += numParams;
    if(m_flipIBIntersegWeights > 0)
    {
      // Second set of weight, one for each move type
      // It's either going to be decoded as sign flips only, or as full weights
      numGenes += numParams;
    }
  }
  
  
  // Reciprocal excitation of autogenic inhibition reflex (+IbInAgMnAn) and Ia to IbIn
  if(m_evolveIBRecExcIa)
  {
    const int numCons = 2; // +IbInAgMnAn, +IbAgIaAg
    if(m_evolveIBINsym)
      numGenes += numReflexes * numCons;
    else
      numGenes += numMuscles * numCons;
  }
  
  // Neural transfer function slopes
  if(m_evolveSigmoidSlope)
  {
    int numSlopes = 0;
    numSlopes += m_evolveIBINsym ? 1 : 2;
    numSlopes += m_evolveIAINsym ? 1 : 2;
    numSlopes += m_evolveRenshawSym ? 1 : 2;
    numSlopes += m_evolveMNAsNeuron ? (m_evolveMNsym ? 1 : 2) : 0;
    
    numGenes += numReflexes * numSlopes; // 2 reflexes
  }
  
  // motor neurons as neuron (or just rectified sum of inputs)
  if(m_evolveMNAsNeuron)
  {
    const int numMNparams = 2; // bias and tau
    if(m_evolveMNsym)
      numGenes += numReflexes * numMNparams;
    else
      numGenes += numMuscles * numMNparams;
  }
  
  if(m_goMode == Reflex::kGo_weighted)
  {
    numGenes += numReflexes * 4;
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
    
    if(m_openLoopTauMaxAct > 0)
    {
      numGenes += 2;
    }
  }
  
  // Intersegmental inputs (weight to mn)
  if(m_evolveIntersegmentInputs)
  {
    int moveMult = m_evolveIntersegPerMove ? m_numMoves : 1;
    if(m_evolveIntersegSym)
      numGenes += moveMult * numReflexes;
    else
      numGenes += moveMult * numMuscles;
  }
  
  if(m_evolveDistalCommandDelay)
  {
    numGenes += m_numMoves;
  }
  
  return numGenes;
};
  
  
}
