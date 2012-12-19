/*
 *  EvoArmCoCon.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 1/18/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "EvoArmCoCon.h"
#include "MathUtils.h"

namespace dmx
{
  
float ELB_SWITCH_TRIAL = 99;

//----------------------------------------------------------------------------------------------------------------------  
const std::string EvoArmCoCon::PhaseNames [kMvPh_numPhases] = 
{
  "LeadIn",
  "Move",
  "LeadOut",
  "End",
};
  
  
//----------------------------------------------------------------------------------------------------------------------
EvoArmCoCon::EvoArmCoCon() : m_arm(0), m_time(0) 
{ 
  m_arm = new dmx::ArmReflex(); 
  init();
};

// Define distance measure for fitness function
//----------------------------------------------------------------------------------------------------------------------
struct DiffSq
{
  double operator() (double d1, double d2) { return sqr(d1-d2); };
};  
  
struct VecDist
{
  double operator() (ci::Vec2f t1, ci::Vec2f t2) { return (t1-t2).lengthSquared(); };
  //double operator() (ci::Vec2f t1, ci::Vec2f t2) { return (t1-t2).length(); };
};  
  
struct VecNorm
{
  VecNorm(float dt = 1) : m_rdt(1.0f/dt) {};
  double operator() (ci::Vec2f t1) { return t1.length() * m_rdt; };
  
  float m_rdt;
};    

// Returns fitness based on trajectory between time t0 and tmax only
//----------------------------------------------------------------------------------------------------------------------
float EvoArmCoCon::getFitness(float start, float end) 
{
  const double dt = 1.0 / (double)SETTINGS->getChild("Config/Globals/FrameRate").getAttributeValue<int>("Value");
  
  int minDelay = -100;
  int maxDelay = 100;
  
  // Convert time [s] into array indices
  int t0 = start / dt;
  int tmax = end / dt;  
  int numSteps = tmax - t0;
  
  // Position-based evaluation
  std::vector<double> posCorr = crossCorrelation(m_desiredPositions, m_actualPositions, VecDist(), minDelay, maxDelay, t0, tmax);
  std::vector<double>::iterator optElem = min_element(posCorr.begin(), posCorr.end());
  double optPosVal = *optElem / numSteps;
  double fitness = 1.0 - sqrt(optPosVal);
  assert(fitness <= 1.0);
  
  // Store what time delay lead to best fitness (for later plotting).
  int optAt = minDelay + std::distance(posCorr.begin(), optElem);
  m_bestFitDelay = optAt * dt;
  
  // Velocity based evaluation, based on desired velocity profile
  if(m_fitnessEvalVel)
  {
    std::vector<double> tangentialVels;
    differences(m_actualPositions, tangentialVels, VecNorm(dt), true, t0, tmax);
    
    std::vector<double> desTangentialVels;
    differences(m_desiredPositions, desTangentialVels, VecNorm(dt), true, t0, tmax);
    
    // Correlation between start and end of differences, as we have already extracted data for the correct period
    std::vector<double> velCorr = crossCorrelation(desTangentialVels, tangentialVels, DiffSq(), minDelay, maxDelay);
    optElem = min_element(velCorr.begin(), velCorr.end());  
    double optVelVal = *optElem / numSteps;
    double velFitness = 1.0 - sqrt(optVelVal) / 10.0;
    assert(velFitness <= 1.0);    
    
    // Store what time delay lead to best fitness (for later plotting).
    //int optAt = minDelay + std::distance(velCorr.begin(), optElem);
    //m_bestFitDelay = optAt * dt;
    
    // Overall fitness: since each fitness can be negative, avoid multiplication of two negs to pos:
    double fac = ((sign(fitness) < 0) && sign(velFitness) < 0) ? -1 : 1;
    fitness = fac * (fitness * velFitness);
  }
  
  // Minimise coactivation
  if(m_fitnessEvalCoact)
  {
    double avgElbCoact = max(m_actualCoactivationsElb, t0, tmax);
    double avgShdCoact = max(m_actualCoactivationsShd, t0, tmax);
    double elbCoactFit = 1.0 - std::max(avgElbCoact - m_fitnessEvalCoactMax, 0.0);
    double shdCoactFit = 1.0 - std::max(avgShdCoact - m_fitnessEvalCoactMax, 0.0);

    fitness *= elbCoactFit * shdCoactFit;
  }
  
  return fitness;
}    
  
//----------------------------------------------------------------------------------------------------------------------
float EvoArmCoCon::getFitness() 
{ 
  // Only calculate once. Otherwise early out.
  if(fabs(m_fitness - MAX_NEG_FLOAT) > 1.0)
  {
    return m_fitness;
  }
  
  float fitness = 1;
  
  if(m_hasHitLimit)
  {
    return 0;
  }
  
  // Muscle geometry
  if(m_evolveMuscles) 
  {
    const double maxLengthRatio = 2.2;
    for(int i = 0; i < 4; i++)
    {
      const double lengthOpt = m_arm->getMuscle(i)->getOptimalLength();
      const double lengthMinNorm = m_arm->getMuscle(i)->getLengthMin() / lengthOpt;
      const double lengthMaxNorm = m_arm->getMuscle(i)->getLengthMax() / lengthOpt;
      if((lengthMinNorm < 0.5) || (lengthMaxNorm > 1.5))
      {
        return 0;
      }
      double lengthRatio = lengthMaxNorm / lengthMinNorm;
      fitness *= 1.0 - std::min(1.0, std::max(lengthRatio - maxLengthRatio, 0.0));
    }
  }
  
  if(m_resetEachMove)
  {
    // We evaluate fitness over each movement separately
    for(int i = 0; i < m_numMoves; i++)
    {
      // Assumes each move consists of 4 trajectory points            
      float start = m_desiredTrajectory[i * 4 + 0].time; 
      float end = m_desiredTrajectory[i * 4 + 3].time;       
      if(m_fitnessEndPointOnly)
      {
        const double dt = 1.0 / (double)SETTINGS->getChild("Config/Globals/FrameRate").getAttributeValue<int>("Value");        

        // Minimise position error at end
        int j = (end / dt) - 1;
        const ci::Vec2f& desEnd = m_desiredPositions[j];
        const ci::Vec2f& actEnd = m_actualPositions[j];
        fitness *= 1.0 - desEnd.distance(actEnd);     
        // Minimise velocity at end
        const ci::Vec2f& prevActEnd = m_actualPositions[j - 1];
        double endVelFitness = 1.0 - (actEnd.distance(prevActEnd) / dt); 
        double fac = ((sign(fitness) < 0) && sign(endVelFitness) < 0) ? -1 : 1;
        fitness = fac * (fitness * endVelFitness);
        
        // Minimise position error at start
        j = (start / dt) + 1;
        const ci::Vec2f& desStart = m_desiredPositions[j];
        const ci::Vec2f& actStart = m_actualPositions[j];
        double startPosFitness = 1.0 - desStart.distance(actStart);     
        fac = ((sign(fitness) < 0) && sign(startPosFitness) < 0) ? -1 : 1;
        fitness = fac * (fitness * startPosFitness);
        
        // Minimise velocity at start
        const ci::Vec2f& prevAct = m_actualPositions[j - 1];
        double startVelFitness = 1.0 - actStart.distance(prevAct); 
        fac = ((sign(fitness) < 0) && sign(startVelFitness) < 0) ? -1 : 1;
        fitness = fac * (fitness * startVelFitness);
      }
      else
      {
        const bool evalEndOnly = false;
        if(evalEndOnly)
        {
          const double endPeriod = 0.4;
          start = end - endPeriod;
        }

        float  moveFitness = getFitness(start, end);
        fitness *= moveFitness;
        if(moveFitness < 0)
        {
          // Ensure a single negative multiplier always leads to negative fitness
          fitness = -std::abs(fitness);
        }
        if(SETTINGS->getChild("Config/Globals/DebugLevel").getAttributeValue<int>("Value",0) >= 3)
        {
          std::cout << "Fitness Mv " << i << ": " << moveFitness << ". AccFit: " << fitness << std::endl;
        }
      }
    }
  }
  else
  {
    // We evaluate fitness over the whole run
    fitness = getFitness(0, m_actualPositions.size());
  }

  m_fitness = fitness;
  return m_fitness;
};

//----------------------------------------------------------------------------------------------------------------------    
void EvoArmCoCon::readRange(Range& range, const std::string& xmlElem)
{
  // defaults
  range.min = 0.0;
  range.max = 1.0;
  
  if(SETTINGS->hasChild(xmlElem))
  {
    range.min = SETTINGS->getChild(xmlElem).getAttributeValue<double>("min", 0.0);
    range.max = SETTINGS->getChild(xmlElem).getAttributeValue<double>("max", 1.0);
  }
}

// Reads from config file data constraining the decoding of a genome into this model 
//----------------------------------------------------------------------------------------------------------------------  
void EvoArmCoCon::readDecodeLimits()
{
  if (SETTINGS->hasChild("Config/GA/Evolvable/EvoFlags/DecodeLimits"))
  {
    // Arm limits
    std::string nm = "Config/GA/Evolvable/EvoFlags/DecodeLimits/ArmLimits/";
    if(SETTINGS->hasChild(nm))
    {
      readRange(m_decodeLimits.arm.jointRadius, nm + "jointRadius");
      readRange(m_decodeLimits.arm.jointFriction, nm + "jointFriction");
    }
    else 
    {
      m_decodeLimits.arm.jointRadius = Range(0.01, 0.05);
      m_decodeLimits.arm.jointFriction = Range(0.01, 0.2);
    }

    // Muscle limits
    nm = "Config/GA/Evolvable/EvoFlags/DecodeLimits/MuscleLimits/";
    readRange(m_decodeLimits.muscle.attach, nm + "attach");
    readRange(m_decodeLimits.muscle.force, nm + "force");
    readRange(m_decodeLimits.muscle.optLength, nm + "optLength");
    readRange(m_decodeLimits.muscle.maxVel, nm + "maxVel");
    
    // Spindle limits
    nm = "Config/GA/Evolvable/EvoFlags/DecodeLimits/SpindleLimits/";
    readRange(m_decodeLimits.spindle.pos, nm + "pos");    
    readRange(m_decodeLimits.spindle.dmp, nm + "dmp");    
    readRange(m_decodeLimits.spindle.exp, nm + "exp");    
    readRange(m_decodeLimits.spindle.weight, nm + "weight");
    if(m_evolveVelRef)
      readRange(m_decodeLimits.spindle.vel, nm + "vel");    
    else 
      m_decodeLimits.spindle.vel = Range(0,0);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::init() 
{ 
  assert(m_arm); 
  m_arm->init(); 
  
  m_bestFitDelay = -1.0f;
  
  // Read settings
  if (SETTINGS->hasChild("Config/GA/Evolvable/EvoFlags")) 
  {   
    const ci::XmlTree& xmlFlags = SETTINGS->getChild("Config/GA/Evolvable/EvoFlags");    
    m_evolveMuscles = xmlFlags.getChild("Muscles").getValue<bool>();
    m_symmetricMuscles = xmlFlags.getChild("Muscles").getAttributeValue<bool>("symmetric");
    m_evolveHillParams = xmlFlags.getChild("Muscles").getAttributeValue<bool>("hillParams");
    m_evolveFriction = xmlFlags.getChild("Friction").getValue<bool>();
    m_evolveRampDuration = xmlFlags.getChild("RampDuration").getValue<bool>();    
    m_evolveRampDurationPerMove = xmlFlags.getChild("RampDuration").getAttributeValue<bool>("perMove");    
    m_evolveSpindles = xmlFlags.getChild("Spindles").getValue<bool>();
    m_evolveSymmetricSpindles = xmlFlags.getChild("Spindles").getAttributeValue<int>("symmetric");    
    m_evolveSymmetricSpindleWeights = xmlFlags.getChild("Spindles").getAttributeValue<int>("symmetricWeights");
    m_evolveSpindlesPerMove = xmlFlags.getChild("Spindles").getAttributeValue<int>("perMove");
    m_evolveVelRef = xmlFlags.getChild("Spindles").getAttributeValue<bool>("velRef");
    m_evolveIAIN = xmlFlags.getChild("Iain").getValue<bool>();
    m_evolveIAINsimple = xmlFlags.getChild("Iain").getAttributeValue<bool>("simple");
    m_evolveIAINsym = xmlFlags.getChild("Iain").getAttributeValue<bool>("symmetric");    
    m_evolveRenshaw = xmlFlags.getChild("Renshaw").getValue<bool>();
    m_evolveRenshawSym = xmlFlags.getChild("Renshaw").getAttributeValue<bool>("symmetric", 0);
    m_evolveIBIN = xmlFlags.getChild("Ibin").getValue<bool>();
    m_evolveIBINsym = xmlFlags.getChild("Ibin").getAttributeValue<bool>("symmetric");
    m_evolveIBINderiv = xmlFlags.getChild("Ibin").getAttributeValue<bool>("derivative");
    m_evolveIBInterSeg = xmlFlags.getChild("Ibin").getAttributeValue<bool>("interseg");
    m_evolveIBRecExcIa = xmlFlags.getChild("Ibin").getAttributeValue<bool>("recExcIa");
    m_evolveIFV = xmlFlags.getChild("Ifv").getValue<bool>();
    m_evolveSFV = xmlFlags.getChild("Sfv").getValue<bool>();
    m_evolveOpenLoop = xmlFlags.getChild("OpenLoop").getValue<bool>();
    m_openLoopSymmetry = xmlFlags.getChild("OpenLoop").getAttributeValue<int>("symmetric");
    m_maxOpenLoop = xmlFlags.getChild("OpenLoop").getAttributeValue<float>("max");    
    m_openLoopTauMaxAct = xmlFlags.getChild("OpenLoop").getAttributeValue<float>("tauMaxAct");
    m_openLoopTauMaxDeact = xmlFlags.getChild("OpenLoop").getAttributeValue<float>("tauMaxDeact");
    m_evolveIntersegmentInputs = xmlFlags.getChild("InterSegmentInput").getValue<bool>();
    m_evolveIntersegPerMove = xmlFlags.getChild("InterSegmentInput").getAttributeValue<bool>("perMove");
    m_evolveIntersegSym = xmlFlags.getChild("InterSegmentInput").getAttributeValue<bool>("symmetric");
    m_maxInterseg = xmlFlags.getChild("InterSegmentInput").getAttributeValue<double>("max");
    m_evolveDistalCommandDelay = xmlFlags.getChild("DistalCommandDelay").getValue<bool>();    
    m_maxDistalDelay = xmlFlags.getChild("DistalCommandDelay").getAttributeValue<double>("max");    
    m_evolveMNAsNeuron = xmlFlags.getChild("MN").getValue<bool>(0);
    m_evolveMNsym = xmlFlags.getChild("MN").getAttributeValue<bool>("symmetric");
    m_evolveSigmoidSlope = xmlFlags.getChild("Neurons").getAttributeValue<bool>("sigSlope");
    
    const ci::XmlTree& xml = SETTINGS->getChild("Config/GA/Evolvable");
    m_commandTrajSmooth = xml.getChild("CommandTrajSmooth").getValue<int>();
    m_useMuscleCoords = xml.getChild("UseMuscleCoords").getValue<bool>();    

    m_fitnessEndPointOnly = xml.getChild("FitnessFunction/EndPointOnly").getValue<bool>();    
    m_fitnessEvalVel = xml.getChild("FitnessFunction/Velocity").getValue<bool>();
    m_fitnessEvalCoact = xml.getChild("FitnessFunction/Coactivation").getValue<bool>();
    m_fitnessEvalCoactMax = xml.getChild("FitnessFunction/Coactivation").getAttributeValue<float>("max", 0.5);
    
    // Following function may depend on some of above variables
    readDecodeLimits(); 
    
    // Open-loop activations
    if (SETTINGS->hasChild("Config/Arm/OpenLoopActivations")) 
    {   
      m_openLoopParams.clear();
      
      ci::XmlTree& oplp = SETTINGS->getChild("Config/Arm/OpenLoopActivations");    
      ci::XmlTree::Iter val = oplp.begin("Value");
      for (; val != oplp.end(); ++val)
      {
        const double v = val->getValue<double>();
        m_openLoopParams.push_back(v);
      }
    }
    
    // Intersegmental parameters    
    if (SETTINGS->hasChild("Config/Arm/IntersegmentalParams")) 
    {   
      m_intersegParams.clear();
      
      ci::XmlTree& params = SETTINGS->getChild("Config/Arm/IntersegmentalParams");    
      ci::XmlTree::Iter val = params.begin("Value");
      for (; val != params.end(); ++val)
      {
        const double v = val->getValue<double>();
        m_intersegParams.push_back(v);
      }
    }     
    
    // Distal (elbow) command delays
    if (SETTINGS->hasChild("Config/Arm/DistalDelays")) 
    {   
      m_distalDelays.clear();
      
      ci::XmlTree& params = SETTINGS->getChild("Config/Arm/DistalDelays");    
      ci::XmlTree::Iter val = params.begin("Value");
      for (; val != params.end(); ++val)
      {
        const double v = val->getValue<double>();
        m_distalDelays.push_back(v);
      }
    }         
    
    // Ramp durations
    if (SETTINGS->hasChild("Config/Arm/RampDurations")) 
    {   
      m_rampDurations.clear();
      
      ci::XmlTree& params = SETTINGS->getChild("Config/Arm/RampDurations");    
      ci::XmlTree::Iter val = params.begin("Value");
      for (; val != params.end(); ++val)
      {
        const double v = val->getValue<double>();
        m_rampDurations.push_back(v);
      }
    }             
        
    // Read trajectory from settings file    
    // Requires arm to be setup to do FK, but that's already done in init() above, so should be fine.
    m_numMoves = 0; // First is the start pose for settling!
    if(SETTINGS->hasChild("Config/GA/Evolvable/DesiredTrajectory"))
    {
      const ci::XmlTree& trajXml = SETTINGS->getChild("Config/GA/Evolvable/DesiredTrajectory");
      m_resetEachMove = trajXml["ResetEachMove"].as<bool>();
      bool convertToRadians = trajXml["InDegrees"].as<bool>();     
      
      ci::Vec2f from, to, tmp;      
      for (ci::XmlTree::ConstIter moveIt = trajXml.begin(); moveIt != trajXml.end(); ++moveIt)
      {        
        // Extract move details
        const ci::XmlTree& move = *moveIt;
        float duration = move["Duration"].as<float>();
        float leadIn = move["LeadIn"].as<float>();
        float leadOut = move["LeadOut"].as<float>();
        float elbAngleFrom = (move / "From")["x"].as<float>();
        float shdAngleFrom = (move / "From")["y"].as<float>();
        float elbAngleTo = (move / "To")["x"].as<float>();
        float shdAngleTo = (move / "To")["y"].as<float>();
        if(convertToRadians)
        {
          elbAngleFrom = degreesToRadians(elbAngleFrom);
          shdAngleFrom = degreesToRadians(shdAngleFrom);        
          elbAngleTo = degreesToRadians(elbAngleTo);
          shdAngleTo = degreesToRadians(shdAngleTo);
        }
        
        // Convert from joint angles to end-effector position
        m_arm->forwardKinematics(elbAngleFrom, shdAngleFrom, tmp, from);
        m_arm->forwardKinematics(elbAngleTo, shdAngleTo, tmp, to);
        
        // Add four control points per move (initial, leadIn, shift, leadOut)
        m_desiredTrajectory.add(from, 0.0f, m_numMoves);
        m_desiredTrajectory.add(from, leadIn, m_numMoves);
        m_desiredTrajectory.add(to, duration, m_numMoves);
        m_desiredTrajectory.add(to, leadOut, -1); // past this doesn't belong to this move 
        
        // HACKY!!!!
        if(elbAngleTo < 0 && ELB_SWITCH_TRIAL == 99)
          ELB_SWITCH_TRIAL = m_numMoves * 4;
        
        m_numMoves++;
        
      }
      
      m_desiredTrajectory.setBlend(Trajectory<ci::Vec2f>::kTr_BlendMinJerk);
    }
  }
  else 
  {
    m_evolveMuscles = true;
    m_symmetricMuscles = false;
    m_evolveIAIN = false;
    m_evolveRenshaw = false;
    m_evolveIBIN = false;
    m_evolveIFV = false;
    m_evolveSFV = false;
    m_evolveOpenLoop = false;
    m_maxOpenLoop = 0.25;
    m_evolveSymmetricSpindles = 1;
    m_evolveSymmetricSpindleWeights = 1;
    m_evolveHillParams = false;    
    m_useMuscleCoords = false;
  }
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::createTrajectories() 
{   
  // First clear existing trajectories, as below we will append
  m_commandedTrajectory.clear();
  m_commandedJointAngles.clear();
  
  Trajectory<ci::Vec2f>::BlendType commandBlend;
  if (m_commandTrajSmooth == 0)
    commandBlend = Trajectory<ci::Vec2f>::kTr_BlendLinear;
  else if(m_commandTrajSmooth == 1)
    commandBlend = Trajectory<ci::Vec2f>::kTr_BlendMinJerk;
  else
    commandBlend = Trajectory<ci::Vec2f>::kTr_BlendSmoothStep;
    
  
  // Translate desired into commanded trajectory, based on ramp duration factor
  m_commandedTrajectory.setBlend(commandBlend);
  for(int i = 0; i < m_numMoves; i++)
  {
    int m = i * 4;  // Four control points per move
    
    // Initial and leadIn time are the same
    m_commandedTrajectory.add(m_desiredTrajectory[m]); 
    m_commandedTrajectory.add(m_desiredTrajectory[m + 1]); 

    // Duration of move is shorter
    double rampDurationFactor = m_rampDurations.size() > 1 ? m_rampDurations[i] : m_rampDurations[0]; // Per move or not?
    float duration = m_desiredTrajectory[m + 2].time - m_desiredTrajectory[m + 1].time;
    Target<ci::Vec2f> comMoveTarget = m_desiredTrajectory[m + 2];
    comMoveTarget.time = m_desiredTrajectory[m + 1].time + rampDurationFactor * duration;
    m_commandedTrajectory.add(comMoveTarget);
    
    // End time of move is the same
    m_commandedTrajectory.add(m_desiredTrajectory[m + 3]);
  }
   
  // Transform cartesian effector position into joint angle targets
  for(int i = 0; i < m_commandedTrajectory.size(); i++)
  {
    int elbDir = i < ELB_SWITCH_TRIAL ? 1 : -1;
    double desiredAngles[2];
    m_arm->inverseKinematics(m_commandedTrajectory[i].position, elbDir, desiredAngles[JT_elbow], desiredAngles[JT_shoulder]);  
    m_commandedJointAngles.add(m_commandedTrajectory[i]);
    m_commandedJointAngles[i].position = Pos(desiredAngles[JT_elbow], desiredAngles[JT_shoulder]);
  }
  m_commandedJointAngles.setBlend(commandBlend);
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::reset()
{
  // Set to -1 so we detect change of move or phase at first frame
  m_currentMove = -1;
  m_currentPhase = -1;
  
  m_time = 0.0f;
  
  m_fitness = MAX_NEG_FLOAT;
  m_hasHitLimit = false;
  
  m_desiredPositions.clear();
  m_actualPositions.clear();
  m_actualCoactivationsElb.clear();
  m_actualCoactivationsShd.clear();

  // If possible set arm to initial position 
  float elbStart = 0, shdStart = 0;  
  if(m_commandedJointAngles.size() > 0)
  {
    elbStart = m_commandedJointAngles[0].position.x;
    shdStart = m_commandedJointAngles[0].position.y;
  }
  
  m_arm->resetTo(elbStart, shdStart);
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::update(float dt)
{ 
  // Blends linearly not in cartesian target position
  Target<Pos> desired = m_desiredTrajectory.atTime(m_time);
  Target<Pos> command = m_commandedTrajectory.atTime(m_time);
  
  // Current move and phase
  int prevMove = m_currentMove;
  int prevPhase = m_currentPhase;
  m_currentMove = desired.name;   
  m_currentPhase = desired.id % kMvPh_numPhases;
  
#if 0 // DEBUGGING
  if(m_currentPhase != prevPhase)
    std::cout << "Mv: " << m_currentMove << "| Ph: " << PhaseNames[m_currentPhase] << ". " << std::endl;
#endif
  
  // Calculate current desired angles and muscle lengths and store for later fitness evaluation
  updateCurrentCommand(command, desired);  
  
  // When switching between moves, reset to moves initial position, if so desired
  if(m_resetEachMove && (m_currentMove != prevMove))
  {
    m_arm->resetTo(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
    
    if(m_evolveSpindlesPerMove)
    {
      decodeSpindles(m_currentMove);
    }
  }

  
  // Set open-loop parameters for the current movement
  // Vector of open-loop is arranged like this: [mv1m1 mv1m2 mv1m3 ... mv1mM | mv2m1 ... | mvNmM],
  // where mv = move, and m = muscle
  if(m_openLoopParams.size() > 0)
  {
    // We set open loop activations at beginning of movement phase. Will be reset to 0 in reflex->reset (next trial)
    if((m_currentPhase == kMvPh_move) && (m_currentPhase != prevPhase))
    {
      int i = m_currentMove * 4; // 4 muscles
      m_arm->getReflex(0)->setOpenLoop(m_openLoopParams[i+0], m_openLoopParams[i+1]);
      m_arm->getReflex(1)->setOpenLoop(m_openLoopParams[i+2], m_openLoopParams[i+3]);
    }
    else if(m_currentPhase == kMvPh_leadOut)
    {
      m_arm->getReflex(0)->setOpenLoop(0, 0);
      m_arm->getReflex(1)->setOpenLoop(0, 0);
    }
  }  
  
  // Set intersegmental parameters for the current movement
  if(m_intersegParams.size() > 0)
  {
    // We set interseg. params at beginning of each move, even in leadIn phase these should be fine to use
    if(m_currentMove != prevMove)
    //if(m_currentPhase == kMvPh_move)
    {
      int i = m_evolveIntersegPerMove ? m_currentMove * 4 : 0; // 4 muscles x 1 param each
      m_arm->getReflex(0)->setIntersegmentalParameters(m_intersegParams[i+0], m_intersegParams[i+1]);
      m_arm->getReflex(1)->setIntersegmentalParameters(m_intersegParams[i+2], m_intersegParams[i+3]);      
    }
    /*else 
    {
      m_arm->getReflex(0)->setIntersegmentalParameters(0, 0);
      m_arm->getReflex(1)->setIntersegmentalParameters(0, 0);      
    }*/
  }
  
  // Distal (elbow) command delay
  if(m_distalDelays.size() > 0)
  {
    if(m_currentMove != prevMove)
    {
      m_arm->getReflex(0)->setCommandDelay(m_distalDelays[m_currentMove]);
    }
  }
  
  
  // Update arm simulation
  if(m_useMuscleCoords)
  {
    //m_arm->update(desiredAngles.x, desiredAngles.y, dt);  
  }
  else
  {
    m_arm->update(command.position, dt, command.id < ELB_SWITCH_TRIAL ? 1 : -1);    
  }
  
  // Store for fitness evaluation  
  m_desiredPositions.push_back(desired.position);  
  m_actualPositions.push_back(m_arm->getEffectorPos()); 
  
  const double elbCoact = std::min(m_arm->getReflex(0)->getAlphaOutput(0), m_arm->getReflex(0)->getAlphaOutput(1));
  const double shdCoact = std::min(m_arm->getReflex(1)->getAlphaOutput(0), m_arm->getReflex(1)->getAlphaOutput(1));
  m_actualCoactivationsElb.push_back(elbCoact);
  m_actualCoactivationsShd.push_back(shdCoact);
  
  m_time += dt;
};

// Calculates desired and commanded state in cartesian, joint angle, and muscle length coordinates
//----------------------------------------------------------------------------------------------------------------------    
void EvoArmCoCon::updateCurrentCommand(Target<Pos>& command, Target<Pos>& desired)
{
  m_currentDesiredPos = desired.position;
  m_currentCommandPos = command.position;
  
  // Check whether any joint has hit its limit
  float eps = 0.001;
  if(fabs(m_arm->getJointAngle(JT_elbow) - m_arm->getJointLimitLower(JT_elbow)) < eps ||
     fabs(m_arm->getJointAngle(JT_elbow) - m_arm->getJointLimitUpper(JT_elbow)) < eps ||
     fabs(m_arm->getJointAngle(JT_shoulder) - m_arm->getJointLimitLower(JT_shoulder)) < eps ||
     fabs(m_arm->getJointAngle(JT_shoulder) - m_arm->getJointLimitUpper(JT_shoulder)) < eps)
  {
    m_hasHitLimit = true;
  }
  
  // Transform desired position to joint angles (IK) and muscle lengths
  double currentDesiredAngles[2];
  m_arm->inverseKinematics(desired.position, desired.id < ELB_SWITCH_TRIAL ? 1.0f : -1.0f, currentDesiredAngles[0], currentDesiredAngles[1]);
  m_currentDesiredAngles.x = clamp(currentDesiredAngles[0], m_arm->getJointLimitLower(JT_elbow), m_arm->getJointLimitUpper(JT_elbow));  
  m_currentDesiredAngles.y = clamp(currentDesiredAngles[1], m_arm->getJointLimitLower(JT_shoulder), m_arm->getJointLimitUpper(JT_shoulder));    
  
  double desLength0 = m_arm->getReflex(0)->getMuscle(0)->getLengthFromJointAngles(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
  double desLength1 = m_arm->getReflex(0)->getMuscle(1)->getLengthFromJointAngles(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
  m_currentDesiredElbLenghts = Pos(desLength0, desLength1);
  
  desLength0 = m_arm->getReflex(1)->getMuscle(0)->getLengthFromJointAngles(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
  desLength1 = m_arm->getReflex(1)->getMuscle(1)->getLengthFromJointAngles(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
  m_currentDesiredShdLenghts = Pos(desLength0, desLength1);  
}

//----------------------------------------------------------------------------------------------------------------------  
bool EvoArmCoCon::hasFinished()
{
  return m_time >= m_desiredTrajectory.getDuration();
}

//----------------------------------------------------------------------------------------------------------------------  
void EvoArmCoCon::finish()
{ 
#if DEBUGGING
  std::cout << "Fitness: " << getFitness() << std::endl;
#endif
}  

//----------------------------------------------------------------------------------------------------------------------    
void EvoArmCoCon::toXml(ci::XmlTree& xml)
{
  ci::XmlTree evolvable ("EvoArmCoCon", "");
  evolvable.setAttribute("Fitness", getFitness());
  
  ci::XmlTree bestFitDelay ("BestFitnessMatchDelay", toString(m_bestFitDelay));
  evolvable.push_back(bestFitDelay);  
  
  ci::XmlTree openLoop("OpenLoopActivations", "");
  for(int i = 0; i < m_openLoopParams.size(); i++)
  {
    ci::XmlTree act ("Value", toString(m_openLoopParams[i]));
    openLoop.push_back(act);
  }
  evolvable.push_back(openLoop);
  
  ci::XmlTree interseg("IntersegmentalParams", "");
  for(int i = 0; i < m_intersegParams.size(); i++)
  {
    ci::XmlTree isP ("Value", toString(m_intersegParams[i]));
    interseg.push_back(isP);
  }
  evolvable.push_back(interseg);  
  
  ci::XmlTree delays("DistalDelays", "");
  for(int i = 0; i < m_distalDelays.size(); i++)
  {
    ci::XmlTree delay ("Value", toString(m_distalDelays[i]));
    delays.push_back(delay);
  }
  evolvable.push_back(delays);    
  
  ci::XmlTree rampDurs("RampDurations", "");
  for(int i = 0; i < m_rampDurations.size(); i++)
  {
    ci::XmlTree duration ("Value", toString(m_rampDurations[i]));
    rampDurs.push_back(duration);
  }
  evolvable.push_back(rampDurs);      
  
  
  // Add ArmMuscled xml data
  m_arm->toXml(evolvable);  
  
  xml.push_back(evolvable);
}
  
//----------------------------------------------------------------------------------------------------------------------     
void EvoArmCoCon::record(Recorder& recorder) 
{ 
  // General arm and reflex state
  m_arm->record(recorder); 
  
  // Desired state (different from commanded state in ArmReflex)
  recorder.push_back("desX", m_currentDesiredPos.x);
  recorder.push_back("desY", m_currentDesiredPos.y);
  recorder.push_back("desElbow", m_currentDesiredAngles.x);
  recorder.push_back("desShoulder", m_currentDesiredAngles.y);
  recorder.push_back("desElbowM1", m_currentDesiredElbLenghts.x);
  recorder.push_back("desElbowM2", m_currentDesiredElbLenghts.y);
  recorder.push_back("desShoulderM1", m_currentDesiredShdLenghts.x);
  recorder.push_back("desShoulderM2", m_currentDesiredShdLenghts.y);
}
  
  
} // namespace
