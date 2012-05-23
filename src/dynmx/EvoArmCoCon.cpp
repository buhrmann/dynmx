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
  
#define ELB_SWITCH_TRIAL 99

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
    double velFitness = 1.0 - sqrt(optVelVal);
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
    double maxCoact = 0.2;
    double avgElbCoact = max(m_actualCoactivationsElb, t0, tmax);
    double avgShdCoact = max(m_actualCoactivationsShd, t0, tmax);
    double elbCoactFit = 1.0 - std::max(avgElbCoact - maxCoact, 0.0);
    double shdCoactFit = 1.0 - std::max(avgShdCoact - maxCoact, 0.0);

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
  
  float fitness;
  if(m_resetEachMove)
  {
    // We evaluate fitness over each movement separately
    fitness = 1.0;
    for(int i = 0; i < m_numMoves; i++)
    {
      // Assumes each move consists of 4 trajectory points            
      float start = m_desiredTrajectory[i * 4 + 0].time; 
      float end = m_desiredTrajectory[i * 4 + 3].time;       
      if(m_fitnessEndPointOnly)
      {
        const double dt = 1.0 / (double)SETTINGS->getChild("Config/Globals/FrameRate").getAttributeValue<int>("Value");        

        // Minimise position error at end
        int i = (end / dt) - 1;        
        const ci::Vec2f& desEnd = m_desiredPositions[i];
        const ci::Vec2f& actEnd = m_actualPositions[i];
        fitness *= 1.0 - desEnd.distance(actEnd);     
        // Minimise velocity at end
        const ci::Vec2f& prevActEnd = m_actualPositions[i - 1];
        fitness *= 1.0 - actEnd.distance(prevActEnd); 
        
        // Minimise position error at start
        i = (start / dt) + 1;        
        const ci::Vec2f& desStart = m_desiredPositions[i];
        const ci::Vec2f& actStart = m_actualPositions[i];
        fitness *= 1.0 - desStart.distance(actStart);     
        // Minimise velocity at start
        const ci::Vec2f& prevAct = m_actualPositions[i - 1];
        fitness *= 1.0 - actStart.distance(prevAct); 
        
      }
      else
      {
        float moveFitness = getFitness(start, end);
        fitness *= moveFitness;
        if(moveFitness < 0)
        {
          // Ensure a single negative multiplier always leads to negative fitness
          fitness = -std::abs(fitness);
        }
      }
    }
  }
  else
  {
    // We evaluate fitness over the whole run
    fitness = getFitness(0, m_actualPositions.size());
  }

  
  if(m_enableCoconIncrease)
  {
    float coconFit = 0;
    int N = m_coconIncrements.size();
    for(int i = 0; i < N; i++)
    {
      coconFit += std::max(m_coconIncrements[i], 0.0f);
    }
    coconFit /= N;
    fitness *= coconFit;
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
    m_evolveSpindles = xmlFlags.getChild("Spindles").getValue<bool>();
    m_evolveUniformSpindles = xmlFlags.getChild("Spindles").getAttributeValue<int>("uniform");    
    m_evolveUniformSpindleWeights = xmlFlags.getChild("Spindles").getAttributeValue<int>("uniformWeights");
    m_evolveVelRef = xmlFlags.getChild("Spindles").getAttributeValue<bool>("velRef");
    m_evolveIAIN = xmlFlags.getChild("iain").getValue<bool>();
    m_evolveIAINsimple = xmlFlags.getChild("iain").getAttributeValue<bool>("simple");
    m_evolveIAINsym = xmlFlags.getChild("iain").getAttributeValue<bool>("symmetric");    
    m_evolveRenshaw = xmlFlags.getChild("renshaw").getValue<bool>();
    m_evolveIBIN = xmlFlags.getChild("ibin").getValue<bool>();
    m_evolveIFV = xmlFlags.getChild("ifv").getValue<bool>();
    m_evolveSFV = xmlFlags.getChild("sfv").getValue<bool>();
    m_evolveOpenLoop = xmlFlags.getChild("openLoop").getValue<bool>();
    m_openLoopSymmetry = xmlFlags.getChild("openLoop").getAttributeValue<int>("symmetric");
    m_maxOpenLoop = xmlFlags.getChild("openLoop").getAttributeValue<float>("max");    
    m_evolveIntersegmentInputs = xmlFlags.getChild("interSegmentInput").getValue<bool>();
    m_maxInterseg = xmlFlags.getChild("interSegmentInput").getAttributeValue<double>("max");
        
    const ci::XmlTree& xml = SETTINGS->getChild("Config/GA/Evolvable");
    m_enableCoconIncrease = xml.getChild("enableCoconInc").getValue<bool>();
    m_minCocontraction = xml.getChild("enableCoconInc").getAttributeValue<float>("min");
    m_maxCocontraction = xml.getChild("enableCoconInc").getAttributeValue<float>("max");
    m_commandTrajSmooth = xml.getChild("commandTrajSmooth").getValue<bool>();    
    m_useMuscleCoords = xml.getChild("useMuscleCoords").getValue<bool>();    

    m_fitnessEndPointOnly = xml.getChild("FitnessFunction/EndPointOnly").getValue<bool>();    
    m_fitnessEvalVel = xml.getChild("FitnessFunction/Velocity").getValue<bool>();
    m_fitnessEvalCoact = xml.getChild("FitnessFunction/Coactivation").getValue<bool>();
    
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
        double v = val->getValue<double>();
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
        double v = val->getValue<double>();
        m_intersegParams.push_back(v);
      }
    }      
    
    // Ramp duration factor
    if (SETTINGS->hasChild("Config/Arm/RampDurationFactor")) 
    {   
      m_rampDurationFactor = SETTINGS->getChild("Config/Arm/RampDurationFactor").getValue<double>();    
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
    m_evolveUniformSpindles = 1;
    m_evolveHillParams = false;
    
    m_enableCoconIncrease = false;
    m_minCocontraction = 0.05;
    m_maxCocontraction = 0.25;
    
    m_useMuscleCoords = false;
  }
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::createTrajectories() 
{   
  // First clear existing trajectories, as below we will append
  m_commandedTrajectory.clear();
  m_targetJointAngles.clear();
  m_targetElbLengths.clear();
  m_targetShdLengths.clear();
  
  Trajectory<ci::Vec2f>::BlendType commandBlend;
  if (m_commandTrajSmooth) 
    commandBlend = Trajectory<ci::Vec2f>::kTr_BlendMinJerk;
  else
    commandBlend = Trajectory<ci::Vec2f>::kTr_BlendLinear;
  
  // Translate desired into commanded trajectory, based on ramp duration factor
  m_commandedTrajectory.setBlend(commandBlend);
  for(int i = 0; i < m_numMoves; i++)
  {
    int m = i * 4;  // Four control points per move
    
    // Initial and leadIn time are the same
    m_commandedTrajectory.add(m_desiredTrajectory[m]); 
    m_commandedTrajectory.add(m_desiredTrajectory[m + 1]); 

    // Duration of move is shorter
    float duration = m_desiredTrajectory[m + 2].time - m_desiredTrajectory[m + 1].time;
    Target<ci::Vec2f> comMoveTarget = m_desiredTrajectory[m + 2];
    comMoveTarget.time = m_desiredTrajectory[m + 1].time + m_rampDurationFactor * duration;
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
    m_targetJointAngles.add(m_commandedTrajectory[i]);
    m_targetJointAngles[i].position = Pos(desiredAngles[JT_elbow], desiredAngles[JT_shoulder]);
  }
  m_targetJointAngles.setBlend(commandBlend);
  
  
  // Transform joint angle targets into muscle length targets
  for(int i = 0; i < m_commandedTrajectory.size(); i++)
  {
    Target<Pos> jointTarget = m_targetJointAngles[i];
    
    // Elbow muscles
    double desLength0 = m_arm->getReflex(0)->getMuscle(0)->getLengthFromJointAngles(jointTarget.position.x, jointTarget.position.y);
    double desLength1 = m_arm->getReflex(0)->getMuscle(1)->getLengthFromJointAngles(jointTarget.position.x, jointTarget.position.y);
    m_targetElbLengths.add(jointTarget);
    m_targetElbLengths[i].position = Pos(desLength0, desLength1);
    
    // Shoulder muscles
    desLength0 = m_arm->getReflex(1)->getMuscle(0)->getLengthFromJointAngles(jointTarget.position.x, jointTarget.position.y);
    desLength1 = m_arm->getReflex(1)->getMuscle(1)->getLengthFromJointAngles(jointTarget.position.x, jointTarget.position.y);
    m_targetShdLengths.add(jointTarget);
    m_targetShdLengths[i].position = Pos(desLength0, desLength1);
  }
  m_targetElbLengths.setBlend(commandBlend);  
  m_targetShdLengths.setBlend(commandBlend);    
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::reset()
{
  // Set to -1 so we detect change of move or phase at first frame
  m_currentMove = -1;
  m_currentPhase = -1;
  
  m_time = 0.0f;
  
  m_fitness = MAX_NEG_FLOAT;
  
  m_desiredPositions.clear();
  m_actualPositions.clear();
  m_actualCoactivationsElb.clear();
  m_actualCoactivationsShd.clear();
  m_coconIncrements.clear();
  
  m_coconStarted = false;

  // If possible set arm to initial position 
  float elbStart = 0, shdStart = 0;  
  if(m_targetJointAngles.size() > 0)
  {
    elbStart = m_targetJointAngles[0].position.x;
    shdStart = m_targetJointAngles[0].position.y;
  }
  
  m_arm->resetTo(elbStart, shdStart);
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::update(float dt)
{ 
  // Blend linearly not in target position, as that would lead to asymmetric profiles in joint and muscle space,
  // but blend linearly in joint angle space
  // Get position first
  Target<Pos> desired = m_desiredTrajectory.atTime(m_time);
  Target<Pos> command = m_commandedTrajectory.atTime(m_time);
  
  // Current move and phase
  int prevMove = m_currentMove;
  m_currentMove = desired.name;   
  int prevPhase = m_currentPhase;
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
  }  
  
  // Incremental increase in cocontraction at target position
  const Target<Pos>& coconTarget = desired;
  if(m_enableCoconIncrease)
  {
    m_cocontraction = m_minCocontraction;
    
    if (coconTarget.id % 2 == 0)
    {     
      // Determine how far proportionally we're through with this target
      float duration = m_desiredTrajectory[coconTarget.id + 1].time - coconTarget.time;
      m_cocontraction += m_maxCocontraction * ((m_time - coconTarget.time) / duration);
    } 
    
    // Apply
    m_arm->getReflex(0)->setOpenLoop(m_cocontraction, m_cocontraction);
    m_arm->getReflex(1)->setOpenLoop(m_cocontraction, m_cocontraction);    
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
  
  // Set open-loop parameters for the current movement
  if(m_intersegParams.size() > 0)
  {
    // We set interseg. params at beginning of each move, even in leadIn phase these should be fine to use
    //if(m_currentMove != prevMove)
    if(m_currentPhase == kMvPh_move)
    {
      int i = m_currentMove * 4; // 4 muscles x 1 param each 
      m_arm->getReflex(0)->setIntersegmentalParameters(m_intersegParams[i+0], m_intersegParams[i+1]);
      m_arm->getReflex(1)->setIntersegmentalParameters(m_intersegParams[i+2], m_intersegParams[i+3]);      
    }
    else 
    {
      m_arm->getReflex(0)->setIntersegmentalParameters(0, 0);
      m_arm->getReflex(1)->setIntersegmentalParameters(0, 0);      
    }

  }
  
  
  // Update arm simulation
  if(m_useMuscleCoords)
  {
    Pos commandedElbLengths = m_targetElbLengths.atTime(m_time).position;
    Pos commandedShdLengths = m_targetShdLengths.atTime(m_time).position;  
    m_arm->update(commandedElbLengths.x, commandedElbLengths.y, commandedShdLengths.x, commandedShdLengths.y, dt);  
    //m_arm->update(desiredAngles.x, desiredAngles.y, dt);  
  }
  else
  {
    m_arm->update(command.position, dt, command.id < ELB_SWITCH_TRIAL ? 1 : -1);    
  }
  
  
  // Measure real co-contraction increment between beginning and end of commanded increase
  if(m_enableCoconIncrease)
  {
    if(coconTarget.id % 2 == 0)
    {
      // At start of increase
      if(!m_coconStarted)
      {
        // Cache average activation at start of increase
        m_coconAtStart[0] = 0.5 * (m_arm->getReflex(0)->getAlphaOutput(0) + m_arm->getReflex(0)->getAlphaOutput(1));
        m_coconAtStart[1] = 0.5 * (m_arm->getReflex(1)->getAlphaOutput(0) + m_arm->getReflex(1)->getAlphaOutput(1));
        assert(m_coconAtStart[0] == m_coconAtStart[0]);
        assert(m_coconAtStart[1] == m_coconAtStart[1]);
        m_coconStarted = true;
      }
      
      // End of increase
      float coconTargetStop = m_desiredTrajectory[coconTarget.id + 1].time;
      if((coconTargetStop - m_time) <= dt)
      {
        double coconAtEnd [2];
        coconAtEnd[0] = 0.5 * (m_arm->getReflex(0)->getAlphaOutput(0) + m_arm->getReflex(0)->getAlphaOutput(1));
        coconAtEnd[1] = 0.5 * (m_arm->getReflex(1)->getAlphaOutput(0) + m_arm->getReflex(1)->getAlphaOutput(1));
        double totalInc = 0.5 * ((coconAtEnd[0] - m_coconAtStart[0]) + (coconAtEnd[1] - m_coconAtStart[1]));
        assert(totalInc == totalInc);
        
        //m_coconIncrements[coconTarget.id/2-1] = totalInc; // Normalise to [0,1]. Can be 1 at max per reflex, so 2 in total.
        m_coconIncrements.push_back(totalInc); // Normalise to [0,1]. Can be 1 at max per reflex, so 2 in total.
        m_coconStarted = false;
      }
    }
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
  
  ci::XmlTree rampDur ("RampDurationFactor", toString(m_rampDurationFactor));
  evolvable.push_back(rampDur);
  
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
