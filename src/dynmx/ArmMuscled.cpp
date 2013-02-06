/*
 *  ArmMuscled.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ArmMuscled.h"
#include "MuscleMonoWrap.h"
#include "MuscleBiWrap.h"
#include "EPController.h"

#include "MathUtils.h"

#include "cinder/xml.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
ArmMuscled::~ArmMuscled()
{
  for(int i = 0; i < m_muscles.size(); i++)
  {
    delete m_muscles[i];
  }
  
  m_muscles.clear();
}
  
//----------------------------------------------------------------------------------------------------------------------  
void ArmMuscled::init()
{
  // First let base of arm init
  Arm::init();
  
  ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/Arm"))
  {
    // Use setting from globals file
    const ci::XmlTree& xml = settings->getChild("Config/Arm");
    m_jointRadius[JT_elbow] = xml.getChild("Elbow").getAttributeValue<double>("Radius");
    m_jointRadius[JT_shoulder] = xml.getChild("Shoulder").getAttributeValue<double>("Radius");
    
    ci::XmlTree::Iter muscle = settings->begin("Config/Arm/Muscle");
    for (; muscle != settings->end(); ++muscle)
    {
      // Parameters common to mono- and biarticulate muscles
      std::string name = muscle->getAttributeValue<std::string>("Name");            
      bool isFlexor = muscle->getAttributeValue<bool>("IsFlexor");      
      double origin = muscle->getChild("Attachment").getAttributeValue<double>("Origin");
      double insertion = muscle->getChild("Attachment").getAttributeValue<double>("Insertion");
      double Fmax = muscle->getChild("MaxIsoForce").getValue<double>();
      double L0 = muscle->getChild("Length").getAttributeValue<double>("Optimal");
      double Vmax = muscle->getChild("MaxVelocity").getValue<double>();
      
      if(muscle->getAttributeValue<bool>("IsMono"))
      {
        // Monoarticulate muscles
        std::string jointName = muscle->getAttributeValue<std::string>("Joint"); 
        Joint joint = (jointName == "Elbow") ? JT_elbow : JT_shoulder;

        MuscleMonoWrap* monoMuscle = new MuscleMonoWrap(this, origin, insertion, joint, isFlexor);
        monoMuscle->setParameters(Fmax, L0, Vmax);
        monoMuscle->setName(name);
        monoMuscle->init();
        m_muscles.push_back(monoMuscle);
      }
      else
      {
        // Biarticulate muscles
        MuscleBiWrap* biMuscle = new MuscleBiWrap(this, origin, insertion, isFlexor);
        biMuscle->setParameters(Fmax, L0, Vmax);
        biMuscle->setName(name);
        biMuscle->init();
        m_muscles.push_back(biMuscle);        
      }
      
      // Hill Params (optional)
      if(muscle->hasChild("HillParameters"))
      {
        const ci::XmlTree& hillParams = muscle->getChild("HillParameters");
        double sh = hillParams.getAttributeValue<double>("Shortening");
        double lg = hillParams.getAttributeValue<double>("Lengthening");
        double asymp = hillParams.getAttributeValue<double>("Asymptote");
        double slope = hillParams.getAttributeValue<double>("Slope");
        m_muscles.back()->setHillParameters(sh, lg, asymp, slope);
      }
      
    } // for muscles
  } // if xml
  else 
  {
    m_jointRadius[JT_elbow] = 0.04;
    m_jointRadius[JT_shoulder] = 0.05;  
    
    m_muscles.clear();
    
    // Todo: Temporarily create some muscles here
    // Monoarticulate shoulder muscles
    m_muscles.push_back(new MuscleMonoWrap(this, 0.13, 0.1, JT_shoulder, true));
    m_muscles[0]->setName("ShoulderFlexor");
    m_muscles.push_back(new MuscleMonoWrap(this, 0.1, 0.12, JT_shoulder, false));
    m_muscles[1]->setName("ShoulderExtensor");
    
    // Monoarticulate elbow muscles
    m_muscles.push_back(new MuscleMonoWrap(this, 0.1, 0.2, JT_elbow, true));
    m_muscles[2]->setName("ElbowFlexor");
    m_muscles.push_back(new MuscleMonoWrap(this, 0.2, 0.075, JT_elbow, false));  
    m_muscles[3]->setName("ElbowExtensor");
    
    // Biarticulate muscles
    m_muscles.push_back(new MuscleBiWrap(this, 0.25, 0.15, true));
    m_muscles[4]->setName("BiArtFlexor");
    m_muscles.push_back(new MuscleBiWrap(this, 0.25, 0.15, false));
    m_muscles[5]->setName("BiArtExtensor");
    
    for(int i = 0; i < m_muscles.size(); ++i)
    {
      m_muscles[i]->init();
    }    
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
void ArmMuscled::resetTo(double elbAngle, double shdAngle)
{
  // Let base class do its reset first.
  Arm::resetTo(elbAngle, shdAngle);  
  for(int i = 0; i < m_muscles.size(); ++i)
  {
    m_muscles[i]->reset();
  }   
}
  
//----------------------------------------------------------------------------------------------------------------------
void ArmMuscled::update(float dt)
{
  
  // Accumulate joint torques
  double elbTorque = 0, shdTorque = 0;
  
  // Muscles return torques to apply to joints they span.
  for(size_t i = 0; i < m_muscles.size(); ++i)
  {
    m_muscles[i]->update(dt);
    
    int sign = m_muscles[i]->isFlexor() ? 1 : -1;
    if(m_muscles[i]->isMonoArticulate())
    {
      // Monoarticulate muscle
      MuscleMonoWrap* m = ((MuscleMonoWrap*)m_muscles[i]);
      if(m->getJoint() == JT_elbow)
      {
        elbTorque += sign * m->getMomentArm() * m->getForce(); 
      }
      else
      {
        shdTorque += sign * m->getMomentArm() * m->getForce(); 
      }
    }
    else 
    {
      // Biarticulate muscle      
      MuscleBiWrap* m = ((MuscleBiWrap*)m_muscles[i]);
      elbTorque += sign * m->getMomentArm(JT_elbow) * m->getForce();
      shdTorque += sign * m->getMomentArm(JT_shoulder) * m->getForce();      
    }
  }
  
  Arm::update(dt, elbTorque, shdTorque);
}
  
//----------------------------------------------------------------------------------------------------------------------  
void ArmMuscled::toXml(ci::XmlTree& xml)
{
  Arm::toXml(xml);
  
  if(xml.hasChild("Arm"))
  {
    ci::XmlTree& arm = xml.getChild("Arm");
    arm.getChild("Shoulder").setAttribute("Radius", m_jointRadius[JT_shoulder]);
    arm.getChild("Elbow").setAttribute("Radius", m_jointRadius[JT_elbow]);  
    
    // Write out muscle data
    for(int i = 0; i < m_muscles.size(); i++)
    {
      m_muscles[i]->toXml(arm);
    }
  }
}
  
//----------------------------------------------------------------------------------------------------------------------    
void ArmMuscled::record(Recorder& recorder)
{
  Arm::record(recorder);
  
  for(int i = 0; i < m_muscles.size(); i++)
  {
    m_muscles[i]->record(recorder);
  }
}
  
  
//----------------------------------------------------------------------------------------------------------------------
void ArmMuscled::recordIsometric(Recorder& recorder, const std::vector<float>& muscleAct, float jointIncDeg)
{
  const float dt = 0.01f; // Arbitrary really
  float jointIncRad = degreesToRadians(jointIncDeg);
  
  // Updating muscle with same dt as time constant means activation will instantly be same as excitation, 
  // i.e. no slow dynamics. Will be reset at end of this function.
  for(size_t i = 0; i < m_muscles.size(); ++i)
  {
    m_muscles[i]->setTimeConstants(dt, dt);
  }

  // Loop over range of shoulder and elbow angles  
  double elbMax = getJointLimitUpper(JT_elbow);
  double shdMax = getJointLimitUpper(JT_shoulder);
  double elbAngle = getJointLimitLower(JT_elbow);    
  double shdAngle = getJointLimitLower(JT_shoulder);  

  // First elbow
  while(elbAngle < elbMax)
  {
    // "Move" kinematically
    resetTo(elbAngle, shdAngle);
    
    // Set excitation and update
    m_muscles[0]->setExcitation(muscleAct[0]);
    m_muscles[1]->setExcitation(muscleAct[1]);
    
    m_muscles[0]->update(dt);
    m_muscles[1]->update(dt);    
    
    // Store state
    record(recorder);
    
    elbAngle += jointIncRad;
  }  
  
  // Now shoulder
  while(shdAngle < shdMax)
  {
    // "Move" kinematically
    resetTo(elbAngle, shdAngle);

    // Set excitation and update
    m_muscles[2]->setExcitation(muscleAct[2]);
    m_muscles[3]->setExcitation(muscleAct[3]);    
    
    m_muscles[2]->update(dt);
    m_muscles[3]->update(dt);    
    
    // Store state
    record(recorder);
    
    shdAngle += jointIncRad;
  }  
  
#if 0  
  while(shdAngle < shdMax)
  {
    double elbAngle = m_limits[JT_elbow][0];
    while(elbAngle < elbMax)
    {
      // "Move" kinematically
      resetTo(elbAngle, shdAngle);
      
      // Updating muscle with same dt as time constant means activation will instantly be same as excitation, 
      // i.e. no slow dynamics
      for(size_t i = 0; i < m_muscles.size(); ++i)
      {
        m_muscles[i]->update(dt);        
      }
      
      // Store state
      record(recorder);
      
      elbAngle += jointIncRad;
    }
    shdAngle += jointIncRad;
  }
#endif
  
  // Reset to default time constants
  for(size_t i = 0; i < m_muscles.size(); ++i)
  {
    m_muscles[i]->setTimeConstants(MUSCLE_DEFAULT_TIMECONST_ACT, MUSCLE_DEFAULT_TIMECONST_DEACT);
  }
}

} // namespace dmx
