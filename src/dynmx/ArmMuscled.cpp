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
#include "Reflex.h"

#include "MathUtils.h"

#include "cinder/xml.h"

namespace dmx
{

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
    for (muscle; muscle != settings->end(); ++muscle)
    {
      // Parameters common to mono- and biarticulate muscles
      bool isFlexor = muscle->getAttributeValue<bool>("IsFlexor");      
      double origin = muscle->getChild("Origin").getAttributeValue<double>("Value");
      double insertion = muscle->getChild("Insertion").getAttributeValue<double>("Value");
      double Fmax = muscle->getChild("MaxIsoForce").getAttributeValue<double>("Value");
      double L0 = muscle->getChild("OptimalLength").getAttributeValue<double>("Value");
      double Vmax = muscle->getChild("MaxVelocity").getAttributeValue<double>("Value");
      std::string name = muscle->getAttributeValue<std::string>("Name");      
      
      if(muscle->getAttributeValue<bool>("IsMono"))
      {
        // Monoarticulate muscles
        std::string jointName = muscle->getChild("Joint").getAttributeValue<std::string>("Value"); 
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
    }
  }
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
  
  // Todo: temporary test code
  for (int i = 0; i < m_muscles.size(); ++i) 
  {
    m_epControllers.push_back(new EPController(m_muscles[i], 4.0f, 0.5f, 0.1f, 0));
    m_epControllers[i]->init();
  }
  
  // Todo: temporary test of reflex control
  m_reflexes.push_back(new Reflex(m_muscles[0], m_muscles[1]));
  m_reflexes.push_back(new Reflex(m_muscles[2], m_muscles[3]));
  m_reflexes[0]->init();
  m_reflexes[1]->init();
}
  
//----------------------------------------------------------------------------------------------------------------------
void ArmMuscled::reset(float elbAngle, float shdAngle)
{
  // Let base class do its reset first.
  Arm::reset(elbAngle, shdAngle);  
  for(int i = 0; i < m_muscles.size(); ++i)
  {
    m_muscles[i]->reset();
  }   
  
  for(int i = 0; i < m_epControllers.size(); ++i)
  {  
    m_epControllers[i]->reset();    
  }
  
  for(int i = 0; i < m_reflexes.size(); ++i)
  {  
    m_reflexes[i]->reset();    
  }
  
  m_desiredPos = Pos(0,0);
  m_desiredAngles[JT_elbow] = m_desiredAngles[JT_shoulder] = 0.0;
}

//----------------------------------------------------------------------------------------------------------------------
void ArmMuscled::updatePosition(Pos pos, float dt, int elbPos)
{
  // Get desired joint angles from desired position
  m_desiredPos = pos;
  inverseKinematics(m_desiredPos, elbPos, m_desiredAngles[JT_elbow], m_desiredAngles[JT_shoulder]);
  // Clamp joint angles to limits
  m_desiredAngles[JT_elbow] = clamp(m_desiredAngles[JT_elbow], getJointLimitLower(JT_elbow), getJointLimitUpper(JT_elbow));
  m_desiredAngles[JT_shoulder] = clamp(m_desiredAngles[JT_shoulder], getJointLimitLower(JT_shoulder), getJointLimitUpper(JT_shoulder));  
  
  /*
  for(size_t i = 0; i <  m_epControllers.size(); ++i)
  {
    double desLength = getMuscle(i)->getLengthFromJointAngles(m_desiredAngles[JT_elbow], m_desiredAngles[JT_shoulder]);
    m_epControllers[i]->setDesiredLength(desLength);
    m_epControllers[i]->update(dt);
    const double excitation = m_epControllers[i]->getActivation();
    getMuscle(i)->setExcitation(excitation);
  }
  */
  
  // Run reflexes: these will set the muscles' activations  
  for(int i = 0; i < m_reflexes.size(); i++)
  {
    m_reflexes[i]->update(dt);
  }
  
  // Now let the normal update take over, this will also update the muscles
  update(dt);
  
  // Store desired trajectory
  m_desiredTrajectory.push_back(m_desiredPos);
  if(m_trajectory.size() > MaxTrajPoints)
  {
    m_trajectory.pop_front();
  }  
}
  
//----------------------------------------------------------------------------------------------------------------------
void ArmMuscled::update(float dt)
{
  float elbTorque = 0, shdTorque = 0;
  
  // Test frame order: 
  //update(elbTorque, shdTorque, dt);
  
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

} // namespace dmx
