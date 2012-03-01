/*
 *  ArmReflex.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 10/14/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


#include "ArmReflex.h"
#include "Reflex.h"

#include "MathUtils.h"

#include "cinder/xml.h"

namespace dmx
{

  //----------------------------------------------------------------------------------------------------------------------  
ArmReflex::~ArmReflex()
{
  for (int i = 0; i < m_reflexes.size(); i++)
  {
    delete m_reflexes[i];
  }
  m_reflexes.clear();
}
  
//----------------------------------------------------------------------------------------------------------------------  
void ArmReflex::init()
{
  // First let base of arm init
  ArmMuscled::init();
  
  ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/Arm"))
  {
    // Todo: load stored reflex params
  }
  
  // Todo: temporary test of reflex control
  m_reflexes.push_back(new Reflex(m_muscles[0], m_muscles[1]));
  m_reflexes.push_back(new Reflex(m_muscles[2], m_muscles[3]));
}

//----------------------------------------------------------------------------------------------------------------------
void ArmReflex::resetTo(double elbAngle, double shdAngle)
{
  // Let base class do its reset first.
  ArmMuscled::resetTo(elbAngle, shdAngle);  
  
  for(int i = 0; i < m_reflexes.size(); ++i)
  {  
    m_reflexes[i]->reset();    
  }
  
  m_desiredPos = Pos(0,0);
  m_desiredAngles[JT_elbow] = m_desiredAngles[JT_shoulder] = 0.0;
  
  m_desiredTrajectory.empty();
}

//----------------------------------------------------------------------------------------------------------------------
void ArmReflex::update(Pos pos, float dt, int elbPos)
{
  // Get desired joint angles from desired position
  m_desiredPos = pos;
  inverseKinematics(m_desiredPos, elbPos, m_desiredAngles[JT_elbow], m_desiredAngles[JT_shoulder]);
  m_desiredAngles[JT_elbow] = clamp(m_desiredAngles[JT_elbow], getJointLimitLower(JT_elbow), getJointLimitUpper(JT_elbow));  
  m_desiredAngles[JT_shoulder] = clamp(m_desiredAngles[JT_shoulder], getJointLimitLower(JT_shoulder), getJointLimitUpper(JT_shoulder));  
  
  // Set reflex desired state  
  for(int i = 0; i < m_reflexes.size(); i++)
  {
    m_reflexes[i]->setDesiredAngles(m_desiredAngles[JT_elbow], m_desiredAngles[JT_shoulder]);
  }
  
  // Now let the normal update take over, this will also update the reflexes and muscles  
  update(dt);
  
  // Store desired trajectory
  m_desiredTrajectory.push_back(m_desiredPos);
  if(m_desiredTrajectory.size() >= MaxTrajPoints)
  {
    m_desiredTrajectory.pop_front();
  }   
}

//----------------------------------------------------------------------------------------------------------------------  
void ArmReflex::update(double desElbAngle, double desShdAngle, float dt)
{
  setDesiredJointAngle(JT_elbow, desElbAngle);
  setDesiredJointAngle(JT_shoulder, desShdAngle);
  
  // Set reflex desired state
  for(int i = 0; i < m_reflexes.size(); i++)
  {
    m_reflexes[i]->setDesiredAngles(m_desiredAngles[JT_elbow], m_desiredAngles[JT_shoulder]);
  }
  
  // Now let the normal update take over, this will also update the reflexes and muscles
  update(dt);
  
  // Store desired trajectory (forward kinematics)
  Pos tmp;    
  forwardKinematics(m_desiredAngles[JT_elbow], m_desiredAngles[JT_shoulder], tmp, m_desiredPos);  
  m_desiredTrajectory.push_back(m_desiredPos);
  if(m_desiredTrajectory.size() >= MaxTrajPoints)
  {
    m_desiredTrajectory.pop_front();
  }     
}

//----------------------------------------------------------------------------------------------------------------------  
void ArmReflex::update(double lengthElb0, double lengthElb1, double lengthShd0, double lengthShd1, float dt)
{
  // Set reflex desired state
  m_reflexes[0]->setDesiredLength(lengthElb0, lengthElb1);
  m_reflexes[1]->setDesiredLength(lengthShd0, lengthShd1);
  
  // Now let the normal update take over, this will also update the reflexes and muscles
  update(dt);
}

//----------------------------------------------------------------------------------------------------------------------
void ArmReflex::update(float dt)
{
  // Run reflexes: these will set the muscles' activations  
  for(int i = 0; i < m_reflexes.size(); i++)
  {
    m_reflexes[i]->update(dt);
  }
  
  // Update arm
  ArmMuscled::update(dt);
  
  // Feed some information from arm kinematics or dynamics back to reflexes
  // Elbow reflex receives torque information from shoulder and vice versa
  m_reflexes[0]->setIntersegmentInput(m_state.torques[JT_shoulder], m_state.torques[JT_shoulder]);
  m_reflexes[1]->setIntersegmentInput(m_state.torques[JT_elbow], m_state.torques[JT_elbow]);
}

//----------------------------------------------------------------------------------------------------------------------  
void ArmReflex::toXml(ci::XmlTree& xml)
{
  ArmMuscled::toXml(xml);
  
  if(xml.hasChild("Arm"))
  {
    ci::XmlTree& arm = xml.getChild("Arm");
    
    // Write out reflex data
    for(int i = 0; i < m_reflexes.size(); i++)
    {
      m_reflexes[i]->toXml(arm);      
    }
  }
}
  
//----------------------------------------------------------------------------------------------------------------------     
void ArmReflex::record(Recorder& recorder) 
{ 
  // Let muscles record themselves
  ArmMuscled::record(recorder); 
  
  for(int i = 0; i < m_reflexes.size(); i++)
  {
    m_reflexes[i]->record(recorder);
  }

  recorder.push_back("comX", m_desiredPos.x);
  recorder.push_back("comY", m_desiredPos.y);
  recorder.push_back("comElbow", m_desiredAngles[JT_elbow]);
  recorder.push_back("comShoulder", m_desiredAngles[JT_shoulder]);
}

} // namespace dmx
