/*
 *  Reflex.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/20/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Reflex.h"
#include "ArmMuscled.h"
#include "MathUtils.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
void Reflex::init()
{
  // Spindle positional gain  
  m_Kspp[0] = 1.0;
  m_Kspp[1]  = 1.5;    
  
  // Spindle velocity gain
  m_Kspv[0] = 0.03;
  m_Kspv[1] = 0.05;    
  
  // Spindle velocity exponent 
  m_Espv[0] = 1.0;
  m_Espv[1] = 1.0; 
  
  // Strength and speed of load compensation
  m_Ksfv[0] = 0.0;
  m_Ksfv[1] = 0.0;
  // Reciprocal inhibition of load compensation
  m_Ksfi[0] = 0.0;
  m_Ksfi[0] = 0.0;   
  
  // IaIn Ia inhibitory interneurons
  m_Kiain[0] = 0.0;
  m_Kiain[1] = 0.0;
  // Reciprocal inhibition of same
  m_Kiari[0] = 0.0;
  m_Kiari[1] = 0.0;
  // Integration time constant (really 1/t)  
  m_Tiain[0] = m_Tiain[1] = 50.0;  

  // Inertial force compensation (velocity error)
  m_Kifv[0] = m_Kifv[1] = 0.0;
  m_Bifv[0] = m_Bifv[1] = 0.0;
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------      
void Reflex::reset()
{
  m_IaIn[0] = m_IaIn[1] = 0.0;
  m_sfv[0] = m_sfv[1] = 0.0;  
  m_ifv[0] = m_ifv[1] = 0.0;
  m_alpha[0] = m_alpha[1] = 0.0;
  m_ofpv[0] = m_ofpv[1] = 0.0;
  
  m_desiredLength[0] = m_muscles[0]->getUnitLength();
  m_desiredLength[1] = m_muscles[1]->getUnitLength();
  
  m_desiredLengthPrev[0] = m_desiredLength[0];
  m_desiredLengthPrev[1] = m_desiredLength[1];
  
  m_lengthPrev[0] = m_muscles[0]->getUnitLength();
  m_lengthPrev[1] = m_muscles[1]->getUnitLength();    
  
  m_cocontraction[0] = m_cocontraction[1] = 0.0;
}

//----------------------------------------------------------------------------------------------------------------------    
void Reflex::setDesiredLength(double l0, double l1)  
{ 
  m_desiredLengthPrev[0] = m_desiredLength[0]; 
  m_desiredLength[0] = m_muscles[0]->lengthToUnitLength(l0);
  
  m_desiredLengthPrev[1] = m_desiredLength[1]; 
  m_desiredLength[1] = m_muscles[1]->lengthToUnitLength(l1);
}
  
//----------------------------------------------------------------------------------------------------------------------  
void Reflex::update(float dt)
{
  // Get desired muscle lengths in unit range [0,1] from desired joint angles
  const double desElbAng = getArm()->getDesiredJointAngle(JT_elbow);
  const double desShdAng = getArm()->getDesiredJointAngle(JT_shoulder);
  double desLength0 = m_muscles[0]->getLengthFromJointAngles(desElbAng, desShdAng);
  double desLength1 = m_muscles[1]->getLengthFromJointAngles(desElbAng, desShdAng);
  setDesiredLength(desLength0, desLength1);   
  
  m_desiredVelocity[0] = (m_desiredLength[0] - m_desiredLengthPrev[0]) / dt;
  m_desiredVelocity[1] = (m_desiredLength[1] - m_desiredLengthPrev[1]) / dt;
  
  // Actual current values
  m_length[0] = m_muscles[0]->getUnitLength();
  m_length[1] = m_muscles[1]->getUnitLength();
  
  m_vel[0] = (m_length[0] - m_lengthPrev[0]) / dt;
  m_vel[1] = (m_length[1] - m_lengthPrev[1]) / dt;
  
  m_lengthPrev[0] = m_length[0];
  m_lengthPrev[1] = m_length[1];  
  
  // Gamma motor neuron inputs
  const double gsGain[2] = {1.0, 1.0}; // Static gamma neuron gain (Big Xi)
  const double gdGain[2] = {1.0, 1.0}; // Dynamic gamma neuron gain (ro  0
  
  double gammaStatic[2];  
  gammaStatic[0] = m_desiredLength[0] * gsGain[0];
  gammaStatic[1] = m_desiredLength[1] * gsGain[1];
  
  double gammaDyn[2];
  gammaDyn[0] = m_desiredVelocity[0] * gdGain[0];
  gammaDyn[1] = m_desiredVelocity[1] * gdGain[1];
  
  // Spindle-calculated errors in length and velocity
  m_posErr[0] = m_length[0] - gammaStatic[0];
  m_posErr[1] = m_length[1] - gammaStatic[1];
  m_posErr[0] = max(m_posErr[0], 0.0);    
  m_posErr[1] = max(m_posErr[1], 0.0);
  
  // TODO: Temporary, until the desired velocity always changes smoothly
  m_velErr[0] = m_vel[0] - gammaDyn[0];
  m_velErr[1] = m_vel[1] - gammaDyn[1];
  //m_velErr[0] = m_vel[0];
  //m_velErr[1] = m_vel[1];  
  m_velErr[0] = max(m_velErr[0], 0.0);  
  m_velErr[1] = max(m_velErr[1], 0.0);
  
  m_spindleSec[0] = spindleActivation(m_Kspp[0] * m_posErr[0]);
  m_spindleSec[1] = spindleActivation(m_Kspp[1] * m_posErr[1]);
  
  m_spindlePri[0] = spindleActivation(m_Kspp[0] * m_posErr[0] + m_Kspv[0] * powf(m_velErr[0], m_Espv[0]));
  m_spindlePri[1] = spindleActivation(m_Kspp[1] * m_posErr[1] + m_Kspv[1] * powf(m_velErr[1], m_Espv[1]));
  
  // Ia inhibitory interneurons
  double dIaIn [2];
//  dIaIn[0] = (m_Kiain[0] * (10 - m_IaIn[0]) * ((1-m_desiredLength[0]-m_cocontraction[0]) + m_spindlePri[0])) - ((m_IaIn[0] + 1) * (1 + max(0.0, m_IaIn[1])));
//  dIaIn[1] = (m_Kiain[1] * (10 - m_IaIn[1]) * ((1-m_desiredLength[1]-m_cocontraction[1]) + m_spindlePri[1])) - ((m_IaIn[1] + 1) * (1 + max(0.0, m_IaIn[0])));
//  m_IaIn[0] += dt * dIaIn[0];
//  m_IaIn[1] += dt * dIaIn[1];
  
  dIaIn[0] = -m_IaIn[0] + m_spindlePri[0] - m_Kiari[0] * max(0.0, m_IaIn[1]);
  dIaIn[1] = -m_IaIn[1] + m_spindlePri[1] - m_Kiari[1] * max(0.0, m_IaIn[0]);
  //dIaIn[0] = (-m_IaIn[0] * max(0.0, m_IaIn[1]) + ((1.0 - m_IaIn[0]) * m_spindlePri[0]));
  //dIaIn[1] = (-m_IaIn[1] * max(0.0, m_IaIn[0]) + ((1.0 - m_IaIn[1]) * m_spindlePri[1]));
  m_IaIn[0] += dt * m_Tiain[0] * dIaIn[0];
  m_IaIn[1] += dt * m_Tiain[1] * dIaIn[1];
  
  
  
  // Inertial force vector: adds extra impulse to overcome inertia based on velocity error
  // (mostly at beginning and end of trajectory)
  m_ifv[0] = - m_Bifv[0] + spindleActivation(m_Kspv[0] * powf(m_velErr[0], m_Espv[0])); // or k * (spindlePri - spindleSec) ?
  m_ifv[1] = - m_Bifv[1] + spindleActivation(m_Kspv[1] * powf(m_velErr[1], m_Espv[1])); // or k * (spindlePri - spindleSec) ?
  m_ifv[0] = max(m_ifv[0], 0.0) * m_Kifv[0];  
  m_ifv[1] = max(m_ifv[1], 0.0) * m_Kifv[1];
  
  
  // Static force vector: adds force to overcome gravity based on integration of position error
  double dSfv [2];
  dSfv[0] = -m_Ksfi[0] * m_sfv[0] * (m_sfv[1] + m_spindlePri[1]) + (1.0 - m_sfv[0]) * m_Ksfv[0] * m_spindlePri[0];
  dSfv[1] = -m_Ksfi[1] * m_sfv[1] * (m_sfv[0] + m_spindlePri[0]) + (1.0 - m_sfv[1]) * m_Ksfv[1] * m_spindlePri[1];  
  m_sfv[0] += dt * dSfv[0];
  m_sfv[1] += dt * dSfv[1];
  
  
  // Outflow force and position vector: desired contraction (alpha-activation) + static and inertial forces
  m_ofpv[0] = m_ifv[0] + m_sfv[0];// + (m_length[0] - m_desiredLength[0]); 
  m_ofpv[1] = m_ifv[1] + m_sfv[1];// + (m_length[1] - m_desiredLength[1]);
  
  
  // Alpha motor neuron
  const double Ksrg = 1.0; // stretch reflex gain
  m_alpha[0] = m_cocontraction[0] + m_ofpv[0] + Ksrg * m_spindlePri[0] - m_Kiain[0] * max(0.0, m_IaIn[1]);
  m_alpha[1] = m_cocontraction[1] + m_ofpv[1] + Ksrg * m_spindlePri[1] - m_Kiain[1] * max(0.0, m_IaIn[0]);
  //m_alpha[0] = max(m_alpha[0], 0.0);  
  //m_alpha[1] = max(m_alpha[1], 0.0);
  m_alpha[0] = clamp(m_alpha[0], 0.0, 1.0);
  m_alpha[1] = clamp(m_alpha[1], 0.0, 1.0);
  
  m_muscles[0]->setExcitation(m_alpha[0]);
  m_muscles[1]->setExcitation(m_alpha[1]);  
}
  
}

