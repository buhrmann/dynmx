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
#include "MuscleMonoWrap.h"
#include "MathUtils.h"

namespace dmx
{

#define REFLEX_USE_CONTRACTION_COORDS 1
#define REFLEX_USE_OWN_IAIN_IMPL 1
#define OPENLOOP_AS_CCOMMAND 0
  
//----------------------------------------------------------------------------------------------------------------------      
Reflex::Reflex() 
{ 
  m_muscles[0] = m_muscles[1] = 0; 
  init(); 
};


//----------------------------------------------------------------------------------------------------------------------    
Reflex::Reflex(Muscle* ag, Muscle* an) 
{ 
  m_muscles[0] = ag; 
  m_muscles[1] = an; 
  init(); 
};  
  
  
//----------------------------------------------------------------------------------------------------------------------  
void Reflex::init()
{
  // Spindle 
  m_Kspp[0] = m_Kspp[1] = 1.0;  // positional gain
  m_Kspv[0] = m_Kspv[1] = 0.0;  // velocity gain
  m_Kspd[0] = m_Kspd[1] = 0.1;  // damping gain
  m_Espv[0] = m_Espv[1] = 1.0;  // velocity exponent 
  
  // Load compensation
  m_Ksfv[0] = m_Ksfv[1] = 0.0;
  m_Ksfi[0] = m_Ksfi[1] = 0.0;     // Reciprocal inhibition
  
  // IaIn inhibitory interneurons
  m_Wspia[0] = m_Wspia[1] = 0.0;    // Spindle input
  m_Wiaia[0] = m_Wiaia[1] = 0.0;    // Reciprocal inhibition
  m_Wrnia[0] = m_Wrnia[1] = 0.0;    // Renshaw input  
  m_Waia[0]  = m_Waia[1]  = 0.0;    // Desired contraction input
  m_Biain[0] = m_Biain[1] = 0.0;    // Ia Biases
  m_Tiain[0] = m_Tiain[1] = 100.0;  // Integration time constant (really 1/t)  
  
  // Renshaw
  m_Wmnrn[0] = m_Wmnrn[1] = 0.0f;  
  m_Wrnrn[0] = m_Wrnrn[1] = 0.0f;  
  m_Brn[0] = m_Brn[1] = 0.0;
  m_Trn[0] = m_Trn[1] = 100.0;  
  
  // IbIn
  m_Wglib[0] = m_Wglib[1] = 0.0;   // Golgi inpuit
  m_Wibib[0] = m_Wibib[1] = 0.0;   // Reciprocal inhibition
  m_Bib[0] = m_Bib[1] = 0.0;
  m_Tib[0] = m_Tib[1] = 100.0;  
  
  // Alpha motor neuron
  m_Wiamn[0] = m_Wiamn[1] = 0.0f;
  m_Wrnmn[0] = m_Wrnmn[1] = 0.0f;
  m_Wibmn[0] = m_Wibmn[1] = 0.0f;
  m_Wspmn[0] = m_Wspmn[1] = 0.0f;

  // Inertial force compensation (velocity error)
  m_Kifv[0] = m_Kifv[1] = 0.0;
  m_Bifv[0] = m_Bifv[1] = 0.0;
  
  m_Wisep[0] = m_Wisep[1] = 0.0;
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------      
void Reflex::reset()
{
  m_IaIn[0] = m_IaIn[1] = 0.0;
  m_IaInOut[0] = m_IaInOut[1] = 0.0;
  
  m_Rn[0] = m_Rn[1] = 0.0;
  m_RnOut[0] = m_RnOut[1] = 0.0;  
  
  m_IbIn[0] = m_IbIn[1] = 0.0;
  m_IbInOut[0] = m_IbInOut[1] = 0.0;    
  
  m_sfv[0] = m_sfv[1] = 0.0;  
  m_ifv[0] = m_ifv[1] = 0.0;
  m_alpha[0] = m_alpha[1] = 0.0;
  m_ofpv[0] = m_ofpv[1] = 0.0;
  
  // State in length coordinates
  m_length[0] = m_muscles[0]->getUnitLength();
  m_length[1] = m_muscles[1]->getUnitLength();
  m_lengthPrev[0] = m_length[0];
  m_lengthPrev[1] = m_length[1];      
  m_desiredLength[0] = m_length[0];
  m_desiredLength[1] = m_length[1];  
  m_desiredLengthPrev[0] = m_desiredLength[0];
  m_desiredLengthPrev[1] = m_desiredLength[1];  
  m_desiredVelocity[0] = m_desiredVelocity[1] = 0.0;
  
  // State in contraction coordinates
  m_contraction[0] = 1.0 - m_length[0];
  m_contraction[1] = 1.0 - m_length[1];  
  m_contractionPrev[0] = m_contraction[0];
  m_contractionPrev[1] = m_contraction[1];
  m_desiredContraction[0] = 1.0 - m_desiredLength[0];
  m_desiredContraction[1] = 1.0 - m_desiredLength[1];  
  m_desiredContractionPrev[0] = m_desiredContraction[0];
  m_desiredContractionPrev[1] = m_desiredContraction[1];
  m_desiredContractionVel[0] = m_desiredContractionVel[1] = 0.0f;
  
  // Input state
  m_openLoop[0] = m_openLoop[1] = 0.0;
  m_interSegmentInput[0] = m_interSegmentInput[1] = 0.0;
}

//----------------------------------------------------------------------------------------------------------------------    
void Reflex::setDesiredAngles(double elbAng, double shdAng)  
{
  double desLength0 = m_muscles[0]->getLengthFromJointAngles(elbAng, shdAng);
  double desLength1 = m_muscles[1]->getLengthFromJointAngles(elbAng, shdAng);
  setDesiredLength(desLength0, desLength1);     
}  
  
//----------------------------------------------------------------------------------------------------------------------    
void Reflex::setDesiredLength(double l0, double l1)  
{ 
  // Length coordinates
  m_desiredLengthPrev[0] = m_desiredLength[0]; 
  m_desiredLength[0] = m_muscles[0]->lengthToUnitLength(l0);
  
  m_desiredLengthPrev[1] = m_desiredLength[1]; 
  m_desiredLength[1] = m_muscles[1]->lengthToUnitLength(l1);
  
#if OPENLOOP_AS_CCOMMAND  
  m_desiredLength[0] = clamp(m_desiredLength[0] - m_openLoop[0], 0.0, 1.0);
  m_desiredLength[1] = clamp(m_desiredLength[1] - m_openLoop[0], 0.0, 1.0);
#endif
  
  // Contraction coordinates
  m_desiredContractionPrev[0] = m_desiredContraction[0]; 
  m_desiredContraction[0] = 1.0 - m_desiredLength[0];
  
  m_desiredContractionPrev[1] = m_desiredContraction[1];   
  m_desiredContraction[1] = 1.0 - m_desiredLength[1];
}
  
//----------------------------------------------------------------------------------------------------------------------  
void Reflex::update(float dt)
{   
  // Store original commands before internal modification
  m_commandedLength[0] = m_desiredLength[0];
  m_commandedLength[1] = m_desiredLength[1];
  m_commandedContraction[0] = m_desiredContraction[0];
  m_commandedContraction[1] = m_desiredContraction[1];
  
  // Intersegmental torque feedback modifies virtual (desired) EP
  m_desiredLength[0] -= m_Wisep[0] * m_interSegmentInput[0];  // +
  m_desiredLength[1] += m_Wisep[1] * m_interSegmentInput[1];  // +
  
  m_desiredContraction[0] += m_Wisep[0] * m_interSegmentInput[0]; // -
  m_desiredContraction[1] -= m_Wisep[1] * m_interSegmentInput[1]; // -
  
  // Calculate desired velocities
  m_desiredVelocity[0] = (m_desiredLength[0] - m_desiredLengthPrev[0]) / dt;
  m_desiredVelocity[1] = (m_desiredLength[1] - m_desiredLengthPrev[1]) / dt;
  m_desiredContractionVel[0] = (m_desiredContraction[0] - m_desiredContractionPrev[0]) / dt;
  m_desiredContractionVel[1] = (m_desiredContraction[1] - m_desiredContractionPrev[1]) / dt;

  // Actual current position and velocities
  m_lengthPrev[0] = m_length[0];
  m_lengthPrev[1] = m_length[1];  
  m_contractionPrev[0] = m_contraction[0];
  m_contractionPrev[1] = m_contraction[1];    
  
  m_length[0] = m_muscles[0]->getUnitLength();
  m_length[1] = m_muscles[1]->getUnitLength();
  m_contraction[0] = 1.0 - m_length[0];
  m_contraction[1] = 1.0 - m_length[1];
  
  m_vel[0] = (m_length[0] - m_lengthPrev[0]) / dt;
  m_vel[1] = (m_length[1] - m_lengthPrev[1]) / dt;
  m_contractionVel[0] = (m_contraction[0] - m_contractionPrev[0]) / dt;
  m_contractionVel[1] = (m_contraction[1] - m_contractionPrev[1]) / dt;
  
  // Normalised force sensor
  double f = m_muscles[0]->getForce();
  double fmax = m_muscles[0]->getForceMax();  
  m_golgi[0] =  f / fmax;
  f = m_muscles[1]->getForce();
  fmax = m_muscles[1]->getForceMax();  
  m_golgi[1] =  f / fmax;
  
#if REFLEX_USE_CONTRACTION_COORDS  
  updateInContractionCoords(dt);
#else
  updateInLengthCoords(dt);
#endif
}
 
//----------------------------------------------------------------------------------------------------------------------  
void Reflex::updateInLengthCoords(float dt)
{
  const double gsGain[2] = {1.0, 1.0}; // Static gamma neuron gain (Big Xi)
  const double gdGain[2] = {1.0, 1.0}; // Dynamic gamma neuron gain (ro)
  
  double gammaStatic[2];  
  gammaStatic[0] = m_desiredLength[0] * gsGain[0];
  gammaStatic[1] = m_desiredLength[1] * gsGain[1];
  
  double gammaDyn[2];
  gammaDyn[0] = m_desiredVelocity[0] * gdGain[0];
  gammaDyn[1] = m_desiredVelocity[1] * gdGain[1];
  
  // Spindle-calculated errors in length and velocity
  m_posErr[0] = m_length[0] - gammaStatic[0];
  m_posErr[1] = m_length[1] - gammaStatic[1];
  m_posErr[0] = std::max(m_posErr[0], 0.0);    
  m_posErr[1] = std::max(m_posErr[1], 0.0);
  
  // TODO: Temporary, until the desired velocity always changes smoothly
  m_velErr[0] = m_vel[0] - gammaDyn[0];
  m_velErr[1] = m_vel[1] - gammaDyn[1];
  m_velErr[0] = std::max(m_velErr[0], 0.0);  
  m_velErr[1] = std::max(m_velErr[1], 0.0);
  
  m_spindleSec[0] = spindleActivation(m_Kspp[0] * m_posErr[0]);
  m_spindleSec[1] = spindleActivation(m_Kspp[1] * m_posErr[1]);
  
  m_spindlePri[0] = spindleActivation((m_Kspp[0] * m_posErr[0]) + 
                                      (m_Kspv[0] * powf(m_velErr[0], m_Espv[0])) + 
                                      (m_Kspd[0] * powf(std::max(m_vel[0], 0.0), m_Espv[0])));
  
  m_spindlePri[1] = spindleActivation((m_Kspp[1] * m_posErr[1]) + 
                                      (m_Kspv[1] * powf(m_velErr[1], m_Espv[1])) + 
                                      (m_Kspd[1] * powf(std::max(m_vel[1], 0.0), m_Espv[1])));
  
  // Ia inhibitory interneurons
  double dIaIn [2];
  double dRn [2];
  double dIbIn [2];
#if REFLEX_USE_OWN_IAIN_IMPL
  dIaIn[0] = -m_IaIn[0] + (m_Wspia[0] * m_spindlePri[0]) + (m_Waia[0] * (1.0 - m_desiredLength[0])) - (m_Wiaia[0] * m_IaInOut[1]) - (m_Wrnia[0] * m_RnOut[0]);
  dIaIn[1] = -m_IaIn[1] + (m_Wspia[1] * m_spindlePri[1]) + (m_Waia[1] * (1.0 - m_desiredLength[1])) - (m_Wiaia[1] * m_IaInOut[0]) - (m_Wrnia[1] * m_RnOut[1]);
  m_IaIn[0] += dt * m_Tiain[0] * dIaIn[0];
  m_IaIn[1] += dt * m_Tiain[1] * dIaIn[1];  
  m_IaInOut[0] = neuronActivation(m_IaIn[0] + m_Biain[0]);
  m_IaInOut[1] = neuronActivation(m_IaIn[1] + m_Biain[1]);
  
  dRn[0] = -m_Rn[0] + (m_Wmnrn[0] * m_alpha[0]) - (m_Wrnrn[0] * m_RnOut[1]);
  dRn[1] = -m_Rn[1] + (m_Wmnrn[1] * m_alpha[1]) - (m_Wrnrn[1] * m_RnOut[0]);
  m_Rn[0] += dt * m_Trn[0] * dRn[0];
  m_Rn[1] += dt * m_Trn[1] * dRn[1];
  m_RnOut[0] = neuronActivation(m_Rn[0] + m_Brn[0]);
  m_RnOut[1] = neuronActivation(m_Rn[1] + m_Brn[1]);    
  
  dIbIn[0] = - m_IbIn[0];
  dIbIn[1] = - m_IbIn[1];
  m_IbIn[0] += dt * m_Tib[0] * dIbIn[0];
  m_IbIn[1] += dt * m_Tib[1] * dIbIn[1];
  m_IbInOut[0] = neuronActivation(m_IbIn[0] + m_Bib[0]);
  m_IbInOut[1] = neuronActivation(m_IbIn[1] + m_Bib[1]);  
#else
  dIaIn[0] = ((m_Kiain[0] - m_IaIn[0]) * ((1-m_desiredLength[0]-m_openLoop[0]) + m_spindlePri[0])) - ((m_IaIn[0] + m_Kiari[0]) * (m_Kiari[0] + max(0.0, m_IaIn[1])));
  dIaIn[1] = ((m_Kiain[1] - m_IaIn[1]) * ((1-m_desiredLength[1]-m_openLoop[1]) + m_spindlePri[1])) - ((m_IaIn[1] + m_Kiari[1]) * (m_Kiari[1] + max(0.0, m_IaIn[0])));
  //dIaIn[0] = ((10.0 - m_IaIn[0]) * (m_desiredContraction[0] + m_cocontraction[0] + m_spindlePri[0])) - ((m_IaIn[0] + 1.0) * (1.0 + max(0.0, m_IaIn[1])));
  //dIaIn[1] = ((10.0 - m_IaIn[1]) * (m_desiredContraction[1] + m_cocontraction[1] + m_spindlePri[1])) - ((m_IaIn[1] + 1.0) * (1.0 + max(0.0, m_IaIn[0])));    
  m_IaIn[0] += dt * dIaIn[0];
  m_IaIn[1] += dt * dIaIn[1];  
#endif
  
  
  // Inertial force vector: adds extra impulse to overcome inertia based on velocity error
  // (mostly at beginning and end of trajectory)
  m_ifv[0] = - m_Bifv[0] + spindleActivation(m_Kspv[0] * powf(m_velErr[0], m_Espv[0])); // or k * (spindlePri - spindleSec) ?
  m_ifv[1] = - m_Bifv[1] + spindleActivation(m_Kspv[1] * powf(m_velErr[1], m_Espv[1])); // or k * (spindlePri - spindleSec) ?
  m_ifv[0] = std::max(m_ifv[0], 0.0) * m_Kifv[0];  
  m_ifv[1] = std::max(m_ifv[1], 0.0) * m_Kifv[1];
  
  
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
#if REFLEX_USE_OWN_IAIN_IMPL  
  m_alpha[0] = m_openLoop[0] + m_ofpv[0] + m_Wspmn[0] * m_spindlePri[0] 
  - (m_Wrnmn[0] * m_RnOut[0]) 
  - (m_Wiamn[0] * m_IaInOut[1]) 
  - (m_Wibmn[0] * m_IbInOut[0]);
  
  m_alpha[1] = m_openLoop[1] + m_ofpv[1] + m_Wspmn[1] * m_spindlePri[1] 
  - (m_Wrnmn[1] * m_RnOut[1]) 
  - (m_Wiamn[1] * m_IaInOut[0]) 
  - (m_Wibmn[1] * m_IbInOut[1]);  
#else
  m_alpha[0] = m_openLoop[0] + m_ofpv[0] + m_Wspmn[0] * m_spindlePri[0] - max(0.0, m_IaIn[1]);
  m_alpha[1] = m_openLoop[1] + m_ofpv[1] + m_Wspmn[1] * m_spindlePri[1] - max(0.0, m_IaIn[0]);  
#endif  
  m_alpha[0] = clamp(m_alpha[0], 0.0, 1.0);
  m_alpha[1] = clamp(m_alpha[1], 0.0, 1.0);
  
  m_muscles[0]->setExcitation(m_alpha[0]);
  m_muscles[1]->setExcitation(m_alpha[1]);    
}

//----------------------------------------------------------------------------------------------------------------------  
void Reflex::updateInContractionCoords(float dt)
{
  // Gamma motor neuron inputs
  const double gsGain[2] = {1.0, 1.0}; // Static gamma neuron gain (Big Xi)
  const double gdGain[2] = {1.0, 1.0}; // Dynamic gamma neuron gain (ro  0
  
  double gammaStatic[2];  
  gammaStatic[0] = m_desiredContraction[0] * gsGain[0];
  gammaStatic[1] = m_desiredContraction[1] * gsGain[1];
  
  double gammaDyn[2];
  gammaDyn[0] = m_desiredContractionVel[0] * gdGain[0];
  gammaDyn[1] = m_desiredContractionVel[1] * gdGain[1];
  
  // Spindle-calculated errors in length 
  m_posErr[0] = gammaStatic[0] - m_contraction[0];
  m_posErr[1] = gammaStatic[1] - m_contraction[1];
  //m_posErr[0] = std::max(m_posErr[0], 0.0);    // Rectification only at end of component sum!!!
  //m_posErr[1] = std::max(m_posErr[1], 0.0);
  m_spindlePosRes[0] = m_Kspp[0] * m_posErr[0];
  m_spindlePosRes[1] = m_Kspp[1] * m_posErr[1];
  
  m_spindleSec[0] = spindleActivation(m_spindlePosRes[0]);
  m_spindleSec[1] = spindleActivation(m_spindlePosRes[1]);
  
  // Spindle-calculated errors in velocity
  m_velErr[0] = gammaDyn[0] - m_contractionVel[0];
  m_velErr[1] = gammaDyn[1] - m_contractionVel[1];
  m_spindleVelRes[0] = m_Kspv[0] * m_velErr[0];
  m_spindleVelRes[1] = m_Kspv[1] * m_velErr[1];  
    
  // Non-linear damping term
  double contractionDir0 = sign(m_contractionVel[0]);
  double contractionDir1 = sign(m_contractionVel[1]);
  m_spindleDmpRes[0] = -m_Kspd[0] * contractionDir0 * powf(fabs(m_contractionVel[0]), m_Espv[0]);
  m_spindleDmpRes[1] = -m_Kspd[1] * contractionDir1 * powf(fabs(m_contractionVel[1]), m_Espv[1]);
  
  m_spindlePri[0] = spindleActivation(m_spindlePosRes[0] + m_spindleVelRes[0] + m_spindleDmpRes[0]);
  m_spindlePri[1] = spindleActivation(m_spindlePosRes[1] + m_spindleVelRes[1] + m_spindleDmpRes[1]);
  
  // Ia inhibitory interneurons
  double dIaIn [2];  
  double dRn [2];  
  double dIbIn[2];
#if REFLEX_USE_OWN_IAIN_IMPL
  dIaIn[0] = -m_IaIn[0] + (m_Wspia[0] * m_spindlePri[0]) + (m_Waia[0] * m_desiredContraction[0]) - (m_Wiaia[0] * m_IaInOut[1]) - (m_Wrnia[0] * m_RnOut[0]);
  dIaIn[1] = -m_IaIn[1] + (m_Wspia[1] * m_spindlePri[1]) + (m_Waia[1] * m_desiredContraction[1]) - (m_Wiaia[1] * m_IaInOut[0]) - (m_Wrnia[1] * m_RnOut[1]); 
  m_IaIn[0] += dt * m_Tiain[0] * dIaIn[0];
  m_IaIn[1] += dt * m_Tiain[1] * dIaIn[1];  
  m_IaInOut[0] = neuronActivation(m_IaIn[0] + m_Biain[0]);
  m_IaInOut[1] = neuronActivation(m_IaIn[1] + m_Biain[1]);
  
  dRn[0] = -m_Rn[0] + (m_Wmnrn[0] * m_alpha[0]) - (m_Wrnrn[0] * m_RnOut[1]);
  dRn[1] = -m_Rn[1] + (m_Wmnrn[1] * m_alpha[1]) - (m_Wrnrn[1] * m_RnOut[0]);
  m_Rn[0] += dt * m_Trn[0] * dRn[0];
  m_Rn[1] += dt * m_Trn[1] * dRn[1];
  m_RnOut[0] = neuronActivation(m_Rn[0] + m_Brn[0]);
  m_RnOut[1] = neuronActivation(m_Rn[1] + m_Brn[1]);  
  
  dIbIn[0] = - m_IbIn[0] + (m_Wglib[0] * m_golgi[0]) - (m_Wibib[0] * m_IbInOut[1]);
  dIbIn[1] = - m_IbIn[1] + (m_Wglib[1] * m_golgi[1]) - (m_Wibib[1] * m_IbInOut[0]);
  m_IbIn[0] += dt * m_Tib[0] * dIbIn[0];
  m_IbIn[1] += dt * m_Tib[1] * dIbIn[1];
  m_IbInOut[0] = neuronActivation(m_IbIn[0] + m_Bib[0]);
  m_IbInOut[1] = neuronActivation(m_IbIn[1] + m_Bib[1]);  
#else
  dIaIn[0] = ((m_Kiain[0] - m_IaIn[0]) * (m_desiredContraction[0] + m_openLoop[0] + m_spindlePri[0])) - ((m_IaIn[0] + m_Kiari[0]) * (m_Kiari[0] + max(0.0, m_IaIn[1])));
  dIaIn[1] = ((m_Kiain[1] - m_IaIn[1]) * (m_desiredContraction[1] + m_openLoop[1] + m_spindlePri[1])) - ((m_IaIn[1] + m_Kiari[1]) * (m_Kiari[1] + max(0.0, m_IaIn[0])));
  //dIaIn[0] = ((10.0 - m_IaIn[0]) * (m_desiredContraction[0] + m_cocontraction[0] + m_spindlePri[0])) - ((m_IaIn[0] + 1.0) * (1.0 + max(0.0, m_IaIn[1])));
  //dIaIn[1] = ((10.0 - m_IaIn[1]) * (m_desiredContraction[1] + m_cocontraction[1] + m_spindlePri[1])) - ((m_IaIn[1] + 1.0) * (1.0 + max(0.0, m_IaIn[0])));  
  m_IaIn[0] += dt * dIaIn[0];
  m_IaIn[1] += dt * dIaIn[1];
#endif
  
  
  // Inertial force vector: adds extra impulse to overcome inertia based on velocity error
  // (mostly at beginning and end of trajectory)
  m_ifv[0] = - m_Bifv[0] + spindleActivation(m_Kspv[0] * m_velErr[0]); // or k * (spindlePri - spindleSec) ?
  m_ifv[1] = - m_Bifv[1] + spindleActivation(m_Kspv[1] * m_velErr[1]); // or k * (spindlePri - spindleSec) ?
  m_ifv[0] = std::max(m_ifv[0], 0.0) * m_Kifv[0];  
  m_ifv[1] = std::max(m_ifv[1], 0.0) * m_Kifv[1];
  
  
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
#if REFLEX_USE_OWN_IAIN_IMPL  
  m_alpha[0] =  m_ofpv[0] 
#if !OPENLOOP_AS_CCOMMAND
  + m_openLoop[0] 
#endif
  + (m_Wspmn[0] * m_spindlePri[0])
  - (m_Wrnmn[0] * m_RnOut[0]) 
  - (m_Wiamn[0] * m_IaInOut[1]) 
  - (m_Wibmn[0] * m_IbInOut[0]);
  
  m_alpha[1] = m_ofpv[1] 
#if !OPENLOOP_AS_CCOMMAND  
  + m_openLoop[1] 
#endif
  + (m_Wspmn[1] * m_spindlePri[1])
  - (m_Wrnmn[1] * m_RnOut[1]) 
  - (m_Wiamn[1] * m_IaInOut[0]) 
  - (m_Wibmn[1] * m_IbInOut[1]);    
  
#else
  m_alpha[0] = m_openLoop[0] + m_ofpv[0] + m_Wspmn[0] * m_spindlePri[0] - max(0.0, m_IaIn[1]);
  m_alpha[1] = m_openLoop[1] + m_ofpv[1] + m_Wspmn[1] * m_spindlePri[1] - max(0.0, m_IaIn[0]);  
#endif
  m_alpha[0] = clamp(m_alpha[0], 0.0, 1.0);
  m_alpha[1] = clamp(m_alpha[1], 0.0, 1.0);
  
  m_muscles[0]->setExcitation(m_alpha[0]);
  m_muscles[1]->setExcitation(m_alpha[1]);    
}
  
//----------------------------------------------------------------------------------------------------------------------    
void Reflex::paramToXml(ci::XmlTree& xml, const std::string& str, double* p)
{
  ci::XmlTree params(str, "");
  params.setAttribute("Ag", p[0]);
  params.setAttribute("An", p[1]);
  xml.push_back(params);
}
  
//----------------------------------------------------------------------------------------------------------------------    
void Reflex::paramFromXml(const ci::XmlTree& xml, const std::string& str, double* p)
{
  if(xml.hasChild(str))
  {
    p[0] = xml.getChild(str).getAttributeValue<double>("Ag");
    p[1] = xml.getChild(str).getAttributeValue<double>("An");
  }
}  
  
//----------------------------------------------------------------------------------------------------------------------  
void Reflex::toXml(ci::XmlTree& xml)
{
  
  ci::XmlTree reflex ("Reflex", "");
  reflex.setAttribute("Agonist", m_muscles[0]->getName());
  reflex.setAttribute("Antagonist", m_muscles[1]->getName());

  // Spindle
  ci::XmlTree spindle("Spindle", "");
  paramToXml(spindle, "PosGain", &m_Kspp[0]);
  paramToXml(spindle, "VelGain", &m_Kspv[0]);
  paramToXml(spindle, "DmpGain", &m_Kspd[0]);  
  paramToXml(spindle, "VelExp", &m_Espv[0]);  
  paramToXml(spindle, "Wspmn", &m_Wspmn[0]);  
  reflex.push_back(spindle);

  // Static load compensation  
  ci::XmlTree sfv("Sfv", "");
  paramToXml(sfv, "Gain", &m_Ksfv[0]);
  paramToXml(sfv, "RecInh", &m_Ksfi[0]); 
  reflex.push_back(sfv);  
  
  // Inertial load compensation  
  ci::XmlTree ifv("Ifv", "");
  paramToXml(ifv, "Gain", &m_Kifv[0]);
  paramToXml(ifv, "Bias", &m_Bifv[0]); 
  reflex.push_back(ifv);    

  // IaIn
  ci::XmlTree iain("IaIn", "");
  paramToXml(iain, "Wiaia", &m_Wiaia[0]);
  paramToXml(iain, "Wspia", &m_Wspia[0]);
  paramToXml(iain, "Wrnia", &m_Wrnia[0]);
  paramToXml(iain, "Waia", &m_Waia[0]);
  paramToXml(iain, "Wiamn", &m_Wiamn[0]);  
  paramToXml(iain, "Bias", &m_Biain[0]);  
  paramToXml(iain, "Tau", &m_Tiain[0]);
  reflex.push_back(iain);  
  
  // Renshaw
  ci::XmlTree rn("Renshaw", "");
  paramToXml(rn, "Wmnrn", &m_Wmnrn[0]);
  paramToXml(rn, "Wrnrn", &m_Wrnrn[0]);
  paramToXml(rn, "Wrnmn", &m_Wrnmn[0]);  
  paramToXml(rn, "Bias", &m_Brn[0]);  
  paramToXml(rn, "Tau", &m_Trn[0]);    
  reflex.push_back(rn);    
  
  // IbIn
  ci::XmlTree ibin("IbIn", "");
  paramToXml(ibin, "Wibib", &m_Wibib[0]);
  paramToXml(ibin, "Wglib", &m_Wglib[0]);
  paramToXml(ibin, "Wibmn", &m_Wibmn[0]);
  paramToXml(ibin, "Bias", &m_Bib[0]);  
  paramToXml(ibin, "Tau", &m_Tib[0]);
  reflex.push_back(ibin);    

  // Intersegmental inputs
  ci::XmlTree isin("IsIn", "");
  paramToXml(isin, "Wisep", &m_Wisep[0]);
  reflex.push_back(isin);

  xml.push_back(reflex);
}

//----------------------------------------------------------------------------------------------------------------------        
void Reflex::fromXml(const ci::XmlTree& xml)
{
  // Spindle
  if(xml.hasChild("Spindle"))
  {
    const ci::XmlTree& spindle = xml.getChild("Spindle");
    paramFromXml(spindle, "PosGain", &m_Kspp[0]);
    paramFromXml(spindle, "VelGain", &m_Kspv[0]);
    paramFromXml(spindle, "DmpGain", &m_Kspd[0]);
    paramFromXml(spindle, "VelExp", &m_Espv[0]);
    paramFromXml(spindle, "Wspmn", &m_Wspmn[0]);
  }
  
  // Static load compensation  
  if(xml.hasChild("Sfv"))
  {
    const ci::XmlTree& sfv = xml.getChild("Sfv");
    paramFromXml(sfv, "Gain", &m_Ksfv[0]);
    paramFromXml(sfv, "RecInh", &m_Ksfi[0]); 
  }
  
  // Inertial load compensation  
  if(xml.hasChild("Ifv"))
  {  
    const ci::XmlTree& ifv = xml.getChild("Ifv");
    paramFromXml(ifv, "Gain", &m_Kifv[0]);
    paramFromXml(ifv, "Bias", &m_Bifv[0]); 
  }
  
  // IaIn
  if(xml.hasChild("IaIn"))
  {  
    const ci::XmlTree& iain = xml.getChild("IaIn");    
    paramFromXml(iain, "Wiaia", &m_Wiaia[0]);
    paramFromXml(iain, "Wspia", &m_Wspia[0]);
    paramFromXml(iain, "Wrnia", &m_Wrnia[0]);
    paramFromXml(iain, "Waia", &m_Waia[0]);
    paramFromXml(iain, "Wiamn", &m_Wiamn[0]);  
    paramFromXml(iain, "Bias", &m_Biain[0]);  
    paramFromXml(iain, "Tau", &m_Tiain[0]);
  }
  
  // Renshaw
  if(xml.hasChild("Renshaw"))
  {    
    const ci::XmlTree& rn = xml.getChild("Renshaw");
    paramFromXml(rn, "Wmnrn", &m_Wmnrn[0]);
    paramFromXml(rn, "Wrnrn", &m_Wrnrn[0]);
    paramFromXml(rn, "Wrnmn", &m_Wrnmn[0]);  
    paramFromXml(rn, "Bias", &m_Brn[0]);  
    paramFromXml(rn, "Tau", &m_Trn[0]);    
  } 
  
  // IbIn
  if(xml.hasChild("IbIn"))
  {     
    const ci::XmlTree& ibin = xml.getChild("IbIn");
    paramFromXml(ibin, "Wibib", &m_Wibib[0]);
    paramFromXml(ibin, "Wglib", &m_Wglib[0]);
    paramFromXml(ibin, "Wibmn", &m_Wibmn[0]);
    paramFromXml(ibin, "Bias", &m_Bib[0]);  
    paramFromXml(ibin, "Tau", &m_Tib[0]);
  }
  
  // Intersegmental inputs
  if(xml.hasChild("IsIn"))
  {       
    const ci::XmlTree isin = xml.getChild("IsIn");
    paramFromXml(isin, "Wisep", &m_Wisep[0]);
  }
  
}

//----------------------------------------------------------------------------------------------------------------------      
void Reflex::recordStatePair(Recorder& recorder, const std::string& name, const double* var)
{
  recorder.push_back(name+"Ag", var[0]);
  recorder.push_back(name+"An", var[1]);
}
  
//----------------------------------------------------------------------------------------------------------------------    
void Reflex::record(Recorder& recorder)
{
  // Prefix for all reflex state variables
  std::string jntNm = "Bi";
  if(m_muscles[0]->isMonoArticulate())
  {
    int joint = ((MuscleMonoWrap*)m_muscles[0])->getJoint();
    jntNm = (joint == JT_elbow) ? "Elb" : "Shd";
  }
  std::string prefix = "reflex" + jntNm;
  
  // Add them all
  recordStatePair(recorder, prefix + "PosRes", &m_spindlePosRes[0]);
  recordStatePair(recorder, prefix + "VelRes", &m_spindleVelRes[0]);
  recordStatePair(recorder, prefix + "DmpRes", &m_spindleDmpRes[0]);
  recordStatePair(recorder, prefix + "SpRes", &m_spindlePri[0]);
  recordStatePair(recorder, prefix + "OpenLoop", &m_openLoop[0]);
  recordStatePair(recorder, prefix + "InterSeg", &m_interSegmentInput[0]);
  recordStatePair(recorder, prefix + "ComContraction", &m_commandedContraction[0]);  
  recordStatePair(recorder, prefix + "DesContraction", &m_desiredContraction[0]);
  recordStatePair(recorder, prefix + "ActContraction", &m_contraction[0]);
  recordStatePair(recorder, prefix + "DesContractionVel", &m_desiredContractionVel[0]);
  recordStatePair(recorder, prefix + "ActContractionVel", &m_contractionVel[0]);
  recordStatePair(recorder, prefix + "MN", &m_alpha[0]);
}
  
} // namespace dmx

