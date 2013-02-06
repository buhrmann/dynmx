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
  m_go = 1.0;
  
  // Spindle 
  m_Kspp[0] = m_Kspp[1] = 1.0;  // positional gain
  m_Kspv[0] = m_Kspv[1] = 0.0;  // velocity gain
  m_Kspd[0] = m_Kspd[1] = 0.1;  // damping gain
  m_Espv[0] = m_Espv[1] = 1.0;  // velocity exponent 
  m_Espd[0] = m_Espd[1] = 1.0;  // damping exponent   
  
  // Load compensation
  m_Ksfv[0] = m_Ksfv[1] = 0.0;
  m_Ksfi[0] = m_Ksfi[1] = 0.0;     // Reciprocal inhibition

  // IaIn inhibitory interneurons
  m_Wspia[0] = m_Wspia[1] = 0.0;    // Spindle input
  m_Wiaia[0] = m_Wiaia[1] = 0.0;    // Reciprocal inhibition
  m_Wrnia[0] = m_Wrnia[1] = 0.0;    // Renshaw input  
  m_Waia[0]  = m_Waia[1]  = 0.0;    // Desired contraction input
  m_Biain[0] = m_Biain[1] = 0.0;    // Ia Biases
  m_Kiain[0] = m_Kiain[1] = 1.0;
  m_Tiain[0] = m_Tiain[1] = 100.0;  // Integration time constant (really 1/t)  
  
  // Renshaw
  m_Wmnrn[0] = m_Wmnrn[1] = 0.0f;  
  m_Wrnrn[0] = m_Wrnrn[1] = 0.0f;  
  m_Brn[0] = m_Brn[1] = 0.0;
  m_Trn[0] = m_Trn[1] = 100.0;
  m_Krn[0] = m_Krn[1] = 1.0;
  
  // IbIn
  m_Wglib[0] = m_Wglib[1] = 0.0;   // Golgi force input
  m_Wgdib[0] = m_Wgdib[1] = 0.0;   // Golgi force derivative input
  m_Wiaibin[0] = m_Wiaibin[1] = 0.0;
  m_Wibib[0] = m_Wibib[1] = 0.0;   // Reciprocal inhibition
  m_WisibAg[0] = m_WisibAg[1] = m_WisibAn[0] = m_WisibAn[1] = 0.0; // Intersegmental connections
  m_Bib[0] = m_Bib[1] = 0.0;
  m_Tib[0] = m_Tib[1] = 100.0;
  m_Kib[0] = m_Kib[1] = 1.0;
  
  // Alpha motor neuron
  m_Wiamn[0] = m_Wiamn[1] = 0.0f;
  m_Wrnmn[0] = m_Wrnmn[1] = 0.0f;
  m_Wibmn[0] = m_Wibmn[1] = 0.0f;
  m_Wspmn[0] = m_Wspmn[1] = 0.0f;
  m_WisibAgMn[0] = m_WisibAgMn[1] = m_WisibAnMn[0] = m_WisibAnMn[1] = 0.0; // Intersegmental connections
  m_WeIbinMn[0] = m_WeIbinMn[1] = 0.0;
  m_Bmn[0] = m_Bmn[1] = 0.0;
  m_Tmn[0] = m_Tmn[1] = 100.0;
  m_Kmn[0] = m_Kmn[1] = 1.0;

  // Inertial force compensation (velocity error)
  m_Kifv[0] = m_Kifv[1] = 0.0;
  m_Bifv[0] = m_Bifv[1] = 0.0;
  
  m_Wisep[0] = m_Wisep[1] = 0.0;

  m_MNasNeuron = false;
  m_coconAsCCommand = false;
  m_trqFeedbackPosMod = true;
  m_feedbackDelay = 0.0;
  m_commandDelay = 0.0;
  
  m_openLoopTimeConstants[0] = 0.01;
  m_openLoopTimeConstants[1] = 0.01;
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------      
void Reflex::reset()
{
  m_go = 1.0;
  
  m_IaIn[0] = m_IaIn[1] = 0.0;
  m_IaInOut[0] = neuronActivation(m_IaIn[0] + m_Biain[0], m_Kiain[0]);
  m_IaInOut[1] = neuronActivation(m_IaIn[1] + m_Biain[1], m_Kiain[1]);
  
  m_Rn[0] = m_Rn[1] = 0.0;
  m_RnOut[0] = neuronActivation(m_Rn[0] + m_Brn[0], m_Krn[0]);
  m_RnOut[1] = neuronActivation(m_Rn[1] + m_Brn[1], m_Krn[1]);
  
  m_IbIn[0] = m_IbIn[1] = 0.0;
  m_IbInOut[0] = neuronActivation(m_IbIn[0] + m_Bib[0], m_Kib[0]);
  m_IbInOut[1] = neuronActivation(m_IbIn[1] + m_Bib[1], m_Kib[1]);
  
  m_alphaState[0] = m_alphaState[1] = 0.0;
  if(m_MNasNeuron)
  {
    m_alpha[0] = neuronActivation(m_alphaState[0] + m_Bmn[0], m_Kmn[0]);
    m_alpha[1] = neuronActivation(m_alphaState[1] + m_Bmn[1], m_Kmn[1]);
  }
  else
  {
    m_alpha[0] = m_alpha[1] = 0.0;
  }

  m_sfv[0] = m_sfv[1] = 0.0;
  m_ifv[0] = m_ifv[1] = 0.0;
  m_ofpv[0] = m_ofpv[1] = 0.0;
  
  // State in length coordinates
  // Sensory feedback delay
  m_posDelay[0] = Delay(m_feedbackDelay, 0.001, m_muscles[0]->getUnitLength());
  m_posDelay[1] = Delay(m_feedbackDelay, 0.001, m_muscles[1]->getUnitLength());
  
  // Delay of commanded trajectory (elbow is empirically found to start later than shoulder)
  m_commandDelayed[0] = Delay(m_commandDelay, 0.001, m_muscles[0]->getUnitLength());
  m_commandDelayed[1] = Delay(m_commandDelay, 0.001, m_muscles[1]->getUnitLength());  
                     
  m_length[0] = m_posDelay[0].get();
  m_length[1] = m_posDelay[1].get();
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
  
  m_spindleVelRes[0] = m_spindleVelRes[1] = 0.0;
  
  // Input state 
  m_openLoopPreFilter[0] = m_openLoopPreFilter[1] = 0.0;
  m_openLoopFiltered[0] = m_openLoopFiltered[1] = 0.0;
  
  // Intersegment feedback delay
  m_interSegDelay[0] = Delay(m_feedbackDelay, 0.001, 0.0);
  m_interSegDelay[1] = Delay(m_feedbackDelay, 0.001, 0.0);  
  m_interSegmentInput[0] = m_interSegDelay[0].get();
  m_interSegmentInput[1] = m_interSegDelay[1].get();

  // Force sensor
  m_forceDelay[0] = Delay(m_feedbackDelay, 0.001, m_muscles[0]->getNormalisedForce());
  m_forceDelay[1] = Delay(m_feedbackDelay, 0.001, m_muscles[1]->getNormalisedForce());  
  m_golgi[0] = m_forceDelay[0].get();
  m_golgi[1] = m_forceDelay[1].get();

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
  assert(l0==l0 && l1==l1);
  
  // Length coordinates
  m_desiredLengthPrev[0] = m_desiredLength[0]; 
  m_desiredLength[0] = m_muscles[0]->lengthToUnitLength(l0);
  
  m_desiredLengthPrev[1] = m_desiredLength[1]; 
  m_desiredLength[1] = m_muscles[1]->lengthToUnitLength(l1);
  
  if(m_coconAsCCommand)
  {
    m_desiredLength[0] = clamp(m_desiredLength[0] - m_openLoopFiltered[0], 0.0, 1.0);
    m_desiredLength[1] = clamp(m_desiredLength[1] - m_openLoopFiltered[1], 0.0, 1.0);
  }
  
  // Delay the onset of the commanded trajectory
  m_desiredLength[0] = m_commandDelayed[0].update(m_desiredLength[0]);
  m_desiredLength[1] = m_commandDelayed[1].update(m_desiredLength[1]);
  
  // Contraction coordinates
  m_desiredContractionPrev[0] = m_desiredContraction[0]; 
  m_desiredContraction[0] = 1.0 - m_desiredLength[0];
  
  m_desiredContractionPrev[1] = m_desiredContraction[1];   
  m_desiredContraction[1] = 1.0 - m_desiredLength[1];
}
  
//----------------------------------------------------------------------------------------------------------------------  
void Reflex::update(float dt)
{   
  // Filter open-loop signals
  double filterConst [2];
  filterConst[0] = m_openLoopPreFilter[0] > m_openLoopFiltered[0] ? m_openLoopTimeConstants[0] : m_openLoopTimeConstants[1];
  filterConst[1] = m_openLoopPreFilter[1] > m_openLoopFiltered[1] ? m_openLoopTimeConstants[0] : m_openLoopTimeConstants[1];
  m_openLoopFiltered[0] += ((m_openLoopPreFilter[0] - m_openLoopFiltered[0]) / filterConst[0]) * dt;
  m_openLoopFiltered[1] += ((m_openLoopPreFilter[1] - m_openLoopFiltered[1]) / filterConst[1]) * dt;   
  assert(m_openLoopFiltered[0] == m_openLoopFiltered[0]);
  
  // Store original commands before internal modification
  m_commandedLength[0] = m_desiredLength[0];
  m_commandedLength[1] = m_desiredLength[1];
  m_commandedContraction[0] = m_desiredContraction[0];
  m_commandedContraction[1] = m_desiredContraction[1];
  
  // Intersegmental torque feedback modifies virtual (desired) EP
  if(m_trqFeedbackPosMod)
  {
    double iseg0 = m_interSegDelay[0].update(m_interSegmentInput[0]);
    double iseg1 = m_interSegDelay[1].update(m_interSegmentInput[1]);
    m_desiredLength[0] -= m_Wisep[0] * iseg0;  // +
    m_desiredLength[1] += m_Wisep[1] * iseg1;  // +
    m_desiredContraction[0] += m_Wisep[0] * iseg0; // -
    m_desiredContraction[1] -= m_Wisep[1] * iseg1; // -
  }
  
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
  
  m_length[0] = m_posDelay[0].update(m_muscles[0]->getUnitLength());
  m_length[1] = m_posDelay[1].update(m_muscles[1]->getUnitLength());
  m_contraction[0] = 1.0 - m_length[0];
  m_contraction[1] = 1.0 - m_length[1];
  
  m_vel[0] = (m_length[0] - m_lengthPrev[0]) / dt;
  m_vel[1] = (m_length[1] - m_lengthPrev[1]) / dt;
  m_contractionVel[0] = (m_contraction[0] - m_contractionPrev[0]) / dt;
  m_contractionVel[1] = (m_contraction[1] - m_contractionPrev[1]) / dt;
  
  // Normalised force sensor
  m_golgi[0] =  m_forceDelay[0].update(m_muscles[0]->getNormalisedForce());
  m_golgi[1] =  m_forceDelay[1].update(m_muscles[1]->getNormalisedForce());  
  
  m_golgiDt[0].update(m_golgi[0], dt);
  m_golgiDt[1].update(m_golgi[1], dt);
  
  updateInContractionCoords(dt);
} 

//----------------------------------------------------------------------------------------------------------------------  
void Reflex::updateInContractionCoords(float dt)
{
  // Spindle-calculated errors in length 
  m_posErr[0] = m_desiredContraction[0] - m_contraction[0];
  m_posErr[1] = m_desiredContraction[1] - m_contraction[1];
  m_spindlePosRes[0] = m_Kspp[0] * m_posErr[0];
  m_spindlePosRes[1] = m_Kspp[1] * m_posErr[1];
  
  // Spindle-calculated errors in velocity
  m_velErr[0] = m_desiredContractionVel[0] - m_contractionVel[0];
  m_velErr[1] = m_desiredContractionVel[1] - m_contractionVel[1];
  double velErrSign0 = sign(m_velErr[0]);
  double velErrSign1 = sign(m_velErr[1]);  
  double currentVelErrRes0 = clamp(m_Kspv[0] * m_velErr[0], -1.0, 1.0);
  double currentVelErrRes1 = clamp(m_Kspv[1] * m_velErr[1], -1.0, 1.0);  
  currentVelErrRes0 = velErrSign0 * powf(fabs(currentVelErrRes0), m_Espv[0]);
  currentVelErrRes1 = velErrSign1 * powf(fabs(currentVelErrRes1), m_Espv[1]);  
  // Filter out fast oscillations
  const float filterGain = 0.7f;
  m_spindleVelRes[0] = filterGain * m_spindleVelRes[0] + (1 - filterGain) * currentVelErrRes0;
  m_spindleVelRes[1] = filterGain * m_spindleVelRes[1] + (1 - filterGain) * currentVelErrRes1;  
  
  // Non-linear damping term
  double contractionDir0 = sign(m_contractionVel[0]);
  double contractionDir1 = sign(m_contractionVel[1]);
  m_spindleDmpRes[0] = -m_Kspd[0] * contractionDir0 * powf(fabs(m_contractionVel[0]), m_Espd[0]);
  m_spindleDmpRes[1] = -m_Kspd[1] * contractionDir1 * powf(fabs(m_contractionVel[1]), m_Espd[1]);
  
  m_spindlePri[0] = spindleActivation(m_spindlePosRes[0] + m_spindleVelRes[0] + m_spindleDmpRes[0]);
  m_spindlePri[1] = spindleActivation(m_spindlePosRes[1] + m_spindleVelRes[1] + m_spindleDmpRes[1]);
  
  // Ia inhibitory interneurons
  double dIaIn [2];
  dIaIn[0] = -m_IaIn[0] + (m_Wspia[0] * m_spindlePri[0]) + (m_Waia[0] * m_desiredContraction[0]) - (m_Wiaia[0] * m_IaInOut[1]) - (m_Wrnia[0] * m_RnOut[0]);
  dIaIn[1] = -m_IaIn[1] + (m_Wspia[1] * m_spindlePri[1]) + (m_Waia[1] * m_desiredContraction[1]) - (m_Wiaia[1] * m_IaInOut[0]) - (m_Wrnia[1] * m_RnOut[1]); 
  m_IaIn[0] += dt * m_Tiain[0] * dIaIn[0];
  m_IaIn[1] += dt * m_Tiain[1] * dIaIn[1];  
  m_IaInOut[0] = neuronActivation(m_IaIn[0] + m_Biain[0], m_Kiain[0]);
  m_IaInOut[1] = neuronActivation(m_IaIn[1] + m_Biain[1], m_Kiain[1]);

  // Renshaw neurons
  double dRn [2];    
  dRn[0] = -m_Rn[0] + (m_Wmnrn[0] * m_alpha[0]) - (m_Wrnrn[0] * m_RnOut[1]);
  dRn[1] = -m_Rn[1] + (m_Wmnrn[1] * m_alpha[1]) - (m_Wrnrn[1] * m_RnOut[0]);
  m_Rn[0] += dt * m_Trn[0] * dRn[0];
  m_Rn[1] += dt * m_Trn[1] * dRn[1];
  m_RnOut[0] = neuronActivation(m_Rn[0] + m_Brn[0], m_Krn[0]);
  m_RnOut[1] = neuronActivation(m_Rn[1] + m_Brn[1], m_Krn[1]);
  
  // Ib inhibitory interneurons
  double dIbIn[2];  
  dIbIn[0] = - m_IbIn[0] + (m_Wglib[0] * m_golgi[0]) + (m_Wgdib[0] * m_golgiDt[0].get() / 10.0) - (m_Wibib[0] * m_IbInOut[1]);
  dIbIn[1] = - m_IbIn[1] + (m_Wglib[1] * m_golgi[1]) + (m_Wgdib[1] * m_golgiDt[1].get() / 10.0) - (m_Wibib[1] * m_IbInOut[0]);
  // Add intersegmental activity
  const bool ibisFromGolgi = true;
  if(ibisFromGolgi)
  {
    dIbIn[0] += (m_WisibAg[0] * m_otherReflex->getGolgi(0)) + (m_WisibAn[0] * m_otherReflex->getGolgi(1));
    dIbIn[1] += (m_WisibAg[1] * m_otherReflex->getGolgi(0)) + (m_WisibAn[1] * m_otherReflex->getGolgi(1));
  }
  else
  {
    dIbIn[0] += (m_WisibAg[0] * m_otherReflex->getIbInOutput(0)) + (m_WisibAn[0] * m_otherReflex->getIbInOutput(1));
    dIbIn[1] += (m_WisibAg[1] * m_otherReflex->getIbInOutput(0)) + (m_WisibAn[1] * m_otherReflex->getIbInOutput(1));
  }
  // Add Ia input
  dIbIn[0] += m_Wiaibin[0] + m_IaInOut[0];
  dIbIn[1] += m_Wiaibin[1] + m_IaInOut[1];
  
  m_IbIn[0] += dt * m_Tib[0] * dIbIn[0];
  m_IbIn[1] += dt * m_Tib[1] * dIbIn[1];
  m_IbInOut[0] = neuronActivation(m_IbIn[0] + m_Bib[0], m_Kib[0]);
  m_IbInOut[1] = neuronActivation(m_IbIn[1] + m_Bib[1], m_Kib[1]);
  
  // Intersegmental torque feedback to motor neurons directly
  double interSegDelayed[2] = {0,0};
  if(!m_trqFeedbackPosMod)
  {
    interSegDelayed[0] = m_interSegDelay[0].update(m_interSegmentInput[0]);
    interSegDelayed[1] = m_interSegDelay[1].update(m_interSegmentInput[1]);
  }

  
#if 0  
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
#endif
  
  // Go signal
  double go0 = m_go;
  double go1 = m_go;
  
  const bool useErrorGoSignal = false;
  if(useErrorGoSignal)
  {
    const double inhThresh = 0.1;
    go0 = 1.0 - (std::max(inhThresh - m_posErr[0], 0.0) / inhThresh); // [0,0.1] -> [0,1]
    go1 = 1.0 - (std::max(inhThresh - m_posErr[1], 0.0) / inhThresh); // [0,0.1] -> [0,1]
  }
  
  
  // Alpha motor neuron 
  double aMNinput0 =  m_ofpv[0] 
  + (m_Wspmn[0] * m_spindlePri[0])
  - (m_Wrnmn[0] * m_RnOut[0] * go0)
  - (m_Wiamn[0] * m_IaInOut[1] * go0)
  - (m_Wibmn[0] * m_IbInOut[0] * go0)
  + (m_WeIbinMn[0] * m_IbInOut[1] * go0)
  + (m_WisibAgMn[0] * m_otherReflex->getIbInOutput(0) * go0)
  + (m_WisibAnMn[0] * m_otherReflex->getIbInOutput(1) * go0)
  + (m_Wisep[0] * interSegDelayed[0] * go0);
  
  double aMNinput1 = m_ofpv[1] 
  + (m_Wspmn[1] * m_spindlePri[1])
  - (m_Wrnmn[1] * m_RnOut[1] * go1)
  - (m_Wiamn[1] * m_IaInOut[0] * go1)
  - (m_Wibmn[1] * m_IbInOut[1] * go1)
  + (m_WeIbinMn[1] * m_IbInOut[0] * go1)
  + (m_WisibAgMn[1] * m_otherReflex->getIbInOutput(0) * go1)
  + (m_WisibAnMn[1] * m_otherReflex->getIbInOutput(1) * go1)
  + (m_Wisep[1] * interSegDelayed[1] * go1);
  
  if(!m_coconAsCCommand)
  {
    aMNinput0 += m_openLoopFiltered[0];    
    aMNinput1 += m_openLoopFiltered[1];
  }

  if(m_MNasNeuron)
  {
    m_alphaState[0] += dt * m_Tmn[0] * (-m_alphaState[0] + aMNinput0);
    m_alpha[0] = neuronActivation(m_alphaState[0] + m_Bmn[0], m_Kmn[0]);
    
    m_alphaState[1] += dt * m_Tmn[1] * (-m_alphaState[1] + aMNinput1);
    m_alpha[1] = neuronActivation(m_alphaState[1] + m_Bmn[1], m_Kmn[1]);
  }
  else
  {
    m_alpha[0] = clamp(aMNinput0, 0.0, 1.0);
    m_alpha[1] = clamp(aMNinput1, 0.0, 1.0);
  }
  
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
  reflex.setAttribute("FeedbackDelay", m_feedbackDelay);  
  reflex.setAttribute("CommandDelay", m_commandDelay);
  reflex.setAttribute("OpenLoopTauAct", m_openLoopTimeConstants[0]);
  reflex.setAttribute("OpenLoopTauDeact", m_openLoopTimeConstants[1]);

  // Spindle
  ci::XmlTree spindle("Spindle", "");
  paramToXml(spindle, "PosGain", &m_Kspp[0]);
  paramToXml(spindle, "VelGain", &m_Kspv[0]);
  paramToXml(spindle, "VelExp", &m_Espv[0]);  
  paramToXml(spindle, "DmpGain", &m_Kspd[0]); 
  paramToXml(spindle, "DmpExp", &m_Espd[0]);  
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
  paramToXml(iain, "Slope", &m_Kiain[0]);
  reflex.push_back(iain);  
  
  // Renshaw
  ci::XmlTree rn("Renshaw", "");
  paramToXml(rn, "Wmnrn", &m_Wmnrn[0]);
  paramToXml(rn, "Wrnrn", &m_Wrnrn[0]);
  paramToXml(rn, "Wrnmn", &m_Wrnmn[0]);  
  paramToXml(rn, "Bias", &m_Brn[0]);  
  paramToXml(rn, "Tau", &m_Trn[0]);
  paramToXml(rn, "Slope", &m_Krn[0]);
  reflex.push_back(rn);    
  
  // IbIn
  ci::XmlTree ibin("IbIn", "");
  paramToXml(ibin, "Wibib", &m_Wibib[0]);
  paramToXml(ibin, "Wglib", &m_Wglib[0]);
  paramToXml(ibin, "Wgdib", &m_Wgdib[0]);
  paramToXml(ibin, "Wibmn", &m_Wibmn[0]);
  paramToXml(ibin, "Wiaib", &m_Wiaibin[0]);
  paramToXml(ibin, "WisibAg", &m_WisibAg[0]);
  paramToXml(ibin, "WisibAn", &m_WisibAn[0]);
  paramToXml(ibin, "Bias", &m_Bib[0]);
  paramToXml(ibin, "Tau", &m_Tib[0]);
  paramToXml(ibin, "Slope", &m_Kib[0]);
  reflex.push_back(ibin);
  
  ci::XmlTree amn("aMN", "");
  paramToXml(amn, "Bias", &m_Bmn[0]);
  paramToXml(amn, "Tau", &m_Tmn[0]);
  paramToXml(amn, "Slope", &m_Kmn[0]);
  paramToXml(amn, "WeIbinMn", &m_WeIbinMn[0]);
  paramToXml(amn, "WisibAgMn", &m_WisibAgMn[0]);
  paramToXml(amn, "WisibAnMn", &m_WisibAnMn[0]);
  amn.setAttribute("asNeuron", m_MNasNeuron);
  reflex.push_back(amn);

  // Intersegmental inputs
  ci::XmlTree isin("IsIn", "");
  paramToXml(isin, "Wisep", &m_Wisep[0]);
  reflex.push_back(isin);

  xml.push_back(reflex);
}

//----------------------------------------------------------------------------------------------------------------------        
void Reflex::fromXml(const ci::XmlTree& xml)
{
  m_feedbackDelay = xml.getAttributeValue<double>("FeedbackDelay", 0.0);
  m_commandDelay = xml.getAttributeValue<double>("CommandDelay", 0.0);
  m_openLoopTimeConstants[0] = xml.getAttributeValue<double>("OpenLoopTauAct", 0.01);
  m_openLoopTimeConstants[1] = xml.getAttributeValue<double>("OpenLoopTauDeact", 0.01);
  
  // Spindle
  if(xml.hasChild("Spindle"))
  {
    const ci::XmlTree& spindle = xml.getChild("Spindle");
    paramFromXml(spindle, "PosGain", &m_Kspp[0]);
    paramFromXml(spindle, "VelGain", &m_Kspv[0]);
    paramFromXml(spindle, "VelExp", &m_Espv[0]);
    paramFromXml(spindle, "DmpGain", &m_Kspd[0]);
    paramFromXml(spindle, "DmpExp", &m_Espd[0]);
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
    paramFromXml(iain, "Slope", &m_Kiain[0]);
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
    paramFromXml(rn, "Slope", &m_Krn[0]);
  } 
  
  // IbIn
  if(xml.hasChild("IbIn"))
  {     
    const ci::XmlTree& ibin = xml.getChild("IbIn");
    paramFromXml(ibin, "Wibib", &m_Wibib[0]);
    paramFromXml(ibin, "Wglib", &m_Wglib[0]);
    paramFromXml(ibin, "Wgdib", &m_Wgdib[0]);
    paramFromXml(ibin, "Wibmn", &m_Wibmn[0]);
    paramFromXml(ibin, "Wiaib", &m_Wiaibin[0]);
    paramFromXml(ibin, "WisibAg", &m_WisibAg[0]);
    paramFromXml(ibin, "WisibAn", &m_WisibAn[0]);
    paramFromXml(ibin, "Bias", &m_Bib[0]);
    paramFromXml(ibin, "Tau", &m_Tib[0]);
    paramFromXml(ibin, "Slope", &m_Kib[0]);
  }
  
  if(xml.hasChild("aMN"))
  {
    const ci::XmlTree& amn = xml.getChild("aMN");
    paramFromXml(amn, "Bias", &m_Bmn[0]);
    paramFromXml(amn, "Tau", &m_Tmn[0]);
    paramFromXml(amn, "Slope", &m_Kmn[0]);
    paramFromXml(amn, "WeIbinMn", &m_WeIbinMn[0]);
    paramFromXml(amn, "WisibAgMn", &m_WisibAgMn[0]);
    paramFromXml(amn, "WisibAnMn", &m_WisibAnMn[0]);
    m_MNasNeuron = amn.getAttributeValue<bool>("asNeuron");
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
  recordStatePair(recorder, prefix + "OpenLoop", &m_openLoopFiltered[0]);
  recordStatePair(recorder, prefix + "InterSeg", &m_interSegmentInput[0]);
  recordStatePair(recorder, prefix + "ComContraction", &m_commandedContraction[0]);  
  recordStatePair(recorder, prefix + "DesContraction", &m_desiredContraction[0]);
  recordStatePair(recorder, prefix + "ActContraction", &m_contraction[0]);
  recordStatePair(recorder, prefix + "DesContractionVel", &m_desiredContractionVel[0]);
  recordStatePair(recorder, prefix + "ActContractionVel", &m_contractionVel[0]);
  recordStatePair(recorder, prefix + "MN", &m_alpha[0]);
  recordStatePair(recorder, prefix + "Renshaw", &m_RnOut[0]);  
  recordStatePair(recorder, prefix + "IaIn", &m_IaInOut[0]);  
  recordStatePair(recorder, prefix + "IbIn", &m_IbInOut[0]);
  recordStatePair(recorder, prefix + "Golgi", &m_golgi[0]);
  recorder.push_back(prefix + "GolgiDt" + "Ag", m_golgiDt[0].get());
  recorder.push_back(prefix + "GolgiDt" + "An", m_golgiDt[1].get());
}
  
//----------------------------------------------------------------------------------------------------------------------    
int Reflex::decodeSpindleParams(const std::vector<double>& genome, int I, bool symmetric, bool velRef, const SpindleLimits& lim)
{
  int pI = 0;
  int dI = 1;
  int edI = 2;
  int vI = 3;
  int evI = 4;
  
  const int numSpindleParams = velRef ? 5 : 3;   
  
  // 1: identical for antagonists. 2: identical for all muscles.
  if(symmetric)
  {      
    double kp = map01To(genome[I + pI], lim.pos);
    double kd = map01To(genome[I + dI], lim.dmp);
    double expD = map01To(genome[I + edI], lim.exp);
    double kv = velRef ? map01To(genome[I + vI], lim.vel) : 0.0;      
    double expV = velRef ? map01To(genome[I + evI], lim.exp) : 1.0;
    
    setSpindleParameters(kp, kp, kv, kv, kd, kd, expV, expV, expD, expD);
    return numSpindleParams;
  }
  else 
  {
    int ofs = numSpindleParams;
    double kp[2] = { map01To(genome[I + pI], lim.pos), map01To(genome[I + ofs + pI], lim.pos) };
    double kd[2] = { map01To(genome[I + dI], lim.dmp), map01To(genome[I + ofs + dI], lim.dmp) };
    double expD[2] = { map01To(genome[I + edI], lim.exp), map01To(genome[I + ofs + edI], lim.exp) };
    double kv[2] = {0,0};
    double expV[2] = {1,1};
    if(velRef)
    {
      kv[0] = map01To(genome[I + vI], lim.vel); kv[1] = map01To(genome[I + ofs + vI], lim.vel);        
      expV[0] = map01To(genome[I + evI], lim.exp); expV[1] = map01To(genome[I + ofs + evI], lim.exp);      
    }      
    
    setSpindleParameters(kp[0], kp[1], kv[0], kv[1], kd[0], kd[1], expV[0], expV[1], expD[0], expD[1]);
    return 2 * numSpindleParams;
  }
}
  
  
} // namespace dmx

