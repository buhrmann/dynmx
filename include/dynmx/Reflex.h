/*
 *  Reflex.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/20/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_REFLEX_
#define _DMX_REFLEX_

#include "Dynmx.h"
#include "Muscle.h"

namespace dmx
{
   
//----------------------------------------------------------------------------------------------------------------------
/// A VITE/FLETE reflex network
//----------------------------------------------------------------------------------------------------------------------
class Reflex : public Model
{

public:
  
  Reflex() { m_muscles[0] = m_muscles[1] = 0; init(); };
  Reflex(Muscle* ag, Muscle* an) { m_muscles[0] = ag; m_muscles[1] = an; init(); };
  
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  
  void setDesiredLength(double l0, double l1);
  void setCocontraction(double c0, double c1);  
  void setSpindleParameters(double Kp0, double Kp1, double Kv0, double Kv1, double E0, double E1);
  void setLoadCompensationParameters(double g0, double g1, double inh0, double inh1);
  void setInertiaCompensationParameters(double g0, double g1, double b0, double b1);
  void setIaInParameters(double Wspia0, double Wspia1, double Wiaia0, double Wiaia1, double t1, double t2, double b1, double b2);
  void setMotoNeuronParameters(double Wiamn0, double Wiamn1);
  
  double getAlphaOutput(int i) { return m_alpha[i]; };
  double getCoContraction(int i) { return m_cocontraction[i]; };
  const ArmMuscled* getArm() const { return m_muscles[0]->getArm(); };
  
  // Parameters
  double m_Kspp [2];   // Spindle positional gain  
  double m_Kspv [2];   // Spindle velocity gain  
  double m_Espv [2];   // Spindle velocity exponent
  
  double m_Ksfv [2];   // Strength and speed of sfv load compensation (gravity)
  double m_Ksfi [2];   // Reciprocal inhibition of sfv  load compensation (gravity)
  
  double m_Kifv [2];   // Gain of inertial force compensation (velocity error)
  double m_Bifv [2];   // Bias of inertial force compensation (velocity error)  
  
  double m_Wiaia [2];  // Weight of IaIn reciprocal inhibition
  double m_Wspia [2];  // Weight of spindle input  
  double m_Biain [2];  // Bias of Ia neurons
  double m_Tiain [2];  // time constant (really 1/t)
  
  double m_Wiamn [2];   // IaIn to alpha MN
  
  // Inputs
  double m_cocontraction[2];
  
  // State
  double m_length [2];
  double m_vel [2];
  double m_posErr [2];
  double m_velErr [2];
  double m_spindlePri [2];
  double m_spindleSec [2];
  
  double m_IaIn [2];              // Ia inhibitory interneurons  
  double m_IaInOut [2];
  double m_ifv [2];               // Inertial force compensation
  double m_sfv [2];               // Static force (gravity) compensation
  
  double m_ofpv [2];              // Outflow position
  
  double m_alpha [2];             // Alpha motor neuron outputs
  double m_opv [2];               // y in Bullock et al
  double m_ppv [2];               // x
  double m_dv [2];                // r
  double m_dvv [2];               // u
  
  double m_lengthPrev [2];
  double m_desiredLength [2];     // 
  double m_desiredLengthPrev [2]; // For calculating desired velocity
  double m_desiredVelocity [2];   // Calculated from desired lengths over time  
  
  double m_contraction [2];
  double m_contractionPrev [2];
  double m_contractionVel [2];
  double m_desiredContraction [2];      // In contraction coordinates (1-length), where length in [0,1]
  double m_desiredContractionPrev [2];  // For calculating desired velocity
  double m_desiredContractionVel [2];   // In contraction coordinates (1-length), where length in [0,1]
  
protected:
  
  static double spindleActivation(double x) { return x;/* / (1 + 100 * (x*x));*/ };
  void updateInLengthCoords(float dt);
  void updateInContractionCoords(float dt);
  
  Muscle* m_muscles [2];
};
  
//----------------------------------------------------------------------------------------------------------------------
/// Inline implementations
//----------------------------------------------------------------------------------------------------------------------  
inline void Reflex::setCocontraction(double c0, double c1) 
{ 
  m_cocontraction[0] = c0; 
  m_cocontraction[1] = c1; 
};

//----------------------------------------------------------------------------------------------------------------------    
inline void Reflex::setSpindleParameters(double Kp0, double Kp1, double Kv0, double Kv1, double E0, double E1)
{ 
  m_Kspp[0] = Kp0; 
  m_Kspp[1] = Kp1; 
  m_Kspv[0] = Kv0; 
  m_Kspv[1] = Kv1; 
  m_Espv[0] = E0; 
  m_Espv[1] = E1;
};

//----------------------------------------------------------------------------------------------------------------------    
inline void Reflex::setLoadCompensationParameters(double g0, double g1, double inh0, double inh1)
{
  m_Ksfv[0] = g0; 
  m_Ksfv[1] = g1; 
  m_Ksfi[0] = inh0; 
  m_Ksfi[1] = inh1;
};

//----------------------------------------------------------------------------------------------------------------------      
inline void Reflex::setInertiaCompensationParameters(double g0, double g1, double b0, double b1)
{
  m_Kifv[0] = g0; 
  m_Kifv[1] = g1; 
  m_Bifv[0] = b0;
  m_Bifv[1] = b1;
}

//----------------------------------------------------------------------------------------------------------------------        
inline void Reflex::setIaInParameters(double Wspia0, double Wspia1, double Wiaia0, double Wiaia1, 
                                      double t0, double t1, double b0, double b1)
{
  m_Wspia[0] = Wspia0;
  m_Wspia[1] = Wspia1;
  m_Wiaia[0] = Wiaia0;
  m_Wiaia[1] = Wiaia1;

  m_Tiain[0] = t0;
  m_Tiain[1] = t1;
  
  m_Biain[0] = b0;
  m_Biain[1] = b1;
}

//----------------------------------------------------------------------------------------------------------------------          
inline void Reflex::setMotoNeuronParameters(double Wiamn0, double Wiamn1)
{
  m_Wiamn[0] = Wiamn0;
  m_Wiamn[1] = Wiamn1;
}
  
} // namespace dmx

#endif