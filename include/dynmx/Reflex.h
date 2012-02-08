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
#include "MathUtils.h"

namespace dmx
{
   
//----------------------------------------------------------------------------------------------------------------------
/// A VITE/FLETE reflex network
//----------------------------------------------------------------------------------------------------------------------
class Reflex : public Model
{

public:
  
  Reflex();
  Reflex(Muscle* ag, Muscle* an);
  
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  
  // Setters
  void setDesiredAngles(double elbAngle, double shdAngle); // Get desired length from arm's desired joint angles    
  void setDesiredLength(double l0, double l1); // Will also update desired velocity, contraction etc...
  void setCocontraction(double c0, double c1);  
  void setSpindleParameters(double Kp0, double Kp1, double Kv0, double Kv1, double Kd0, double Kd1, double E0, double E1);
  void setLoadCompensationParameters(double g0, double g1, double inh0, double inh1);
  void setInertiaCompensationParameters(double g0, double g1, double b0, double b1);
  void setIaInParameters(double Wspia0, double Wspia1, double Wiaia0, double Wiaia1, double Waia0, double Waia1, 
                         double Wrnia0, double Wrnia1, double Wiamn0, double Wiamn1, 
                         double t1, double t2, double b1, double b2);
  void setRenshawParameters(double Wmnrn0, double Wmnrn1, double Wrnrn0, double Wrnrn1, double Wrnmn0, double Wrnmn1,
                            double t1, double t2, double b1, double b2);  
  void setIbInParameters(double Wglib0, double Wglib1, double Wibib0, double Wibib1, double Wibmn0, double Wibmn1,
                         double t1, double t2, double b1, double b2);  
  void setMotoNeuronParameters(double Wspmn0, double Wspmn1);
  void setIntersegmentalParameters(double Wismn0, double Wismn1, double Wisia0, double Wisia1);
  
  // Getters
  double getAlphaOutput(int i) { return m_alpha[i]; };
  double getCoContraction(int i) { return m_cocontraction[i]; };
  double getLength(int i) { return m_length[i]; };
  double getDesiredLength(int i) { return m_desiredLength[i]; };
  double getDesiredVelocity(int i) { return m_desiredVelocity[i]; };
  double getPositionError(int i) { return m_posErr[i]; }
  double getVelocityError(int i) { return m_velErr[i]; }
  double getContractionVelocity(int i) { return m_contractionVel[i]; };
  Muscle* getMuscle(int i) { assert(i==0 || i==1); return m_muscles[i];};
  const ArmMuscled* getArm() const { return m_muscles[0]->getArm(); };
  
  // Store output in human readable format
  virtual void toXml(ci::XmlTree& xml);  
  
  // Public parameters
  //-------------------------------------------------
  // Spindle
  double m_Kspp [2];   // Spindle positional gain  
  double m_Kspv [2];   // Spindle velocity gain  
  double m_Kspd [2];   // Spindle damping
  double m_Espv [2];   // Spindle velocity exponent
  
  
protected:
  
  static double spindleActivation(double x) { return x;/* / (1 + 100 * (x*x));*/ };
  static double neuronActivation(double x) { return 1.0 / (1.0 + exp(-x));};
  //static double neuronActivation(double x) { return clamp(x, 0.0, 1.0); };
  //static double neuronActivation(double x) { return smoothStep(0, 1, x); };
  void updateInLengthCoords(float dt);
  void updateInContractionCoords(float dt);  
  
  void paramToXml(ci::XmlTree& xml, const std::string& str, double* p); 
  
  
  // Parameters
  //-------------------------------------------------
  
  // Static load compensation  
  double m_Ksfv [2];   // Strength and speed of sfv load compensation (gravity)
  double m_Ksfi [2];   // Reciprocal inhibition of sfv  load compensation (gravity)
  
  // Dynamic load compensation
  double m_Kifv [2];   // Gain of inertial force compensation (velocity error)
  double m_Bifv [2];   // Bias of inertial force compensation (velocity error)  
  
  // IaIn
  double m_Wiaia [2];  // Weight of IaIn reciprocal inhibition
  double m_Wspia [2];  // Weight of spindle input  
  double m_Wrnia [2];  // Weight of renshaw input    
  double m_Waia  [2];  // Weight for desired contraction
  double m_Biain [2];  // Bias of Ia neurons
  double m_Tiain [2];  // Time constant (really 1/t)
  
  // Renshaw
  double m_Wmnrn [2];  // Input from motor neuron
  double m_Wrnrn [2];  // Reciprocal inhibition
  double m_Brn [2];
  double m_Trn [2];
  
  // IbIn
  double m_Wglib [2];   // Golgi inpuit
  double m_Wibib [2];   // Reciprocal inhibition
  double m_Bib [2];     // Bias
  double m_Tib [2];     // Tau
  
  // Alpha motor neurons
  double m_Wiamn [2];   // IaIn to alpha MN
  double m_Wrnmn [2];   // Renshaw to alpha MN
  double m_Wibmn [2];   // IbIn to alpha MN
  double m_Wspmn [2];   // Stretch reflex (monosynaptic spindle input)
  
  // Intersegmental force feedback
  double m_Wismn [2];  // intersegmental input to alpha MN
  double m_Wisia [2];  // intersegmental input to IaIn
  
  // Inputs
  //-------------------------------------------------  
  double m_cocontraction[2];
  double m_interSegmentInput[2]; 
  
  // State
  //-------------------------------------------------  
  double m_length [2];
  double m_vel [2];
  double m_posErr [2];
  double m_velErr [2];
  double m_spindlePri [2];
  double m_spindleSec [2];
  double m_golgi[2];
  
  double m_IaIn [2];              // Ia inhibitory interneurons  
  double m_IaInOut [2];
  double m_Rn [2];                // Renshaw interneurons  
  double m_RnOut [2];  
  double m_IbIn [2];              // Ib force feedback inhibitory interneurons
  double m_IbInOut [2];
  
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
inline void Reflex::setSpindleParameters(double Kp0, double Kp1, double Kv0, double Kv1, double Kd0, double Kd1, double E0, double E1)
{ 
  m_Kspp[0] = Kp0; 
  m_Kspp[1] = Kp1; 
  m_Kspv[0] = Kv0; 
  m_Kspv[1] = Kv1; 
  m_Kspd[0] = Kd0;
  m_Kspd[1] = Kd1;
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
inline void Reflex::setIaInParameters(double Wspia0, double Wspia1, double Wiaia0, double Wiaia1, double Waia0, double Waia1,
                                      double Wrnia0, double Wrnia1, double Wiamn0, double Wiamn1, 
                                      double t0, double t1, double b0, double b1)
{
  m_Wspia[0] = Wspia0;
  m_Wspia[1] = Wspia1;
  m_Wiaia[0] = Wiaia0;
  m_Wiaia[1] = Wiaia1;
  m_Waia[0] = Waia0;
  m_Waia[1] = Waia1;
  m_Wrnia[0] = Wrnia0;
  m_Wrnia[1] = Wrnia1;
  m_Wiamn[0] = Wiamn0;
  m_Wiamn[1] = Wiamn1;

  m_Tiain[0] = t0;
  m_Tiain[1] = t1;
  
  m_Biain[0] = b0;
  m_Biain[1] = b1;
}

//----------------------------------------------------------------------------------------------------------------------            
inline void Reflex::setRenshawParameters(double Wmnrn0, double Wmnrn1, double Wrnrn0, double Wrnrn1, double Wrnmn0, double Wrnmn1,
                                         double t0, double t1, double b0, double b1)
{
  m_Wmnrn[0] = Wmnrn0;
  m_Wmnrn[1] = Wmnrn1;
  m_Wrnrn[0] = Wrnrn0;
  m_Wrnrn[1] = Wrnrn1;
  m_Wrnmn[0] = Wrnmn0;
  m_Wrnmn[1] = Wrnmn1;
  
  m_Trn[0] = t0;
  m_Trn[1] = t1;
  
  m_Brn[0] = b0;
  m_Brn[1] = b1;
}
  
//----------------------------------------------------------------------------------------------------------------------            
inline void Reflex::setIbInParameters(double Wglib0, double Wglib1, double Wibib0, double Wibib1, double Wibmn0, double Wibmn1,
                                      double t0, double t1, double b0, double b1)
{
  m_Wglib[0] = Wglib0;
  m_Wglib[1] = Wglib1;
  m_Wibib[0] = Wibib0;
  m_Wibib[1] = Wibib1;
  m_Wibmn[0] = Wibmn0;
  m_Wibmn[1] = Wibmn1;
  
  m_Tib[0] = t0;
  m_Tib[1] = t1;
  
  m_Bib[0] = b0;
  m_Bib[1] = b1;
}  

//----------------------------------------------------------------------------------------------------------------------          
inline void Reflex::setMotoNeuronParameters(double Wspmn0, double Wspmn1)
{
  m_Wspmn[0] = Wspmn0;
  m_Wspmn[1] = Wspmn1;
}
  
//----------------------------------------------------------------------------------------------------------------------            
inline void Reflex::setIntersegmentalParameters(double Wismn0, double Wismn1, double Wisia0, double Wisia1)
{
  m_Wismn[0] = Wismn0;
  m_Wismn[1] = Wismn1;
  m_Wisia[0] = Wisia0;
  m_Wisia[1] = Wisia1;
}

  
} // namespace dmx

#endif