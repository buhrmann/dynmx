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
#include "Signal.h"
#include "MathUtils.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
/// A VITE/FLETE reflex network
//----------------------------------------------------------------------------------------------------------------------
class Reflex : public Model
{
  
friend class ArmReflexView;

public:
  
  struct SpindleLimits
  {
    Range pos, vel, dmp, exp, weight;
  };    
  
  
  Reflex();
  Reflex(Muscle* ag, Muscle* an);
  virtual ~Reflex(){};
  
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  
  // Setters
  void setOtherReflex(Reflex* r) { m_otherReflex = r; };
  void setDesiredAngles(double elbAngle, double shdAngle); // Get desired length from arm's desired joint angles    
  void setDesiredLength(double l0, double l1); // Will also update desired velocity, contraction etc...
  void setOpenLoop(double c0, double c1);  
  void setIntersegmentInput(double i1, double i2);
  void setSigmoidSlopes(double ia0, double ia1, double ib0, double ib1, double rn0, double rn1, double mn0, double mn1);
  void setSpindleParameters(double Kp0, double Kp1, double Kv0, double Kv1, double Kd0, double Kd1, double E0, double E1, double Ed0=1, double Ed1=1);
  void setLoadCompensationParameters(double g0, double g1, double inh0, double inh1);
  void setInertiaCompensationParameters(double g0, double g1, double b0, double b1);
  void setIaInParameters(double Wspia0, double Wspia1, double Wiaia0, double Wiaia1, double Waia0, double Waia1, 
                         double Wrnia0, double Wrnia1, double Wiamn0, double Wiamn1, 
                         double t1, double t2, double b1, double b2);
  void setIaInParametersMinimal(double Wspia0, double Wspia1, double Wiaia0, double Wiaia1, double Wiamn0, double Wiamn1, double t1, double t2);
  void setRenshawParameters(double Wmnrn0, double Wmnrn1, double Wrnrn0, double Wrnrn1, double Wrnmn0, double Wrnmn1,
                            double t1, double t2, double b1, double b2);  
  void setIbInParameters(double Wglib0, double Wglib1, double Wibib0, double Wibib1, double Wibmn0, double Wibmn1,
                         double t1, double t2, double b1, double b2);
  void setIbInParametersDeriv(double Wglib0, double Wglib1, double Wgdib0, double Wgdib1, double Wibib0, double Wibib1, double Wibmn0, double Wibmn1,
                         double t1, double t2, double b1, double b2);  
  void setMotoNeuronParameters(double Wspmn0, double Wspmn1);
  void setMNAsNeuron(double t0, double t1, double b0, double b1);
  void setIntersegmentalParameters(double Wisep0, double Wisep1);
  void setIbIntersegParameters(double ib0Ib0, double ib1Ib0, double ib0Ib1, double ib1Ib1,
                               double ib0Mn0, double ib1Mn0, double ib0Mn1, double ib1Mn1);
  void setIbRecExcIaParameters(double IbIn1Mn0, double IbIn0Mn1, double Ia0IbIn0, double Ia1IbIn1);
  void setCoconAsCCommand(bool b) { m_coconAsCCommand = b; };
  void setTorqueFeedbackPosMod(bool b) { m_trqFeedbackPosMod = b; };
  void setOpenLoopTimeConstant(double act, double deact) { m_openLoopTimeConstants[0] = act; m_openLoopTimeConstants[1] = deact; };
  void setCommandDelay(double delay) 
  { 
    m_commandDelayed[0] = Delay(delay, 0.001, m_muscles[0]->getUnitLength());
    m_commandDelayed[1] = Delay(delay, 0.001, m_muscles[1]->getUnitLength());
  };
  
  int decodeSpindleParams(const std::vector<double>& genome, int startIndex,  bool symmetric, bool velRef, const SpindleLimits& spindleLim);
  
  // Getters
  double getAlphaOutput(int i) { return m_alpha[i]; };
  double getIbInOutput(int i) { return m_IbInOut[i]; };
  double getGolgi(int i) { return m_golgi[i]; };
  double getOpenLoop(int i) { return m_openLoopFiltered[i]; };
  double getIntersegmentInput(int i) { return m_interSegmentInput[i]; };
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
  virtual void fromXml(const ci::XmlTree& xml);
  
  void record(Recorder& recorder);  
  
  // Public parameters
  //-------------------------------------------------
  // Spindle
  double m_Kspp [2];   // Spindle positional gain  
  double m_Kspv [2];   // Spindle velocity gain  
  double m_Kspd [2];   // Spindle damping
  double m_Espv [2];   // Spindle velocity exponent
  double m_Espd [2];   // Spindle damping exponent
  
  
protected:
  
  void recordStatePair(Recorder& recorder, const std::string& name, const double* var);
  
  static double spindleActivation(double x) { return x;/* / (1 + 100 * (x*x));*/ };
  //static double neuronActivation(double x) { return 1.0 / (1.0 + exp(5.0 - 12.0*x));};
  static double neuronActivation(double x, double k=1.0) { return 1.0 / (1.0 + exp(-k*x));};
  //static double neuronActivation(double x) { return clamp(x, 0.0, 1.0); };
  //static double neuronActivation(double x) { return smoothStep(0, 1, x); };
  void updateInLengthCoords(float dt);
  void updateInContractionCoords(float dt);  
  
  void paramToXml(ci::XmlTree& xml, const std::string& str, double* p); 
  void paramFromXml(const ci::XmlTree& xml, const std::string& str, double* p); 
  
  // Delay lines
  double m_feedbackDelay;
  double m_commandDelay;
  Delay m_posDelay [2];
  Delay m_interSegDelay [2];
  Delay m_forceDelay [2];
  Delay m_commandDelayed[2];
  
  // Derivative
  Derivative m_golgiDt[2];
  
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
  double m_Kiain [2];  // Sigmoid slope
  
  // Renshaw
  double m_Wmnrn [2];  // Input from motor neuron
  double m_Wrnrn [2];  // Reciprocal inhibition
  double m_Brn [2];
  double m_Trn [2];
  double m_Krn [2];
  
  // IbIn
  double m_Wglib [2];   // Golgi force input
  double m_Wgdib [2];   // Golgi force-derivative input
  double m_Wiaibin [2]; // Ia afferents to IbIn
  double m_Wibib [2];   // Reciprocal inhibition
  double m_WisibAg [2]; // Interjoint connections between other joint and agonist
  double m_WisibAn [2]; // Interjoint connections between other joint and antagonist
  double m_Bib [2];     // Bias
  double m_Tib [2];     // Tau
  double m_Kib [2];
  
  // Alpha motor neurons
  double m_Wiamn [2];   // IaIn to alpha MN
  double m_Wrnmn [2];   // Renshaw to alpha MN
  double m_Wibmn [2];   // IbIn to alpha MN
  double m_WeIbinMn[2]; // +IbIn to alpha MN (reciprocal excitation omitting extra interneuron)
  double m_WisibAgMn [2]; // Interjoint connections between other joint and agonist
  double m_WisibAnMn [2]; // Interjoint connections between other joint and antagonist
  double m_Wspmn [2];   // Stretch reflex (monosynaptic spindle input)
  double m_Bmn [2];
  double m_Tmn [2];
  double m_Kmn [2];
  
  // Intersegmental force feedback
  double m_Wisep [2];  // intersegmental influence on desired position
  
  bool m_MNasNeuron;
  bool m_coconAsCCommand;
  bool m_trqFeedbackPosMod;
  
  // Inputs
  //-------------------------------------------------  
  double m_openLoopPreFilter[2];
  double m_openLoopFiltered[2];
  double m_openLoopTimeConstants[2];
  double m_interSegmentInput[2]; 
  
  // State
  //-------------------------------------------------  
  double m_length [2];
  double m_vel [2];
  double m_posErr [2];
  double m_velErr [2];
  double m_spindlePri [2];
  double m_spindlePosRes [2];
  double m_spindleVelRes [2];
  double m_spindleDmpRes [2];
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
  double m_alphaState[2];
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
  
  // For distinguishing centrally specified position, and internally modified one
  double m_commandedLength[2];
  double m_commandedContraction[2];
  
  Muscle* m_muscles [2];
  Reflex* m_otherReflex;
};
  
//----------------------------------------------------------------------------------------------------------------------
/// Inline implementations
//----------------------------------------------------------------------------------------------------------------------
inline void Reflex::setIbRecExcIaParameters(double IbIn1Mn0, double IbIn0Mn1, double Ia0IbIn0, double Ia1IbIn1)
{
  m_WeIbinMn[0] = IbIn1Mn0;
  m_WeIbinMn[1] = IbIn0Mn1;
  m_Wiaibin[0] = Ia0IbIn0;
  m_Wiaibin[1] = Ia1IbIn1;
}
  
inline void Reflex::setIbIntersegParameters(double ib0Ib0, double ib1Ib0, double ib0Ib1, double ib1Ib1,
                                            double ib0Mn0, double ib1Mn0, double ib0Mn1, double ib1Mn1)
{
  m_WisibAg[0] = ib0Ib0;
  m_WisibAg[1] = ib0Ib1;
  m_WisibAn[0] = ib1Ib0;
  m_WisibAn[1] = ib1Ib1;
  
  m_WisibAgMn[0] = ib0Mn0;
  m_WisibAgMn[1] = ib0Mn1;
  m_WisibAnMn[0] = ib1Mn0;
  m_WisibAnMn[1] = ib1Mn1;
}
  
//----------------------------------------------------------------------------------------------------------------------
inline void Reflex::setSigmoidSlopes(double ia0, double ia1, double ib0, double ib1,
                                     double rn0, double rn1, double mn0, double mn1)
{
  m_Kiain[0] = ia0;
  m_Kiain[1] = ia1;
  m_Kib[0] = ib0;
  m_Kib[1] = ib1;
  m_Krn[0] = rn0;
  m_Krn[1] = rn1;
  m_Kmn[0] = mn0;
  m_Kmn[1] = mn1;
};

//----------------------------------------------------------------------------------------------------------------------
inline void Reflex::setIntersegmentInput(double i0, double i1)
{
  m_interSegmentInput[0] = i0;
  m_interSegmentInput[1] = i1;
};

//----------------------------------------------------------------------------------------------------------------------      
inline void Reflex::setOpenLoop(double c0, double c1) 
{ 
  m_openLoopPreFilter[0] = c0; 
  m_openLoopPreFilter[1] = c1;  
};

//----------------------------------------------------------------------------------------------------------------------    
inline void Reflex::setSpindleParameters(double Kp0, double Kp1, double Kv0, double Kv1, double Kd0, double Kd1, double E0, double E1, double Ed0, double Ed1)
{ 
  m_Kspp[0] = Kp0; 
  m_Kspp[1] = Kp1; 
  m_Kspv[0] = Kv0; 
  m_Kspv[1] = Kv1; 
  m_Kspd[0] = Kd0;
  m_Kspd[1] = Kd1;
  m_Espv[0] = E0; 
  m_Espv[1] = E1;
  m_Espd[0] = Ed0; 
  m_Espd[1] = Ed1;  
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
inline void Reflex::setIaInParametersMinimal(double Wspia0, double Wspia1, double Wiaia0, double Wiaia1, 
                                             double Wiamn0, double Wiamn1, double t0, double t1)
{
  m_Wspia[0] = Wspia0;
  m_Wspia[1] = Wspia1;
  m_Wiaia[0] = Wiaia0;
  m_Wiaia[1] = Wiaia1;
  m_Wiamn[0] = Wiamn0;
  m_Wiamn[1] = Wiamn1;
  m_Tiain[0] = t0;
  m_Tiain[1] = t1;
};

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
  
inline void Reflex::setIbInParametersDeriv(double Wglib0, double Wglib1, double Wgdib0, double Wgdib1, double Wibib0, double Wibib1, double Wibmn0, double Wibmn1,
                                           double t0, double t1, double b0, double b1)
{
  m_Wglib[0] = Wglib0;
  m_Wglib[1] = Wglib1;
  m_Wgdib[0] = Wgdib0;
  m_Wgdib[1] = Wgdib1;
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


inline void Reflex::setMNAsNeuron(double t0, double t1, double b0, double b1)
{
  m_MNasNeuron = true;
  m_Tmn[0] = t0;
  m_Tmn[1] = t1;
  m_Bmn[0] = b0;
  m_Bmn[1] = b1;
}

//----------------------------------------------------------------------------------------------------------------------            
inline void Reflex::setIntersegmentalParameters(double Wisep0, double Wisep1)
{
  m_Wisep[0] = Wisep0;
  m_Wisep[1] = Wisep1;
}

  
} // namespace dmx

#endif