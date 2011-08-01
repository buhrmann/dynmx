/*
 *  Muscle.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_MUSCLE_
#define _DMX_MUSCLE_

#include "Dynmx.h"
#include "Model.h"
#include "cinder/Vector.h"

#include <iostream>
#include <fstream>

namespace dmx
{

class ArmMuscled;
struct MusclePathPoint;
class Recorder;
  
//----------------------------------------------------------------------------------------------------------------------
// Hill-type non-linear muscle model
//----------------------------------------------------------------------------------------------------------------------
class Muscle : public Model
{

friend class Arm3dView;
  
public:
  
  typedef ci::Vec2f Pos;   
  
  // Inherited from class Model
  virtual void init();
  virtual void reset();  
  virtual void update(float dt);
  
  void setExcitation(double e) { assert(e >=0 && e <= 1); m_excitation = e; };
  
  void setParameters(double maxIsoForce, double optimalLength, double maxVelocity);
  
  // Getters
  double getLength() const { return m_length; };
  double getNormalisedLength() const { return m_lengthNorm; };
  double getVelocity() const { return m_velocity; };
  double getNormalisedVelocity() const { return m_velocityNorm; };
  double getForce() const { return m_force; };
  double getActiveForce() const { return m_activeForceNorm; };
  double getPassiveForce() const { return m_passiveForceNorm; };
  double getVelocityForce() const { return m_velocityForceNorm; };
  double getExcitation() const { return m_excitation; };
  double* getExcitation() { return &m_excitation; }; 
  double getActivation() const { return m_activation; };
  bool isMonoArticulate() const { return m_isMonoArticulate; };
  bool isFlexor() const { return m_isFlexor; };
  
  virtual Pos getOriginWorld() = 0;
  virtual Pos getInsertionWorld() = 0;
  
protected:

  virtual void updateLengthAndMomentArm() = 0;
  
  double calcActiveForceNorm(double lengthNorm);
  double calcPassiveForceNorm(double lengthNorm);
  double calcVelocityForceNorm(double velNorm);
  double calcActivation(double activation, double excitation, float dt);

  ArmMuscled* m_arm;

  // Parameters different per muscle
  double m_maxForce; 
  double m_maxVelocity; 
  double m_lengthOpt;
  
  bool m_isMonoArticulate;
  bool m_isFlexor;  
  
  // Parameters automatically determined for all muscles
  double m_passiveForceGain;  // How much passive elasticity is produced
  double m_tauAct;            // time constant describing increasing activation
  double m_tauDeact;          // time constant describing decreasing activation
  
  // State
  double m_excitation;        // neural input
  double m_activation;        // muscle activation
  double m_length;
  double m_lengthNorm;        // scaled by optimal length
  double m_velocity;
  double m_velocityNorm;      // scaled my maximum velocity
  double m_passiveForceNorm;  //
  double m_activeForceNorm;   //
  double m_velocityForceNorm; //
  double m_momentArm;
  double m_force;
  double m_torque[2];     // around the two joints
};

} // namespace dmx

#endif