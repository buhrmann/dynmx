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
  
public:
  
  typedef ci::Vec2f Pos;   
  
  // Inherited from class Model
  virtual void init();
  virtual void reset();  
  virtual void update(float dt);
  
  void setExcitation(double e)
  { 
    assert(e >=0 && e <= 1);
    m_excitation = e; 
  };
  
  void setParameters(double maxIsoForce, double optimalLength, double maxVelocity);
  void setHillParameters(double hSh, double hLn, double hMax, double hSlope);
  void setOriginInsertion(double origin, double insertion) { m_originJointDist = origin; m_insertJointDist = insertion; };
  
  // Getters
  double getLength() const { return m_length; };
  double getLengthMin() const { return m_lengthMin; };
  double getLengthMax() const { return m_lengthMax; };
  double getNormalisedLength() const { return m_lengthNorm; };
  double getUnitLength() const { return m_lengthUnit; };
  double getOptimalLength() const { return m_lengthOpt; };
  
  double getVelocity() const { return m_velocity; };
  double getVelocityMax() const { return m_maxVelocity; };
  double getNormalisedVelocity() const { return m_velocityNorm; };
  
  double getForce() const { return m_force; };
  double getForceMax() const { return m_maxForce; };
  double getActiveForce() const { return m_activeForceNorm; };
  double getPassiveForce() const { return m_passiveForceNorm; };
  double getVelocityForce() const { return m_velocityForceNorm; };
  
  double getExcitation() const { return m_excitation; };
  double getActivation() const { return m_activation; };
  
  bool isMonoArticulate() const { return m_isMonoArticulate; };
  bool isFlexor() const { return m_isFlexor; };
  double getOrigin() const { return m_originJointDist; };
  double getInsertion() const { return m_insertJointDist; };
  virtual Pos getOriginWorld() = 0;
  virtual Pos getInsertionWorld() = 0;
  
  // Conversion between unit length [0,1] and actual length
  double lengthToUnitLength (double l) { return (l - m_lengthMin) / (m_lengthMax - m_lengthMin); };
  double unitLengthToLength (double l) { return m_lengthMin + l * (m_lengthMax - m_lengthMin); };
  
  // "InverseKinematics"
  virtual double getLengthFromJointAngles(double elbAngle, double shdAngle) = 0;
  
  const ArmMuscled* getArm() const { return m_arm; };
  
  // Store output in human readable format
  virtual void toXml(ci::XmlTree& xml);  
  
  // Parameters different per muscle
  double m_maxForce; 
  double m_lengthOpt;  
  
protected:

  virtual void updateLengthAndMomentArm() = 0;
  virtual void calculateMinMaxLength(double& min, double& max) = 0;  
  
  double calcActiveForceNorm(double lengthNorm);
  double calcPassiveForceNorm(double lengthNorm);
  double calcVelocityForceNorm(double velNorm);
  double calcActivation(double activation, double excitation, float dt);

  ArmMuscled* m_arm;

  // Parameters different per muscle
  double m_maxVelocity; 
  double m_originJointDist;
  double m_insertJointDist;
  double m_hillSh;
  double m_hillLn;
  double m_hillMax;
  double m_hillSlope;
  double m_hillSub;
  
  bool m_isMonoArticulate;
  bool m_isFlexor;  
  
  // Parameters automatically determined for all muscles
  double m_passiveForceGain;  // How much passive elasticity is produced
  double m_tauAct;            // time constant describing increasing activation
  double m_tauDeact;          // time constant describing decreasing activation
  double m_lengthMin;         // Based on arm's joint limits what's the min/max length 
  double m_lengthMax;
  
  // State
  double m_excitation;        // neural input
  double m_activation;        // muscle activation
  double m_length;
  double m_lengthNorm;        // scaled by optimal length
  double m_lengthUnit;        // scaled by min and max length
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