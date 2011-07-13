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
#include "cinder/Vector.h"

namespace dmx
{

class Arm;
struct MusclePathPoint;

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
struct MusclePathPoint
{
  ci::Vec2f local;  /// Specified as local distance from joint along bone and perpendicular to bone.
  ci::Vec2f world;  /// Converted from local point into world;
  int joint;        /// Joint the point is local to.
};

//----------------------------------------------------------------------------------------------------------------------
// Hill-type non-linear muscle model
//----------------------------------------------------------------------------------------------------------------------
class Muscle
{

public:

  Muscle(Arm* arm) : m_arm(arm), m_length(0.0), m_lengthOpt(1.0), m_velocity(0.0), m_excitation(0.0) {};
  
  void addPathPoint(const ci::Vec2f& point, int jointId);  
  
  void init();
  
  void update(float dt);
  
  double getLength() const { return m_length; };
  double getNormalisedLength() const { return m_lengthNorm; };
  double getVelocity() const { return m_velocity; };
  double getForce() const { return m_force; };
  int getNumPathPoints() const { return m_path.size(); };
  const ci::Vec2f& getPathPointWorld(int i) const { assert(i < m_path.size()); return m_path[i].world; };
  
protected:

  void transformPathToWorld();
  double calcLength();
  
  double calcActiveForceNorm();
  double calcPassiveForceNorm();
  double calcVelocityForceNorm();
  double calcActivation();

  Arm* m_arm;
  
  // Each Pos defines a waypoint along the muscle path. X specifies the distance from the root joint along the bone, 
  // and Y the perpendicular distance from the bone.
  std::vector<MusclePathPoint> m_path;

  // Parameters different per muscle
  double m_maxForce; 
  double m_maxVelocity; 
  double m_lengthOpt;
  
  // Parameters automatically determined for all muscles
  double m_passiveForceGain;  // How much passive elasticity is produced
  double m_tauAct;            // time constant describing increasing activation
  double m_tauDeact;          // time constant describing decreasing activation
  
  // State
  double m_excitation;    // neural input
  double m_activation;    // muscle activation
  double m_length;
  double m_lengthNorm;    // scaled by optimal length
  double m_velocity;
  double m_velocityNorm;  // scaled my maximum velocity
  double m_force;
  double m_torque[2];     // around the two joints
};

} // namespace dmx

#endif