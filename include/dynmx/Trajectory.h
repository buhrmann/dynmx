/*
 *  Trajectory.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/19/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TRAJECTORY_
#define _DMX_TRAJECTORY_

#include "Dynmx.h"
#include <vector>

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// Smoothest motion over given duration (2d)
//----------------------------------------------------------------------------------------------------------------------
class MinJerkTrajectory
{
public:
  MinJerkTrajectory() : duration(1.0f), time(0.0f) {};
  void setNew(const Pos& initPos, const Pos& finalPos, float duration); 
  void update(float dt); 
  
  static Pos getPosition(const Pos& initPos, const Pos& finalPos, float duration, float time);
  
protected:  
  Pos current;
  Pos initial;
  Pos target;
  float duration;
  float time;
};

//----------------------------------------------------------------------------------------------------------------------  
inline Pos MinJerkTrajectory::getPosition(const Pos& initPos, const Pos& finalPos, float duration, float time)
{
  assert(time >= 0.0);
  if (time > duration)
  {
    time = duration;
  }
  
  float t = time / duration;
  float amplitude = 15*t*t*t*t - 6*t*t*t*t*t - 10*t*t*t;
  
  Pos current;
  current.x = initPos.x + (initPos.x - finalPos.x) * amplitude;
  current.y = initPos.y + (initPos.y - finalPos.y) * amplitude;  
  return current;
}
  
//----------------------------------------------------------------------------------------------------------------------
inline void MinJerkTrajectory::update(float dt)
{
  time += dt;
  current = getPosition(initial, target, duration, time);
}

//----------------------------------------------------------------------------------------------------------------------
inline void MinJerkTrajectory::setNew(const Pos& initPos, const Pos& finalPos, float dur)
{
  initial = initPos;
  target = finalPos;
  duration = dur;
  time = 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
// Point in a generic trajectory
//----------------------------------------------------------------------------------------------------------------------  
template <class T>
struct Target
{
  Target() {};
  Target(T pos, float t, int nm=0) : position(pos), time(t), name(nm) {};
  
  T position;
  float time;
  int id;
  int name;
};
  
//----------------------------------------------------------------------------------------------------------------------
/// A list of targets with option to blend between them and loop
//----------------------------------------------------------------------------------------------------------------------
template <class T>
class Trajectory
{
  
public:
  
  enum BlendType
  {
    kTr_BlendNone,
    kTr_BlendLinear,
    kTr_BlendMinJerk
  };
  
  Trajectory() : m_loop(false), m_blend(kTr_BlendNone), m_duration(0.0) {};
  
  // Setters
  void add(Target<T> pos);
  void add(T pos, float time, int name = 0);  
  void setLoop(bool l) { m_loop = l; };
  void setBlend(BlendType b) { m_blend = b; };
  void clear() { m_targets.clear(); m_duration = 0.0; };
  
  // Getters
  Target<T>& operator[] (const int id) { assert(id < m_targets.size()); return m_targets[id]; };  
  const Target<T>& operator[] (const int id) const { assert(id < m_targets.size()); return m_targets[id]; };  
  std::vector<Target<T> >& getTargets() { return m_targets; };
  
  T getPositionAtTime(float time);
  Target<T> atTime(float time);
  float getDuration() { return m_targets.back().time; };
  float size() const { return m_targets.size(); };
  
protected:
  
  std::vector<Target<T> > m_targets;
                                  
  bool m_loop;
  BlendType m_blend;
  float m_duration;
  
}; // class Trajectory
  

//----------------------------------------------------------------------------------------------------------------------      
// Inline implementations  
//----------------------------------------------------------------------------------------------------------------------      
template <class T>
void Trajectory<T>::add(T pos, float time, int name)
{
  float accTime = m_duration + time;
  add(Target<T> (pos, accTime, name));
};  

//----------------------------------------------------------------------------------------------------------------------        
template <class T>
void Trajectory<T>::add(Target<T> target)
{  
  m_targets.push_back(target);  
  m_duration = m_targets.back().time;  
};    

//----------------------------------------------------------------------------------------------------------------------      
template <class T>
T Trajectory<T>::getPositionAtTime(float time)
{
  return atTime(time).position;
};
  
//----------------------------------------------------------------------------------------------------------------------      
template <class T>
Target<T> Trajectory<T>::atTime(float time)
{  
  Target<T> target;
  
  // Loop around
  if(m_loop && (time > m_duration))
  {
    time = fmod(time, m_duration);
  }
  
  if(time < m_duration)
  {
    // Within duration of set of targets
    for(int i = 0; i < m_targets.size() - 1; i++)
    {
      if(time >= m_targets[i].time && time < m_targets[i+1].time)
      {
        target = m_targets[i];
        target.id = i;
        const Target<T>& nextTarget = m_targets[i+1];
        
        // Blend between current and next target
        if((m_blend > kTr_BlendNone) && (target.position != nextTarget.position))
        {
          float duration = nextTarget.time - target.time;
          if(m_blend == kTr_BlendLinear)
          {
            // Blend linearly
            float timeProp = (time - target.time) / duration;
            target.position = target.position + timeProp * (nextTarget.position - target.position);
          }
          else 
          {
            // Create minimum jerk trajectory
            float t = time - target.time;
            target.position = MinJerkTrajectory::getPosition(target.position, nextTarget.position, duration, t);
          }
        }
        
        // We've found it!
        // If two targets have the same position in time, then time couldn't be greater than the first and
        // less than the second. So only the second will be returned.
        break;
      }
    }
  }
  
  return target;
}
  
} // namespace dmx

#endif
