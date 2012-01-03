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
struct MinJerkTrajectory
{
  MinJerkTrajectory() : duration(1.0f), time(0.0f) {};
  void setNew(const Pos& initPos, const Pos& finalPos, float duration); 
  void update(float dt); 
  
  static Pos getPosition(const Pos& initPos, const Pos& finalPos, float duration, float time);
  
  Pos current;
  Pos initial;
  Pos target;
  float duration;
  float time;
};

//----------------------------------------------------------------------------------------------------------------------  
Pos MinJerkTrajectory::getPosition(const Pos& initPos, const Pos& finalPos, float duration, float time)
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
void MinJerkTrajectory::update(float dt)
{
  time += dt;
  current = getPosition(initial, target, duration, time);
}

//----------------------------------------------------------------------------------------------------------------------
void MinJerkTrajectory::setNew(const Pos& initPos, const Pos& finalPos, float duration)
{
  initial = initPos;
  target = finalPos;
  duration = duration;
  time = 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
// Point in a generic trajectory
//----------------------------------------------------------------------------------------------------------------------  
template <class T>
struct Target
{
  T position;
  float start;
  float stop;
  int id;
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
  
  Trajectory() : m_loop(false), m_blend(kTr_BlendNone) {};
  
  // Setters
  void add(Target<T> pos);
  void add(T pos, float duration);  
  void setLoop(bool l) { m_loop = l; };
  void setBlend(BlendType b) { m_blend = b; };
  
  // Getters
  Target<T>& operator[] (const int id) { assert(id < m_targets.size()); return m_targets[id]; };  
  const Target<T>& operator[] (const int id) const { assert(id < m_targets.size()); return m_targets[id]; };  
  
  T getPositionAtTime(float time);
  Target<T> atTime(float time);
  float getDuration() { return m_targets.back().stop; };
  float size() const { return m_targets.size(); };
  
protected:
  
  std::vector<Target<T> > m_targets;
                                  
  bool m_loop;
  BlendType m_blend;
  
}; // class Trajectory
  

//----------------------------------------------------------------------------------------------------------------------      
// Inline implementations  
//----------------------------------------------------------------------------------------------------------------------      
template <class T>
void Trajectory<T>::add(T pos, float duration)
{
  Target<T> target;
  target.position = pos;
  target.start = 0.0f;
  if(m_targets.size() > 0)
  {
    // starts when previous has finished
    target.start = m_targets.back().stop;
  }
  target.stop = target.start + duration;
  m_targets.push_back(target);
  
};  

//----------------------------------------------------------------------------------------------------------------------        
template <class T>
void Trajectory<T>::add(Target<T> target)
{
  m_targets.push_back(target);  
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
  if(m_loop && time > m_targets.back().stop)
  {
    time = fmod(time, m_targets.back().stop);
  }
  
  if(time < m_targets.back().stop)
  {
    // Within duration of set of targets
    for(int i = 0; i < m_targets.size(); i++)
    {
      if(time >= m_targets[i].start && time <= m_targets[i].stop)
      {
        target = m_targets[i];
        target.id = i;
        
        // Blend between current and next target
        if((m_blend > kTr_BlendNone) && (i < m_targets.size() - 1))
        {
          float duration = target.stop - target.start;
          if(m_blend == kTr_BlendLinear)
          {
            // Blend linearly
            float timeProp = (time - target.start) / duration;
            target.position = target.position + timeProp * (m_targets[i+1].position - target.position);
          }
          else 
          {
            // Create minimum jerk trajectory
            float t = time - m_targets[i].start;
            target.position = MinJerkTrajectory::getPosition(target.position, m_targets[i+1].position, duration, t);
          }
        }
        
        // We've found it
        break;
      }
    }
  }
  
  return target;
}
  
} // namespace dmx

#endif
