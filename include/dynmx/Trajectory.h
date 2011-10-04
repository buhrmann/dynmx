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
template <class T>
struct Target
{
  T position;
  float start;
  float stop;
  int id;
};
  
//----------------------------------------------------------------------------------------------------------------------
/// A list of targets with option to linearly shift between them
//----------------------------------------------------------------------------------------------------------------------
template <class T>
class Trajectory
{
  
public:
  
  Trajectory() : m_loop(false), m_blend(false) {};
  
  // Setters
  void add(T pos, float duration);  
  void setLoop(bool l) { m_loop = l; };
  void setBlend(bool b) { m_blend = b; };
  
  // Getters
  T getPositionAt(float time);
  Target<T> at(float time);
  const Target<T>& getTarget(int id) const { assert(id < m_targets.size()); return m_targets[id]; };
  float getDuration() { return m_targets.back().stop; };
  
  
protected:
  
  std::vector<Target<T> > m_targets;
                                  
  bool m_loop;
  bool m_blend;
  
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
T Trajectory<T>::getPositionAt(float time)
{
  return at(time).position;
};
  
//----------------------------------------------------------------------------------------------------------------------      
template <class T>
Target<T> Trajectory<T>::at(float time)
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
        
        // Linearly blend between current and next target
        if(m_blend && i < m_targets.size() - 1)
        {
          float duration = m_targets[i].stop - m_targets[i].start;
          float timeProp = (time - m_targets[i].start) / duration;
          target.position = m_targets[i].position + timeProp * (m_targets[i+1].position - m_targets[i].position);
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
