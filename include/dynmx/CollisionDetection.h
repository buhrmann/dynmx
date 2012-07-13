/*
 *  CollisionDetection.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _COLLISION_DETECTION_
#define _COLLISION_DETECTION_

#include "Dynmx.h"
#include <math.h>
#include "MathUtils.h"
#include "cinder/Vector.h"
#include "SMCEnvironment.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------  
static inline float pointToGaussianDist(ci::Vec2f& res, const ci::Vec2f& p, const Gaussian& g)
{
  float projection = (p - g.getPosition()).dot(g.getDir());
  float y = g.getValueAt(projection);
  if (y > 0.0)
  {
    res = g.getPointFromCoords(projection, y);
    return (res - p).length();
  }

  return -1;
}
  
// Returns whether intersection occured in return argument and intersection point in first argument
// For details see http://paulbourke.net/geometry/lineline2d/
//----------------------------------------------------------------------------------------------------------------------
static inline int lineSegmentIntersect(ci::Vec2f& res, const ci::Vec2f& p1, const ci::Vec2f& p2, const ci::Vec2f& p3, const ci::Vec2f& p4)
{
  float p43x = (p4.x - p3.x);  
  float p43y = (p4.y - p3.y);
  float p21x = (p2.x - p1.x);
  float p21y = (p2.y - p1.y);
  float denom = (p43y * p21x) - (p43x * p21y);
  
  // lines are parallel
  if(denom == 0)
  {
    return false;
  }
  
  float p13y = (p1.y - p3.y);
  float p13x = (p1.x - p3.x);
  float ua = (p43x * p13y) - (p43y * p13x);
  ua /= denom;
  float ub = (p21x * p13y) - (p21y * p13x);
  ub /= denom;
  
  // ua and ub have to lie in [0,1] for intersection of the segments (rather than just infinite lines)
  bool intersectWithinSegments = (ua >=0 && ua <= 1) && (ub >=0 && ub <= 1);
  if(!intersectWithinSegments)
  {
    return 0;
  }
  else
  {
    res = p1 + ci::Vec2f(ua, ua) * (p2 - p1);
    return 1;
  }
}

// Returns intersection points in first argument
//----------------------------------------------------------------------------------------------------------------------
static inline void lineSegmentTriangleIntersect(std::vector<ci::Vec2f>& res, const ci::Vec2f& l1, const ci::Vec2f& l2, 
                                               const ci::Vec2f& p1, const ci::Vec2f& p2, const ci::Vec2f& p3)
{
  ci::Vec2f intersection;
  if( lineSegmentIntersect(intersection, l1, l2, p1, p2))
  {
    res.push_back(intersection);
  }
  if( lineSegmentIntersect(intersection, l1, l2, p2, p3))
  {
    res.push_back(intersection);
  }
  if( lineSegmentIntersect(intersection, l1, l2, p3, p1))
  {
    res.push_back(intersection);
  }
}

// Returns closest intersection between ray segment and triangle and its distance (or -1 in case of no intersection)
//----------------------------------------------------------------------------------------------------------------------
static inline float raySegmentTriangleIntersect(ci::Vec2f& res, const ci::Vec2f& start, const ci::Vec2f& end, 
                                               const ci::Vec2f& p1, const ci::Vec2f& p2, const ci::Vec2f& p3)
{
  // Get all intersections
  std::vector<ci::Vec2f> intersections;
  lineSegmentTriangleIntersect(intersections, start, end, p1, p2, p3);

  if(intersections.size() == 0)
    return -1;
  
  // Find closest
  float closestDistSq = intersections[0].distanceSquared(start);
  int closestIntersection = 0;
  for(int i = 1; i < intersections.size(); i++)
  {
    float distSq = intersections[i].distanceSquared(start);
    if(distSq < closestDistSq)
    {
      closestDistSq = distSq;
      closestIntersection = i;
    }
  }

  res = intersections[closestIntersection];
  return sqrt(closestDistSq);
}

// Returns closest intersection between circle and line segment (and distance or -1 if no intersection)
// The distance return is that between the intersection and the start of the line p1
// Also see http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
// and http://paulbourke.net/geometry/sphereline/
//----------------------------------------------------------------------------------------------------------------------
static inline float lineSegmentCircleClosestIntersect(ci::Vec2f& res, const ci::Vec2f& p1, const ci::Vec2f& p2, 
                                                      const ci::Vec2f& c, float r)
{
  ci::Vec2f line = p2 - p1;
  float length = line.length();
  ci::Vec2f lineDir = line / length;
  ci::Vec2f toCircle = c - p1;
  float projDist = toCircle.dot(lineDir);
  
  ci::Vec2f closestOnLine; 
  if (projDist < 0)
  {
    closestOnLine = p1;
  }
  else if (projDist > length)
  {
    closestOnLine = p2;
  }
  else
  {
    closestOnLine = p1 + lineDir * projDist;
  }
  
  float distSq = closestOnLine.distanceSquared(c);
  if(distSq < r*r)
  {
    // We have an intersection. Solve line=circle equation.
    float a = (p2 - p1).lengthSquared();
    float b = 2 * ( ((p2.x - p1.x) * (p1.x - c.x)) + ((p2.y - p1.y) * (p1.y - c.y)) );
    float d = c.lengthSquared() + p1.lengthSquared() - 2 * (c.x*p1.x + c.y*p1.y) - sqr(r);
    float i = sqr(b) - 4 * a * d;
    
    if (i == 0.0)
    {
      // Exactly one intersection
      float mu = -b/(2*a);
      res = p1 + mu * (p2 - p1);
      return res.distance(p1);
    }
    else if ( i > 0.0 )
    {
      // Two intersections
      float root = sqrt(sqr(b) - 4.0f*a*d);
      float denom = 1.0f / (2.0f*a);
      
      // First intersection
      float mu1 = (-b + root) * denom;
      ci::Vec2f isct1 = p1 + mu1 * (p2 - p1);
      float dist1 = isct1.distance(p1);
      
      // Second intersection
      float mu2 = (-b - root) * denom;
      ci::Vec2f isct2 = p1 + mu2 * (p2 - p1);
      float dist2 = isct2.distance(p1);      
      
      // Return closer
      if(dist1 < dist2)
      {
        res = isct1;
        return dist1;
      }
      else
      {
        res = isct2;
        return dist2;
      }
    }
    else 
    {
      // No intersection
      return -1;
    }
  }
  else
  {
    // No intersection
    return -1;
  }
}

} // namespace

#endif