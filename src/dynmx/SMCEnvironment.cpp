/*
 *  SMCEnvironment.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 3/9/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */


#include "SMCEnvironment.h"
#include "Random.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------    
// Positionable  
//----------------------------------------------------------------------------------------------------------------------    
void Positionable::randomisePosition() 
{ 
  const float randXOffset = UniformRandom(-m_positionVar.x, m_positionVar.x);
  const float randYOffset = UniformRandom(-m_positionVar.y, m_positionVar.y);
  ci::Vec2f randOffset (randXOffset, randYOffset);
  
  setPosition(m_positionMean + randOffset);
}

//----------------------------------------------------------------------------------------------------------------------      
void Positionable::randomiseAngle() 
{ 
  const float randAngle = UniformRandom(-m_angleVar, m_angleVar);
  setAngle(m_angleMean + randAngle);
}
  
//----------------------------------------------------------------------------------------------------------------------    
// Environment  
//----------------------------------------------------------------------------------------------------------------------  
void SMCEnvironment::fromXml(const ci::XmlTree& xml)
{
  for(ci::XmlTree::ConstIter elem = xml.begin(); elem != xml.end(); ++elem)
  {
    const ci::XmlTree& obj = *elem;
    ci::Vec2f pos = ci::Vec2f(obj["PosX"].as<float>(), obj["PosY"].as<float>());
    
    if(obj.getTag() == "Line")
    {
      Line* l = new Line (pos, degreesToRadians(obj["Angle"].as<float>()), obj["Length"].as<float>());
      m_objects.push_back(l);
    }
    else if (obj.getTag() == "Circle")
    {
      Circle* c = new Circle (pos, obj["Radius"].as<float>());
      m_objects.push_back(c);      
    }
    else if (obj.getTag() == "Triangle")
    {
      Triangle* t = new Triangle (pos, obj["Size"].as<float>());
      m_objects.push_back(t);      
    }
    else if (obj.getTag() == "Gaussian")
    {
      Gaussian* g = new Gaussian (pos, obj["Width"].as<float>(), obj["Height"].as<float>(), ci::Vec2f(obj["DirX"].as<float>(), obj["DirY"].as<float>()));
      m_objects.push_back(g);      
    }    
    else 
    {
      // Not recognized as supported object
      continue;
    }

    bool visibility = obj.getAttributeValue<bool>("Visible", true);    
    m_objects[m_objects.size() - 1]->setVisibility(visibility);
    
    float posVarX = obj.getAttributeValue<float>("PosVarX", 0.0);
    float posVarY = obj.getAttributeValue<float>("PosVarY", 0.0);
    m_objects[m_objects.size() - 1]->setPositionVariance(ci::Vec2f(posVarX, posVarY));
    
    float angVar = degreesToRadians(obj.getAttributeValue<float>("AngleVar", 0.0));
    m_objects[m_objects.size() - 1]->setAngleVariance(angVar);

  }
}

} // namespace