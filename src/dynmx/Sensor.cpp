//
//  Sensor.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 07/03/14.
//
//

#include "Sensor.h"
#include "Random.h"

namespace dmx
{

// Statics
//----------------------------------------------------------------------------------------------------------------------
std::string Sensor::s_transFuncNames [kTF_NumFunctions] = {"Identity", "Inverse", "Binary"};
std::string Sensor::s_modeNames [kSM_NumModes] = {"Identity", "Derivative", "Delayed"};

//----------------------------------------------------------------------------------------------------------------------
void Sensor::init()
{
  m_noiseLevel = 0.0f;
  m_mode = kSM_Identity;
  setTransferFunction(kTF_Inverse);
  
  m_activationDelayed = Delay(0.06, 0.01, m_activation);
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void Sensor::reset()
{
  m_activation = 0.0f;
  m_activationDt.reset();
  m_activationDelayed.reset();
}

//----------------------------------------------------------------------------------------------------------------------
float Sensor::sense(SMCEnvironment& environment, float dt)
{
  updateActivation(environment, dt);
  
  m_activation = m_activation + UniformRandom(-m_noiseLevel, m_noiseLevel);
  
  m_activationDt.update(m_activation, dt);
  m_activationDelayed.update(m_activation);
  
  return getLevel();
}

//----------------------------------------------------------------------------------------------------------------------
float Sensor::getLevel() const
{
  if (m_mode == kSM_Identity)
    return m_activation;
  else if (m_mode == kSM_Derivative)
    return m_activationDt.get();
  else
    return m_activationDelayed.get();
}
  
//----------------------------------------------------------------------------------------------------------------------
void Sensor::setTransferFunction(int actFuncName)
{
  switch (actFuncName)
  {
    case kTF_Binary:
      m_transferFunction = &tfBinary;
      break;
    case kTF_Identity:
      m_transferFunction = &tfIdentity;
      break;
    case kTF_Inverse:
    default:
      m_transferFunction = &tfInverse;
      break;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void Sensor::setTransferFunction(const std::string& name)
{
  for(int i = 0; i < kTF_NumFunctions; ++i)
  {
    if(name == s_transFuncNames[i])
      setTransferFunction(i);
  }
}

//----------------------------------------------------------------------------------------------------------------------
const std::string& Sensor::getTransferFunctionName() const
{
  if(m_transferFunction == &tfBinary)
    return s_transFuncNames[kTF_Binary];
  else if(m_transferFunction == &tfIdentity)
    return s_transFuncNames[kTF_Identity];
  else
    return s_transFuncNames[kTF_Inverse];
}
  
//----------------------------------------------------------------------------------------------------------------------
const std::string& Sensor::getModeName() const
{
  return s_modeNames[m_mode];
}
  
//----------------------------------------------------------------------------------------------------------------------
void Sensor::fromXml(const ci::XmlTree& xml)
{
  m_noiseLevel = xml.getAttributeValue<float>("NoiseLevel", 0.0);
  
  std::string modeStr = xml.getAttributeValue<std::string>("Mode", "Identity");
  if (modeStr == "Identity")
    m_mode = kSM_Identity;
  else if (modeStr == "Derivative")
    m_mode = kSM_Derivative;
  else if (modeStr == "Delayed")
    m_mode = kSM_Delayed;
  
  setTransferFunction(xml.getAttributeValue<std::string>("TransferFunc", "Binary"));
}
  
//----------------------------------------------------------------------------------------------------------------------
void Sensor::toXml(ci::XmlTree& xml)
{
  xml.setAttribute("NoiseLevel", m_noiseLevel);
  xml.setAttribute("TransferFunc", getTransferFunctionName());
  xml.setAttribute("Mode", getModeName());
}
  

} // namespace