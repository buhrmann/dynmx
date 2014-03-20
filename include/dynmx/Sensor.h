//
//  Sensor.h
//  dynmx
//
//  Created by Thomas Buhrmann on 07/03/14.
//
//

#ifndef dynmx_Sensor_h
#define dynmx_Sensor_h

#include "Dynmx.h"
#include "Signal.h"
#include "SMCEnvironment.h"
#include "cinder/Vector.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// A 2d distance sensor that performs variois collision detection routines
// ---------------------------------------------------------------------------------------------------------------------
class Sensor
{
  
public:
  
  // Define a type of pointer to transfer function
  typedef double (*TransferFunction)(double);
  
  // Name the activation functions
  enum TransferFunctions
  {
    kTF_Identity,
    kTF_Inverse,
    kTF_Binary,
    kTF_NumFunctions
  };
  
  enum SensorMode
  {
    kSM_Identity = 0,
    kSM_Derivative = 1,
    kSM_Delayed = 2,
    kSM_NumModes
  };
  
  static std::string s_transFuncNames [kTF_NumFunctions];
  static std::string s_modeNames [kSM_NumModes];
  
  Sensor() : m_position(0.0f, 0.0f) { init(); };
  Sensor(const ci::Vec2f& position) : m_position(position) { init(); };
  virtual ~Sensor() {};
  
  virtual void init();
  virtual void reset();
  virtual float sense(SMCEnvironment& environment, float dt);
  
  virtual void toXml(ci::XmlTree& xml);
  virtual void fromXml(const ci::XmlTree& xml);
  
  void setNoiseLevel(float level) { m_noiseLevel = level; };
  void setPosition(const ci::Vec2f& pos) { m_position = pos; };
  void setTransferFunction(double (*pt2Func)(double)) { m_transferFunction = pt2Func; };
  void setTransferFunction(int actFuncName);
  void setTransferFunction(const std::string& name);
  
  float getLevel() const;
  float getActivation() const { return m_activation; };
  float getDerivative() const { return m_activationDt.get(); };
  float getDelayed() const { return m_activationDelayed.get(); };
  const ci::Vec2f& getPosition() const { return m_position; };
  
  const std::string& getTransferFunctionName() const;
  const std::string& getModeName() const;
  
protected:
  
  virtual void updateActivation(SMCEnvironment& environment, float dt) = 0;
  
  static inline double tfIdentity(double v) { return v; };
  static inline double tfInverse(double v) { return 1.0 - v; };
  static inline double tfBinary(double v) { return  v > 0; };
  
  // State
  ci::Vec2f m_position;
  float m_activation;
  Derivative m_activationDt;
  Delay m_activationDelayed;
  
  // Parameters
  float m_noiseLevel;
  int m_mode;
  TransferFunction m_transferFunction;
  
}; // class
  
} // namespace

#endif
