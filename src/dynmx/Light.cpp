/*
 *  Light.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Light.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
Light::Light(int id) : m_id(id), m_enabled(true), m_lightType(LIGHT_POINT) 
{ 
  init(); 
}

//----------------------------------------------------------------------------------------------------------------------
void Light::init()
{
  m_type = NODE_LIGHT;
  m_ambient = Vec4f(0.3, 0.3, 0.3, 1.0);
  m_ambientGlobal = Vec4f(0.4, 0.4, 0.4, 1.0);
  m_diffuse = Vec4f(1,1,1,1);
  m_specular = Vec4f(1,1,1,1);

  m_spotCutoff = 180.0f;
  m_spotExponent = 0.0f;
  m_attenuationConst = 1;
  m_attenuationLinear = 0;
  m_attenuationQUad = 0;
#ifdef _DEBUG
  print();
#endif
}    

//----------------------------------------------------------------------------------------------------------------------
void Light::update()
{
  if (m_enabled)
  {
    GLenum light = GL_LIGHT0 + m_id;

    glEnable(light);

    glLightfv( light, GL_AMBIENT, &m_ambient.x);
    glLightfv( light, GL_DIFFUSE, &m_diffuse.x);
    glLightfv( light, GL_SPECULAR, &m_specular.x);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, &m_ambientGlobal.x);

    Vec4f position = Vec4f(m_TM.getTranslation());
    Vec4f direction = m_TM.getRow(0);

    switch (m_lightType)
    {
    case LIGHT_POINT:
      position[3] = 1.0;
      glLightfv(light, GL_POSITION, position );
      glLightf(light, GL_SPOT_CUTOFF, 180.0f);
      break;
    case LIGHT_DIRECTIONAL:
      position[3] = 0.0;
      glLightfv(light, GL_POSITION, position );
      glLightf(light, GL_SPOT_CUTOFF, 180.0f);
      break;
    case LIGHT_SPOT:
      position[3] = 1.0;
      glLightfv(light, GL_POSITION, position );
      glLightf(light, GL_SPOT_CUTOFF, m_spotCutoff);
      glLightfv(light, GL_SPOT_DIRECTION, &direction.x);
      glLightf(light, GL_SPOT_EXPONENT, m_spotExponent);
      break;
    default:
      break;
    }

    glLighti( light, GL_CONSTANT_ATTENUATION, m_attenuationConst );
    glLighti( light, GL_LINEAR_ATTENUATION, m_attenuationLinear );
    glLighti( light, GL_QUADRATIC_ATTENUATION, m_attenuationQUad );

  }
}

} // namespace dmx