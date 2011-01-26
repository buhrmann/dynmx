/*
 *  Light.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Scene.h"

namespace dmx
{

// Inherits a TM from ofxNode, therefore has translation already,
// and direction is stored in x-Axis of TM...
class Light : public Node
{

public:

	enum LightType
	{
		LIGHT_POINT,
		LIGHT_DIRECTIONAL,
		LIGHT_SPOT
	};

	Light(int id = 0);
	virtual ~Light(){};


  virtual void update() const;
	void toggle() { m_enabled = !m_enabled; };  

	int m_id;

	Vec4f m_ambient;
	Vec4f m_ambientGlobal;
	Vec4f m_diffuse;
	Vec4f m_specular;
	GLint m_attenuationConst;
	GLint m_attenuationLinear;
	GLint m_attenuationQUad;

	float m_spotCutoff;
	float m_spotExponent;

	LightType m_lightType;
	bool m_enabled;

protected:
  virtual void init();
  
};

} // namespace dmx
