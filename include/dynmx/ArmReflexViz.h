/*
 *  ArmReflexViz.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 10/14/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _DMX_ARM_REFLEX_VIZ_
#define _DMX_ARM_REFLEX_VIZ_

#include "Scene.h"
#include "ArmMuscledViz.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// 3d visualisation of the Arm model
//----------------------------------------------------------------------------------------------------------------------
class ArmReflexViz : public ArmMuscledViz
{
  
public:
  
  ArmReflexViz(ArmReflex* model) : ArmMuscledViz((ArmMuscled*)model), m_armRx(model) { ArmMuscledViz::init(); };
  
  void setModel(ArmReflex* model) { m_arm = model; };
  virtual void update();
  
protected:
  
  ArmReflex* m_armRx;
  
};
  
//----------------------------------------------------------------------------------------------------------------------
// Inline implementation
//----------------------------------------------------------------------------------------------------------------------
inline void ArmReflexViz::update()
{
  ArmMuscledViz::update();
  
  if(m_drawOverlays)
  {
    // Overlay desired state
    glDisable(GL_DEPTH_TEST);
    glPushMatrix();
    glMultMatrixf(*m_pTM);
    
    // Draw desired trajectory
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);  
    const std::deque<Pos>& desTrajectory = m_armRx->getDesiredTrajectory();  
    int numPoints =  desTrajectory.size();    
    float lineVertsDes[numPoints*2];
    float colorsDes[numPoints*4];
    glVertexPointer(2, GL_FLOAT, 0, lineVertsDes); // 2d positions
    glColorPointer(4, GL_FLOAT, 0, colorsDes);     // 4d colors  
    for(size_t i = 0; i < numPoints; i++)
    {
      lineVertsDes[i*2 + 0] = desTrajectory[i].x;
      lineVertsDes[i*2 + 1] = desTrajectory[i].y;
      float c = 0.5f * (float)i / (float)numPoints;
      colorsDes[i*4 + 0] = 0.5;
      colorsDes[i*4 + 1] = 0.5;
      colorsDes[i*4 + 2] = 0.5;
      colorsDes[i*4 + 3] = c;
    }
    glDrawArrays( GL_LINE_STRIP, 0, numPoints);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    
    // Restore gl state
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);     
  }
}
  
} // namespace dmx

#endif