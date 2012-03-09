
#include "Scene.h"
#include "cinder/gl/gl.h"

namespace dmx
{

// Statics
// ---------------------------------------------------------------------------------------------------------------------
int Node::NUM_NODES = 0;
int Node::UNIQUE_IDS = 0;


// ---------------------------------------------------------------------------------------------------------------------
// Global render state
// ---------------------------------------------------------------------------------------------------------------------
int RenderState::g_polygonMode = GL_FLAT;
int RenderState::g_shadingMode = GL_SMOOTH;
int RenderState::g_lineMode = 1;
RenderState::RenderPass RenderState::g_renderPass = RENDER_SURFACE;

// ---------------------------------------------------------------------------------------------------------------------
void RenderState::startSurfaceMode()
{
  g_renderPass = RENDER_SURFACE;
  glPushAttrib(GL_LIGHTING_BIT);
  glEnable(GL_LIGHTING);
  glShadeModel(g_shadingMode);
}

// ---------------------------------------------------------------------------------------------------------------------
void RenderState::startOutlineMode()
{
  g_renderPass = RENDER_OUTLINES;
  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT | GL_POLYGON_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable(GL_LIGHTING);
  if(g_lineMode == GL_LINE_SMOOTH)
  {
    glEnable (GL_LINE_SMOOTH);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
  }
  glPolygonMode (GL_BACK, GL_LINE);	// Draw Backfacing Polygons As Wireframes
  glCullFace (GL_FRONT);				    // Don't Draw Any Front-Facing Polygons
  glDepthFunc (GL_LEQUAL);
  glEnable( GL_POLYGON_OFFSET_FILL );
  glPolygonOffset( -1, -1 );
}

// ---------------------------------------------------------------------------------------------------------------------
void RenderState::startWireframeMode()
{
  g_renderPass = RENDER_WIREFRAME;
  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT | GL_POLYGON_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable(GL_LIGHTING);
  if(g_lineMode == GL_LINE_SMOOTH) 
    glEnable (GL_LINE_SMOOTH);
  glPolygonMode (GL_FRONT, GL_LINE);	// Draw Frontfacing Polygons As Wireframes
  glDepthFunc (GL_LEQUAL);
  glEnable( GL_POLYGON_OFFSET_FILL );
  glPolygonOffset( -1, -1 );
}

// ---------------------------------------------------------------------------------------------------------------------
void RenderState::startShadowMode(const Vec4f& normal, const Vec4f& origin, const Vec4f& light)
{
  g_renderPass = RENDER_SHADOWS;
  glPushAttrib(GL_LIGHTING_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_BLEND);
  glDisable(GL_LIGHTING);
  glEnable( GL_POLYGON_OFFSET_FILL );
  glPolygonOffset( 0.5, 0.5 );

  glPushMatrix();
  setShadowTransform(normal, origin, light);
}


// ---------------------------------------------------------------------------------------------------------------------
// ofxNode
// ---------------------------------------------------------------------------------------------------------------------
Node::Node() :
  m_pTM(&m_TM),
  m_type(NODE_INVALID),
  m_uniqueID(UNIQUE_IDS++),
  m_isDriven(false)
{
  m_TM.setToIdentity();
  NUM_NODES++;
}

// ---------------------------------------------------------------------------------------------------------------------
void Node::print()
{ 
#ifdef _DEBUG
  printf("Node %i: %s | N: %i U: %i \n", m_uniqueID, NodeTypeNames[m_type], NUM_NODES, UNIQUE_IDS); 
#endif
}

// ---------------------------------------------------------------------------------------------------------------------
// ofxNodeGroup
// ---------------------------------------------------------------------------------------------------------------------
NodeGroup::NodeGroup() 
{ 
  init(); 
};

//----------------------------------------------------------------------------------------------------------------------
void NodeGroup::init()
{
  m_type = NODE_GROUP;
  m_isRightAligned = false;
#ifdef _DEBUG
  print();
#endif
}

//----------------------------------------------------------------------------------------------------------------------
void NodeGroup::update()
{
  // Make right-aligned if so desired
  cinder::Matrix44f m = *m_pTM;
  if(m_isRightAligned)
  {
    ci::Vec4f pos = m.getTranslate();      
    float x = ci::app::getWindowSize().x - pos.x;
    pos.x = x;
    m.setTranslate(pos);
  }
  
  glPushMatrix();
  glMultMatrixf(m);
  for(size_t i = 0; i < m_children.size(); i++)
  {
    m_children[i]->update();
  }
  glPopMatrix();
}

//----------------------------------------------------------------------------------------------------------------------
ci::Vec4f NodeGroup::toLocalPos(const ci::Vec4f pos)
{
  // Transform position into local space
  Vec4f currentPos = m_pTM->getTranslate(); 
  if(m_isRightAligned)
  {
    currentPos.x = ci::app::getWindowSize().x - currentPos.x;
  }
  
  return pos - currentPos;
}
  
//----------------------------------------------------------------------------------------------------------------------
  void NodeGroup::onMouseMove(const cinder::Vec4f& mPos)
{
  Vec4f mPosLocal = toLocalPos(mPos);
  for(size_t i = 0; i < m_children.size(); i++)
  {
    m_children[i]->onMouseMove(mPosLocal);
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
void NodeGroup::onMouseDrag(ci::app::MouseEvent e)
{
  for(size_t i = 0; i < m_children.size(); i++)
  {
    m_children[i]->onMouseDrag(e);
  }
}  

//----------------------------------------------------------------------------------------------------------------------
void NodeGroup::onMousePress(const cinder::Vec4f& mPos)
{
  Vec4f mPosLocal = toLocalPos(mPos);
  for(size_t i = 0; i < m_children.size(); i++)
  {
    m_children[i]->onMousePress(mPosLocal);
  }
}    
  
//----------------------------------------------------------------------------------------------------------------------
void NodeGroup::onKeyPress(ci::app::KeyEvent e)
{
  for(size_t i = 0; i < m_children.size(); i++)
  {
    m_children[i]->onKeyPress(e);
  }  
}
  
//----------------------------------------------------------------------------------------------------------------------
void NodeGroup::onResize(ci::app::ResizeEvent e)
{
  for(size_t i = 0; i < m_children.size(); i++)
  {
    m_children[i]->onResize(e);
  }  
}  

//----------------------------------------------------------------------------------------------------------------------
Node* NodeGroup::getNode(int pickID)
{
  Node* pickedNode = 0;
  for(size_t i = 0; i < m_children.size(); i++)
  {
    pickedNode = m_children[i]->getNode(pickID);
    if(pickedNode != 0)
    {
      //pickedNode = this; // test for highlighting all children at the same time
      break;
    }
  }
  return pickedNode;
}

//----------------------------------------------------------------------------------------------------------------------
// ofxNodeGeometry
//----------------------------------------------------------------------------------------------------------------------
NodeGeometry::NodeGeometry() :
  m_color(1,1,1,1),
  m_outlineColor(0,0,0,1),
  m_pickColor(1,0.4,0,1),
  m_outlineWidth(1.0),
  m_picked(false),
  m_hasShadow(false),
  m_isSelectable(true),
  m_dl(-1)
{
}

//----------------------------------------------------------------------------------------------------------------------
void NodeGeometry::update()
{
  assert(m_dl > -1);

  if((RenderState::g_renderPass == RenderState::RENDER_SHADOWS)  && !m_hasShadow)
  {
    return;
  }

  // set Color
  glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_LINE_BIT);
  if (RenderState::g_renderPass == RenderState::RENDER_SURFACE)
  {
    glColor4f(m_color.x, m_color.y, m_color.z, m_color.w);
  }
  else if (RenderState::g_renderPass == RenderState::RENDER_OUTLINES)
  {
    if(m_picked)
    {
      glColor4f(m_pickColor.x, m_pickColor.y, m_pickColor.z, m_pickColor.w);
      glLineWidth(4 * m_outlineWidth);
    }
    else
    {
      glColor4f(m_outlineColor.x, m_outlineColor.y, m_outlineColor.z, m_outlineColor.w);
      glLineWidth(m_outlineWidth);
    }
  }
  else if(RenderState::g_renderPass == RenderState::RENDER_SHADOWS)
  {
    glColor4f(0,0,0,1);
  }
  else if (RenderState::g_renderPass == RenderState::RENDER_WIREFRAME)
  {
    glColor4f(m_outlineColor.x, m_outlineColor.y, m_outlineColor.z, m_outlineColor.w);
    glLineWidth(m_outlineWidth);
  }

  // draw geometry
  glPushName(m_uniqueID); // name for picking
  glPushMatrix();
    glMultMatrixf(*m_pTM);
    glCallList(m_dl);
  glPopMatrix();
  glPopAttrib();
  glPopName();
}

//----------------------------------------------------------------------------------------------------------------------
Node* NodeGeometry::getNode(int pickID)
{ 
  if((m_uniqueID == pickID) && m_isSelectable) 
    return this; 
  else 
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------
// Axes
//----------------------------------------------------------------------------------------------------------------------
void Axes::createGeometry()
{
  m_dl = glGenLists(1);
  glNewList(m_dl, GL_COMPILE);
    drawBasis(m_size);
  glEndList();
}

void Axes::init()
{
  m_type = NODE_AXES;
#ifdef _DEBUG
  print();
#endif
};

//----------------------------------------------------------------------------------------------------------------------
// A box
//----------------------------------------------------------------------------------------------------------------------
void Box::createGeometry()
{
  m_dl = glGenLists(1);
  glNewList(m_dl, GL_COMPILE);
    drawBox(m_lx, m_ly, m_lz);
  glEndList();
}


void Box::init()
{
  m_type = NODE_BOX;
#ifdef _DEBUG
  print();
#endif
}

//----------------------------------------------------------------------------------------------------------------------
// A sphere
//----------------------------------------------------------------------------------------------------------------------
void Sphere::createGeometry()
{
  m_dl = glGenLists(1);
  glNewList(m_dl,GL_COMPILE);
    glutSolidSphere(m_radius, m_resolution, m_resolution);
  glEndList();
}

void Sphere::init()
{
  m_type = NODE_SPHERE;
#ifdef _DEBUG
  print();
#endif
}

//----------------------------------------------------------------------------------------------------------------------
// A disk
//----------------------------------------------------------------------------------------------------------------------
void Disk::createGeometry()
{
  m_dl = glGenLists(1);
  glNewList(m_dl,GL_COMPILE);
  if(m_sweepAngle > 0)
  {
    drawPartialDisk(m_radius2, m_radius1, m_resolution, 2, m_startAngle, m_sweepAngle);
  }
  else 
  {
    drawDisk(m_radius1, m_radius2, m_resolution, 2);  
  }

  glEndList();
}

void Disk::init()
{
  m_type = NODE_DISK;
#ifdef _DEBUG
  print();
#endif
}  
  
//----------------------------------------------------------------------------------------------------------------------
// A cylinder with flat ends
//----------------------------------------------------------------------------------------------------------------------
void Cylinder::createGeometry()
{
  m_dl = glGenLists(1);
  glNewList(m_dl,GL_COMPILE);
    drawCylinder(m_length, m_radius1, m_radius2, m_slices, m_stacks, GLU_FILL);
  glEndList();
}

void Cylinder::init()
{
  m_type = NODE_CYLINDER;
#ifdef _DEBUG
  print();
#endif
}

//----------------------------------------------------------------------------------------------------------------------
// A cylinder half-spheres at ends
//----------------------------------------------------------------------------------------------------------------------
void Capsule::createGeometry()
{
  m_dl = glGenLists(1);
  glNewList(m_dl,GL_COMPILE);
    drawCapsule(m_length, m_radius, m_resolution, m_resolution, GLU_FILL);
  glEndList();
}

//----------------------------------------------------------------------------------------------------------------------
void Capsule::init()
{
  m_type = NODE_CAPSULE;
#ifdef _DEBUG
  print();
#endif
}

//----------------------------------------------------------------------------------------------------------------------
// A grid
//----------------------------------------------------------------------------------------------------------------------
void Grid::createGeometry()
{
  float x = m_xl / 2;
  float y = m_yl / 2;
  GLfloat grid[2][2][3] = { {{-x, 0.0f, -y}, {x, 0.0f, -y}}, {{-x, 0.0f, y}, {x, 0.0f, y}} };
  m_dl = glGenLists(1);
  glNewList(m_dl,GL_COMPILE);
    drawEvalGrid(&grid[0][0][0], m_Nx, m_Ny);
  glEndList();
}

void Grid::init()
{
  m_type = NODE_GRID;
#ifdef _DEBUG
  print();
#endif
}


//----------------------------------------------------------------------------------------------------------------------
// vertex array/buffer
//----------------------------------------------------------------------------------------------------------------------
RealMatrixViz::RealMatrixViz(double **d, int N, int M, float width, double maxVal) : 
  m_data(d), m_N(N), m_M(M), m_maxVal(maxVal), m_iSel(-1), m_jSel(-1)
{
  init();
  
  // choose the larger side as the one that's specified 
  m_scale = N > M ? width / N : width / M;
  
  // stack space for vertex data
  const int numVerts = N * M * 4;
  const int numVertCoords = numVerts * 2;
  const int numColorCoords = numVerts * 3;   
  GLfloat* verts = new GLfloat[numVerts*2];
  GLfloat* cols = new GLfloat[numVerts*3];
  
  float s = m_scale;
  
  // create vertices
  for(int i = 0; i < N; i++)
  {
    for(int j = 0; j < M; j++)
    {
      int sId = i*M+j;
      int vId = sId*8; // (2x4 coords)

      verts[vId + 0] = i*s; 
      verts[vId + 1] = j*s; 
      verts[vId + 2] = i*s; 
      verts[vId + 3] = j*s + s; 
      verts[vId + 4] = i*s + s; 
      verts[vId + 5] = j*s + s; 
      verts[vId + 6] = i*s + s; 
      verts[vId + 7] = j*s; 
      
      int cId = sId*12; // (3x4 coords)
      double color = m_data[i][j] / m_maxVal;// (float)sId / (float)(N*M);
      for(int c = 0; c < 12; c++)
      {
        cols[cId + c] = color;
      }
    }
  }
  
  // create vertex and color buffers
  glGenBuffersARB(2, &m_vbo[0]);
  
  // copy vertex data
  glBindBufferARB(GL_ARRAY_BUFFER_ARB, m_vbo[0]);
  //glBufferDataARB(GL_ARRAY_BUFFER_ARB, sizeof(verts), verts, GL_STATIC_DRAW_ARB);
  glBufferDataARB(GL_ARRAY_BUFFER_ARB, numVertCoords * sizeof(GLfloat), verts, GL_STATIC_DRAW_ARB);  
  
  
  // copy color data
  glBindBufferARB(GL_ARRAY_BUFFER_ARB, m_vbo[1]);
  //glBufferDataARB(GL_ARRAY_BUFFER_ARB, sizeof(cols), cols, GL_STATIC_DRAW_ARB);
  glBufferDataARB(GL_ARRAY_BUFFER_ARB, numColorCoords * sizeof(GLfloat), cols, GL_STATIC_DRAW_ARB);

}

//----------------------------------------------------------------------------------------------------------------------
void RealMatrixViz::update()
{
  glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_LIGHTING_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);

  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // draw vertex array
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY); 

  glBindBufferARB(GL_ARRAY_BUFFER_ARB, m_vbo[0]);
  glVertexPointer(2, GL_FLOAT, 0, 0);  
  
  glBindBufferARB(GL_ARRAY_BUFFER_ARB, m_vbo[1]);
  glColorPointer(3, GL_FLOAT, 0, 0);

  glDrawArrays(GL_QUADS, 0, m_N * m_M * 4); 
  
  glDisableClientState(GL_COLOR_ARRAY);      
  glDisableClientState(GL_VERTEX_ARRAY); 
  
  // reset
  glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
  
  glPopMatrix();
  glPopAttrib();
}

//----------------------------------------------------------------------------------------------------------------------
// A 3d model in .obj format loaded from file
//----------------------------------------------------------------------------------------------------------------------
//void ofx3dModel::createGeometry()
//{
//  m_dl = glGenLists(1);
//  glNewList(m_dl,GL_COMPILE);
//    m_obj.draw();
//  glEndList();
//}

//----------------------------------------------------------------------------------------------------------------------
//void ofx3dModel::init()
//{
//  m_type = NODE_3DMODEL;
//#ifdef _DEBUG
//  print();
//#endif
//}


//----------------------------------------------------------------------------------------------------------------------
// A graph/plot
//----------------------------------------------------------------------------------------------------------------------
Plot::Plot(float w, float h, int nr, int N) :
  m_points(nr, std::vector<float>()), 
  m_names(nr, std::string()), 
  m_nr(nr), m_N(N), 
  m_w(w), m_h(h), m_maxY(-1), m_minY(1)
{ 

  init(); 
}

//----------------------------------------------------------------------------------------------------------------------
void Plot::update()
{
  glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_LIGHTING_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);

  glPushMatrix();
  glMultMatrixf(*m_pTM);
  
  // outputs label    
  const float labelHeight = 16;
  ci::ColorA labelColor (0.0f, 0.0f, 0.0f, 0.5f);
  ci::gl::color(labelColor);
  ci::gl::drawSolidRect(ci::Rectf(0, 0, m_w, labelHeight));         
  ci::gl::drawString("Graph: " + m_title, ci::Vec2f(3,3), ci::Color(1,1,1), m_font);    
  
  glPushMatrix();
  glTranslatef(0, labelHeight,0);
  
  // Now draw graphs
  float widthRec = 1.0 / (float)m_N * m_w;
  float scale = m_maxY - m_minY;
  float scaleRec = scale > 0 ? 1.0 / scale : 0.0;
  for(int pl = 0; pl < m_nr; pl++)
  {
    const int numPoints =  m_points[pl].size();
    if(numPoints > 1)
    {
      // line plot
      Vec3f col = getColorMapRainbow((float)pl / m_nr);
      glColor4f(col.x, col.y, col.z, 1.0);
      float lineVerts[numPoints*2];
      glEnableClientState( GL_VERTEX_ARRAY );
      glVertexPointer( 2, GL_FLOAT, 0, lineVerts );      
      for(size_t i = 0; i < numPoints; i++)
      {
        lineVerts[i*2 + 0] = i * widthRec;
        lineVerts[i*2 + 1] = m_h - m_h * ((m_points[pl][i]   - m_minY) * scaleRec);
      }
      glDrawArrays( GL_LINE_STRIP, 0, numPoints);
      glDisableClientState( GL_VERTEX_ARRAY);       
      
#if DEBUGGING      
      // show current value
      float current = m_points[pl][numPoints - 1];
      char str [64]; 
      if(m_names[pl] != "")
        sprintf(str, "%s: %2.4f", m_names[pl].c_str(), current);
      else
        sprintf(str, "%i: %2.4f", pl, current);        
      
      const int h = 8;
      const bool legendInside = true;
      const float xoffset = legendInside ? 0 : m_w;
      glPushMatrix();
      glTranslatef(xoffset + 2*h, pl*1.5*h + h, 0.0);
      glColor4f(col.x, col.y, col.z, 1.0);      
      drawRectangle(h, h);
      glPopMatrix();
      ci::gl::drawString(str, ci::Vec2f(xoffset + 3*h, pl*1.5*h /* + h*/ + 3), ci::Color(col.x, col.y, col.z), m_font);    
#endif
    }
  }
  
  // box
  glColor4f(0.0, 0.0, 0.0, 1);
  ci::gl::drawLine(Vec3f(0.5, m_h+0.5, 0), Vec3f(m_w+0.5, m_h+0.5, 0)); // xAxis
  ci::gl::drawLine(Vec3f(0.5, m_h, 0), Vec3f(0.5, 0, 0));               // yAxis on left
  ci::gl::drawLine(Vec3f(m_w - 0.5, m_h, 0), Vec3f(m_w - 0.5, 0, 0));   // yAxis on right

  // Show min and max on axes
#if 0
  char str [32]; 
  sprintf(str, "%+2.3f", m_minY);
  drawString(Vec3f(0.5-40, m_h + 2.5, 0), str);  
  sprintf(str, "%+2.3f", m_maxY);
  drawString(Vec3f(0.5-40, 2.5, 0), str);    
#endif
  
  glPopMatrix();
  
  glPopMatrix();
  glPopAttrib();
  glPopName();
}

//----------------------------------------------------------------------------------------------------------------------
void Plot::addPoint(float p, int pID)
{
  assert(pID < m_nr);

  m_points[pID].push_back(p);
  
  if(p > m_maxY)
    m_maxY = p;
  if(p < m_minY)
    m_minY = p;
    
  if(m_points[pID].size() > m_N)
    m_points[pID].erase(m_points[pID].begin());    
}

//----------------------------------------------------------------------------------------------------------------------  
void Plot::setLabel(int pId, const std::string& name)
{
  assert(pId >= 0 && pId < m_names.size());
  m_names[pId] = name;
}

//----------------------------------------------------------------------------------------------------------------------
void Plot::init()
{
  m_type = NODE_PLOT;
  m_font = ci::Font(ci::app::loadResource("pf_tempesta_seven.ttf"), 8);  
#ifdef _DEBUG
  print();
#endif
}

//----------------------------------------------------------------------------------------------------------------------
// A matrix viz node
//----------------------------------------------------------------------------------------------------------------------





}
