
#include "Scene.h"

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
void RenderState::startShadowMode(const Vec3f& normal, const Vec3f& origin, const Vec3f& light)
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
  printf("Node %i: %s | N: %i U: %i \n", m_uniqueID, NodeTypeNames[m_type], NUM_NODES, UNIQUE_IDS); 
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
#ifdef _DEBUG
  print();
#endif
}

//----------------------------------------------------------------------------------------------------------------------
void NodeGroup::update() const
{
  glPushMatrix();
  glMultMatrixf(m_pTM->m);
  for(int i = 0; i < m_children.size(); i++)
  {
    m_children[i]->update();
  }
  glPopMatrix();
}

//----------------------------------------------------------------------------------------------------------------------
void NodeGroup::onMouseMove(const Vec3f& mPos)
{
  // update mouse info: transform mouse position into local space
  Vec3f mPosLocal = m_pTM->getTranslation();
  mPosLocal = mPos - mPosLocal;
  for(int i = 0; i < m_children.size(); i++)
  {
    m_children[i]->onMouseMove(mPosLocal);
  }
}

//----------------------------------------------------------------------------------------------------------------------
Node* NodeGroup::getNode(int pickID)
{
  Node* pickedNode = 0;
  for(int i = 0; i < m_children.size(); i++)
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
  m_hasShadow(true),
  m_isSelectable(true),
  m_dl(-1)
{
}

//----------------------------------------------------------------------------------------------------------------------
void NodeGeometry::update() const
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
      glLineWidth(4*m_outlineWidth);
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
    glMultMatrixf(m_pTM->m);
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
// A cylinder with flat ends
//----------------------------------------------------------------------------------------------------------------------
void Cylinder::createGeometry()
{
  m_dl = glGenLists(1);
  glNewList(m_dl,GL_COMPILE);
    drawCylinder(m_length, m_radius1, m_radius2, m_resolution, m_resolution, GLU_FILL);
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
  float x = m_xl/2;
  float y = m_yl/2;
  GLfloat grid[2][2][3] = { {{-x, -y, 0.0}, {x, -y, 0.0}}, {{-x, y, 0.0}, {x, y, 0.0}} };
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
  m_nr(nr), m_N(N), m_points(nr, std::vector<float>()), m_w(w), m_h(h), m_maxY(1), m_minY(0)
{ 
  init(); 
}

//----------------------------------------------------------------------------------------------------------------------
void Plot::update() const
{
  glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_LIGHTING_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);

  glPushMatrix();
  glMultMatrixf(m_pTM->m);

  float widthRec = 1.0 / (float)m_N * m_w;
  float scale = m_maxY - m_minY;
  float scaleRec = scale > 0 ? 1.0 / scale : 0.0;
  for(int pl = 0; pl < m_nr; pl++)
  {
    if(m_points[pl].size() > 1)
    {
      // line plot
      Vec3f col = getColorMapRainbow((float)pl / m_nr);
      glColor4f(col.x, col.y, col.z, 1.0);
      glBegin(GL_LINE_STRIP);
      for(int i = 0; i < m_points[pl].size() - 1; i++)
      {
        float x1 = (i + 1) * widthRec;
        float x2 = (i + 2) * widthRec;
        float y1 = m_h * ((m_points[pl][i]   - m_minY) * scaleRec);
        float y2 = m_h * ((m_points[pl][i+1] - m_minY) * scaleRec);
        glVertex3f(x1, y1, 0);
        glVertex3f(x2, y2, 0);
#ifdef _DEBUG
  //printf("Plot: %f %f\n", x1, y1);
#endif
      }
      glEnd();
    }
  }
  
  // box
  glColor4f(0.0, 0.0, 0.0, 1);
//  drawLine(Vec3f(0, 0, 0), Vec3f(m_w, 0, 0)); // xAxis
//  drawLine(Vec3f(0, 0, 0), Vec3f(0, m_h, 0)); // yAxis
//  drawLine(Vec3f(0, m_h, 0), Vec3f(m_h/20, m_h, 0)); // end
//  drawLine(Vec3f(m_w, 0, 0), Vec3f(m_w, m_h/20, 0)); // end

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
void Plot::init()
{
  m_type = NODE_PLOT;
#ifdef _DEBUG
  print();
#endif
}

//----------------------------------------------------------------------------------------------------------------------
// A matrix viz node
//----------------------------------------------------------------------------------------------------------------------





}
