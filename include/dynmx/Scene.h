#ifndef _OFXNODE_
#define _OFXNODE_

#include <vector>
#include "Dynmx.h" 

//#include "ofx3DModelLoader.h"
#include "cinder/Matrix.h"
#include "cinder/Path2d.h"
#include "cinder/app/App.h"

#include "Render.h"

namespace dmx 
{

enum NodeType
{
  NODE_INVALID,
  NODE_GROUP,
  NODE_AXES,
  NODE_BOX,
  NODE_SPHERE,
  NODE_CYLINDER,
  NODE_CAPSULE,
  NODE_GRID,
  NODE_3DMODEL,
  NODE_PLOT,
  NODE_CUSTOM,
  NODE_VALUEMATRIX,
  NODE_VALUEVECTOR,
  NODE_LIGHT,
  NUM_NODETYPES
};

#ifdef _DEBUG
static const char* NodeTypeNames [NUM_NODETYPES] =
{
  "Invalid!",
  "Group",
  "Box",
  "Sphere",
  "Cylinder",
  "Capsule",
  "Grid",
  "3D-Model",
  "Plot",
  "Custom",
  "Value-Matrix",
  "Value-Vector",
  "Light",
};
#endif

// Global render state
// ---------------------------------------------------------------------------------------------------------------------
class RenderState
{
public :

  enum RenderPass
  {
    RENDER_INVALID = -1,
    RENDER_SURFACE,
    RENDER_OUTLINES,
    RENDER_WIREFRAME,
    RENDER_SHADOWS,
    RENDER_PICKING,
    NUM_RENDER_PASSES
  };

  static int g_polygonMode;
  static int g_shadingMode;
  static int g_lineMode;
  static RenderPass g_renderPass;

  // switching between render modes
  static void startSurfaceMode();
  static void stopSurfaceMode() { glPopAttrib(); };
  static void startOutlineMode();
  static void stopOutlineMode() { glPopAttrib(); };
  static void startWireframeMode();
  static void stopWireframeMode() { glPopAttrib(); };
  static void startShadowMode(const cinder::Vec4f& normal, const cinder::Vec4f& origin, const cinder::Vec4f& light);
  static void stopShadowMode() { glPopMatrix(); glPopAttrib(); };

}; // class RenderState

// ---------------------------------------------------------------------------------------------------------------------
// Base class for all nodes in scene-graph
// ---------------------------------------------------------------------------------------------------------------------
class Node
{
public:

  static int NUM_NODES;
  static int UNIQUE_IDS;

  virtual ~Node() { NUM_NODES--; };
  virtual void update() = 0;

  // only effective when not externally driven !
  virtual void translate(const cinder::Vec3f& p) { m_TM.translate(p); };

  void attachDriver(cinder::Matrix44f* m) { m_pTM = m; m_isDriven = true; };
  void detachDriver() { m_pTM = &m_TM; m_isDriven = false; };

  virtual Node* getNode(int pickID) { return 0; };
  
  virtual void onMouseMove(const cinder::Vec3f& mousePos) {};
  virtual void onKeyPress(cinder::app::KeyEvent e) {};

  virtual void print();

  cinder::Matrix44f m_TM;
  cinder::Matrix44f* m_pTM;

  NodeType m_type;
  int m_uniqueID;
  bool m_isDriven;

protected:

  Node();

};

//----------------------------------------------------------------------------------------------------------------------
// Node for grouping children
//----------------------------------------------------------------------------------------------------------------------
class NodeGroup : public Node
{
public:

  NodeGroup();
  virtual void update();

  // recursively descend into tree to look for picked node
  virtual Node* getNode(int pickID);
  virtual void onMouseMove(const cinder::Vec3f& mousePos);
  virtual void onKeyPress(ci::app::KeyEvent e);
  
  std::vector<Node*> m_children;

protected:
  virtual void init();
};

//----------------------------------------------------------------------------------------------------------------------
// Base class for all actual geometries in the scene-graph
//----------------------------------------------------------------------------------------------------------------------
class NodeGeometry : public Node
{
public:
  virtual ~NodeGeometry(){ glDeleteLists(m_dl,1); };
  virtual void update();

  virtual Node* getNode(int pickID);
  virtual void createGeometry() = 0;

  cinder::Vec4f m_color;
  cinder::Vec4f m_outlineColor;
  cinder::Vec4f m_pickColor;
  float m_outlineWidth;
  bool m_picked;
  bool m_hasShadow;
  bool m_isSelectable;
  int m_dl;

protected:

  NodeGeometry();

};

//----------------------------------------------------------------------------------------------------------------------
// An axes object
//----------------------------------------------------------------------------------------------------------------------
class Axes : public NodeGeometry
{
public:

  Axes() : m_size(1.0f){ init(); };
  Axes(float size) : m_size(size){ init(); };
  virtual void createGeometry();

  float m_size;

protected:
  virtual void init();
};

//----------------------------------------------------------------------------------------------------------------------
// A box
//----------------------------------------------------------------------------------------------------------------------
class Box : public NodeGeometry
{
public:

  Box() : m_lx(1.0f), m_ly(1.0f), m_lz(1.0f){ init(); };
  Box(float x, float y, float z) : m_lx(x), m_ly(y), m_lz(z){ init(); };
  virtual void createGeometry();

  float
    m_lx,
    m_ly,
    m_lz;

protected:
  virtual void init();
};

//----------------------------------------------------------------------------------------------------------------------
// A sphere
//----------------------------------------------------------------------------------------------------------------------
class Sphere : public NodeGeometry
{
public:

  Sphere() : m_radius(1), m_resolution(16) { init(); };
  Sphere(float radius, int resolution = 16) : m_radius(radius), m_resolution(resolution) { init(); };
  virtual void createGeometry();

  float m_radius;
  int m_resolution;

protected:
  virtual void init();
};

//----------------------------------------------------------------------------------------------------------------------
// A cylinder with flat ends
//----------------------------------------------------------------------------------------------------------------------
class Cylinder : public NodeGeometry
{
public:

  Cylinder() : m_radius1(1), m_radius2(1), m_length(1), m_resolution(16) { init(); };
  Cylinder(float r, float l, int res = 16) : m_radius1(r), m_radius2(r), m_length(l), m_resolution(res) { init(); };
  Cylinder(float r1, float r2, float l, int res = 16) : m_radius1(r1), m_radius2(r2), m_length(l), m_resolution(res) { init(); };
  virtual void createGeometry();

  float m_radius1;
  float m_radius2;
  float m_length;
  int m_resolution;

protected:
  virtual void init();
};

//----------------------------------------------------------------------------------------------------------------------
// A cylinder half-spheres at ends
//----------------------------------------------------------------------------------------------------------------------
class Capsule: public NodeGeometry
{
public:

  Capsule() : m_radius(1), m_length(1), m_resolution(16) { init(); };
  Capsule(float r, float l, int res = 16) : m_radius(r), m_length(l), m_resolution(res) { init(); };
  virtual void createGeometry();

  float m_radius;
  float m_length;
  int m_resolution;

protected:
  virtual void init();
};

//----------------------------------------------------------------------------------------------------------------------
// A grid
//----------------------------------------------------------------------------------------------------------------------
class Grid : public NodeGeometry
{
public:

  Grid() : m_xl(2), m_yl(2), m_Nx(2), m_Ny(2) { init(); };
  Grid(float xl, float yl, int nX = 2, int nY = 2) : m_xl(xl), m_yl(yl), m_Nx(nX), m_Ny(nY)  { init(); };
  virtual void createGeometry();

  float m_xl;
  float m_yl;

  int m_Nx;
  int m_Ny;

protected:
  virtual void init();
};

//----------------------------------------------------------------------------------------------------------------------
// A 3d model in .obj format loaded from file
//----------------------------------------------------------------------------------------------------------------------
//class ofx3dModel: public ofxNodeGeometry
//{
//public:
//
//  ofx3dModel() { init(); };
//  virtual void createGeometry();
//
//protected:
//  void load(string name, float scale = 1.0f){ m_obj.loadModel(name, scale); };
//  virtual void init();
//
//  ofx3DModelLoader m_obj;
//};

//----------------------------------------------------------------------------------------------------------------------
// A graph/plot
//----------------------------------------------------------------------------------------------------------------------
class Plot : public Node
{
public:

  Plot(float w = 2.0, float h = 1.0, int nr = 1, int N = 100);

  virtual void update();
  void addPoint(float p, int pID = 0);
  virtual Node* getNode(int pickID) { if (m_uniqueID == pickID) return this; else return 0; };

protected:
  virtual void init();

  std::vector<std::vector<float> > m_points;
  //std::vector<ci::Path2d> m_paths;
  int m_nr;
  uint8_t m_N;
  float m_w;
  float m_h;
  float m_maxY;
  float m_minY;
};


//----------------------------------------------------------------------------------------------------------------------
// A matrix viz node
//----------------------------------------------------------------------------------------------------------------------
class RealMatrixViz : public Node
{
public:

  RealMatrixViz(double **data, int n, int m, float width = 100.0f, double maxVal = 1.0);
  virtual Node* getNode(int pickID){ if(m_uniqueID == pickID) return this; else return 0;};
  virtual void update();
  
protected:
  virtual void init()
  {
    m_type = NODE_VALUEMATRIX;
  #ifdef _DEBUG
    print();
  #endif
  };
    
public:  
  int m_iSel, m_jSel;  

protected:
  double** m_data;
  float m_scale;
  double m_maxVal;
  int m_N, m_M;
  GLuint m_vbo[2];
  //GLfloat *m_vertices, *m_colors;

};
  

//----------------------------------------------------------------------------------------------------------------------
// A matrix viz node
//----------------------------------------------------------------------------------------------------------------------
template<class Type>
class MatrixView : public Node
{
public:

  MatrixView(Type **d, int n, int m, float s = 1.0f, float maxVal = 1) : 
    m_data(d), m_N(n), m_M(m), m_scale(s/n), m_maxVal(maxVal), m_iSel(-1), m_jSel(-1) { init(); };

  virtual Node* getNode(int pickID){ if(m_uniqueID == pickID) return this; else return 0;};
  
  virtual void update()
  {
    glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);
    glLineWidth(1);

    glPushMatrix();
    glMultMatrixf(m_pTM->m);

    glScalef(m_scale, m_scale, m_scale);
    
    // Align top left corner
    glTranslatef(0.5, 0.5, 0.0);
    
    cinder::Vec3f col;
    for(int i = 0; i < m_N; i++)
    {
      for(int j = 0; j < m_M; j++)
      {
        glPushMatrix();
        glTranslatef(i, j, 0.0);
        
        float w = m_data[i][j] / m_maxVal;
        col = w > 0 ? Vec3f(0,0,0) : Vec3f(235.0/255.0, 0.0/255.0, 103.0/255.0); //Vec3f(215.0/255.0, 19.0/255.0, 69.0/255.0);
        glColor4f(col.x, col.y, col.z, 1);
        drawRectangle(w*0.98, w*0.98);
        //ci::gl::drawSolidRect(ci::Rectf(ci::Area(ci::Vec2f(0.0f,0.0f), ci::Vec2f(1.0f, 1.0f))), false);
        if (i == m_iSel && j == m_jSel)
        {
          glLineWidth(2.0f);
          glColor3f(1, 1, 1);         
          drawFrame(1, 1);  
        }
        glPopMatrix();
      }
    }
    
    glPopMatrix();
    glPopAttrib();
  };  
  
  virtual void onMouseMove(const Vec3f& mP)
  {
    const Vec3f& pos = m_pTM->getTranslation();
    const float xMax = pos.x + m_N * m_scale;
    const float yMax = pos.y + m_M * m_scale;
    if((mP.x > pos.x) && (mP.x < xMax) && (mP.y > pos.y) && (mP.y < yMax))
    {      
      Vec3f mPosLocal = mP - pos;
      m_iSel = (int) (mPosLocal.x / m_scale);
      m_jSel = (int) (mPosLocal.y / m_scale);
      assert(m_iSel < m_N && m_jSel < m_M);
    }
    else 
    {
      m_iSel = m_jSel = -1;
    }

  };
  
  int
    m_iSel,
    m_jSel;  

protected:
  virtual void init()
  {
    m_type = NODE_VALUEMATRIX;
  #ifdef _DEBUG
    print();
  #endif
  };

  Type** m_data;
  float m_scale;
  float m_maxVal;
  int
    m_N,
    m_M;

};

//----------------------------------------------------------------------------------------------------------------------
// A vector viz node
//----------------------------------------------------------------------------------------------------------------------
template<class Type>
class VectorView : public Node
{
public:

  VectorView(Type *d, int n, float s = 1.0f, float maxVal = 1.0f) : 
    m_data(d), m_N(n), m_scale(s/n), m_maxVal(maxVal), m_iSel(-1)  { init(); };

  void update()
  {
    glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);
    glLineWidth(1);

    glPushMatrix();
    glMultMatrixf(m_pTM->m);
    glScalef(m_scale, m_scale, m_scale);

    // Aligh top left corner
    glTranslatef(0.5, 0.5, 0.0);
    
    glColor3f(0,0,0);
    glLineWidth(2.0);
    for(int i = 0; i < m_N; i++)
    {   
      float w = m_data[i] / m_maxVal;        
      drawRectangle(w*0.98, w*0.98);       
      if(i == m_iSel)
      {
        glColor3f(1,1,1);
        drawFrame(1, 1);
        glColor3f(0,0,0);
      }
      glTranslatef(1,0,0); // move on to next
    }
    glPopMatrix();
    glPopAttrib();
  }
  
  virtual void onMouseMove(const Vec3f& p)
  {
    Vec3f pos = m_pTM->getTranslation();
    const float xMax = pos.x + m_N * m_scale;
    const float yMax = pos.y + m_scale;
    if((p.x > pos.x) && (p.x < xMax) && (p.y > pos.y) && (p.y < yMax))
    {      
      Vec3f posLocal = p - pos;
      m_iSel = (int) (posLocal.x / m_scale);
      assert(m_iSel < m_N);
    }
    else
    {
      m_iSel = -1;
    }

  };

  virtual Node* getNode(int pickID){ if(m_uniqueID == pickID) return this; else return 0;};

  int m_iSel;
  
protected:
  virtual void init()
  {
    m_type = NODE_VALUEVECTOR;
#ifdef _DEBUG
    print();
#endif
  };

  Type* m_data;
  float m_scale;
  float m_maxVal;
  int
    m_N;
};


// A multiplot node
// -------------------------------------------------------------------
/*class ofxMultiplot : public ofxNodeGroup
{
public:

  ofxMultiplot (float w = 2.0, float h = 1.0, int nr=1, int N=100) :
    m_w(w), m_h(h), m_nr(nr), m_N(N) { init(); };

  virtual void update() const
  {
    for(int i=0; i < m_nn.size; i++)
    {
      ((ofxPlot*)m_children[i])->addPoint(1 - m_nn.outputs[i+1]);
    }
    ofxNodeGroup::update();
  }

protected:
  virtual void init()
  {
    m_type = NODE_CUSTOM;
#ifdef _DEBUG
    print();
#endif
    for(int i=0; i < m_nn.size; i++)
    {
      ofxPlot* plot = new ofxPlot(4*m_scale, m_scale, 100); // 2d
      plot->translate(2*m_scale, -m_nn.size*m_scale/2 + i*m_scale, 0);
      m_children.push_back(plot);
    }
    ofxValueVector* outputs = new ofxValueVector(m_nn.outputs.get(), m_nn.size, m_scale);
    ofxValueMatrix* weights = new ofxValueMatrix(m_nn.weights.get(), m_nn.size, m_nn.size, m_scale);
    weights->translate(-m_scale*m_nn.size,0,0);
    m_children.push_back(weights);
    m_children.push_back(outputs);
  };

  CTRNN& m_nn;
  float m_scale;
  float m_w;
  float m_h;
  int m_nr;
  int m_N;  
};
 */

} //namespace dmx
#endif

