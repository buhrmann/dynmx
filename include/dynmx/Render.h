/*
 *  Render.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
  
#ifndef __OFX_3d__H_
#define __OFX_3d__H_

#include "Dynmx.h"

#ifdef DYNMX_MAC
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <glut.h>
#endif

#include "cinder/Matrix.h"
#include "cinder/Color.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"

namespace dmx
{

  typedef cinder::Matrix44f Mat4f;
  typedef cinder::Vec3f Vec2f;
  typedef cinder::Vec3f Vec3f;
  typedef cinder::Vec4f Vec4f;

  // static draw functions ...
  // ---------------------------------------------------------
  static void setTransform(const Mat4f& matrix)
	{
		glMultMatrixf(matrix);
	}
  
  static void translate(const Vec4f& pos)
  {
    glTranslatef(pos.x, pos.y, pos.z);
  }
  
  static void translate(const Vec3f& pos)
  {
    glTranslatef(pos.x, pos.y, pos.z);
  }

  static void setColor3(const Vec3f& c)
	{
		glColor3f(c[0], c[1], c[2]);
	}

	static void setColor4(const Vec4f& c)
	{
		glColor4f(c[0], c[1], c[2], c[3]);
	}

	static void setColorMaterial(float r, float g, float b, float alpha)
	{
		GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
		light_ambient[0] = r*0.3f;
		light_ambient[1] = g*0.3f;
		light_ambient[2] = b*0.3f;
		light_ambient[3] = alpha;
		light_diffuse[0] = r*0.7f;
		light_diffuse[1] = g*0.7f;
		light_diffuse[2] = b*0.7f;
		light_diffuse[3] = alpha;
		light_specular[0] = r*0.2f;
		light_specular[1] = g*0.2f;
		light_specular[2] = b*0.2f;
		light_specular[3] = alpha;
		glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
		glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
		glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
		glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0f);
	}
	static void setColorMaterial(const Vec3f& c){ setColorMaterial(c[0], c[1], c[2], c[3]);};

	static void drawBasis(float l, bool annotate = false)
	{
		glBegin(GL_LINES);
			glColor3f(1,0,0);
			glVertex3f(0,0,0);
			glVertex3f(l,0,0);
      
			glColor3f(0,1,0);
			glVertex3f(0,0,0);
			glVertex3f(0,l,0);
      
			glColor3f(0,0,1);
			glVertex3f(0,0,0);
			glVertex3f(0,0,l);
		glEnd();

    if(annotate)
    {
      const char* c = "xyz";
      glColor3f(1,0,0);
      glRasterPos3f(l*1.1f, 0.0f, 0.0f);
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, c[0]);

      glColor3f(0.0f,1.0f,0.0f);
      glRasterPos3f(0.0f, l*1.1f, 0.0f);
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, c[1]);

      glColor3f(0.0f, 0.0f , 1.0f);
      glRasterPos3f(0.0f, 0.0f, l*1.1f);
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, c[2]);
    }
	}

  static void drawBox(float x, float y, float z)
  {
		float lx = x*0.5f;
		float ly = y*0.5f;
		float lz = z*0.5f;

  	glBegin(GL_QUADS);

		glNormal3f (0,1,0);
		glVertex3f( lx, ly,-lz);
		glVertex3f(-lx, ly,-lz);
		glVertex3f(-lx, ly, lz);
		glVertex3f( lx, ly, lz);

		glNormal3f (0,-1,0);
		glVertex3f( lx,-ly, lz);
		glVertex3f(-lx,-ly, lz);
		glVertex3f(-lx,-ly,-lz);
		glVertex3f( lx,-ly,-lz);

		glNormal3f (0,0,1);
		glVertex3f( lx, ly, lz);
		glVertex3f(-lx, ly, lz);
		glVertex3f(-lx,-ly, lz);
		glVertex3f( lx,-ly, lz);

		glNormal3f (0,0,-1);
		glVertex3f( lx,-ly,-lz);
		glVertex3f(-lx,-ly,-lz);
		glVertex3f(-lx, ly,-lz);
		glVertex3f( lx, ly,-lz);

		glNormal3f (-1,0,0);
		glVertex3f(-lx, ly, lz);
		glVertex3f(-lx, ly,-lz);
		glVertex3f(-lx,-ly,-lz);
		glVertex3f(-lx,-ly, lz);

		glNormal3f (1,0,0);
		glVertex3f( lx, ly,-lz);
		glVertex3f( lx, ly, lz);
		glVertex3f( lx,-ly, lz);
		glVertex3f( lx,-ly,-lz);
		glEnd();
  }

  static void drawString(const Vec3f& p, const char *s)
	{
		char c;
		glRasterPos3f(p[0], p[1], p[2]);
		for ( ; (c = *s) != '\0'; s++ )
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
	}

  // in the y plane
	static void drawCross(float l)
	{
	  l *= 0.5;
	  glBegin(GL_LINES);
    glNormal3f (0,1,0);
    glVertex3f( l, 0, 0.0f);
    glVertex3f(-l, 0, 0.0f);
    glVertex3f(0, l, 0.0f);
    glVertex3f(0, -l, 0.0f);
	  glEnd();
	}
  
  // in the y plane
	static void drawRectangle(float lx, float ly, GLenum mode = GL_QUADS)
	{
	  lx *= 0.5;
	  ly *= 0.5;

    // TODO: performance hit ?
    // as 2d, make sure it's drawn on both side
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_CULL_FACE);
	  glBegin(mode);
      glNormal3f (0,0,1);
      glVertex3f( lx, ly, 0.0f);
      glVertex3f(-lx, ly, 0.0f);
      glVertex3f(-lx, -ly, 0.0f);
      glVertex3f( lx, -ly, 0.0f);
	  glEnd();
	  glPopAttrib();
	}
  
  // in the y plane
	static void drawRectangleCentred(float lx, float ly, ci::ColorA tl, ci::ColorA tr, ci::ColorA bl, ci::ColorA br)
	{
	  lx *= 0.5;
	  ly *= 0.5;

	  glBegin(GL_QUADS);
      glNormal3f (0,0,1);
      glColor4f(br.r, br.g, br.g, br.a);      
      glVertex3f( lx, ly, 0.0f);
      glColor4f(bl.r, bl.g, bl.g, bl.a);      
      glVertex3f(-lx, ly, 0.0f);
      glColor4f(tl.r, tl.g, tl.g, tl.a);
      glVertex3f(-lx, -ly, 0.0f);
      glColor4f(tr.r, tr.g, tr.g, tr.a);
      glVertex3f( lx, -ly, 0.0f);
	  glEnd();
	}  

	static void drawRectangle(float lx, float ly, ci::ColorA tl, ci::ColorA tr, ci::ColorA bl, ci::ColorA br)
	{
	  glBegin(GL_QUADS);
      glNormal3f (0,0,1);
      glColor4f(br.r, br.g, br.g, br.a);      
      glVertex3f( lx, ly, 0.0f);
      glColor4f(bl.r, bl.g, bl.g, bl.a);      
      glVertex3f(0, ly, 0.0f);
      glColor4f(tl.r, tl.g, tl.g, tl.a);
      glVertex3f(0, 0, 0.0f);
      glColor4f(tr.r, tr.g, tr.g, tr.a);
      glVertex3f( lx, 0, 0.0f);
	  glEnd();
	}  
  
  static void drawSquares(int N=100, int M=100)
  {
  int dl = glGenLists(1);
  glNewList(dl, GL_COMPILE);
    glPushMatrix();
    float scale = 300.0f / N;
    glScalef(scale,scale,scale);
    glTranslatef(N/2, M/2, 0);
    for(int i=0; i<N; i++)
    {
      for(int j=0; j<M; j++)
      {
        glPushMatrix();
        glTranslatef((float)i, (float)j, 0);
        float id = (float)i*M+j;
        float c = id / (N*M);
        glColor3f(c,c,c);
        drawRectangle(1,1);
        glPopMatrix();
      }
    }
    glPopMatrix();
  glEndList();
  glCallList(dl);
  } 
  
  static GLuint vbo[2];
  static bool vboInited = false;
  
  static void drawSquaresVB(float width, int N=200, int M=200)
  {
    float s = width/N;
    const int numVerts = N*M*4;
    GLfloat* verts = new GLfloat[numVerts*2];
    GLfloat* cols = new GLfloat[numVerts*3];

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
        float color =  (float)sId / (float)(N*M);
        for(int c = 0; c < 12; c++)
        {
          cols[cId + c] = color;
        }
      }
    }
    
    bool useVA = true;
    if(useVA)
    {
      glEnableClientState( GL_VERTEX_ARRAY );
      glEnableClientState( GL_COLOR_ARRAY );      
      
      glColorPointer(3, GL_FLOAT, 0, cols);
      glVertexPointer(2, GL_FLOAT, 0, verts);

      glDrawArrays( GL_QUADS, 0, numVerts );
      
      glDisableClientState(GL_COLOR_ARRAY);
      glDisableClientState(GL_VERTEX_ARRAY);
    }
    else
    {
      if(!vboInited)
      {
        glGenBuffersARB(2, &vbo[0]);
        vboInited = true;        
      }
      
      // render into fbo
      const bool useFbo = false;                     
      ci::gl::Fbo::Format format;
      format.enableDepthBuffer(false);
      format.enableColorBuffer(true);
      ci::gl::Fbo myFbo( 640, 480, format );          
      if(useFbo)
      {          
        myFbo.bindFramebuffer();  
      }
            
      glEnableClientState(GL_VERTEX_ARRAY);
      glEnableClientState(GL_COLOR_ARRAY); 

      // copy vertex data
      glBindBufferARB(GL_ARRAY_BUFFER_ARB, vbo[0]);
      glBufferDataARB(GL_ARRAY_BUFFER_ARB, sizeof(verts), verts, GL_STATIC_DRAW_ARB);
      glVertexPointer(2, GL_FLOAT, 0, 0);  
      
      // copy color data
      glBindBufferARB(GL_ARRAY_BUFFER_ARB, vbo[1]);
      glBufferDataARB(GL_ARRAY_BUFFER_ARB, sizeof(cols), cols, GL_STATIC_DRAW_ARB);      
      glColorPointer(3, GL_FLOAT, 0, 0);

      glDrawArrays(GL_QUADS, 0, numVerts); 
      
      glDisableClientState(GL_COLOR_ARRAY);      
      glDisableClientState(GL_VERTEX_ARRAY); 
      
      // reset
      glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);  
      
      if(useFbo)
      {
        myFbo.unbindFramebuffer(); 
        ci::gl::draw(myFbo.getTexture(0), myFbo.getTexture(0).getBounds());
//      ci::gl::GlslProg blurShader;
//      blurShader.uniform( "blurRadius", 5.0f );
//      blurShader.bind();
//      ci::gl::draw( myFbo.getTexture() ); // draw the contents of the Fbo by using it as a texture
//      blurShader.unbind();          
      }          
    }
    delete [] verts;
    delete [] cols;
  }     


	static void drawFrame(float lx, float ly)
	{
    lx *= 0.5;
    ly *= 0.5;
    glBegin(GL_LINE_STRIP);
      glVertex3f( -lx, -ly, 0.0f);
      glVertex3f( -lx, ly, 0.0f);
      glVertex3f(  lx, ly, 0.0f);
      glVertex3f(  lx, -ly, 0.0f);
      glVertex3f( -lx, -ly, 0.0f);
	  glEnd();
	}

//	static void drawLine(const Vec3f& from, const Vec3f& to)
//	{
//	  glBegin (GL_LINES);
//      glVertex3fv(from.data);
//      glVertex3fv(to.data);
//    glEnd ();
//	}
//
	static void drawTriangle(const Vec3f& a, const Vec3f& b, const Vec3f& c)
	{
	  glBegin(GL_TRIANGLES);
      glVertex3fv(a);
      glVertex3fv(b);
      glVertex3fv(c);
    glEnd();
	};

	// equilateral in z-plane
	static void drawTriangle(float s)
	{
    glBegin(GL_TRIANGLES);
      glVertex3f( s,  0.0f,  0.0f);
      glVertex3f(-s, 0.0f, -s);
      glVertex3f(-s, 0.0f, s);      
    glEnd();
	}
  
  static void drawLineFromTo(const Vec3f& p1, const Vec3f& p2)
	{
	  glBegin(GL_LINES);
    glVertex3f(p1[0], p1[1], p1[2]);
    glVertex3f(p2[0], p2[1], p2[2]);
	  glEnd();
	}

  static void drawLineAt(const Vec3f& p, const Vec3f& v)
	{
	  glBegin(GL_LINES);
    glVertex3f(p[0], p[1], p[2]);
    glVertex3f(p[0]+v[0], p[1]+v[1], p[2]+v[2]);
	  glEnd();
	}
  
  static void drawVectorAt(const Vec3f& p, const Vec3f& v)
	{
	  drawLineAt(p, v);
	}

  // Draws a point
	static void drawPoint(const Vec3f& p, float s)
  {
    glPointSize(s);
    glBegin(GL_POINTS);
    glVertex3f(p.x, p.y, p.z);
    glEnd();
  }
  
	// Draws a point as 3 intersecting lines
	static void drawCross(const Vec3f& p, float s)
	{
	  glBegin(GL_LINES);
      glVertex3f(p[0]-s, p[1], p[2]);
      glVertex3f(p[0]+s, p[1], p[2]);

      glVertex3f(p[0], p[1]-s, p[2]);
      glVertex3f(p[0], p[1]+s, p[2]);

      glVertex3f(p[0], p[1], p[2]-s);
      glVertex3f(p[0], p[1], p[2]+s);
	  glEnd();
	}

	/// Draws a grid of size x*y with distance between lines equal to res
	static void drawGrid(float x, float y, float res)
	{
	  x *= 0.5;
    y *= 0.5;
    for(float i = -x; i <= x; i += res)
	  {
      glBegin(GL_LINES);
        glVertex3f(-x, 0.0f, i);
        glVertex3f(x,  0.0f, i);
        glVertex3f(i, 0.0f, -y);
        glVertex3f(i, 0.0f, y);
      glEnd();
	  }
	}

	// uses corner control points to evaluate a grid on board
	static void drawEvalGrid(const GLfloat* grid, int n, int m)
	{
      glMap2f(GL_MAP2_VERTEX_3, 0.0, 1.0, 3, 2, 0.0, 1.0, 2 * 3, 2, grid);

      // when rendering surface mesh
      glEnable(GL_MAP2_VERTEX_3);
      glMapGrid2f(n, 0.0, 1.0, m, 0.0, 1.0);
      glEvalMesh2(GL_LINE, 0, n, 0, m);
	}

  static void drawDisk(float radius1, float radius2, int slices, int rings, GLenum mode=GLU_FILL, GLenum shade=GLU_SMOOTH )
	{
	  GLUquadricObj* obj = gluNewQuadric();
	  gluQuadricDrawStyle(obj, mode);
	  gluQuadricNormals(obj, shade);
		gluDisk(obj, radius2, radius1, slices, rings);
    gluDeleteQuadric(obj);
	}
  
  static void drawPartialDisk(float innerRadius, float outerRadius, int slices, int rings, float startAngle, float sweepAngle, 
    GLenum mode=GLU_FILL, GLenum shade=GLU_SMOOTH )
	{
	  GLUquadricObj* obj = gluNewQuadric();
	  gluQuadricDrawStyle(obj, mode);
	  gluQuadricNormals(obj, shade);
		gluPartialDisk(obj, innerRadius, outerRadius, slices, rings, startAngle, sweepAngle);
    gluDeleteQuadric(obj);
	}  

	static void drawSphere(float radius, int slices, int stacks, GLenum mode=GLU_FILL, GLenum shade=GLU_SMOOTH)
	{
	  GLUquadricObj* obj = gluNewQuadric();
	  gluQuadricDrawStyle(obj, mode);
	  gluQuadricNormals(obj, shade);
		gluSphere(obj, radius, slices, stacks);
    gluDeleteQuadric(obj);
	}

	// along z-Axis
	static void drawCylinder(float length, float radius1, float radius2, int slices, int stacks, GLenum mode=GLU_FILL)
	{
	  //draws hollow cylinder
	  GLUquadricObj* obj = gluNewQuadric();
	  gluQuadricDrawStyle(obj, mode);
	  gluQuadricNormals(obj, GLU_SMOOTH);
	  glPushMatrix();
		glTranslatef(0,0,-length/2);    
		gluCylinder(obj, radius1, radius2, length, slices, stacks);

#if 1    
		//bottom disk
		glFrontFace(GL_CW);
		gluDisk(obj, 0, radius1, slices, 4);
		glFrontFace(GL_CCW);
		//top disk
		glPushMatrix();
			glTranslatef( 0, 0, length );
			gluDisk(obj, 0, radius2, slices, 4);
		glPopMatrix();
#endif
    
    glPopMatrix();
	  gluDeleteQuadric(obj);
	}

  static void drawCylinder(const Vec3f& p1, const Vec3f& p2, float radius1, float radius2, int slices, int stacks, GLenum mode=GLU_FILL)
  {
    float length = (p2 - p1).length(); //length
    Vec3f v1(p2 - p1);  //vector showing from p2 to p1
		Vec3f v2(0.0, 0.0, 1.0);  //vector showing in z-axis
    v1.normalize();
    Mat4f tm;
		//if v1 is not lying on z-axis, calculate rotation matrix
		/*(if( !(v1.x == 0 && v1.y == 0) ){
			XVector n( (v1^v2) / (v1^v2).Length() );

			double tempACos = (v1*v2) / (v1.Length() * v2.Length());
			double alpha =  acos( tempACos );

			tm.makeRotate(alpha, -n[0], -n[1], -n[2]);
		}*/
		Vec3f n = v1.cross(v2);// n perp to v1
		//v2 = v1^n;
		tm.setRow(0, cinder::Vec4f(v2, 1));
		tm.setRow(1, cinder::Vec4f(n, 1));
		tm.setRow(2, cinder::Vec4f(v1, 1));

		//find middle point between two given points and move object to location
		Vec3f pos = p1 + (p2 - p1) * 0.5f;
    tm.setRow(4, cinder::Vec4f(pos, 1));

		glPushMatrix();
		glMultMatrixf(tm);
		drawCylinder(length, radius1, radius2, slices, stacks, mode);
		glPopMatrix();
  }

	static void drawCapsule(float length, float radius, int slices, int stacks, GLenum mode=GLU_FILL)
	{
	  GLUquadricObj* obj = gluNewQuadric();
	  gluQuadricDrawStyle(obj, mode);
	  gluQuadricNormals(obj, GLU_SMOOTH);

	  // cylinder
	  glPushMatrix();
	  glTranslatef(0,0,-length/2);
		  gluCylinder(obj, radius, radius, length, slices, stacks);
	  glPopMatrix();

	  // first end-cap
	  glPushMatrix();
	  glTranslatef( 0, 0, length*0.5 );
		gluSphere(obj, radius, slices, stacks);
	  glPopMatrix();

	  glPushMatrix();
	  glTranslatef( 0, 0, -length*0.5 );
		gluSphere(obj, radius, slices, stacks);
	  glPopMatrix();

	  gluDeleteQuadric(obj);

	}

	// forward declaration: implementation at bottom of file
	static void setShadowTransform(const Vec4f& ground, const Vec4f& origin, const Vec4f& light)
	{
    /*float  dot;
    float  shadowMat[4][4];

    Vec4f ground = normal;
    Vec3f n3 (normal.x, normal.y, normal.z);
    Vec3f o3 (origin.x, origin.y, origin.z); 
    ground[3] = - n3.dot(o3);

    dot = ground.dot(light);

    shadowMat[0][0] = dot - light[0] * ground[0];
    shadowMat[1][0] = 0.0 - light[0] * ground[1];
    shadowMat[2][0] = 0.0 - light[0] * ground[2];
    shadowMat[3][0] = 0.0 - light[0] * ground[3];

    shadowMat[0][1] = 0.0 - light[1] * ground[0];
    shadowMat[1][1] = dot - light[1] * ground[1];
    shadowMat[2][1] = 0.0 - light[1] * ground[2];
    shadowMat[3][1] = 0.0 - light[1] * ground[3];

    shadowMat[0][2] = 0.0 - light[2] * ground[0];
    shadowMat[1][2] = 0.0 - light[2] * ground[1];
    shadowMat[2][2] = dot - light[2] * ground[2];
    shadowMat[3][2] = 0.0 - light[2] * ground[3];

    shadowMat[0][3] = 0.0 - light[3] * ground[0];
    shadowMat[1][3] = 0.0 - light[3] * ground[1];
    shadowMat[2][3] = 0.0 - light[3] * ground[2];
    shadowMat[3][3] = dot - light[3] * ground[3];

    glMultMatrixf((const GLfloat*)shadowMat);
    */
    float  dot;
    float  shadowMat[4][4];

    dot = ground[0] * light[0] +
          ground[1] * light[1] +
          ground[2] * light[2] +
          ground[3] * light[3];
    
    shadowMat[0][0] = dot - light[0] * ground[0];
    shadowMat[1][0] = 0.0 - light[0] * ground[1];
    shadowMat[2][0] = 0.0 - light[0] * ground[2];
    shadowMat[3][0] = 0.0 - light[0] * ground[3];
    
    shadowMat[0][1] = 0.0 - light[1] * ground[0];
    shadowMat[1][1] = dot - light[1] * ground[1];
    shadowMat[2][1] = 0.0 - light[1] * ground[2];
    shadowMat[3][1] = 0.0 - light[1] * ground[3];
    
    shadowMat[0][2] = 0.0 - light[2] * ground[0];
    shadowMat[1][2] = 0.0 - light[2] * ground[1];
    shadowMat[2][2] = dot - light[2] * ground[2];
    shadowMat[3][2] = 0.0 - light[2] * ground[3];
    
    shadowMat[0][3] = 0.0 - light[3] * ground[0];
    shadowMat[1][3] = 0.0 - light[3] * ground[1];
    shadowMat[2][3] = 0.0 - light[3] * ground[2];
    shadowMat[3][3] = dot - light[3] * ground[3];

    glMultMatrixf((const GLfloat*)shadowMat); 
  }

  static Vec3f getColorMapRainbow(float yval)
  {
    if (yval < 0.2) // purple to blue ramp
    {
      return Vec3f(0.5*(1.0-yval/0.2), 0.0, 0.5+(0.5*yval/0.2));
    }
    else if ((yval >= 0.2) && (yval < 0.40)) // blue to cyan ramp
    {
      return Vec3f(0.0, (yval-0.2)*5.0, 1.0);
    }
    else if ((yval >= 0.40) && (yval < 0.6)) // cyan to green ramp
    {
      return Vec3f(0.0, 1.0, (0.6-yval)*5.0);
    }
    else if ((yval >= 0.6) && (yval < 0.8 )) // green to yellow ramp
    {
      return Vec3f((yval-0.6)*5.0, 1.0, 0.0);
    }
    else // yellow to red ramp^
    {
      return Vec3f(1.0, (1.0-yval)*5.0, 0.0);
    }
  }

  static Vec3f getColorMapLuminance(float yval)
  {
    if (yval < 0.30)
    {
      return Vec3f(yval/0.3, 0, 0);
    }
    else if ((yval>=0.30) && (yval < 0.89))
    {
      return Vec3f(1.0, (yval-0.3)/0.59, 0.0);
    }
    else
    {
      return Vec3f(1.0, 1.0, (yval-0.89)/0.11);
    }
  }
  
  // assumes val in [-1, 1]
  static Vec3f getColorMapBlueRed(float val)
  {
    assert(fabs(val) <= 1.0);
    
    //static const Vec3f positive (180.0/255.0, 4.0/255.0, 38.0/255.0);
    //static const Vec3f negative (59.0/255.0, 76.0/255.0, 192.0/255.0);
    //static const Vec3f neutral (221.0/255.0, 221.0/255.0, 221.0/255.0);
    static const Vec3f positive (255.0/255.0, 0.0/255.0, 0.0/255.0);
    static const Vec3f negative (0.0/255.0, 0.0/255.0, 255.0/255.0);
    static const Vec3f neutral (255.0/255.0, 255.0/255.0, 255.0/255.0);
    if(val >= 0)
    {
      return neutral + val * (positive - neutral);
    }
    else
    {
      return neutral - val * (negative - neutral);
    }
  };

}

#endif
