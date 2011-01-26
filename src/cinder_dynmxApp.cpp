
#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/MayaCamUI.h"
#include "cinder/gl/gl.h"
#include "Render.h"

using namespace ci;
using namespace ci::app;

class cinder_dynmxApp : public AppBasic 
{
public:
	void setup();
	void prepareSettings( Settings *settings );  
  void resize( ResizeEvent event );
	void mouseDown( MouseEvent event );	
	void mouseMove( MouseEvent event );
	void mouseDrag( MouseEvent event );  
	void update();
	void draw();
	void drawGrid(float size=100.0f, float step=10.0f);  
    
	MayaCamUI	mMayaCam;
	Vec2i		mMousePos; 
};

void cinder_dynmxApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
	settings->setFullScreen( false );
}

void cinder_dynmxApp::setup()
{
	// set up the camera
	CameraPersp cam;
	cam.setEyePoint( Vec3f(5.0f, 10.0f, 10.0f) );
	cam.setCenterOfInterestPoint( Vec3f(0.0f, 0.0f, 0.0f) );
	cam.setPerspective( 60.0f, getWindowAspectRatio(), 1.0f, 1000.0f );
	mMayaCam.setCurrentCam( cam );
}

void cinder_dynmxApp::resize( ResizeEvent event )
{
	// adjust aspect ratio
	CameraPersp cam = mMayaCam.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam( cam );
}

void cinder_dynmxApp::update()
{

}

void cinder_dynmxApp::draw()
{
	// clear out the window with black
	gl::clear( Color::black() ); 
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	
  gl::pushMatrices();
	gl::setMatrices( mMayaCam.getCamera() );

	// enable the depth buffer (after all, we are doing 3D)
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableAlphaBlending();
	glDisable( GL_TEXTURE_2D );  
  glDisable(GL_LIGHTING);  

	// draw the grid on the floor
	drawGrid();
  
	glEnable( GL_LIGHTING );
	glEnable( GL_LIGHT0 );
	GLfloat light_position[] = { mMousePos.x, mMousePos.y, 75.0f, 1.0f };
  glLightfv( GL_LIGHT0, GL_POSITION, light_position );
  glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;      
  
  glColor3f(1,1,1);
  //gl::drawCube( Vec3f::zero(), Vec3f( 2.0f, 2.0f, 2.0f ) );
  dmx::drawCylinder(2.0,0.0,1.0,16,16);
}

void cinder_dynmxApp::drawGrid(float size, float step)
{
	gl::color( Colorf(0.2f, 0.2f, 0.2f) );
	for(float i = -size; i <= size; i += step)
  {
		gl::drawLine( Vec3f(i, 0.0f, -size), Vec3f(i, 0.0f, size) );
		gl::drawLine( Vec3f(-size, 0.0f, i), Vec3f(size, 0.0f, i) );
	}
}

void cinder_dynmxApp::mouseMove( MouseEvent event )
{
	// keep track of the mouse
	mMousePos = event.getPos();
}

void cinder_dynmxApp::mouseDown( MouseEvent event )
{	
	// let the camera handle the interaction
	mMayaCam.mouseDown( event.getPos() );
}

void cinder_dynmxApp::mouseDrag( MouseEvent event )
{
	// keep track of the mouse
	mMousePos = event.getPos();

	// let the camera handle the interaction
	mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}


//CINDER_APP_BASIC( cinder_dynmxApp, RendererGl )
