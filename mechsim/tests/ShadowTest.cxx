#include "ShadowTest.h"
#include "GLHelper.h"
#include <iostream>

const int ShadowTest::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int ShadowTest::INT_PER_STEP = 16;
const char ShadowTest::TITLE[] = "Shadow test";

void STState::Draw(int) const
{
    Box obj0(1., 2., .5);
    Cylinder obj1(1., 2.);
    
    obj0.setCenter(0., -1.2, 4.);
    obj0.setRotation(Eigen::Quaternionf(cos(fT/20.), sin(fT/20.), 0., 0.));
    
    obj1.setCenter(0., 1.2, 4.);
    obj1.setRotation(Eigen::Quaternionf(cos(fT/20.), sin(fT/20.), 0., 0.));
    
    Scene sc;
    sc.addObject(&obj0);
    sc.addObject(&obj1);
    
    GLfloat LightPos[] = { 5.0f, 5.0f, 20.0f, 1.0f };
    GLfloat LightAmb[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat LightDif[] = { 0.6f, 0.6f, 0.6f, 1.0f };
    GLfloat LightSpc[] = { -0.2f, -0.2f, -0.2f, 1.0f };
    GLfloat MatAmb[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat MatDif[] = { 0.2f, 0.6f, 0.9f, 1.0f };
    GLfloat MatSpc[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    
    Eigen::Vector3f SpherePos( 0.f, 0.f, 0.f );
    
    /* BEGIN INIT */
    glShadeModel( GL_SMOOTH );
    glClearColor( 0.0f, 0.0f, 0.0f, 0.5f );
    glClearDepth( 1.0f );
    glClearStencil( 0 );
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LEQUAL );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glLightfv( GL_LIGHT1, GL_POSITION, LightPos );
    glLightfv( GL_LIGHT1, GL_AMBIENT,  LightAmb );
    glLightfv( GL_LIGHT1, GL_DIFFUSE,  LightDif );
    glLightfv( GL_LIGHT1, GL_SPECULAR, LightSpc );
    glEnable( GL_LIGHT1 );
    glEnable( GL_LIGHTING );

    glMaterialfv( GL_FRONT, GL_AMBIENT, MatAmb );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, MatDif );
    glMaterialfv( GL_FRONT, GL_SPECULAR, MatSpc );
    // glMaterialfv( GL_FRONT, GL_SHININESS, ... );

    glCullFace( GL_BACK );
    glEnable(GL_CULL_FACE);
    glClearColor(0.f, 0.f, 0.f, 1.0f);
    
    /* END INIT */
    
    GLUquadricObj *q;
    q = gluNewQuadric();
    gluQuadricNormals(q, GL_SMOOTH);
    gluQuadricTexture(q, GL_FALSE);
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    
    glLightfv( GL_LIGHT1, GL_POSITION, LightPos );
    sc.setLightPos(LightPos[0], LightPos[1], LightPos[2]);
    
    /* Draw sphere on ground */
    glPushMatrix();
    glTranslatef( SpherePos[0], SpherePos[1], SpherePos[2] );
    gluSphere( q, 1.5f, 32, 16);
    glPopMatrix();
    
    /* Draw floor */
    GL::drawCheckerboardFloor();
    
    /* Draw objects */
    glColor4f( 0.7f, 0.4f, 0.0f, 1.0f );
    sc.drawObjects();
    
    /* Draw shadow */
    sc.castShadow();
    
    /* Draw sphere representing the light source */
    glColor4f(0.7f, 0.4f, 0.0f, 1.0f);
    glDisable(GL_LIGHTING);
    glDepthMask(GL_FALSE);
    glTranslatef(LightPos[0], LightPos[1], LightPos[2]);
    gluSphere( q, 0.2f, 16, 8);
    glEnable(GL_LIGHTING);
    glDepthMask(GL_TRUE);
    
    gluDeleteQuadric(q);
}

ShadowTest::ShadowTest()
    : fCurStep(0)
{
}

STState ShadowTest::GetCurrentState()
{
    STState state;
    state.fParent = this;
    state.fT = fCurStep;
    return state;
}
