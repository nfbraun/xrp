#include "GLHelper.h"
#include <GL/gl.h>
#include <GL/glu.h>

namespace GL {

void Rotate(const Rotation& r)
{
    double mat[4][4];
    double a = r.a(), b = r.b(), c = r.c(), d = r.d();
    
    // see http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    mat[0][0] = 1. - 2.*c*c - 2.*d*d;
    mat[0][1] = 2.*b*c - 2.*a*d;
    mat[0][2] = 2.*b*d + 2.*a*c;
    mat[0][3] = 0.;
    
    mat[1][0] = 2.*b*c + 2.*a*d;
    mat[1][1] = 1. - 2.*b*b - 2.*d*d;
    mat[1][2] = 2.*c*d - 2.*a*b;
    mat[1][3] = 0.;
    
    mat[2][0] = 2.*b*d - 2.*a*c;
    mat[2][1] = 2.*c*d + 2.*a*b;
    mat[2][2] = 1. - 2.*b*b - 2.*c*c;
    mat[2][3] = 0.;
    
    mat[3][0] = 0.;
    mat[3][1] = 0.;
    mat[3][2] = 0.;
    mat[3][3] = 1.;
    
    glMultMatrixd(&mat[0][0]);
}

void drawDiscSegment(const double r, double h, const double alpha)
{
    const int n = 8;
    const double alpha2n = alpha / (2 * n);
    const double rc = r * cos(alpha / 2);
    double phi;
    double x1, x2, x3, x4;
    double y1, y2, y3, y4;
    
    h /= 2.;
    
    phi = -n * alpha2n;
    x3 = rc * tan(phi);
    y3 = rc;
    x4 = r * sin(phi);
    y4 = r * cos(phi);

    glBegin(GL_QUADS);
    for(int i=-n+1; i<=n; ++i) {
        phi = i * alpha2n;
    
        x1 = x3; y1 = y3; x2 = x4; y2 = y4;
        x3 = rc * tan(phi);
        y3 = rc;
        x4 = r * sin(phi);
        y4 = r * cos(phi);
        
        glNormal3f(0., 0., 1.);
        glVertex3f(x1, y1, h);
        glVertex3f(x2, y2, h);
        glVertex3f(x4, y4, h);
        glVertex3f(x3, y3, h);
        
        glNormal3f(0., 0., -1.);
        glVertex3f(x1, y1, -h);
        glVertex3f(x2, y2, -h);
        glVertex3f(x4, y4, -h);
        glVertex3f(x3, y3, -h);
        
        glNormal3f(cos(phi), sin(phi), 0.);
        glVertex3f(x2, y2, h);
        glVertex3f(x4, y4, h);
        glVertex3f(x4, y4, -h);
        glVertex3f(x2, y2, -h);
    }
    
    glNormal3f(0., -1., 0.);
    x1 = -r * sin(alpha/2.);
    y1 = rc;
    x3 = -x1;
    y3 = rc;
    glVertex3f(x1, y1, h);
    glVertex3f(x3, y3, h);
    glVertex3f(x3, y3, -h);
    glVertex3f(x1, y1, -h);
    
    glEnd();
}

void drawCheckerboardFloor()
{
    glBegin(GL_QUADS);
        glNormal3f(0., 0., 1.);
        
        for(int x=-5; x <= 5; ++x) {
            for(int y=-5; y <= 5; ++y) {
                if((x+y)%2)
                    glColor3f(1., 0., 0.);
                else
                    glColor3f(0., 0., 1.);
                
                glVertex3f((x-1)*10., (y-1)*10., 0.);
                glVertex3f( x*10.,    (y-1)*10., 0.);
                glVertex3f( x*10.,     y*10.,    0.);
                glVertex3f((x-1)*10.,  y*10.,    0.);
            }
        }
    glEnd();
}

void drawCheckerboardFloorOutline()
{
    glBegin(GL_LINES);
        for(int x=-6; x <= 5; ++x) {
            glVertex3f(x*10., -60., 0.);
            glVertex3f(x*10,   50., 0.);
        }
        
        for(int y=-6; y <=5; ++y) {
            glVertex3f(-60., y*10., 0.);
            glVertex3f( 50., y*10., 0.);
        }
    glEnd();
}

void drawTube(double r, Vector3 p1, Vector3 p2)
{
    Vector3 d = p2 - p1;
    double angle = asin(sqrt(d.x()*d.x() + d.y()*d.y()) / d.mag()) * RAD_TO_DEG;
    if(d.z() < 0) angle = 180. - angle;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(p1.x(), p1.y(), p1.z());
    glRotatef(angle, -d.y(), d.x(), 0.);

    GLUquadric *quad = gluNewQuadric();
    gluCylinder(quad, r, r, d.mag(), 8, 4);
    gluDeleteQuadric(quad);
    
    glPopMatrix();
}

void drawSphere(double r, Vector3 p)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glTranslatef(p.x(), p.y(), p.z());
    GLUquadric *quad = gluNewQuadric();
    gluSphere(quad, r, 16, 16);
    gluDeleteQuadric(quad);
    
    glPopMatrix();
}

void drawODEBox(dGeomID id, double lx, double ly, double lz)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    const dReal* pos = dGeomGetPosition(id);
    const dReal* rot = dGeomGetRotation(id);
    
    GLfloat mat[] = { rot[0], rot[4], rot[8], 0.,
                      rot[1], rot[5], rot[9], 0.,
                      rot[2], rot[6], rot[10], 0.,
                      pos[0], pos[1], pos[2], 1. };

    glMultMatrixf(mat);
    glScalef(lx, ly, lz);
    
    drawUnitCube();
    
    glPopMatrix();
}

void drawBox(Vector3 p1, Vector3 p2)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    Vector3 p = .5 * (p1 + p2);
    glTranslatef(p.x(), p.y(), p.z());
    Vector3 d = p2 - p1;
    glScalef(d.x(), d.y(), d.z());
    
    drawUnitCube();
    
    glPopMatrix();
}

void drawUnitCube()
{
    glBegin(GL_QUADS);
    
    glNormal3f(-1., 0., 0.);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f(-.5, -.5,  .5);
    glVertex3f(-.5,  .5,  .5);
    glVertex3f(-.5,  .5, -.5);
    
    glNormal3f(0., 0., -1.);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5,  .5, -.5);
    glVertex3f(-.5,  .5, -.5);
    
    glNormal3f(0., -1., 0.);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5, -.5,  .5);
    glVertex3f(-.5, -.5,  .5);
    
    glNormal3f(1., 0., 0.);
    glVertex3f( .5,  .5,  .5);
    glVertex3f( .5,  .5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5, -.5,  .5);

    glNormal3f(0., 0., 1.);
    glVertex3f( .5,  .5,  .5);
    glVertex3f(-.5,  .5,  .5);
    glVertex3f(-.5, -.5,  .5);
    glVertex3f( .5, -.5,  .5);

    glNormal3f(0., 1., 0.);
    glVertex3f( .5,  .5,  .5);
    glVertex3f(-.5,  .5,  .5);
    glVertex3f(-.5,  .5, -.5);
    glVertex3f( .5,  .5, -.5);

    glEnd();
}

void shadowsBeginFloor()
{
    glClear(GL_STENCIL_BUFFER_BIT);
    glEnable(GL_STENCIL_TEST);
    
    glStencilFunc(GL_ALWAYS, 1, 0xffffffff);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
}

void shadowsBeginObjects(Vector3 floor_n, double floor_d)
// Floor is plane with n_x*x + n_y*y + n_z*z = d
{
    GLfloat mod[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, mod);
    
    // Position of light in eye coordinates
    const float px = 1., py =  1., pz = 1.;
    
    // (lx, ly, lz) is position of light in modelview coordinates
    // NOTE: assumes modelview matrix is orthogonal
    float lx =  mod[0]*px +  mod[1]*py +  mod[2]*pz;
    float ly =  mod[4]*px +  mod[5]*py +  mod[6]*pz;
    float lz =  mod[8]*px +  mod[9]*py + mod[10]*pz;
    
    const float nx = floor_n.x(), ny = floor_n.y(), nz = floor_n.z();
    float dot = nx*lx + ny*ly + nz*lz;
    
    glStencilFunc(GL_LESS, 0, 0xffffffff);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    
    glPolygonOffset(-1., -1.);
    glEnable(GL_POLYGON_OFFSET_FILL);
    
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0., 0., 0., .5);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    const GLfloat mat[] = { dot - nx*lx, -nx*ly,       -nx*lz,       0,
                           -ny*lx,        dot - ny*ly, -ny*lz,       0,
                           -nz*lx,       -nz*ly,        dot - nz*lz, 0,
                            floor_d*lx,   floor_d*ly,   floor_d*lz,  dot };
    glMultMatrixf(mat);
}

void shadowsEnd()
{
    glPopMatrix();
    
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_STENCIL_TEST);
}

} // end namespace GL

